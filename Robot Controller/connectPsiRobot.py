import zmq
import msgpack
import runSequence
import time
import sys
import os
import threading
from playsound import playsound
import queue

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Session_pb2, Base_pb2, BaseCyclic_pb2

# This program bridges between perception system (PSI) and the robot, using Kinova Kortex API. Much of
# specific commands here are tailored for the particular task, error, and robot (Kinova Gen3).

# Connecting to PSI to receive commands (e.e., indicators for errors, object requests, query answers)
input = zmq.Context().socket(zmq.SUB)
input.setsockopt_string(zmq.SUBSCRIBE, "commands")
# Fill in the X's for the appropriate IP address to communicate between PSI and this program
input.connect("tcp://XXX.XXX.X.XX:X")

# Connecting to PSI to send notification of whether the robot is moving
output = zmq.Context().socket(zmq.PUB)
# Fill in the X's for the appropriate IP address to communicate between PSI and this program
output.bind("tcp://XXX.XXX.X.XX:X")

# Counter for number of commands for yellow pipes
yellowCount = 0
# Counter for number of commands for green pipes
greenCount = 0
# Flag indicating for the yellow error
isErrorYellow = 0
# Flag indicating for the missing (failing to grab the pipe) error
isErrorMissing = 0
# Flag indicating if the recovery to the yellow error has been executed
isYellowRecover = -1
# Flag indicating if the recovery to the missing error has been executed
isMissingRecover = -1
# Queue logging the sent back to PSI
isWrittenQ = queue.Queue()
# Stores the originating time from the previous message sent from PSI
oldOrigTime = 0
# Separate thread to run the execution commands to the robot
runRobotThread = threading.Thread()
# Flag for whether a possible robot error has been detected
isPossible = 0


# This function reads in messages sent from PSI and outputs the command and the originating
# that was sent through the message. This function is blocking.
def readCommand():
    [topic, payload] = input.recv_multipart()
    message = msgpack.unpackb(payload, raw=True)
    command = message[b"message"]
    originatingTime = message[b"originatingTime"]
    command = command.decode('utf-8')
    return (command, originatingTime)

# This function sends a message to PSI indicating whether the robot is done executing a command
# (whether the robot is moving or not). The inputs are:
# isDone: Flag indicating whether robot is done executing a command
# originatingTime: Time recieved from the incoming message PSI sent that this one is replying to
# In addition, it check to make sure that the message has not already been sent.
def writeRobotDone(isDone, originatingTime):
    # Checks if the reply message has already been sent
    if originatingTime not in isWrittenQ.queue:
        # Logs that the message has been replied to
        isWrittenQ.put(originatingTime)
        # Send the message
        payload = {u"message": isDone, u"originatingTime": originatingTime}
        output.send_multipart(["isDone".encode(), msgpack.dumps(payload)])

# This function loads all of the sequence handles for the pre-programmed robot actions from the
# robot's computer into this program. The input is the base controller for the robot. The outputs
# handles for the pick, error, and recovery sequences.
def sequenceListing(base):
    # Loads the list of pre-progammed sequences stored
    sequenceList = base.ReadAllSequences()
    retractIdx = 0
    greenIdx = 0
    yellowIdx = 0
    missingIdx = 0
    yErrIdx = 0
    mRecoverIdx = 0
    yCRecoverIdx = 0
    yORecoverIdx = 0
    gCRecoverIdx = 0
    gORecoverIdx = 0
    # Iterates through the pre-progammed sequence list
    for i in range(len(sequenceList.sequence_list)):
        seq = sequenceList.sequence_list[i]
        # Checks if the name of the current sequence matches one needed for the task
        if seq.name == "My Retract":
            retractIdx = i
        elif seq.name == "Green_PVC_Pick":
            greenIdx = i
        elif seq.name == "Yellow_PVC_Pick":
            yellowIdx = i
        elif seq.name == "Missing_Green_Error":
            missingIdx = i
        elif seq.name == "Missing_Recovery":
            mRecoverIdx = i
        elif seq.name == "Wrong_Y_Recovery_Close":
            yCRecoverIdx = i
        elif seq.name == "Wrong_Y_Recovery_Open":
            yORecoverIdx = i
        elif seq.name == "General_Recovery_Close":
            gCRecoverIdx = i
        elif seq.name == "General_Recovery_Open":
            gORecoverIdx = i
        elif seq.name == "Yellow_PVC_Error":
            yErrIdx = i
    retractHandle = sequenceList.sequence_list[retractIdx].handle
    greenHandle = sequenceList.sequence_list[greenIdx].handle
    yellowHandle = sequenceList.sequence_list[yellowIdx].handle
    missingHandle = sequenceList.sequence_list[missingIdx].handle
    mRecoverHandle = sequenceList.sequence_list[mRecoverIdx].handle
    yCRecoverHandle = sequenceList.sequence_list[yCRecoverIdx].handle
    yORecoverHandle = sequenceList.sequence_list[yORecoverIdx].handle
    gCRecoverHandle = sequenceList.sequence_list[gCRecoverIdx].handle
    gORecoverHandle = sequenceList.sequence_list[gORecoverIdx].handle
    yErrHandle = sequenceList.sequence_list[yErrIdx].handle
    return (retractHandle, greenHandle, yellowHandle, missingHandle, mRecoverHandle, yCRecoverHandle, gCRecoverHandle, gORecoverHandle, yORecoverHandle, yErrHandle)

# This function sends the command to the robot to run the specified sequence and also send a message afterwards to PSI
# that the execution is done. The inputs are:
# base: Robot's computer client
# base_cyclic: Robot's computer client that waits for a run sequence to be sent
# commandHandle: Sequence handle for the sequence that needs to be executed by the robot
# originatingTime: Command messages' originating time as sent by PSI
def runRobotCommand(base, base_cyclic, commandHandle, originatingTime):
    # Calls function to execute the robot sequence and returns value when done. This function is blocking.
    isDone = runSequence.run_sequence(base, base_cyclic, commandHandle)
    # Calls function to send response to command message stating execution is done.
    writeRobotDone(isDone, originatingTime)
    return

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
import utilities

# Parse arguments
args = utilities.parseConnectionArguments()
    
# Create connection to the robot and get the router
with utilities.DeviceConnection.createTcpConnection(args) as router:
    # Create required services for robot to run the sequences
    base = BaseClient(router)
    base_cyclic = BaseCyclicClient(router)
    
    # Load the gripper object
    gripperRequest = Base_pb2.GripperRequest()
    # Gets the status of the robot's gripper: position of opening
    gripperRequest.mode = Base_pb2.GRIPPER_POSITION
    
    # Calls function to load all of the sequence handles needed for the task
    retractHandle, greenHandle, yellowHandle, missingHandle, mRecoverHandle, yCRecoverHandle, gCRecoverHandle, gORecoverHandle, yORecoverHandle, yErrHandle = sequenceListing(base)
    # Run an initial retract sequence on the robot to bring it to a neutral position
    runSequence.run_sequence(base, base_cyclic, retractHandle)
    originatingTime = 0
    
    # Continous while that waits for commands to come in
    while True:
        # Sets the previous message's originating time
        oldOrigTime = originatingTime
        # Calls function to read in incoming message
        command, originatingTime = readCommand()
        
        # Set sequence handle to default retract
        commandHandle = retractHandle
        # Checks if the command is requesting a pipe
        if command == "yellow" or command == "green":
            # Checks if the command requests a yellow pipe
            if command == "yellow":
                # Sets the sequence handle to the sequence that retrieves the yellow pipe
                commandHandle = yellowHandle
                # Increases the yellow command count
                yellowCount+=1
            # Checks if the command requests a green pipe
            elif command == "green":
                # Sets the sequence handle to the sequence that retrieves the green pipe
                commandHandle = greenHandle
                # Increases the green command count
                greenCount+=1

            # Manually pre-programmed error override statement for having the robot run the errors
            # Checks if the yellow pipe command count is 9, if the error has already happened, and if it already
            # has recovered from that error. Replaces the sequence handle with the corresponding error sequence handle
            if yellowCount == 9 and isErrorYellow == 0 and isYellowRecover == -1:
                isErrorYellow = 1
                # Runs the wrong object error (grabbing green instead of yellow) in a separate thread
                runRobotThread = threading.Thread(target=runRobotCommand, args=(base, base_cyclic, yErrHandle, originatingTime))
            # Checks if the green pipe command count is 10, if the error has already happened, and if it already
            # has recovered from that error. Replaces the sequence handle with the corresponding error sequence handle
            elif greenCount == 10 and isErrorMissing == 0 and isMissingRecover == -1:
                isErrorMissing = 1
                # Runs the wrong object error (grabbing green instead of yellow) in a separate thread
                runRobotThread = threading.Thread(target=runRobotCommand, args=(base, base_cyclic, missingHandle, originatingTime))
            # Otherwise runs the requested pipe's pick retrieval sequence
            else:
                runRobotThread = threading.Thread(target=runRobotCommand, args=(base, base_cyclic, commandHandle, originatingTime))
            
            # Initiates the thread to run the robot
            runRobotThread.start()
        
        # Checks if the command sent from PSI indicates that an error has been detected
        elif command == "error":
            # Checks if the error was detected using the explicit indicator
            if isPossible == 0:
                # Calls function to send message stating robot is no longer moving
                writeRobotDone(False, oldOrigTime)
            # Checks if the error was detected using the implicit indicator (socials signal ml algorithm and domain-specific input)
            elif isPossible == 1:
                base.ResumeSequence()
                isPossible = 0
                
            # Stops the current sequence
            base.StopSequence()
            # Fill in audio if want to play sound after error is detected
            playsound("XXXXX.mp3")
            
            # Gets the current position of the gripper fingers
            gripperMeasure = base.GetMeasuredGripperMovement(gripperRequest)

            # Checks if the error is the wrong object (yellow) error
            if isErrorYellow == 1:
                isErrorYellow = 0
                # Sets that the error is being recovered from
                isYellowRecover = 1
                # Checks if the gripper is closed
                if len(gripperMeasure.finger) and (gripperMeasure.finger[0].value > 0.009):
                    # Call function to run the appropriate recovery sequence knowing the gripper is closed
                    isDone = runSequence.run_sequence(base, base_cyclic, yCRecoverHandle)
                else:
                    # Call function to run the appropriate recovery sequence knowing the gripper is opem
                    isDone = runSequence.run_sequence(base, base_cyclic, yORecoverHandle)
            # Checks if the error is failing to grab the pipe
            elif isErrorMissing == 1:
                isErrorMissing = 0
                # Sets that the error is being recovered from
                isMissingRecover = 1
                # Calls function to run the appropriate recovery sequence
                isDone = runSequence.run_sequence(base, base_cyclic, mRecoverHandle)
            # If error detected is neither, then robot executes a generic recovery
            else:
                # Check if gripper is closed
                if len(gripperMeasure.finger) and (gripperMeasure.finger[0].value > 0.009):
                    isDone = runSequence.run_sequence(base, base_cyclic, gCRecoverHandle)
                else:
                    isDone = runSequence.run_sequence(base, base_cyclic, gORecoverHandle)
            # Calls function to send message indicating that the robot is done running
            writeRobotDone(True, originatingTime)
        # Checks if the command sent from PSI indicates that an error has been detected by the social signal ml algorithm
        elif command == "possible":
            # Pause the robot from moving
            base.PauseSequence()
            # Flag that a possible error has occured
            isPossible = 1
            # Fill in audio, if want to query the participant whether the error indeed happen
            playsound("XXXX.mp3")
            # Send message that the previous pipe command was recieved
            writeRobotDone(False, oldOrigTime)
            # Send message that the possible detection message has been recieved
            writeRobotDone(False, originatingTime)
        # Checks if the command sent from PSI indicates that the participant stated no error has occurred post social signal ml algorithm detection
        elif command == "resume":
            isPossible = 0
            # Continue executing the pipe command
            base.ResumeSequence()
            # Send message that the possible detection message has been recieved
            writeRobotDone(True, originatingTime)
            
