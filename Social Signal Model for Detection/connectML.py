import zmq
import msgpack
import torch
import torch.nn as nn
import random
import queue

# This program detects if a potential error has occured based on AUs.
# It contains a binary classier that classifies AUs per timestep as
# either an error or a non-error timestep. Then classified timesteps
# are fed into a sliding window that determines if an error has occured.
# The program communicates with PSI.

# Subscribe socket that sends AUs per timestep and whether or not the robot is
# moving as one 1D array of size 18 (isMoving and 17 AUs)
input = zmq.Context().socket(zmq.SUB)
input.setsockopt_string(zmq.SUBSCRIBE, "AUs Intensities")
# Fill in the X's for the appropriate IP address to communicate between PSI and this program
input.connect("tcp://XXX.X.X.X:X")

# Publish to socket that outputs new error or not
output = zmq.Context().socket(zmq.PUB)
# Fill in the X's for the appropriate IP address to communicate between PSI and this program
output.bind("tcp://XXX.X.X.X:X")

# Checking to see if gpu is available
gpuBoole = torch.cuda.is_available()
print(gpuBoole)
device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

torch.manual_seed(20)
random.seed(20)

# A timestep comprised of 17 AUs is the input for the algorithm
inputSize = 17
# Threshold for the sliding window to determine if an error as occured
threshold = 6
# Size of the sliding window
windowSize = 11
# Represents the contents for the sliding window
slidingWindowQPred = queue.Queue()
slidingWindowQConf = queue.Queue()
count = 0
# Sum of the confidence classifications for the window
runningWindowSum = 0
# Previous error detected timestep
prevStopNum = -1
# Previous estimated error start
prevStartReact = -1

# Define the archetecture of the binary classifier
class BinaryClassifier(nn.Module):
  def __init__(self,):
    super(BinaryClassifier, self).__init__()
    self.layer1 = nn.Linear(inputSize, 64)
    self.relu = nn.ReLU()
    self.layer2 = nn.Linear(64, 128)
    self.layer3 = nn.Linear(128, 64)
    self.outL = nn.Linear(64, 2)
    self.dropout = nn.Dropout(p=0.1)
    self.softmax = nn.Softmax()

  def forward(self, x):
    l1 = self.relu(self.layer1(x))
    l2 = self.relu(self.layer2(l1))
    l2 = self.dropout(l2)
    l3 = self.relu(self.layer3(l2))
    l3 = self.dropout(l3)
    out = self.outL(l3)
    out = self.softmax(out)
    return out

# Initialize model
model = BinaryClassifier()
# Load the model
model.load_state_dict(torch.load('CaseFullDict'))
model.eval()
model.to(device)


# This function reads in messages sent from PSI and outputs the 1D array of
# calculated AUs for the timestep and the originating
# that was sent through the message. This function is blocking.
def readAUCalced():
    [topic, payload] = input.recv_multipart()
    message = msgpack.unpackb(payload, raw=True)
    AUsCalced = message[b"message"]
    oT = message[b"originatingTime"]
    return (AUsCalced, oT)

# This function sends a message to PSI indicating whether an error has been
# detected. The inputs are:
# isError: 1D array comprised of [whether the robot is moving, classification of
# timestep, classification confidence, is new error detected]
# originatingTime: Time recieved from the incoming message PSI sent that this one is replying to.
def writeCommand(isError, originatingTime):
    isNewError = ",".join([str(i) for i in isError])
    payload = {u"message":isNewError, u"originatingTime":originatingTime}
    output.send_multipart(["isNewError".encode(), msgpack.dumps(payload)])

# This function is the binary classifier and classifies a set of 17 AUs (1 timestep)
# as either error or no error. The arguments are:
# model: The classifier loaded from file.
# timeStepAU: 1D list of AU intensities
# gpuBoole: Flag indicating the gpu
# The outputs are the classification (1 for error, 0 for no error) and the weighted
# classfication confidence
def runML(model, timeStepAU, gpuBoole):
    intensitiesTimeStep = torch.tensor(timeStepAU)
    with torch.no_grad():
        intensitiesTimeStep = intensitiesTimeStep.view(-1, inputSize).to(torch.float)
        if gpuBoole:
            intensitiesTimeStep = intensitiesTimeStep.cuda()
        # Classify the timestep
        y_hat = model(intensitiesTimeStep)
        predicted = y_hat.argmax(dim=1)
    return float(predicted.item()), float((predicted * y_hat[:, 1]).item())

# This function is the sliding window that determines if a new error has been
# detected. A new error can only be detected if the robot is moving (specific to
# the task). The inputs are:
# predicted: Timestep classification (1 or 0)
# confidence: Classification confidence
# index: Current timestep index over the entire data collection
# isMoving: Flag indicating if the robot is moving at the timestep
# The outputs are the timestep index if it was a new error and estimated index
# of error start. If it is not a new error, then the function outputs -1 for both.
def postPredictionWindow(predicted, confidence, index, isMoving):
    global runningWindowSum
    global prevStopNum
    global prevStartReact

    # Check if the robot is not moving
    if isMoving == 0:
        # Reset the sliding window because the robot is no longer moving
        slidingWindowQPred.queue.clear()
        slidingWindowQConf.queue.clear()
        runningWindowSum = 0
    else:
        # Check if the window is the correct size
        if slidingWindowQPred.qsize() == windowSize:
            # Remove oldest value from the queue
            p = slidingWindowQPred.get()
            c = slidingWindowQConf.get()
            # Remove that value from the running weighted sum
            runningWindowSum = runningWindowSum - c
        # Add the currently newest classified value into the window
        slidingWindowQPred.put(predicted)
        slidingWindowQConf.put(confidence)
        # Add the new timestep classification to the running sum
        runningWindowSum = runningWindowSum + confidence

        # Check if the running sum is the above the threshold to see if error
        # is detected
        if runningWindowSum >= threshold:
            # Set the detected error timestep
            stopNum = index
            queueCount = 0
            # Iterate through queue and to determine the estimated error start
            for i in slidingWindowQPred.queue:
                # Check if the current timestep in the window is an error one
                if i == 1:
                    # Calculate the current estimated error start for the detection
                    startReact = index - (slidingWindowQPred.qsize() - queueCount) + 1
                    break
                queueCount += 1
                
            # Check if the estimated error start and detected error timestep are
            #within one of the previously indicated one to see if it is the same
            # error.
            if stopNum != prevStopNum + 1 and startReact != prevStartReact + 1:
                prevStopNum = stopNum
                prevStartReact = startReact
                return stopNum, startReact
            prevStopNum = stopNum
            prevStartReact = startReact
    # return -1 -1 if no error was detected
    return -1, -1

# Continous while that waits for AU timesteps to come in
while True:
    AUsCalced, oT = readAUCalced()
    # Extract the 17 AUs from the message payload
    timeStepAU = AUsCalced[1:]
    # Extract whether the robot was moving
    isMoving = AUsCalced[0]
    # Calls function to run the binary classifier on the AUs
    predictedClass, predictedConfidence = runML(model, timeStepAU, gpuBoole)
    # Calls function to run the sliding window on the output from the binary classifier
    stopNum, estimatedStart = postPredictionWindow(predictedClass, predictedConfidence, count, isMoving)
    # Checks if a new error was detected
    if stopNum != -1 and estimatedStart != -1:
        # Calls function to send reply to PSI stating the error was detected
        writeCommand([float(isMoving), float(predictedClass), float(predictedConfidence), float(1)], oT)
    else:
        # Calls function to send reply to PSI stating no error was detected
        writeCommand([float(isMoving), float(predictedClass), float(predictedConfidence), float(0)], oT)
    count += 1



