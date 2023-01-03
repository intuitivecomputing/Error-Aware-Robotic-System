import time
import sys
import os
import threading

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.BaseCyclicClientRpc import BaseCyclicClient

from kortex_api.autogen.messages import Session_pb2, Base_pb2, BaseCyclic_pb2

## Adapted from Kinova Kortex API examples sequence.py:
# https://github.com/Kinovarobotics/kortex/tree/master/api_python/examples
# Code specifically relevant for Kinova Gen3 robot arm

# Maximum allowed waiting time during actions (in seconds)
TIMEOUT_DURATION = 240

# Create closure to set an event after an END or an ABORT
def check_for_sequence_end_or_abort(e):
    """Return a closure checking for END or ABORT notifications on a sequence
    Arguments:
    e -- event to signal when the action is completed
        (will be set when an END or ABORT occurs)
    """

    def check(notification, e = e):
        event_id = notification.event_identifier
        task_id = notification.task_index
        if event_id == Base_pb2.SEQUENCE_TASK_COMPLETED:
            print("Sequence task {} completed".format(task_id))
        elif event_id == Base_pb2.SEQUENCE_ABORTED:
            print("Sequence aborted with error {}:{}"\
                .format(\
                    notification.abort_details,\
                    Base_pb2.SubErrorCodes.Name(notification.abort_details)))
            e.set()
        elif event_id == Base_pb2.SEQUENCE_COMPLETED:
            print("Sequence completed.")
            e.set()
    return check

# This function sends command to the robot to run the specified sequence.
# The sequences were pre-programmed.
def run_sequence(base, base_cyclic, commandHandle):
    e = threading.Event()
    notification_handle = base.OnNotificationSequenceInfoTopic(
        check_for_sequence_end_or_abort(e),
        Base_pb2.NotificationOptions()
    )

    print("Executing it")
    base.PlaySequence(commandHandle)

    print("Waiting for movement to finish ...")
    finished = e.wait(TIMEOUT_DURATION)
    base.Unsubscribe(notification_handle)

    if not finished:
        print("Timeout on action notification wait")
    return finished

def main():
    # Import the utilities helper module
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    import utilities

    # Parse arguments
    args = utilities.parseConnectionArguments()
    
    # Create connection to the device and get the router
    with utilities.DeviceConnection.createTcpConnection(args) as router:

        # Create required services
        base = BaseClient(router)
        base_cyclic = BaseCyclicClient(router)

        # Example core
        success = True
        success &= run_sequence(base, base_cyclic, item)
        
        #return 0 if success else 1

#if __name__ == "__main__":
    #exit(main())
    #main()
