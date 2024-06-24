import rospy

from demos.pycram_gpsr_demo import track_human, nlp_listening
from pycram.utilities.robocup_utils import StartSignalWaiter
from demos.pycram_gpsr_demo.setup_demo import *
import utils


# --- main control ---
# TODO: test on real robot
def gpsr():
    # init params

    rospy.loginfo("Main GPSR High-Level-Plan started")
    # wait for start signal
    # Create an instance of the StartSignalWaiter
    start_signal_waiter = StartSignalWaiter()
    # Wait for the start signal
    start_signal_waiter.wait_for_startsignal()
    # Once the start signal is received, continue with the rest of the script
    rospy.loginfo("Start signal received, now proceeding with tasks.")

    # TODO this should be looped for multiple tasks
    # go to initial point
    move.query_pose_nav(instruction_point)

    # listen to instructions
    tts.pub_now("Hello, I am Toya. Please step in front of me.")
    image_switch.pub_now(0)

    track_human() # look for a human and track them

    tts.pub_now("Please talk to me after the beep.")
    image_switch.pub_now(1)
    sound_pub.publish_sound_request() # this is the beep

    # listen to instructions
    instruction_list = nlp_listening()

    # TODO iterate over list of instructions and do stuff
    for instruction in instruction_list:
        rospy.loginfo("in instruction loop")
        # do stuff
        # match instruction to plan
        utils.call_plan_by_name(plan_list, "cleaning", "test")

    # execute instructions


def test_nlp_repl():
    image_switch.pub_now(0)
    tts.pub_now("testing.")
    image_switch.pub_now(1)
    temp = nlp_listening()
    if len(temp) == 0:
        tts.pub_now("I am sorry, I didn't quite get that")
    else:
        tts.pub_now("I have heared: " + temp['sentence'])

