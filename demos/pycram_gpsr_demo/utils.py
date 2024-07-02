import inspect

import rospy
from pycram.pose import Pose

from . import high_level_plans


# generate a list of all plans instead of having to hardcode them
# this is required for the mapping between NLP and PyCRAM
def get_plans(module):
    module_name = module.__name__
    return {name: obj for name, obj in inspect.getmembers(module)
            if inspect.isfunction(obj) and obj.__module__ == module_name}


def call_plan_by_name(plan_list, name, *args, **kwargs):
    func = plan_list.get(name)
    if func:
        func(*args, **kwargs)
        rospy.loginfo(f"[CRAM] plan {name} found and executed.")
    else:
        rospy.logerr(f"[CRAM] Plan {name} not found.")


def kpose_to_pose_stamped(k_pose):
    # find a better way? maybe via knowrob?
    pose = Pose(frame=k_pose.get('Frame'), position=k_pose.get('Pose'), orientation=k_pose.get('Quaternion'))
    return pose


# --- repl specific ---
def reset():
    #global todo_list, plans_list
    todo_list = []
    plans_list = get_plans(high_level_plans)
    rospy.loginfo("[CRAM] reset done.")