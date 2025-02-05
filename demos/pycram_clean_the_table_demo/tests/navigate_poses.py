import rospy

from demos.pycram_serve_breakfast_demo.utils.misc import get_bowl, sort_objects, try_pick_up, get_free_spaces
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.designators.object_designator import *
from pycram.external_interfaces import giskard
from pycram.process_module import real_robot, semi_real_robot
from pycram.ros_utils.viz_marker_publisher import VizMarkerPublisher
from pycram.ros_utils.robot_state_updater import RobotStateUpdater
from pycram.utils import axis_angle_to_quaternion
from pycram.world_concepts.world_object import Object
from pycram.worlds.bullet_world import BulletWorld

# Initialize the Bullet world for simulation
world = BulletWorld()

# Visualization Marker Publisher for ROS
v = VizMarkerPublisher()

# Create and configure the robot object
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
# Update robot state
RobotStateUpdater("/tf", "/giskard_joint_states")

#robot.set_color([0.5, 0.5, 0.9, 1])

# TODO: change urdf
# Create environmental objects
apartment = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_2024_1.urdf")

# Define orientation for objects
object_orientation = axis_angle_to_quaternion([0, 0, 1], 180)


# name of the dishwasher handle and dishwasher door
handle_name = "sink_area_dish_washer_door_handle"
door_name = "sink_area_dish_washer_door"
dishwasher_main_name = "sink_area_dish_washer_main"


class NavigatePose(Enum):
    DISHWASHER_CLOSED = Pose([2.75, -2.1, 0], [0, 0, -1, 1])
    DISHWASHER = Pose([2.65, -1.85, 0], [0, 0, -1, 1])
    SHELF = Pose([4.5, 3.95, 0], [0, 0, 0, 1])
    TRESSE = Pose([1.95, 4, 0], [0, 0, 0.7, 0.7])
    LONG_TABLE = Pose([1.7, 0.8, 0], [0, 0, 1, 0])


class NavigateOrientation(Enum):
    RIGHT = [0, 0, -1, 1]
    LEFT = [0, 0, -1, 1]
    ABOVE = [0, 0, -1, 1]
    BELOW = [0, 0, -1, 1]

# TODO: change postions of navigating, pickup, placing, etc.
with (real_robot):
    print(robot.get_pose().pose)
    # NavigateAction([Pose([2.8, -2.1, 0], [0, 0, -1, 1])]).resolve().perform()
