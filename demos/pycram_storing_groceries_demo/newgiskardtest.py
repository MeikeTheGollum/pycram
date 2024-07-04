import numpy as np
import rospy

from demos.pycram_storing_groceries_demo.utils.misc import *
from pycram.designators.location_designator import find_placeable_pose
from pycram.language import Code

from pycram.ros.robot_state_updater import RobotStateUpdater, KitchenStateUpdater
from pycram.designators.action_designator import *
from pycram.enums import ObjectType
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard_new as giskardpy
# import pycram.external_interfaces.giskard as giskardpy_old

import pycram.external_interfaces.robokudo as robokudo
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, StartSignalWaiter, \
    HSRBMoveGripperReal
from pycram.designator import LocationDesignatorDescription
import random

world = BulletWorld()
v = VizMarkerPublisher()
kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "suturo_lab_version_15.urdf")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

talk = TextToSpeechPublisher()
img_swap = ImageSwitchPublisher()
start_signal_waiter = StartSignalWaiter()
move = PoseNavigator()
gripper = HSRBMoveGripperReal()
robot = Object("hsrb", "robot", "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"])
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
KitchenStateUpdater("/tf", "/iai_kitchen/joint_states")

giskardpy.init_giskard_interface()
# #robokudo.init_robokudo_interface()
# #rospy.sleep(2)
# giskardpy.spawn_kitchen()
#
# # Shelf Variables
# shelf_pose = Pose([4.417880842356951,4.913135736923778, 0] ,[0, 0, 0, 1])
# lower_compartment = ShelfCompartmentDescription(height=0.09, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
# middle_compartment = ShelfCompartmentDescription(height=0.37, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
# upper_compartment = ShelfCompartmentDescription(height=0.71, placing_areas=[[5.1, 5.5], [4.8, 5.1], [4.35, 4.8]])
# shelves = [lower_compartment, middle_compartment, upper_compartment]


shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0], [0.0, 0.0, 0, 1])
rotated_shelf_pose = Pose([4.375257854937237, 4.991582584825204, 0.0],
                          [0.0, 0.0, 0.7220721042045632, 0.6918178057332686])
table_pose = Pose([2.862644998141083, 5.046512935221523, 0.0], [0.0, 0.0, 0.7769090622619312, 0.6296128246591604])

# List of objects
objects = [
    "Fork", "Pitcher", "Bleachcleanserbottle", "Crackerbox", "Minisoccerball",
    "Baseball", "Mustardbottle", "Jellochocolatepuddingbox", "Wineglass", "Orange",
    "Coffeepack", "Softball", "Metalplate", "Pringleschipscan", "Strawberry",
    "Glasscleanerspraybottle", "Tennisball", "Spoon", "Metalmug", "Abrasivesponge",
    "Jellobox", "Dishwashertab", "Knife", "Cerealbox", "Metalbowl", "Sugarbox",
    "Coffeecan", "Milkpackja", "Apple", "Tomatosoupcan", "Tunafishcan",
    "Gelatinebox", "Pear", "Lemon", "Banana", "Pottedmeatcan", "Peach",
    "Plum", "Rubikscube", "Mueslibox", "Cupblue", "Cupgreen", "Largemarker",
    "Masterchefcan", "Scissors", "Scrubcleaner", "Grapes", "Cup_small",
    "screwdriver", "clamp", "hammer", "wooden_block", "Cornybox*"
]

popcorn_frame = "popcorn_table:p_table:table_center"
# Group objects by similarity
groups = {
    "Kitchen Utensils and Tools": ["Fork", "Spoon", "Knife"],
    "Containers and Drinkware": ["Pitcher", "Metalplate", "Metalbowl", "Metalmug", "Wineglass", "Cupblue", "Cupgreen",
                                 "Cup_small"],
    "Cleaning Supplies": ["Bleachcleanserbottle", "Glasscleanerspraybottle", "Abrasivesponge", "Scrubcleaner"],
    "Packaged Food and Beverages": ["Crackerbox", "Mustardbottle", "Jellochocolatepuddingbox", "Coffeepack",
                                    "Pringleschipscan", "Jellobox", "Dishwashertab", "Cerealbox", "Sugarbox",
                                    "Coffeecan", "Milkpackja", "Tomatosoupcan", "Tunafishcan", "Gelatinebox",
                                    "Mueslibox", "Cornybox*", "Masterchefcan", "Pottedmeatcan"],
    "Fruits": ["Orange", "Strawberry", "Apple", "Pear", "Lemon", "Banana", "Peach", "Plum", "Grapes"],
    "Sports Equipment": ["Minisoccerball", "Baseball", "Softball", "Tennisball"],
    "Miscellaneous": ["Rubikscube", "Largemarker", "Scissors", "screwdriver", "clamp", "hammer", "wooden_block"]
}

links_from_shelf = [
    'shelf:shelf:shelf_floor_0',
    'shelf:shelf:shelf_floor_1',
    'shelf:shelf:shelf_floor_2',
]


def find_group(obj):
    obj_lower = obj.lower()
    for group, items in groups.items():
        for item in items:
            if obj_lower in item.lower() or item.lower() in obj_lower:
                return group
    return None


def get_closet_link_to_pose(obj_pose):
    position_distance = 30
    nearest_link = None
    for link in links_from_shelf:
        link_pose = kitchen.get_link_pose(link)
        posez = obj_pose.position.z

        dis = abs(link_pose.pose.position.z - posez)
        if dis <= position_distance:
            position_distance = dis
            nearest_link = link
    return nearest_link


def get_closest_pose(obj_pose, pose_list):
    position_distance = float('inf')
    nearest_pose = None

    for pose in pose_list:
        dis = ((obj_pose.pose.position.x - pose.position.x) ** 2 +
               (obj_pose.pose.position.y - pose.position.y) ** 2 +
               (obj_pose.pose.position.z - pose.position.z) ** 2) ** 0.5
        if dis < position_distance:
            position_distance = dis
            nearest_pose = pose

    return nearest_pose


def pickerino(object_desig, grasp, arm, talk, frame):
    lt = LocalTransformer()
    robot = BulletWorld.robot
    # Retrieve object and robot from designators
    object = object_desig.bullet_world_object
    # Calculate the object's pose in the map frame
    oTm = object.get_pose()
    execute = True

    if grasp == "top":
        oTm.pose.position.z += 0.035

    grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
    oTb = lt.transform_pose(oTm, kitchen.get_link_tf_frame(frame))
    oTb.orientation = grasp_rotation
    oTmG = lt.transform_pose(oTb, "map")

    rospy.logwarn("Opening Gripper")
    gripper.pub_now("open")

    rospy.logwarn("Picking up now")
    BulletWorld.current_bullet_world.add_vis_axis(oTmG)

    tool_frame = robot_description.get_tool_frame(arm)
    special_knowledge_offset = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))
    y = 0.04
    special_knowledge_offset.pose.position.y -= y
    if grasp == "top":
        z = 0.025
        special_knowledge_offset.pose.position.z -= z

    push_baseTm = lt.transform_pose(special_knowledge_offset, "map")
    special_knowledge_offsetTm = lt.transform_pose(special_knowledge_offset, "map")
    liftingTm = push_baseTm
    liftingTm.pose.position.z += 0.03
    talk.pub_now("Pick Up now!" + object.type)
    giskard_return = giskardpy.achieve_sequence_pick_up(oTmG, special_knowledge_offsetTm)
    giskardpy.achieve_attached(object_desig)
    tip_link = 'hand_gripper_tool_frame'
    BulletWorld.robot.attach(object=object_desig.bullet_world_object, link=tip_link)
    gripper.pub_now("close")


def pakerino(torso_z=0.15):
    config = {'arm_lift_joint': torso_z, 'arm_flex_joint': 0, 'arm_roll_joint': -1.2, 'wrist_flex_joint': -1.5,
              'wrist_roll_joint': 0}
    giskardpy.avoid_all_collisions()
    giskardpy.achieve_joint_goal(config)
    print("Parking done")


def pakerino(torso_z=0.18):
    config = {'arm_lift_joint': torso_z, 'arm_flex_joint': 0, 'arm_roll_joint': -1.2, 'wrist_flex_joint': -1.5,
              'wrist_roll_joint': 0}
    giskardpy.avoid_all_collisions()
    giskardpy.achieve_joint_goal(config)
    print("Parking done")


def monitor_func_place():
    der = fts.get_last_value()
    if abs(der.wrench.force.x) > 10.30:
        return SensorMonitoringCondition
    return False


def placerino(object_desig, grasp, arm, talk, target_location, frame):
    lt = LocalTransformer()
    robot = BulletWorld.robot
    oTm = target_location
    oTm.pose.position.z += 0.015
    # if grasp == "top":
    #     oTm.pose.position.z += 0.035

    grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
    oTb = lt.transform_pose(oTm, kitchen.get_link_tf_frame(frame))
    oTb.orientation = grasp_rotation
    special_knowledge_offset = oTb
    # x = 0.04
    # special_knowledge_offset.pose.position.x -= x
    # if grasp == "top":
    #     y = 0.025
    #     special_knowledge_offset.pose.position.y -= y
    oTmG = lt.transform_pose(oTb, "map")
    # push_baseTm = lt.transform_pose(special_knowledge_offset, "map")

    # tool_frame = robot_description.get_tool_frame(arm)
    # special_knowledge_offset = lt.transform_pose(oTmG, robot.get_link_tf_frame(tool_frame))

    # push_baseTm = lt.transform_pose(special_knowledge_offset, "map")
    # special_knowledge_offsetTm = lt.transform_pose(special_knowledge_offset, "map")
    # push_baseTm.pose.position.z -= 1
    # world.current_bullet_world.add_vis_axis(oTmG)
    # world.current_bullet_world.add_vis_axis(push_baseTm)
    talk.pub_now("Placing now!")
    # giskardpy.achieve_placing_without_prepose(oTmG, object_desig, kitchen)
    giskardpy.achieve_sequence_te(oTmG, object_desig)
    gripper.pub_now("open")
    # try:
    #     plan = Code(lambda: giskardpy.achieve_sequence_te(push_baseTm)) >> Monitor(monitor_func_place)
    #     plan.perform()
    # except SensorMonitoringCondition:
    #     rospy.logwarn("interrupted")
    # talk.pub_now("opening my gripper")
    # MoveGripperMotion(motion="open", gripper="left").resolve().perform()
    # robot.detach_all()


def multiply_quaternions(q1, q2):
    """
    Multiply two quaternions.

    Parameters:
    q1 (tuple): First quaternion (x1, y1, z1, w1).
    q2 (tuple): Second quaternion (x2, y2, z2, w2).

    Returns:
    tuple: The product of the two quaternions.
    """

    x1, y1, z1, w1 = q1.x, q1.y, q1.z, q1.w
    x2, y2, z2, w2 = q2.x, q2.y, q2.z, q2.w

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
    return (x, y, z, w)


# Example usage:


def demo(step):
    with real_robot:
        # Wait for the start signal
        gripper.pub_now("close")
        pakerino()

        locationtoplace = Pose([2.6868790796016738, 5.717920690528091, 0.715])

        # #giskardpy.spawn_kitchen()
        groups_in_shelf = {
            "Kitchen Utensils and Tools": None,
            "Containers and Drinkware": None,
            "Cleaning Supplies": None,
            "Packaged Food and Beverages": None,
            "Fruits": None,
            "Sports Equipment": None,
            "Miscellaneous": None
        }
        groups_on_table = {}

        if step <= 3:
            rospy.logerr("driving")
            talk.pub_now("driving", True)
            # move.pub_now(table_pose)

            look_pose = kitchen.get_link_pose("popcorn_table:p_table:table_center")
            # look_pose.pose.position.x += 0.5
            giskardpy.move_head_to_pose(locationtoplace)
            # giskardpy.move_head_to_pose(look_pose)
            # LookAtAction(targets=[look_pose]).resolve().perform()  # 0.18
            # LookAtAction(targets=[look_pose]).resolve().perform()
            table_obj = DetectAction(technique='all').resolve().perform()
            first, *remaining = table_obj
            for dictionary in remaining:
                for value in dictionary.values():
                    try:
                        group = find_group(value.type)
                        groups_on_table[value.name] = [value, group]

                    except AttributeError:
                        pass
        obj_to_place = None
        if step <= 4:

        if step <= 4:
            # print(groups_on_table.keys())
            for obj in groups_on_table.values():
                lt = LocalTransformer()
                robot = BulletWorld.robot
                # Retrieve object and robot from designators
                object = obj[0].bullet_world_object
                # Calculate the object's pose in the map frame
                oTm = object.get_pose()
                execute = True

                oTb = lt.transform_pose(oTm, kitchen.get_link_tf_frame(popcorn_frame))

                # angle = helper.quaternion_to_angle((oTb.pose.orientation.x, oTb.pose.orientation.y,
                #                                     oTb.pose.orientation.z, oTb.pose.orientation.w))
                # print(object.name)
                # print("Angle of rotation:", angle)
                # object_height = object.get_object_dimensions()[2]
                # print(object_height)
                # # if object_height < 0.07:
                # #     grasp = "top"
                # #     #todo dann musst du hier eign vom dem support surface nehmengeiilo
                # if angle > 75:
                #     grasp = "top"
                #     oTb.pose.position.z += (object_height/ 2 / 2)
                # else: grasp = "front"
                #
                # if object_height < 0.05:
                #     grasp = "top"
                #     oTb.pose.position.z = 0.03
                # grasp_rotation = robot_description.grasps.get_orientation_for_grasp(grasp)
                # grasp_q = Quaternion(grasp_rotation[0],grasp_rotation[1],grasp_rotation[2],grasp_rotation[3])
                # oTb.orientation = multiply_quaternions(oTb.pose.orientation, grasp_q)
                oTmG = lt.transform_pose(oTb, "map")

                # result =
                # oTm.pose.orientation = result
                BulletWorld.current_bullet_world.add_vis_axis(oTmG)

                # pickerino(obj[0], "front", "left", talk, popcorn_frame)
                # giskardpy.avoid_all_collisions()
                # pakerino()
                #
                #
                # obj_to_place = obj[0]
                # # PickUpAction(obj[0], ["left"], ["front"]).resolve().perform()
                # locationtoplace = Pose([2.7868790796016738, 5.617920690528091, 0.815])
                # placerino(obj_to_place, "front", "left", talk, locationtoplace, popcorn_frame)
                # pakerino()
                gripper.pub_now("close")


demo(0)
