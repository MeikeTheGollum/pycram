from threading import Lock
from threading import Lock
from typing import Any

import numpy as np
import rospy
from tmc_control_msgs.msg import GripperApplyEffortActionGoal
from tmc_msgs.msg import Voice

import pycram.bullet_world_reasoning as btr
from ..designators.motion_designator import *
from ..enums import JointType, ObjectType
from ..external_interfaces import giskard
from ..external_interfaces.ik import request_ik
from ..external_interfaces.robokudo import queryEmpty, queryHuman, stop_queryHuman
from ..helper import _apply_ik
from ..local_transformer import LocalTransformer
from ..external_interfaces.navigate import queryPoseNav
from ..process_module import ProcessModule


def _park_arms(arm):
    """
    Defines the joint poses for the parking positions of the arms of HSRB and applies them to the
    in the BulletWorld defined robot.
    :return: None
    """

    robot = BulletWorld.robot
    if arm == "left":
        for joint, pose in robot_description.get_static_joint_chain("left", "park").items():
            robot.set_joint_state(joint, pose)


class HSRBNavigation(ProcessModule):
    """
    The process module to move the robot from one position to another.
    """

    def _execute(self, desig: MoveMotion.Motion):
        robot = BulletWorld.robot
        robot.set_pose(desig.target)


class HSRBPickUp(ProcessModule):
    """
    This process module is for picking up a given object.
    The object has to be reachable for this process module to succeed.
    """

    def _execute(self, desig: PickUpMotion.Motion):
        object = desig.object_desig.bullet_world_object
        robot = BulletWorld.robot
        grasp = robot_description.grasps.get_orientation_for_grasp(desig.grasp)
        target = object.get_pose()
        target.orientation.x = grasp[0]
        target.orientation.y = grasp[1]
        target.orientation.z = grasp[2]
        target.orientation.w = grasp[3]

        arm = desig.arm
        arm_short = "r" if arm == "right" else "l"

        _move_arm_tcp(target, robot, arm)
        tool_frame = robot_description.get_tool_frame(arm)
        robot.attach(object, tool_frame)


class HSRBPlace(ProcessModule):
    """
    This process module places an object at the given position in world coordinate frame.
    """

    def _execute(self, desig: PlaceMotion.Motion):
        """

        :param desig: A PlaceMotion
        :return:
        """
        object = desig.object.bullet_world_object
        robot = BulletWorld.robot
        arm = desig.arm

        # Transformations such that the target position is the position of the object and not the tcp
        object_pose = object.get_pose()
        local_tf = LocalTransformer()
        tcp_to_object = local_tf.transform_pose(object_pose,
                                                robot.get_link_tf_frame(robot_description.get_tool_frame(arm)))
        target_diff = desig.target.to_transform("target").inverse_times(tcp_to_object.to_transform("object")).to_pose()

        _move_arm_tcp(target_diff, robot, arm)
        robot.detach(object)


class HSRBMoveHead(ProcessModule):
    """
    This process module moves the head to look at a specific point in the world coordinate frame.
    This point can either be a position or an object.
    """

    def _execute(self, desig: LookingMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x ** 2 + pose_in_tilt.position.y ** 2) * -1

        current_pan = robot.get_joint_state("head_pan_joint")
        current_tilt = robot.get_joint_state("head_tilt_joint")

        robot.set_joint_state("head_pan_joint", new_pan + current_pan)
        robot.set_joint_state("head_tilt_joint", new_tilt + current_tilt)


class HSRBMoveGripper(ProcessModule):
    """
    This process module controls the gripper of the robot. They can either be opened or closed.
    Furthermore, it can only moved one gripper at a time.
    """

    def _execute(self, desig: MoveGripperMotion.Motion):
        robot = BulletWorld.robot
        gripper = desig.gripper
        motion = desig.motion
        for joint, state in robot_description.get_static_gripper_chain(gripper, motion).items():
            robot.set_joint_state(joint, state)


class HSRBDetecting(ProcessModule):
    """
    This process module tries to detect an object with the given type. To be detected the object has to be in
    the field of view of the robot.
    """

    def _execute(self, desig: DetectingMotion.Motion):
        rospy.loginfo("Detecting technique: {}".format(desig.technique))
        robot = BulletWorld.robot
        object_type = desig.object_type
        # Should be "wide_stereo_optical_frame"
        cam_frame_name = robot_description.get_camera_frame()
        # should be [0, 0, 1]
        front_facing_axis = robot_description.front_facing_axis
        if desig.technique == 'all':
            rospy.loginfo("Fake detecting all generic objects")
            objects = BulletWorld.current_bullet_world.get_all_objets_not_robot()
        elif desig.technique == 'human':
            rospy.loginfo("Fake detecting human -> spawn 0,0,0")
            human = []
            human.append(Object("human", ObjectType.HUMAN, "human_male.stl", pose=Pose([0, 0, 0])))
            object_dict = {}

            # Iterate over the list of objects and store each one in the dictionary
            for i, obj in enumerate(human):
                object_dict[obj.name] = obj
            return object_dict

        else:
            rospy.loginfo("Fake -> Detecting specific object type")
            objects = BulletWorld.current_bullet_world.get_objects_by_type(object_type)

        object_dict = {}

        perceived_objects = []
        for obj in objects:
            if btr.visible(obj, robot.get_link_pose(cam_frame_name), front_facing_axis):
                perceived_objects.append(ObjectDesignatorDescription.Object(obj.name, obj.type, obj))
        # Iterate over the list of objects and store each one in the dictionary
        for i, obj in enumerate(perceived_objects):
            object_dict[obj.name] = obj

        rospy.loginfo("returning dict objects")
        return object_dict


class HSRBMoveTCP(ProcessModule):
    """
    This process moves the tool center point of either the right or the left arm.
    """

    def _execute(self, desig: MoveTCPMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot

        _move_arm_tcp(target, robot, desig.arm)


class HSRBMoveArmJoints(ProcessModule):
    """
    This process modules moves the joints of either the right or the left arm. The joint states can be given as
    list that should be applied or a pre-defined position can be used, such as "parking"
    """

    def _execute(self, desig: MoveArmJointsMotion.Motion):

        robot = BulletWorld.robot
        if desig.right_arm_poses:
            robot.set_joint_states(desig.right_arm_poses)
        if desig.left_arm_poses:
            robot.set_joint_states(desig.left_arm_poses)


class HSRBMoveJoints(ProcessModule):
    """
    Process Module for generic joint movements, is not confined to the arms but can move any joint of the robot
    """

    def _execute(self, desig: MoveJointsMotion.Motion):
        robot = BulletWorld.robot
        robot.set_joint_states(dict(zip(desig.names, desig.positions)))


class HSRBWorldStateDetecting(ProcessModule):
    """
    This process module detectes an object even if it is not in the field of view of the robot.
    """

    def _execute(self, desig: WorldStateDetectingMotion.Motion):
        obj_type = desig.object_type
        return list(filter(lambda obj: obj.type == obj_type, BulletWorld.current_bullet_world.objects))[0]


class HSRBOpen(ProcessModule):
    """
    Low-level implementation of opening a container in the simulation. Assumes the handle is already grasped.
    """

    def _execute(self, desig: OpeningMotion.Motion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[1] - 0.05}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(container_joint)[1])


class HSRBClose(ProcessModule):
    """
    Low-level implementation that lets the robot close a grasped container, in simulation
    """

    def _execute(self, desig: ClosingMotion.Motion):
        part_of_object = desig.object_part.bullet_world_object

        container_joint = part_of_object.find_joint_above(desig.object_part.name, JointType.PRISMATIC)

        goal_pose = btr.link_pose_for_joint_config(part_of_object, {
            container_joint: part_of_object.get_joint_limits(container_joint)[0]}, desig.object_part.name)

        _move_arm_tcp(goal_pose, BulletWorld.robot, desig.arm)

        desig.object_part.bullet_world_object.set_joint_state(container_joint,
                                                              part_of_object.get_joint_limits(container_joint)[0])


def _move_arm_tcp(target: Pose, robot: Object, arm: str) -> None:
    gripper = robot_description.get_tool_frame(arm)

    joints = robot_description.chains[arm].joints

    inv = request_ik(target, robot, joints, gripper)
    _apply_ik(robot, inv, joints)


###########################################################
########## Process Modules for the Real HSRB ###############
###########################################################


class HSRBNavigationReal(ProcessModule):
    """
    Process module for the real HSRB that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion.Motion) -> Any:
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        #giskard.achieve_cartesian_goal(designator.target, robot_description.base_link, "map")
        queryPoseNav(designator.target)

class HSRBNavigationSemiReal(ProcessModule):
    """
    Process module for the real HSRB that sends a cartesian goal to giskard to move the robot base
    """

    def _execute(self, designator: MoveMotion.Motion) -> Any:
        rospy.logdebug(f"Sending goal to giskard to Move the robot")
        giskard.achieve_cartesian_goal(designator.target, robot_description.base_link, "map")
        #queryPoseNav(designator.target)

class HSRBPickUpReal(ProcessModule):

    def _execute(self, designator: PickUpMotion.Motion) -> Any:
        pass


class HSRBPlaceReal(ProcessModule):

   # def _execute(self, designator: MotionDesignatorDescription.Motion) -> Any:
    #    pass
    def _execute(self, designator: PlaceMotion.Motion) -> Any:
          giskard.avoid_all_collisions()
          giskard.place_objects(designator.object, designator.target, designator.grasp)


class HSRBMoveHeadReal(ProcessModule):
    """
    Process module for the real robot to move that such that it looks at the given position. Uses the same calculation
    as the simulated one
    """

    def _execute(self, desig: LookingMotion.Motion):
        target = desig.target
        robot = BulletWorld.robot

        local_transformer = LocalTransformer()
        pose_in_pan = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_pan_link"))
        pose_in_tilt = local_transformer.transform_pose(target, robot.get_link_tf_frame("head_tilt_link"))

        new_pan = np.arctan2(pose_in_pan.position.y, pose_in_pan.position.x)
        new_tilt = np.arctan2(pose_in_tilt.position.z, pose_in_tilt.position.x + pose_in_tilt.position.y)

        current_pan = robot.get_joint_state("head_pan_joint")
        current_tilt = robot.get_joint_state("head_tilt_joint")

        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(
            {"head_pan_joint": new_pan + current_pan, "head_tilt_joint": new_tilt + current_tilt})
        giskard.achieve_joint_goal(
            {"head_pan_joint": new_pan + current_pan, "head_tilt_joint": new_tilt + current_tilt})


class HSRBDetectingReal(ProcessModule):
    """
    Process Module for the real HSRB that tries to detect an object fitting the given object description. Uses Robokudo
    for perception of the environment.
    """

    def _execute(self, desig: DetectingMotion.Motion) -> Any:
        # todo at the moment perception ignores searching for a specific object type so we do as well on real
        if desig.technique == 'human' and (desig.state == "start" or desig.state == None):
            human_pose = queryHuman()
            pose = Pose.from_pose_stamped(human_pose)
            pose.position.z = 0
            human = []
            human.append(Object("human", ObjectType.HUMAN, "human_male.stl", pose=pose))
            object_dict = {}

            # Iterate over the list of objects and store each one in the dictionary
            for i, obj in enumerate(human):
                object_dict[obj.name] = obj
            return object_dict

            return human_pose
        elif desig.technique == 'human' and desig.state == "stop":
            stop_queryHuman()
            return "stopped"


        query_result = queryEmpty(ObjectDesignatorDescription(types=[desig.object_type]))
        perceived_objects = []
        for i in range(0, len(query_result.res)):
            # this has to be pose from pose stamped since we spawn the object with given header
            obj_pose = Pose.from_pose_stamped(query_result.res[i].pose[0])
            #obj_pose.orientation = [0, 0, 0, 1]
            # obj_pose_tmp = query_result.res[i].pose[0]
            obj_type = query_result.res[i].type
            obj_size = query_result.res[i].shape_size
            obj_color = query_result.res[i].color[0]
            color_switch = {
                "red": [1, 0, 0, 1],
                "green": [0, 1, 0, 1],
                "blue": [0, 0, 1, 1],
                "black": [0, 0, 0, 1],
                "white": [1, 1, 1, 1],
                # add more colors if needed
            }
            color = color_switch.get(obj_color)
            if color is None:
                color = [0, 0, 0, 1]

            # atm this is the string size that describes the object but it is not the shape size thats why string
            def extract_xyz_values(input_string):
                # Split the input string by commas and colon to separate key-value pairs
                #key_value_pairs = input_string.split(', ')

                # Initialize variables to store the X, Y, and Z values
                x_value = None
                y_value = None
                z_value = None

                for key in input_string:
                    x_value = key.dimensions.x
                    y_value = key.dimensions.y
                    z_value = key.dimensions.z

                #
                # # Iterate through the key-value pairs to extract the values
                # for pair in key_value_pairs:
                #     key, value = pair.split(': ')
                #     if key == 'x':
                #         x_value = float(value)
                #     elif key == 'y':
                #         y_value = float(value)
                #     elif key == 'z':
                #         z_value = float(value)

                return x_value, y_value, z_value

            x, y, z = extract_xyz_values(obj_size)
            size = (x, z/2, y)
            size_box = (x/2, z/2, y/2)
            hard_size= (0.02, 0.02, 0.03)
            id = BulletWorld.current_bullet_world.add_rigid_box(obj_pose, hard_size, color)
            box_object = Object(obj_type + "_" + str(rospy.get_time()), obj_type, pose=obj_pose, color=color, id=id,
                                customGeom={"size": [hard_size[0], hard_size[1], hard_size[2]]})
            box_object.set_pose(obj_pose)
            box_desig = ObjectDesignatorDescription.Object(box_object.name, box_object.type, box_object)

            perceived_objects.append(box_desig)

        object_dict = {}

        # Iterate over the list of objects and store each one in the dictionary
        for i, obj in enumerate(perceived_objects):
            object_dict[obj.name] = obj
        return object_dict


class HSRBMoveTCPReal(ProcessModule):
    """
    Moves the tool center point of the real HSRB while avoiding all collisions
    """

    def _execute(self, designator: MoveTCPMotion.Motion) -> Any:
        lt = LocalTransformer()
        pose_in_map = lt.transform_pose(designator.target, "map")
        giskard.avoid_all_collisions()
        if designator.allow_gripper_collision:
            giskard.allow_gripper_collision(designator.arm)
        giskard.achieve_cartesian_goal(pose_in_map, robot_description.get_tool_frame(designator.arm),
                                       "map")


class HSRBMoveArmJointsReal(ProcessModule):
    """
    Moves the arm joints of the real HSRB to the given configuration while avoiding all collisions
    """

    def _execute(self, designator: MoveArmJointsMotion.Motion) -> Any:
        joint_goals = {}
        if designator.left_arm_poses:
            joint_goals.update(designator.left_arm_poses)
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(joint_goals)


class HSRBMoveJointsReal(ProcessModule):
    """
    Moves any joint using giskard, avoids all collisions while doint this.
    """

    def _execute(self, designator: MoveJointsMotion.Motion) -> Any:
        name_to_position = dict(zip(designator.names, designator.positions))
        giskard.avoid_all_collisions()
        giskard.achieve_joint_goal(name_to_position)


class HSRBMoveGripperReal(ProcessModule):
    """
     Opens or closes the gripper of the real HSRB with the help of giskard.
     """

    def _execute(self, designator: MoveGripperMotion.Motion) -> Any:
        if (designator.motion == "open"):
            pub_gripper = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                          queue_size=10)
            rate = rospy.Rate(10)
            rospy.sleep(2)
            msg = GripperApplyEffortActionGoal()  # sprechen joint gripper_controll_manager an, indem wir goal publishen type den giskard fürs greifen erwartet
            msg.goal.effort = 0.8
            pub_gripper.publish(msg)

        elif (designator.motion == "close"):
            pub_gripper = rospy.Publisher('/hsrb/gripper_controller/grasp/goal', GripperApplyEffortActionGoal,
                                          queue_size=10)
            rate = rospy.Rate(10)
            rospy.sleep(2)
            msg = GripperApplyEffortActionGoal()
            msg.goal.effort = -0.8
            pub_gripper.publish(msg)

        # if designator.allow_gripper_collision:
        #     giskard.allow_gripper_collision("left")
        # giskard.achieve_gripper_motion_goal(designator.motion)


class HSRBOpenReal(ProcessModule):
    """
    Tries to open an already grasped container
    """

    def _execute(self, designator: OpeningMotion.Motion) -> Any:
        giskard.achieve_open_container_goal(robot_description.get_tool_frame(designator.arm),
                                            designator.object_part.name)


class HSRBCloseReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: ClosingMotion.Motion) -> Any:
        giskard.achieve_close_container_goal(robot_description.get_tool_frame(designator.arm),
                                             designator.object_part.name)


class HSRBTalkReal(ProcessModule):
    """
    Tries to close an already grasped container
    """

    def _execute(self, designator: TalkingMotion.Motion) -> Any:
        pub = rospy.Publisher('/talk_request', Voice, queue_size=10)

        # fill message of type Voice with required data:
        texttospeech = Voice()
        # language 1 = english (0 = japanese)
        texttospeech.language = 1
        texttospeech.sentence = designator.cmd

        rospy.sleep(1)
        pub.publish(texttospeech)


class HSRBManager(ProcessModuleManager):

    def __init__(self):
        super().__init__("hsrb")
        self._navigate_lock = Lock()
        self._pick_up_lock = Lock()
        self._place_lock = Lock()
        self._looking_lock = Lock()
        self._detecting_lock = Lock()
        self._move_tcp_lock = Lock()
        self._move_arm_joints_lock = Lock()
        self._world_state_detecting_lock = Lock()
        self._move_joints_lock = Lock()
        self._move_gripper_lock = Lock()
        self._open_lock = Lock()
        self._close_lock = Lock()
        self._talk_lock = Lock()

    def navigate(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBNavigation(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBNavigationReal(self._navigate_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBNavigationSemiReal(self._navigate_lock)

    def pick_up(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBPickUp(self._pick_up_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBPickUpReal(self._pick_up_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBPickUpReal(self._pick_up_lock)

    def place(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBPlace(self._place_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBPlaceReal(self._place_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBPlaceReal(self._place_lock)

    def looking(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveHead(self._looking_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveHeadReal(self._looking_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveHeadReal(self._looking_lock)

    def detecting(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBDetecting(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBDetectingReal(self._detecting_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBDetecting(self._detecting_lock)

    def move_tcp(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveTCP(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveTCPReal(self._move_tcp_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveTCPReal(self._move_tcp_lock)

    def move_arm_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveArmJoints(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveArmJointsReal(self._move_arm_joints_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveArmJointsReal(self._move_arm_joints_lock)

    def world_state_detecting(self):
        if ProcessModuleManager.execution_type == "simulated" or ProcessModuleManager.execution_type == "real":
            return HSRBWorldStateDetecting(self._world_state_detecting_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBWorldStateDetecting(self._world_state_detecting_lock)

    def move_joints(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveJoints(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveJointsReal(self._move_joints_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveJointsReal(self._move_joints_lock)

    def move_gripper(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBMoveGripper(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBMoveGripperReal(self._move_gripper_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBMoveGripperReal(self._move_gripper_lock)

    def open(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBOpen(self._open_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBOpenReal(self._open_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBOpenReal(self._open_lock)

    def close(self):
        if ProcessModuleManager.execution_type == "simulated":
            return HSRBClose(self._close_lock)
        elif ProcessModuleManager.execution_type == "real":
            return HSRBCloseReal(self._close_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBCloseReal(self._close_lock)

    def talk(self):
        if ProcessModuleManager.execution_type == "real":
            return HSRBTalkReal(self._talk_lock)
        elif ProcessModuleManager.execution_type == "semi_real":
            return HSRBTalkReal(self._talk_lock)