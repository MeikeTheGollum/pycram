import unittest

import numpy as np
from pycram.bullet_world import BulletWorld
from pycram.world import Object, fix_missing_inertial
from pycram.pose import Pose
from pycram.robot_descriptions import robot_description
from pycram.process_module import ProcessModule
from pycram.enums import ObjectType
import os
import xml.etree.ElementTree as ET
import tf


class BulletWorldTest(unittest.TestCase):

    world: BulletWorld

    @classmethod
    def setUpClass(cls):
        cls.world = BulletWorld("DIRECT")
        cls.robot = Object(robot_description.name, ObjectType.ROBOT, robot_description.name + ".urdf")
        cls.kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "kitchen.urdf")
        cls.milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([1.3, 1, 0.9]))
        cls.cereal = Object("cereal", ObjectType.BREAKFAST_CEREAL, "breakfast_cereal.stl", pose=Pose([1.3, 0.7, 0.95]))
        ProcessModule.execution_delay = False

    def setUp(self):
        self.world.reset_world()

    def test_object_movement(self):
        self.milk.set_position(Pose([0, 1, 1]))
        self.assertEqual(self.milk.get_position_as_list(), [0, 1, 1])

    def test_robot_orientation(self):
        self.robot.set_pose(Pose([0, 1, 1]))
        head_position = self.robot.links['head_pan_link'].position.z
        #self.robot.set_orientation(list(2*tf.transformations.quaternion_from_euler(0, 0, np.pi, axes="sxyz")))
        self.robot.set_orientation(Pose(orientation=[0, 0, 1, 1]))
        self.assertEqual(self.robot.links['head_pan_link'].position.z, head_position)

    def test_save_and_restore_state(self):
        self.robot.attach(self.milk)
        state_id = self.world.save_state()
        robot_link = self.robot.get_root_link()
        milk_link = self.milk.get_root_link()
        cid = robot_link.constraint_ids[milk_link]
        self.assertTrue(cid == self.robot.attachments[self.milk].constraint_id)
        self.world.remove_constraint(cid)
        self.world.restore_state(state_id)
        cid = robot_link.constraint_ids[milk_link]
        self.assertTrue(milk_link in robot_link.constraint_ids)
        self.assertTrue(self.milk in self.robot.attachments)
        self.assertTrue(cid == self.robot.attachments[self.milk].constraint_id)

    def tearDown(self):
        self.world.reset_world()

    @classmethod
    def tearDownClass(cls):
        cls.world.exit()


class XMLTester(unittest.TestCase):

    def setUp(self) -> None:
        self.urdf_string = open(os.path.join("..", "resources", "pr2.urdf"), "r").read()

    def test_inertial(self):
        result = fix_missing_inertial(self.urdf_string)
        resulting_tree = ET.ElementTree(ET.fromstring(result))
        for element in resulting_tree.iter("link"):
            self.assertTrue(len([*element.iter("inertial")]) > 0)
