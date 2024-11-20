import json
import math
from typing import Optional, Dict

import actionlib
import rospy
from geometry_msgs.msg import WrenchStamped, PoseStamped, PointStamped, PoseWithCovarianceStamped
#from robokudo_msgs.msg import QueryAction, QueryGoal
from std_msgs.msg import String

from pycram.bullet_world import Object
from pycram.designators.action_designator import fts, ParkArmsAction, DetectAction, LookAtAction, HeadFollowAction, \
    NavigateAction
from pycram.designators.motion_designator import MoveGripperMotion
from pycram.designators.object_designator import  HumanDescription
from pycram.enums import Arms, ImageEnum, ObjectType
from pycram.fluent import Fluent
from pycram.language import Monitor, Code
from pycram.local_transformer import LocalTransformer
from pycram.plan_failures import SensorMonitoringCondition
from pycram.pose import Pose
from pycram.process_module import real_robot, simulated_robot
from demos.pycram_hsrb_real_test_demos.utils.startup import startup
import pycram.external_interfaces.giskard_new as giskardpy
import pycram.external_interfaces.robokudo as robokudo

import rospy
import tf
from geometry_msgs.msg import PoseStamped

from pycram.ros.viz_marker_publisher import ManualMarkerPublisher

# Initialize the necessary components
world, v, text_to_speech_publisher, image_switch_publisher, move = startup()
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/hsrb.urdf", pose=Pose([0, 0, 0]))
pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)
response = ""
callback = False
waiving = False
giskardpy.init_giskard_interface()
robokudo.init_robokudo_interface()
marker = ManualMarkerPublisher()
listener = tf.TransformListener()


class CustomerDescription(HumanDescription):
    """
    Class that represents customers, which ar
    """
    def __init__(self,pose: Pose,  num: int,orders: Optional = None):
        super().__init__("", pose)
        self.orders = orders
        self.num = num
    def set_order(self, orders: str):
        """
        Sets the order for the unique customer
        :param orders: The order of the customer
        """
        self.orders= orders

    def get_order(self):
        """
        Gets the order for the unique customer
        """
        return self.orders
    def get_num(self):
        return self.num

class Restaurant:

    global customers

    def __init__(self):
        self.toya_pose = Fluent()
        self.human_pose = None
        self.toya_pose_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.toya_pose_cb)

    def toya_pose_cb(self, msg):
        #print("updating")
        self.toya_pose.set_value(robot.get_pose())
        rospy.sleep(0.5)

    def distance(self):
        print("toya posget_valuee:" + str(self.toya_pose.get_value().pose))
        if self.human_pose:
            dis = math.sqrt((self.human_pose.pose.position.x - self.toya_pose.get_value().pose.position.x) ** 2 +
                            (self.human_pose.pose.position.y - self.toya_pose.get_value().pose.position.y) ** 2)
            print("dis: " + str(dis))
            return dis
        else:
            rospy.logerr("Cant calculate distance, no human pose found")


restaurant = Restaurant()
customer1 = CustomerDescription(1, "Tea")


def transform_pose(input_pose, from_frame, to_frame):
    """
    Transforms a pose from 'from_frame' to 'to_frame'.
    """
    try:
        # Ensure that the transform is available
        now = rospy.Time.now()
        listener.waitForTransform(to_frame, from_frame, now, rospy.Duration(4.0))

        # Perform the transform
        transformed_pose = listener.transformPose(to_frame, input_pose)
        return transformed_pose

    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        rospy.loginfo("Transform error: %s", e)
        return None


def monitor_func():
    if restaurant.distance() < 1:
        return SensorMonitoringCondition
    return False


rospy.loginfo("Waiting for action server")
rospy.loginfo("You can start your demo now")

def convert_String(input : str) -> Dict:
    return json.loads(input)

def take_order(num: int, customer: CustomerDescription):
    global callback

    DetectAction(technique='human').resolve().perform()
    HeadFollowAction('start').resolve().perform()

    pub_nlp.publish("start listening")
    rospy.sleep(3)
    tmp_res = convert_String(response)
    order_response = tmp_res['Order']
    amount_response = tmp_res['Amount']

def look_around(increase: float):
    angle = 0
    while angle <= 1:
        LookAtAction([Pose([robot.get_pose().pose.position.x, robot.get_pose().pose.position.y, 0],
                           [robot.get_pose().orientation.x, robot.get_pose().orientation.y,
                            robot.get_pose().orientation.z, angle])])
        rospy.sleep(1)
        print(robot.get_pose().orientation)
        angle += increase


def data_cb(data):
    global response
    global callback
    global waiving

    image_switch_publisher.pub_now(ImageEnum.HI.value)
    response = data.data.split(",")
    response.append("None")
    callback = True



def demo(step):
    look_at_frames = [Pose([restaurant.toya_pose.get_value().pose.position.x, restaurant.toya_pose.get_value().pose.position.y,])]
    global order
    with real_robot:
        talk = True

        start_pose = robot.get_pose()
        if step <= 0:
            text_to_speech_publisher.pub_now("Starting Restaurant Demo.", talk)
            # ParkArmsAction([Arms.LEFT]).resolve().perform()

        if step <= 1:
            # todo look from left to right
            # LookAtAction(targets=[look_pose]).resolve().perform()
            config_left = {'head_pan_joint': -0.5}
            config_right = {'head_pan_joint': 0.5}
            print(robot.get_pose().orientation)
            look_around(0.5)



            image_switch_publisher.pub_now(ImageEnum.WAVING.value)
            success = False
            while not success:
                try:
                    text_to_speech_publisher.pub_now("Please wave you hand. I will come to you", talk)

                    restaurant.human_pose = None
                    human_pose = robokudo.query_waving_human()
                    human_poseTm = transform_pose(human_pose, "head_rgbd_sensor_rgb_frame", "map")
                    human_p = human_poseTm
                    human_p.pose.position.z = 0
                    human_p.pose.orientation.x = 0
                    human_p.pose.orientation.y = 0
                    human_p.pose.orientation.z = 0
                    human_p.pose.orientation.w = 1

                    if human_p:
                        success = True
                        restaurant.human_pose = human_p

                        # print(human_p.pose.position.x)
                except Exception as e:
                    print("An error occurred from perception:", e)
        if step <= 2:
            text_to_speech_publisher.pub_now("Driving", talk)
            image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
            world.current_bullet_world.add_vis_axis(Pose.from_pose_stamped(human_p))
            #print(restaurant.distance())
            marker.publish(Pose.from_pose_stamped(human_p))
            world.current_bullet_world.add_vis_axis(Pose.from_pose_stamped(human_p))
            try:
                plan = Code(lambda: move.pub_now(human_p)) >> Monitor(monitor_func)
                plan.perform()
            except SensorMonitoringCondition:
                print("ABBRUCH")
                move.interrupt()
        if step <= 3:
            text_to_speech_publisher.pub_now("Arrived at customer", talk)
            image_switch_publisher.pub_now(ImageEnum.WAVING.value)


        if step <= 4:
            text_to_speech_publisher.pub_now("What can I get for you", talk)
            


            cust_new = CustomerDescription()
            # robot_pose_to_human = Pose.from_pose_stamped(calc_distance(human_p, 0.5))

            # world.current_bullet_world.add_vis_axis(robot_pose_to_human)
        #     move.query_pose_nav(robot_pose_to_human)
        #     image_switch_publisher.pub_now(ImageEnum.ORDER.value)
        #     text_to_speech_publisher.pub_now("Hi what do you want to have.", talk)
        #     # nlp stuff
        # if step <= 3:
        #     text_to_speech_publisher.pub_now("Driving", talk)
        #     image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
        #     move.query_pose_nav(start_pose)
        #     # nlp stuff insert here
        #     image_switch_publisher.pub_now(ImageEnum.HANDOVER.value)
        #     text_to_speech_publisher.pub_now("The guest wants: apple, milk", talk)
        #
        # if step <= 4:
        #     text_to_speech_publisher.pub_now("Driving", talk)
        #     image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
        #     move.query_pose_nav(robot_pose_to_human)
        #     # nlp stuff insert here
        #     image_switch_publisher.pub_now(ImageEnum.HANDOVER.value)
        #     text_to_speech_publisher.pub_now("Take out your order", talk)
        #
        # if step <= 5:
        #     text_to_speech_publisher.pub_now("Driving", talk)
        #     image_switch_publisher.pub_now(ImageEnum.DRIVINGBACK.value)
        #     move.query_pose_nav(start_pose)
        #     demo(step=1)


demo(0)
