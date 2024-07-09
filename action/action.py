#!/usr/bin/env python3
# Replace with the actual path to catkin_ws/src
import sys
sys.path.append('/root/HSR/catkin_ws/src/gpsr/scripts')
# Replace with the actual path to catkin_ws/src
sys.path.append('/root/HSR/catkin_ws/src/robocup_utils/scripts/')
import predefined_utils
import rospy
import robocup_utils.robot
from hsrb_interface import Robot, settings
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, String
from llm_manager import LLMTaskPlanner, LLMWhatToDo, LLMAnswerYourSelf
from gpsr_utils.control.end_effector_wrapper import GripperWrapper
from gpsr_utils.control.joint_group_wrapper import JointGroupWrapper
from gpsr_utils.control.mobile_base_wrapper import MobileBaseWrapper
from weblab_hsr_msgs.srv import StringTrigger, SoundPlay
from std_srvs.srv import Trigger
from world_functions import GPSRFunctions
from world_modules import GPSRModules
import subprocess
from Detic import GPSRDetection as gpsr_detection
from enum import Enum
from vision.my_typing import Direction


# self.gpsr_detection = gpsr_detection

# from robocup_utils.scripts.Detic import GPSRDetection

# class for task manager


class Action:
    # constructor
    def __init__(self):
        """
        Role: basic settings
        Args: none
        Returns: none
        """
        #rospy.init_node('my_action', anonymous=True)

        # Connect to Robot
        rospy.loginfo("Connecting to robot ..")
        robot = Robot()
        rospy.loginfo("Connecting to robot 1/4")
        self.omni_base = MobileBaseWrapper(robot.get('omni_base'))
        rospy.loginfo("Connecting to robot 2/4")
        self.whole_body = JointGroupWrapper(robot.get('whole_body'))
        rospy.loginfo("Connecting to robot 3/4")
        gripper = GripperWrapper(robot.get('gripper'))
        rospy.loginfo("Connected")
        self.robot = robocup_utils.robot.Robot(
            robot, self.omni_base, self.whole_body, gripper)
        self.base_link_id = settings.get_frame('base')
        print("Robot Initialized.")

        # set language HSR speaks
        rospy.set_param("spoken_language", '0')  # 0: Japanese, 1: English

        # tuning core files for the environment
        gpsr_modules = GPSRModules(
            robot=self.robot,
            location_info_list=[],  # self.locations,
            # self.objects,
            # self.names,
            # self.categories
            # 0, #detic_mode
            # '' #prompt_by_llm
        )
        self.gpsr_functions = GPSRFunctions(self.robot, gpsr_modules)
        # rosparam
        rospy.set_param('/scaling', True)
        rospy.loginfo("EGPSR Task Manager Initialized")

        self.init_pose = None

    def start(self):
        # self.monitor_instruction("battery") # str -> bool
        self.robot.whole_body.move_to_neutral()

        # self.robot.speak("おはようございます。", wait=True)
        # self.robot.speak("HCI2024", wait=True)

        # spottingした場所への移動 (optional)
        # location_name = "instruction point"
        # self.gpsr_functions.go_to_location(location_name)
        """
        # 首の角度を変更
        self.gpsr_functions.gpsr_modules.move_joints_with_exception(
            {"head_tilt_joint": -0.6, "head_pan_joint": 0.0})

        # object_nameの物体を見つける (左右に首振りして探す、上下は動かないので上の首の角度変更する関数で対応)
        object_name = "bottle"
        self.gpsr_functions.find_concrete_name_objects(
            object_name=object_name,
            room=None,
            complement=None
        )
        # object_nameの物体を掴む
        self.gpsr_functions.pick(object_name, location_name=None)
        """

    def speak(self, text: str):
        self.robot.speak(text, wait=True)

    def rotate_head(self, direction: Direction):
        if direction == Direction.LEFT_DOWN:
            self.gpsr_functions.gpsr_modules.move_joints_with_exception(
                {"head_tilt_joint": -0.5, "head_pan_joint": -1.5})
        elif direction == Direction.LEFT_UP:
            self.gpsr_functions.gpsr_modules.move_joints_with_exception(
                {"head_tilt_joint": 0.5, "head_pan_joint": -1.5})
        elif direction == Direction.RIGHT_UP:
            self.gpsr_functions.gpsr_modules.move_joints_with_exception(
                {"head_tilt_joint": 0.5, "head_pan_joint": 1.5})
        else:
            self.gpsr_functions.gpsr_modules.move_joints_with_exception(
                {"head_tilt_joint": -0.5, "head_pan_joint": 1.5})

    def init_head(self):
        self.gpsr_functions.gpsr_modules.move_joints_with_exception(
            {"head_tilt_joint": 0.0, "head_pan_joint": 0.0})

    def rotate_body(self, direction: Direction, wait=False):
        try:
            if direction == Direction.LEFT_DOWN:
                self.robot.rotate_omni_base(-90, wait=wait)
            elif direction == Direction.LEFT_UP:
                self.robot.rotate_omni_base(-90, wait=wait)
            elif direction == Direction.RIGHT_UP:
                self.robot.rotate_omni_base(90, wait=wait)
            else:
                self.robot.rotate_omni_base(90, wait=wait)
        except Exception as e:
            print(e)

    def rotate_body_inverse(self, direction: Direction, wait=False):
        try:
            if direction == Direction.LEFT_DOWN:
                self.robot.rotate_omni_base(90, wait=wait)
            elif direction == Direction.LEFT_UP:
                self.robot.rotate_omni_base(90, wait=wait)
            elif direction == Direction.RIGHT_UP:
                self.robot.rotate_omni_base(-90, wait=wait)
            else:
                self.robot.rotate_omni_base(-90, wait=wait)
        except Exception as e:
            print(e)

    def look(self, direction: Direction):
        action.rotate_head(direction)
        action.rotate_body(direction)
        action.init_head()

    def find_and_pick(self, object_name: str):
        self.gpsr_functions.find_concrete_name_objects(
            object_name=object_name,
            room=None,
            complement=None
        )
        self.gpsr_functions.pick(object_name, location_name=None)

    """
    def bring(self, x: float, y: float):
        theta = math.atan2(y, x)
        # rad2deg = 180 / math.pi
        theta = math.degrees(theta)
        self.robot.go_rel(x=x, y=y, theta=theta)
        # self.robot.omni_base.go_rel(x=x, y=y, theta=theta)
        while True:
            self.speak("僕の手を触ってね")
            is_pushed = self.gpsr_functions.gpsr_modules.call_push_checker(5.0)
            if is_pushed:
                break
        self.speak("はい、どうぞ！")
        self.robot.open_gripper()
    """

    def bring(self, distance: float, angle: float):
        self.robot.omni_base.go_rel(x=distance, y=0, theta=angle)
        while True:
            self.speak("僕の手を触ってね")
            is_pushed = self.gpsr_functions.gpsr_modules.call_push_checker(5.0)
            if is_pushed:
                break
        self.speak("はい、どうぞ！")
        self.robot.open_gripper()

    def register_initial_position(self):
        self.init_pose = self.robot.omni_base.pose

    def return_to_initial_position(self):
        if self.init_pose is None:
            return
        self.robot.omni_base.go_abs(
            self.init_pose[0], self.init_pose[1], self.init_pose[2])


if __name__ == '__main__':
    action = Action()
    action.start()
    action.speak("Hello")
    action.rotate_head(Direction.LEFT_UP)
    print("Rotating head: LEFT_UP")
    action.rotate_body(Direction.LEFT_UP)
    print("Rotating body: LEFT_UP")
    action.init_head()
    # action.rotate_body(Direction.LEFT_UP)
    # print("Rotating body: LEFT_UP")
    # action.rotate_head(Direction.RIGHT_DOWN)
    # print("Rotating head: RIGHT_DOWN")
    # action.rotate_body(Direction.RIGHT_DOWN)
    # print("Rotating body: RIGHT_DOWN")
    rospy.spin()
