#!/usr/bin/env python3
import sys
# Replace with the actual path to catkin_ws/src
sys.path.append('/root/HSR/catkin_ws/src/gpsr/scripts')
# Replace with the actual path to catkin_ws/src
sys.path.append('/root/HSR/catkin_ws/src/robocup_utils/scripts/')


from Detic import GPSRDetection as gpsr_detection
import subprocess
from world_modules import GPSRModules
from world_functions import GPSRFunctions
from std_srvs.srv import Trigger
from weblab_hsr_msgs.srv import StringTrigger, SoundPlay
from utils.control.mobile_base_wrapper import MobileBaseWrapper
from utils.control.joint_group_wrapper import JointGroupWrapper
from utils.control.end_effector_wrapper import GripperWrapper
from llm_manager import LLMTaskPlanner, LLMWhatToDo, LLMAnswerYourSelf
from std_msgs.msg import Empty, String
from sensor_msgs.msg import Image
from hsrb_interface import Robot, settings
import utils.robot
import rospy
import predefined_utils
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
        rospy.init_node('my_action', anonymous=True)

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
        self.robot = utils.robot.Robot(
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

    def start(self):
        # self.monitor_instruction("battery") # str -> bool
        self.robot.whole_body.move_to_neutral()

        self.robot.speak("おはようございます。", wait=True)
        self.robot.speak("HCI2024", wait=True)

        # spottingした場所への移動 (optional)
        # location_name = "instruction point"
        # self.gpsr_functions.go_to_location(location_name)

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


if __name__ == '__main__':
    action = Action()
    action.start()
    rospy.spin()
