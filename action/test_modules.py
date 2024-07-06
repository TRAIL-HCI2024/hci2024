#!/usr/bin/env python3
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
import sys
# Replace with the actual path to catkin_ws/src
sys.path.append('/root/HSR/catkin_ws/src/gpsr/scripts')
# Replace with the actual path to catkin_ws/src
sys.path.append('/root/HSR/catkin_ws/src/robocup_utils/scripts/')


# self.gpsr_detection = gpsr_detection

# from robocup_utils.scripts.Detic import GPSRDetection

# class for task manager

class TestModules:
    # constructor
    def __init__(self):
        """
        Role: basic settings
        Args: none
        Returns: none
        """
        rospy.init_node('test_manager', anonymous=True)

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

        # location_file_path = "/root/HSR/catkin_ws/src/gpsr/scripts/spotting_data/kawa3.json"

        # predefined_file_path = "/root/HSR/catkin_ws/src/robocup_utils/predefined_info/paper.json"
        # print(f"location_file_path: {location_file_path}")
        # print(f"predefined_file_path: {predefined_file_path}")

        # predefined locations, objects, names, categories info
        # self.locations = \
        #     predefined_utils.load_info_from_json(location_file_path, predefined_file_path)

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

        self.llm_task_planner = LLMTaskPlanner(
            []
        )

        self.llm_what_to_do = LLMWhatToDo(
            []
        )

        self.llm_answer_yourself = LLMAnswerYourSelf(
            []
        )

        # mode

        # rosparam
        rospy.set_param('/scaling', True)
        rospy.loginfo("EGPSR Task Manager Initialized")

# -----------------------------------------------------------------------------------------
# -----------------------------------------------------------------------------------------

    # Flow Starts Here
    def test_run(self):
        # self.monitor_instruction("battery") # str -> bool
        self.robot.whole_body.move_to_neutral()

        # HSR notify that the program is the modified version for HSRT-X research project experiment.
        self.robot.speak("Module test mode.", wait=True)

        self.robot.speak("Pease say the location name.", wait=True)
        self.robot.speak("Tap my hand when you finish speaking.", wait=True)
        # give prompts to Whisper
        # self.set_whisper_prompt(self.whisper_prompt_command)

        # whisperによる聞き取り
        # HSRのdisplay
        is_listen_success, sentence = self.gpsr_functions.gpsr_modules.call_listen_service()
        print(f"Whisperによる聞き取り: {sentence}")

        # self.robot.speak("Pease say the object name.", wait=True)
        # self.robot.speak("Tap my hand when you finish speaking.", wait=True)
        # give prompts to Whisper
        # self.set_whisper_prompt(self.whisper_prompt_command)
        # is_listen_success, obj_sentence = self.gpsr_functions.gpsr_modules.call_listen_service()

        # spottingした場所への移動 (optional)
        # location_name = "instruction point"
        # self.gpsr_functions.go_to_location(location_name)

        # 首の角度を変更
        self.gpsr_functions.gpsr_modules.move_joints_with_exception(
            {"head_tilt_joint": -0.6, "head_pan_joint": 0.0})

        # function_dict = {
        #     "function": "find_concrete_name_objects",
        #     "objects": "book"
        # }
        # self.gpsr_functions.execute_function(function_dict, robot_at="test")

        # object_nameの物体を見つける (左右に首振りして探す、上下は動かないので上の首の角度変更する関数で対応)
        object_name = "book"
        self.gpsr_functions.find_concrete_name_objects(
            object_name=object_name,
            room=None,
            complement=None
        )
        # object_nameの物体を掴む
        self.gpsr_functions.pick(object_name, location_name=None)


if __name__ == '__main__':
    task_manager = TestModules()
    task_manager.test_run()
    rospy.spin()
