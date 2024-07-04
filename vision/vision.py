import datetime
import threading
from dataclasses import dataclass
from enum import Enum
from typing import List, Tuple, Union

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from .mymediapipe import MyMediaPipe
from .yolo import YOLO


class Direction(Enum):
    LEFT_UP = 0
    LEFT_DOWN = 1
    RIGHT_UP = 2
    RIGHT_DOWN = 3
    NONE = -1


@dataclass
class Position:
    x: float
    y: float
    z: float


class Vision:
    def __init__(self):
        rospy.init_node('my_vision')
        topic_name = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
        self._bridge = CvBridge()
        self._input_image = None

        # Subscribe color image data from HSR
        self._image_sub = rospy.Subscriber(
            topic_name, Image, self._color_image_cb)
        # Wait until connection
        rospy.wait_for_message(topic_name, Image, timeout=5.0)

        self.yolo = YOLO()
        self.mmp = MyMediaPipe()

        self.directs = []
        self.lim_directs = 150

    def start(self):
        thread = threading.Thread(target=self._main)
        thread.start()

    def _main(self):
        while not rospy.is_shutdown():
            direct = self.extimate_pose()
            timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S.%f")
            if len(self.directs) >= self.lim_directs:
                self.directs.pop(0)
            self.directs.append((direct, timestamp))
            rospy.sleep(0.1)

    def _color_image_cb(self, data):
        try:
            self._input_image = self._bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)

    def search_direction_at(self, timestamp: str) -> Direction:
        return self._search_direction_at(timestamp, self.directs)

    def _search_direction_at(
        self,
        timestamp: str,
        ls: List[Tuple[Direction, str]]
    ) -> Direction:
        if len(ls) == 1:
            return ls[0][0]

        if len(ls) == 0:
            return Direction.NONE

        if len(ls) > 2:
            median = len(ls) // 2
            if ls[median][1] > timestamp:
                return self._search_direction_at(timestamp, ls[:median])
            else:
                return self._search_direction_at(timestamp, ls[median:])

    def get_image(self):
        return self._input_image

    def is_person_detected(self) -> bool:
        image = self.get_image()
        boxes = self.yolo._detect(image)
        boxes = boxes[boxes['name'] == "person"]
        return len(boxes) > 0

    def _estimate_pose(self) -> Union[Tuple[Position, Position], None]:
        return self.mmp.estimate_right_arm()

    def extimate_pose(self) -> Direction:
        """
        目的地の方向 (left, right): str
        喋るメッセージ: str
        """
        if not self.is_person_detected():
            return Direction.NONE

        _pose = self._estimate_pose()
        if _pose is None:
            return Direction.NONE

        shoulder, wrist = _pose
        if shoulder.x > wrist.x and shoulder.y < wrist.y:
            return Direction.LEFT_UP
