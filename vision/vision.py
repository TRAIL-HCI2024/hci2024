import datetime
import threading
from copy import deepcopy
from typing import List, Tuple, Union

import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from .my_typing import Direction, Position
from .mymediapipe import MyMediaPipe


class Vision:
    def __init__(self):
        rospy.init_node('my_vision')
        topic_name1 = '/hsrb/head_rgbd_sensor/rgb/image_rect_color'
        topic_name2 = '/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw'
        self._bridge = CvBridge()
        self._input_image = None

        # Subscribe color image data from HSR
        self._image_sub = rospy.Subscriber(
            topic_name1, Image, self._color_image_cb)
        # Wait until connection
        rospy.wait_for_message(topic_name1, Image, timeout=5.0)

        self._depth_sub = rospy.Subscriber(
            topic_name2, Image, self._depth_image_cb)
        rospy.wait_for_message(topic_name2, Image, timeout=5.0)

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

    def _depth_image_cb(self, data):
        try:
            self._depth_image = self._bridge.imgmsg_to_cv2(data, "passthrough")
        except CvBridgeError as cv_bridge_exception:
            rospy.logerr(cv_bridge_exception)

    def search_direction_at(self, timestamp: str) -> Direction:
        if len(timestamp.split("-")) > 1:
            timestamp = timestamp.split("-")[0]
        copyof_directs = deepcopy(self.directs)  # 別スレッドからアクセスされるのでコピーしておく
        return self._search_direction_at(timestamp, copyof_directs)

    def _search_direction_at(
        self,
        timestamp: str,
        ls: List[Tuple[Direction, str]]
    ) -> Direction:
        if len(ls) == 1:
            return ls[0][0]

        if len(ls) == 0:
            return Direction.NONE

        if len(ls) >= 2:
            median = len(ls) // 2
            dt1 = datetime.datetime.strptime(ls[median][1], "%Y%m%d_%H%M%S")
            dt2 = datetime.datetime.strptime(timestamp, "%Y%m%d_%H%M%S")
            if dt1 < dt2:
                return self._search_direction_at(timestamp, ls[:median])
            else:
                return self._search_direction_at(timestamp, ls[median:])
        return Direction.NONE

    def get_image(self):
        return self._input_image

    def get_depth(self):
        return self._depth_image

    def _estimate_pose(self) -> Union[Tuple[Position, Position], None]:
        return self.mmp.estimate_right_arm(self.get_image())

    def extimate_pose(self) -> Direction:
        """
        目的地の方向 (left, right): str
        喋るメッセージ: str
        """

        _pose = self._estimate_pose()
        if _pose is None:
            return Direction.NONE

        shoulder, wrist = _pose
        if shoulder.x > wrist.x and shoulder.y < wrist.y:
            return Direction.LEFT_UP
        elif shoulder.x < wrist.x and shoulder.y < wrist.y:
            return Direction.RIGHT_UP
        elif shoulder.x > wrist.x and shoulder.y > wrist.y:
            return Direction.LEFT_DOWN
        elif shoulder.x < wrist.x and shoulder.y > wrist.y:
            return Direction.RIGHT_DOWN
        else:
            return Direction.NONE

    def estimate_distance(self) -> float:
        image = self.get_image()
        depth = self.get_depth()

        mask = self.mmp.get_mask(image)

        if mask is None:
            print("Cannot estimate distance")
            return -1

        depth_map = depth * mask
        depth_map = depth_map[depth_map != 0]
        return depth_map.mean() / 1000  # mm -> m
