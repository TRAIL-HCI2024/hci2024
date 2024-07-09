import cv2
from cv2.typing import MatLike
from typing import Tuple, Union
from .my_typing import Position, Bone

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision

import numpy as np


class MyMediaPipe:
    def __init__(self):
        # STEP 2: Create an PoseLandmarker object.
        base_options = python.BaseOptions(
            model_asset_path='./pose_landmarker_lite.task')
        options = vision.PoseLandmarkerOptions(
            base_options=base_options,
            output_segmentation_masks=True)
        self.detector = vision.PoseLandmarker.create_from_options(options)

    def detect(self, image: MatLike):
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        # STEP 4: Detect pose landmarks from the input image.
        return self.detector.detect(mp_image)

    def estimate_right_arm(
            self,
            image: MatLike
    ) -> Union[Tuple[Position, Position], None]:
        results = self.detect(image)
        if results.pose_landmarks is None:
            return None

        if len(results.pose_landmarks) == 0:
            return None

        # Extract the landmarks of the right shoulder and right wrist.
        right_shoulder = results.pose_landmarks[0][int(
            Bone.RIGHT_SHOULDER.value)]
        right_wrist = results.pose_landmarks[0][int(Bone.RIGHT_WRIST.value)]

        # Calculate the 3D coordinates of the right shoulder and right wrist.
        right_shoulder_x = right_shoulder.x * image.shape[1]
        right_shoulder_y = right_shoulder.y * image.shape[0]
        right_shoulder_z = right_shoulder.z
        right_wrist_x = right_wrist.x * image.shape[1]
        right_wrist_y = right_wrist.y * image.shape[0]
        right_wrist_z = right_wrist.z

        pos1 = Position(right_shoulder_x, right_shoulder_y, right_shoulder_z)
        pos2 = Position(right_wrist_x, right_wrist_y, right_wrist_z)

        return pos1, pos2

    def get_mask(self, image: MatLike) -> Union[np.ndarray, None]:
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=image)
        results = self.detector.detect(mp_image)
        if results.segmentation_masks is None:
            return None
        nv = results.segmentation_masks[0].numpy_view()
        nv = np.where(nv == 0, 0, 1).astype(np.uint8)
        return nv


if __name__ == '__main__':
    # STEP 3: Load the input image.
    image = cv2.imread('./a.jpg')
    mmp = MyMediaPipe()
    print(mmp.detect(image))
