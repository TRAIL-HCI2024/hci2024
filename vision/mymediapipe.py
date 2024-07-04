import cv2
from cv2.typing import MatLike

import mediapipe as mp
from mediapipe.tasks import python
from mediapipe.tasks.python import vision


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


if __name__ == '__main__':
    # STEP 3: Load the input image.
    image = cv2.imread('./a.jpg')
    mmp = MyMediaPipe()
    print(mmp.detect(image))
