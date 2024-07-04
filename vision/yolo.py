from typing import List, Tuple, Union

import cv2
import pandas as pd
import torch
from cv2.typing import MatLike


class YOLO:
    def __init__(self, model_name="yolov5s"):
        self.model_name = model_name
        self.model = torch.hub.load('ultralytics/yolov5', model_name)
        self.target_label = None

    def set_target_label(self, target_label: str):
        self.target_label = target_label

    def _detect(self, img: MatLike) -> pd.DataFrame:
        results = self.model(img)
        return results.pandas().xyxy[0]

    def detect(
        self,
        img: MatLike
    ) -> Union[List[Tuple[int, int, int, int]], None]:

        if self.target_label is not None:
            boxes = self._detect(img)
            boxes = boxes[boxes['name'] == self.target_label]
            if len(boxes) == 0:
                return None
            results = []
            for i, box in boxes.iterrows():
                results.append(
                    (
                        int(box['xmin']),
                        int(box['ymin']),
                        int(box['xmax']),
                        int(box['ymax'])
                    )
                )
            return results
        else:
            return None
