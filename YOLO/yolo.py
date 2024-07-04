import cv2
from cv2.typing import MatLike
import torch

import pandas as pd

model_path = './models/yolov5m.pt'


def draw_bounding_boxes_with_class(
    img: MatLike, boxes: pd.DataFrame
) -> MatLike:
    for i, box in boxes.iterrows():
        x1, y1, x2, y2 = int(box['xmin']), int(
            box['ymin']), int(box['xmax']), int(box['ymax'])
        img = cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        img = cv2.putText(
            img,
            box['name'],
            (x1, y1),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0, 255, 0),
            2,
            cv2.LINE_AA
        )
    return img


if __name__ == '__main__':
    model = torch.hub.load('ultralytics/yolov5', 'yolov5m', pretrained=True)
    model.eval()

    # streaming video
    cap = cv2.VideoCapture(0)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # Inference
        results = model(frame)
        im = draw_bounding_boxes_with_class(
            results.ims[0], results.pandas().xyxy[0])

        # Display
        cv2.imshow('frame', im)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
