from vision import vision
import cv2
import numpy as np
import rospy
import time

vis = vision.Vision()
vis.start()
time.sleep(1)
# 画像撮影
image = vis.get_image()
cv2.imwrite("image.jpg", image)
depth = vis.get_depth()
depth_array = np.array(depth, dtype=np.uint16)
print(depth_array.shape)
print(np.max(depth_array))
depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=0.03), cv2.COLORMAP_JET)
cv2.imwrite("depth_img.jpg", depth_colormap)
# 色方向が1次元のため、グレースケールに変換
cv2.imwrite("depth.jpg", depth)
print("Done")
# rospy.spin()
