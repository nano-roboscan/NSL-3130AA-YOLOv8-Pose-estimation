from collections import defaultdict

import cv2
import numpy as np
from ultralytics import YOLO
import os
from PIL import Image
import torch


# Load the YOLOv8 model
model = YOLO('yolov8n.pt')
model.to('cuda')

# Open the video file
#video_path = "./sample.mp4"
#cap = cv2.VideoCapture(video_path)
file_path = "/home/nanosystem/disk/img"
file_names = os.listdir(file_path)


# Store the track history
track_history = defaultdict(lambda: [])

total_cnt = 0
success_cnt = 0
fail_cnt = 0

print('succes count = ', success_cnt, 'fail_cnt =', fail_cnt)
#print('fail_cnt count = ', fail_cnt)

# Loop through the video frames
for filename in file_names:
	if os.path.splitext(filename)[1] == '.png':
		imgname = '/home/nanosystem/disk/img/' + filename

		full_image = Image.open(imgname)
		image_r = full_image.crop((0,0,640,480))
		image = image_r
		#image = image_r.resize((640,640))
		
		results = model.track(image, persist=True, classes=0, conf=0.4)

		clss = results[0].boxes.cls.cpu().tolist()

		if len(clss) == 0:
			print(filename)
			fail_cnt = fail_cnt + 1
		else:
			success_cnt = success_cnt + 1

		total_cnt = total_cnt + 1

		print('count = ', total_cnt, 'succes count = ', success_cnt, 'fail_cnt count = ', fail_cnt )
		
		annotated_frame = results[0].plot()
		cv2.imshow("YOLOv8 Local Test", annotated_frame)
		if cv2.waitKey(1) & 0xFF == ord("q"):
			break
	else:
		break

print('total count = ', total_cnt, 'succes count = ', success_cnt, 'fail_cnt count = ', fail_cnt )
cv2.destroyAllWindows()

