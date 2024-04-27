from ultralytics import YOLO
import time 
import os
from PIL import Image

model = YOLO('yolov8n.pt')
source = '/home/zzhfro/code/semantic_slam/yolo/yolonodetest.png'
results = model(source, save=True,device=0,conf=0.6)
print(results[0].boxes.xywh)
print(results[0].boxes.cls)
print(results[0].boxes.conf)
print(results[0].keypoints)

