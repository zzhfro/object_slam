from ultralytics import YOLO

# Load a model
model = YOLO('yolov8n-seg.pt')  # load an official model
#model = YOLO('path/to/best.pt')  # load a custom model

# Predict with the model
results = model('/home/zzhfro/code/semantic_slam/yolo/test/000027.png')  # predict on an image

