from ultralytics import YOLO

# Load a model
model = YOLO('yolov8n.pt')  # pretrained YOLOv8n model

# Run batched inference on a list of images
results = model(['/home/zzhfro/code/semantic_slam/yolo/test/000313.png',
                 '/home/zzhfro/code/semantic_slam/yolo/test/000314.png'
                 ] )  # return a list of Results objects

# Process results list
for result in results:
    boxes = result.boxes  # Boxes object for bounding box outputs
    masks = result.masks  # Masks object for segmentation masks outputs
    keypoints = result.keypoints  # Keypoints object for pose outputs
    probs = result.probs  # Probs object for classification outputs
    
    print(boxes.cpu().cls)

    result.show()  # display to screen
    result.save(filename='result.jpg')  # save to disk