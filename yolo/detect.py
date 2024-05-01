from ultralytics import YOLO
import time 
import os
from PIL import Image

def process_image(image_path):
    # 在此处定义你的处理函数
    # 这里只是一个示例，将图片转换为灰度图像
    img = Image.open(image_path).convert('L')
    img.save(image_path)

def process_images_in_folder(folder_path,output_file):
    with open(output_file, "w") as f:
        # 遍历文件夹中的所有文件
        for file_name in sorted( os.listdir(folder_path)):
            # 确保文件是图片文件
            if file_name.endswith('.jpg') or file_name.endswith('.png'):
                # 获取图片的完整路径
                image_name = os.path.join(folder_path, file_name)
                # 调用处理函数
                results = model(image_name, save=True,device=0,conf=0.6)  # list of Results objects
                #results = model(image_path, save=True,device=0)  # predict on an image
                f.write(image_name+"\n")
                f.write(" ".join(map(str, results[0].boxes.xywh.view(-1).tolist())) + "\n")  # 将边界框展平并写入文件
                f.write(" ".join(map(str, results[0].boxes.cls.tolist())) + "\n")  # 写入类别
                f.write(" ".join(map(str, results[0].boxes.conf.tolist())) + "\n")  # 写入置信度
                #print(results[0].boxes.xywh)
                #print(results[0].boxes.cls)
                #print(results[0].boxes.conf)
                

            
# 调用函数处理图片



# Load a pretrained YOLOv8n model
#model = YOLO('yolov8n-seg.pt')
output_file = "output_static.txt"
if os.path.exists(output_file):
    # 如果存在，则删除文件
    os.remove(output_file)                


model = YOLO('yolov8n.pt')

# Define path to the image file
source = '/home/zzhfro/data/rgbd_dataset_freiburg2_desk_with_person/rgb'
# 结果文件路径


start_time = time.time()
process_images_in_folder(source,output_file)
# Run inference on the source


end_time = time.time()
elapsed_time = end_time - start_time
print("Elapsed time: {:.2f} seconds".format(elapsed_time))



