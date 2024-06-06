#include <ros/ros.h>
#include <orb_slam2_ros/ImageDetect.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <chrono>


int main(int argc, char** argv)
{
    ros::init(argc, argv, "detectClient");        //解析参数，命名节点
    ros::NodeHandle nh;                       //创建句柄，实例化node
    ros::ServiceClient image_client = nh.serviceClient<orb_slam2_ros::ImageDetect>("detect_objects");

    cv::Mat image = cv::imread("/home/zzhfro/code/semantic_slam/yolo/yolonodetest.png", cv::IMREAD_COLOR);
    if(!image.data)
    {
      ROS_ERROR("Failed to read image");
      return -1;
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    
    orb_slam2_ros::ImageDetect srv;
    srv.request.image_raw=*msg;
    if( image_client.call(srv))
    {
     for(int i=0;i<srv.response.image_detect.num_boxes;++i)
     {
      std::cout<<"-----xywh"<<std::endl;
      for(int j=0;j<4;++j)
      {
       std::cout<<srv.response.image_detect.boxes_xywh[j+i*4]<<std::endl;
      }
      std::cout<<srv.response.image_detect.boxes_conf[i]<<std::endl;
      std::cout<<srv.response.image_detect.boxes_cls[i]<<std::endl;

     }
    }
    else
    {
        ROS_ERROR("no detect msg");
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Code execution time: " << duration.count() << " milliseconds" << std::endl;

    return 0;
}