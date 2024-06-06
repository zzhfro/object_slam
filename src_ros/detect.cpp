#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <mutex>

// 图像处理函数
void imageProcessingFunction(cv::Mat leftImage, cv::Mat rightImage, double timestamp, std::mutex& mtx, std::string& response) {
    // 在这里执行图像处理任务，例如调用 TrackStereo 函数
    // TrackStereo(leftImage, rightImage, timestamp);

    // 在这里执行 ROS 服务客户端，并在获取响应后设置 response 变量
    ros::NodeHandle nh;
    ros::ServiceClient client = nh.serviceClient<your_package::YourServiceMsg>("your_service_name");
    your_package::YourServiceMsg srv;
    // 设置请求数据
    if (client.call(srv)) {
        // 获取响应数据
        std::lock_guard<std::mutex> lock(mtx); // 锁定互斥量
        response = srv.response.data;
    } else {
        ROS_ERROR("Failed to call service");
    }
}

// 图像回调函数
void imageCallback(const sensor_msgs::ImageConstPtr& leftImageMsg, const sensor_msgs::ImageConstPtr& rightImageMsg, std::mutex& mtx, std::string& response) {
    cv_bridge::CvImageConstPtr leftCvImgPtr, rightCvImgPtr;
    try {
        leftCvImgPtr = cv_bridge::toCvShare(leftImageMsg);
        rightCvImgPtr = cv_bridge::toCvShare(rightImageMsg);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    double timestamp = leftImageMsg->header.stamp.toSec();
    
    std::thread imageProcessingThread(imageProcessingFunction, leftCvImgPtr->image, rightCvImgPtr->image, timestamp, std::ref(mtx), std::ref(response));
    imageProcessingThread.detach(); // 在后台运行图像处理线程
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_processing_node");
    ros::NodeHandle nh;

    std::mutex responseMutex;
    std::string response;

    ros::Subscriber leftImageSub = nh.subscribe<sensor_msgs::Image>("/left_image_topic", 1, boost::bind(imageCallback, _1, _2, std::ref(responseMutex), std::ref(response)));
    ros::Subscriber rightImageSub = nh.subscribe<sensor_msgs::Image>("/right_image_topic", 1, boost::bind(imageCallback, _1, _2, std::ref(responseMutex), std::ref(response)));

    ros::spin();

    return 0;
}
