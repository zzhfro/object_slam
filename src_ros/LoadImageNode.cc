#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <string>
#include <opencv2/opencv.hpp>
#include<opencv2/core/core.hpp>
#include<iostream>
#include<fstream>

using namespace std;

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;

    image_transport::ImageTransport it(nh);

    image_transport::Publisher left_pub = it.advertise("/cam_front/left/image_rect_color", 10);
    image_transport::Publisher right_pub = it.advertise("/cam_front/right/image_rect_color", 10);

    vector<string> vstrImageLeft;
    vector<string> vstrImageRight;
    vector<double> vTimestamps;
    string image_folder;
    bool got_image=nh.getParam("image_publisher/image_folder_name", image_folder);
    int nImages = 0;

    if(got_image)
    {
        LoadImages(image_folder, vstrImageLeft, vstrImageRight, vTimestamps);
        nImages = vstrImageLeft.size();
    }
    else
    {
        ROS_ERROR ("Failed to get image folder name from the launch file.");
        throw std::runtime_error("No image folder");
    }

    ros::Rate loop_rate(5); // 发布频率为 10Hz
    int ni=0;
    

    while (ros::ok())
    {    cv_bridge::CvImage left_cv_image;
         cv_bridge::CvImage right_cv_image;
        // 读取左侧图片
        cv::Mat imLeft = cv::imread(vstrImageLeft[ni],cv::IMREAD_GRAYSCALE);
        cv::Mat imRight = cv::imread(vstrImageRight[ni],cv::IMREAD_GRAYSCALE);
        if (!imLeft.empty())
        {
            // 发布左侧图片
             
            left_cv_image.encoding = sensor_msgs::image_encodings::MONO8;
            left_cv_image.image = imLeft;
            left_cv_image.header.stamp = ros::Time(vTimestamps[ni]);
            sensor_msgs::ImagePtr left_msg = left_cv_image.toImageMsg();
           
            left_pub.publish(left_msg);
        }
        else
        {
            ROS_ERROR("Failed to read left image from folder");
        }

        // 读取右侧图片
       
        if (!imRight.empty())
        {
            // 发布右侧图片
                  
            right_cv_image.encoding = sensor_msgs::image_encodings::MONO8;
            right_cv_image.image = imRight;
            right_cv_image.header.stamp = ros::Time(vTimestamps[ni]);

            sensor_msgs::ImagePtr right_msg = right_cv_image.toImageMsg();


            right_pub.publish(right_msg);
        }
        else
        {
            ROS_ERROR("Failed to read right image from folder");
        }
        ni++;
        if (ni>= nImages)
        {
           
            ROS_INFO("images end.");

            break;
        }
         
        
        loop_rate.sleep();
    }

    return 0;
}

