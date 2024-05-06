#include"ObjectColorManager.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
int main()
{
 ORB_SLAM2::ObjectColorManager color_manager(80,"/home/zzhfro/code/object_slam/ORB_SLAM2/Examples/RGB-D/obj.txt");
 
 std::cout<<"init success"<<std::endl;
 int category_id1=0;
 int category_id2=60;
 auto info = color_manager.getObjectInfo(category_id1);
    cv::Scalar color = info.first;
    std::string name = info.second;

 std::cout<<"catrgory_id1"<<std::endl;
 std::cout << "Scalar: (" << color[0] << ", " << color[1] << ", " << color[2] << ", " << color[3] << ")" << std::endl;
 std::cout<<name<<std::endl;

 info = color_manager.getObjectInfo(category_id2);
 color = info.first;
 name = info.second;

 std::cout<<"catrgory_id2"<<std::endl;
 std::cout << "Scalar: (" << color[0] << ", " << color[1] << ", " << color[2] << ", " << color[3] << ")" << std::endl;
 std::cout<<name<<std::endl;

 



}