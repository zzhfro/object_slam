#ifndef OBJECT_COLOR_MANAGER
#define OBJECT_COLOR_MANAGER
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/core/core.hpp>
/**
 * created by zzh 2024.5
 * 
 * 
*/
namespace ORB_SLAM2
{
  class ObjectColorManager {
private:
    std::vector<cv::Scalar> color_vector;
    std::vector<std::string> name_vector;

public:
ObjectColorManager(int category_num,const std::string& filename) {
        initializeColorVector(category_num);
        std::cout<<"color init success"<<std::endl;
        initializeNameVector(filename);
    }
    ObjectColorManager()
    {
        
    }

    void initializeColorVector(int category_num) {
        // 这里使用简单的固定颜色生成算法，可以根据需要自定义
        for (int i = 0; i < category_num; ++i) {
            int b = i * 10 % (256+i*1);
            int g = i * 30 % (256+i*3);
            int r = i * 20 % (256+i*2);
            cv::Scalar color(b, g, r);
            color_vector.push_back(color);
        }
    }
     void initializeNameVector(const std::string& filename) 
     {
        // 这里假设物体类别的名称是一个通用的名称，例如 "object" + 类别ID
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Unable to open file " << filename << std::endl;
            return;
        }

        std::string name;
        while (std::getline(file, name)) {
            name_vector.push_back(name);
        }
        file.close();
    }

    std::pair<cv::Scalar, std::string> getObjectInfo(int category_id) 
    {
        cv::Scalar color(0, 0, 0); // 默认颜色为黑色
        std::string name = "Unknown";

        color=color_vector[category_id];

        name=name_vector[category_id];

        return std::make_pair(color, name);
    }
};



}
#endif