#ifndef OBJECT_COLOR_MANAGER
#define OBJECT_COLOR_MANAGER
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <unordered_map>
#include <opencv2/core/core.hpp>
namespace ORB_SLAM2
{
class ObjectColorManager {
private:
    std::unordered_map<int, cv::Scalar> color_map;
    std::unordered_map<int, std::string> name_map;

public:
    ObjectColorManager(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cerr << "Error: Unable to open file " << filename << std::endl;
            return;
        }

        int category_id;
        int r, g, b;
        std::string name;
        while (file >> category_id >> r >> g >> b >> name) {
            cv::Scalar color(b, g, r); // 注意颜色顺序为 BGR
            color_map[category_id] = color;
            name_map[category_id] = name;
        }
        file.close();
    }

    std::pair<cv::Scalar, std::string> getObjectInfo(int category_id) {
        cv::Scalar color(0, 0, 0); // 默认黑色
        std::string name = "Unknown";

        auto it_color = color_map.find(category_id);
        if (it_color != color_map.end()) {
            color = it_color->second;
        }

        auto it_name = name_map.find(category_id);
        if (it_name != name_map.end()) {
            name = it_name->second;
        }

        return std::make_pair(color, name);
    }
};



}
#endif