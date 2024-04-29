#ifndef UTILES_H
#define UTILES_H
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>

namespace ORB_SLAM2{

 /**
  * 
  * From T to [R,t]
  * 
 */
 Eigen::Matrix<double, 3, 4> RtFromT(const cv::Mat& cvMat) {
    Eigen::Matrix<double, 3, 4> eigenMat;

    // 提取旋转部分
    Eigen::Matrix3d R;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            R(i, j) = cvMat.at<double>(i, j);
        }
    }

    // 提取平移部分
    Eigen::Vector3d t;
    for (int i = 0; i < 3; ++i) {
        t(i) = cvMat.at<double>(i, 3);
    }

    // 将旋转部分和平移部分组合成一个 3x4 的矩阵 [R, t]
    eigenMat.block<3, 3>(0, 0) = R;
    eigenMat.block<3, 1>(0, 3) = t;

    return eigenMat;
   }
}
#endif