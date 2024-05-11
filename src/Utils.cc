#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <unordered_set>
#include <unordered_map>
#include "Utils.h"

namespace ORB_SLAM2
{
Eigen::Matrix<double, 3, 4> RtFromT(const cv::Mat& cvMat) 
  {
    Eigen::MatrixXd eigenTcw;
    cv::cv2eigen(cvMat, eigenTcw);
    Eigen::Matrix3d R;
    Eigen::Vector3d t;
    // 提取旋转矩阵R（前3列）
    R = eigenTcw.block<3, 3>(0, 0).cast<double>();

    // 提取平移向量t（第4列）
    t = eigenTcw.block<3, 1>(0, 3).cast<double>();
   
    Eigen::MatrixXd Rt(3, 4);
    Rt << R, t;
    return Rt;
   }




}