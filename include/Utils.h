#ifndef UTILES_H
#define UTILES_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <unordered_set>
#include <unordered_map>
namespace ORB_SLAM2{

 /**
  * 
  * From T to [R,t]
  * 
 */
 Eigen::Matrix<double, 3, 4> RtFromT(const cv::Mat& cvMat) ;
template <class T, class V>   
int count_set_map_intersection(const std::unordered_set<T>& s0, const std::unordered_map<T, V>& s1) {
    int res = 0;
    for (const auto& x : s0) {
        res += s1.count(x);
    }
    return res;
}

template <class T, class V>
int count_map_intersection(const std::unordered_map<T, V>& s0, const std::unordered_map<T, V>& s1) {
    int res = 0;
    for (const auto& x : s0) {
        res += s1.count(x.first);
    }
    return res;
}
}
#endif