#ifndef DISTANCE_H
#define DISTANCE_H
#include "Ellipse.h"
#include <Eigen/Dense>
namespace ORB_SLAM2{


double gaussian_wasserstein_2d(const Ellipse& ell1, const Ellipse& ell2)
{   
    auto result1=ell1.GetGAussian();
    Eigen::Vector2d mu1=result1.first;
    Eigen::Matrix2d sigma1=result1.second;

    auto result2=ell2.GetGAussian();
    Eigen::Vector2d mu2=result2.first;
    Eigen::Matrix2d sigma2=result2.second;

    Eigen::Matrix2d sigma1_sqrt = sigma1.sqrt();
    Eigen::Matrix2d s121 = sigma1_sqrt * sigma2 * sigma1_sqrt;
    Eigen::Matrix2d sigma121 = s121.sqrt();

    double d = (mu1-mu2).squaredNorm() + (sigma1 + sigma2 - 2 * sigma121).trace();
    return d;
}



};

#endif