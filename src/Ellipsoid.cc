#include "Ellipsoid.h"
#include <iostream>
#include <Eigen/Dense>

#define TO_RAD(x) 0.01745329251 * (x)
#define TO_DEG(x) 57.2957795131 * (x)
namespace ORB_SLAM2{

Ellipsoid::Ellipsoid(const Eigen::Vector3d& axes,const Eigen::Matrix3d& R,const Eigen::Vector3d& center)
{

          Eigen::Matrix4d Two=Eigen::Matrix4d::Identity();
    
          Eigen::Matrix4d Q_star = Eigen::Matrix4d(Eigen::Vector4d(1/std::pow(axes[0], 2), 
                                                                 1/std::pow(axes[1], 2),
                                                                 1/std::pow(axes[2], 2),
                                                                 -1.0).asDiagonal());
          Two.block<3, 3>(0, 0) = R;
          Two.block<3, 1>(0,3)=center;
          Eigen::Matrix4d Two_inverse=Two.inverse();
          Q=Two_inverse.transpose()*Q_star*Two_inverse;
          Q = 0.5 * (Q_star + Q_star.transpose());
          Q /= -Q(3, 3);

          this->axes= axes;
          this->R = R;
          this->center = center; 



}


Eigen::Matrix<double, Eigen::Dynamic, 3> Ellipsoid::generate_ellipsoid_points(int azimuths, int elevations, int sampling) {
    int n = (azimuths + elevations) * (sampling);
    Eigen::Matrix<double, Eigen::Dynamic, 3> pts(n, 3);
    int k = 0;

    double a_step = 360.0 / azimuths;

    for (int i = 0; i < azimuths; ++i) {
        double az = TO_RAD(a_step * i);
        double e_step = 180.0 / (sampling-1);
        for (int j = 0; j < sampling; ++j) {
            double elev = TO_RAD(-90.0 + e_step * j);
            pts(k, 0) = std::cos(az) * std::cos(elev);
            pts(k, 1) = std::sin(az) * std::cos(elev);
            pts(k, 2) = std::sin(elev);
            ++k;
        }
    }

    double e_step = 180.0 / elevations;
    for (int i = 0; i < elevations; ++i) {
        double el = TO_RAD(-90.0 + e_step * i);
        double a_step = 360.0 / (sampling-1);
        for (int j = 0; j < sampling; ++j) {
            double az = TO_RAD(a_step * j);
            pts(k, 0) = std::cos(az) * std::cos(el);
            pts(k, 1) = std::sin(az) * std::cos(el);
            pts(k, 2) = std::sin(el);
            ++k;
        }
    }
    return pts;
}
Eigen::Matrix<double, Eigen::Dynamic, 3> Ellipsoid::generate_point_cloud(int sampling) 
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> pts;
    
    pts =  this->generate_ellipsoid_points(8, 8, sampling);   

    pts.col(0) *= axes[0];
    pts.col(1) *= axes[1];
    pts.col(2) *= axes[2];

    Eigen::Matrix<double, 3, Eigen::Dynamic> pts_transf =get_R()*pts.transpose();
    pts_transf.colwise() += get_center();
   
    return pts_transf.transpose();
}


};
