#include "Ellipsoid.h"
#include <iostream>
#include <Eigen/Dense>

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


};
