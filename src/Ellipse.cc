#include"Ellipse.h"
#include <Eigen/Dense>
#include <iostream>
namespace ORB_SLAM2
{
Ellipse::Ellipse(const Eigen::Vector2d& axes,double angle, const Eigen::Vector2d& center)
          //counterclockwise if angle >0
         {
          Eigen::Matrix3d Two;
          Eigen::Matrix3d C_star = Eigen::Matrix3d(Eigen::Vector3d(1/std::pow(axes[0], 2), 
                                                                 1/std::pow(axes[1], 2),
                                                                 -1.0).asDiagonal());
          Two << std::cos(angle), -std::sin(angle), center[0],
                std::sin(angle),  std::cos(angle), center[1],
                0.0, 0.0, 1.0;
          Eigen::Matrix3d Two_inverse=Two.inverse();
          C=Two_inverse.transpose()*C_star*Two_inverse;
          C = 0.5 * (C_star + C_star.transpose());
          C /= -C(2, 2);

          this->axes= axes;
          this->angle = angle;
          this->center = center;  
         
         }

};
