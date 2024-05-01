#ifndef ELLIPSE_H
#define ELLIPSE_H

#include <Eigen/Dense>
#include <iostream>
namespace ORB_SLAM2
{
  class  Ellipse
  { 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    public:
        Ellipse()
        {
          C=Eigen::Matrix3d::Identity();
          C(2, 2) = -1;
        }
         Ellipse(const Eigen::Matrix3d& C_) 
         {
            if ((C.transpose() - C).cwiseAbs().sum() > 1e-3) 
            {
                std::cout << "Warning: Matrix should be symmetric" << "\n";
            }
            C= C;
            C/= -C(2, 2);
        
         }
         Ellipse(const Eigen::Vector2d& axes,double angle, const Eigen::Vector2d& center);

         Eigen::Matrix3d get_C()
         {
          return C;
         }
         
         double get_angle()
         {
          return angle;
         }
         Eigen::Vector2d get_axes()
         {
          return axes;
         }
         Eigen::Vector2d get_center()
         {
          return center;
         }

           
    public:
        Eigen::Matrix3d C; //primal
        double angle=0; 
        Eigen::Vector2d axes=Eigen::Vector2d(-1, -1);
        Eigen::Vector2d center = Eigen::Vector2d(-1, -1);
       


  };


}


#endif