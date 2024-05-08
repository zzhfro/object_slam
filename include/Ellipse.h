#ifndef ELLIPSE_H
#define ELLIPSE_H
#include "BoundingBox.h"
#include <Eigen/Dense>
#include <iostream>
namespace ORB_SLAM2
{
  class  Ellipse
  { 
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    public:
        Ellipse();
        
        Ellipse(const Eigen::Matrix3d& C_) ;

        Ellipse(const Eigen::Vector2d& axes,double angle, const Eigen::Vector2d& center);
        
        BoundingBox compute_box();

        static Ellipse compute_ellipse(BoundingBox box,double angle=0); 

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
        
        std::pair<Eigen::Vector2d, Eigen::Matrix2d> GetGAussian() const;
           
    public:
        Eigen::Matrix3d C; //primal
        double angle=0;   //-pi/2~pi/2
        Eigen::Vector2d axes=Eigen::Vector2d(-1, -1);
        Eigen::Vector2d center = Eigen::Vector2d(-1, -1);
       


  };


}


#endif