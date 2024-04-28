#ifndef ELLIPSE_H
#define ELLIPSE_H

#include <Eigen/Dense>
namespace ORB_SLAM2
{
  class  Ellipse
  {

   
           
    private:
        Eigen::Matrix3d C;
        double angle=0;
        Eigen::Vector2d axes=Eigen::Vector2d(-1, -1);
        Eigen::Vector2d center_ = Eigen::Vector2d(-1, -1);
       


  };


}


#endif