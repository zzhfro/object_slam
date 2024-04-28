#ifndef ELLIPSOID_H
#define ELLIPSOID_H

#include <Eigen/Dense>

namespace ORB_SLAM2
{
  class  Ellipsoid
  {

   
           
    private:
        Eigen::Matrix4d Q;
        Eigen::Vector3d axes;
        Eigen::Matrix3d R;
        Eigen::Vector3d center;
       


  };


}


#endif