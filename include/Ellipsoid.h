#ifndef ELLIPSOID_H
#define ELLIPSOID_H

#include <Eigen/Dense>
#include <iostream>
namespace ORB_SLAM2
{
  class  Ellipsoid
  {
   public: 
    

    Ellipsoid() {
        Q= Eigen::Matrix4d::Identity();
        Q(3, 3) = -1;
        
    }
    Ellipsoid(const Eigen::Matrix4d& Q_) {
        if ((Q_.transpose() - Q_).cwiseAbs().sum() > 1e-3) {
            std::cerr << "Warning: Matrix should be symmetric" << "\n";
        }
        Q = Q_;
        Q /= -Q(3, 3);
        
    }
    Ellipsoid(const Eigen::Vector3d& axes,const Eigen::Matrix3d& R,const Eigen::Vector3d& center);
   
           
    public:
        Eigen::Matrix4d Q;
        Eigen::Vector3d axes;
        Eigen::Matrix3d R;
        Eigen::Vector3d center;
       


  };


}


#endif