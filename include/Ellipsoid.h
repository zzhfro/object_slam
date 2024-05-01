#ifndef ELLIPSOID_H
#define ELLIPSOID_H

#include <Eigen/Dense>
#include <iostream>
namespace ORB_SLAM2
{
  class  Ellipsoid
  {
   EIGEN_MAKE_ALIGNED_OPERATOR_NEW; 
   public: 
    

    Ellipsoid() {
        Q= Eigen::Matrix4d::Identity();
        Q(3, 3) = -1;
        
    }
    Ellipsoid(const Eigen::Matrix4d& Q_) {
        if ((Q_.transpose() - Q_).cwiseAbs().sum() > 1e-3) {
            std::cout << "Warning: Matrix should be symmetric" << "\n";
        }
        Q = Q_;
        Q /= -Q(3, 3);
        
    }
    Ellipsoid(const Eigen::Vector3d& axes,const Eigen::Matrix3d& R,const Eigen::Vector3d& center);
     
    void operator= (const Ellipsoid& ell) {
        Q= ell.Q;

        axes = ell.axes;
        R = ell.R;
        center = ell.center;
    }
     
    Eigen::Matrix4d get_Q()
    {
      return Q;

    }
    
    Eigen::Vector3d get_axes()
    {
      return axes;
    }
    Eigen::Matrix3d get_R()
    {
      return R;
    }
   Eigen::Vector3d get_center()
   {
    return center;
   }
   Eigen::Matrix<double, Eigen::Dynamic, 3> generate_ellipsoid_points(int azimuths, int elevations, int sampling);
   Eigen::Matrix<double, Eigen::Dynamic, 3> generate_point_cloud(int sampling);



    public:
        Eigen::Matrix4d Q;
        Eigen::Vector3d axes=Eigen::Vector3d(-1,-1,-1);
        Eigen::Matrix3d R=Eigen::Matrix3d::Identity();
        Eigen::Vector3d center=Eigen::Vector3d(-1,-1,-1);
       


  };


}


#endif