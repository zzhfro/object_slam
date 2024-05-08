#include"Ellipse.h"
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
namespace ORB_SLAM2
{
Ellipse::Ellipse()
        {
          C=Eigen::Matrix3d::Identity();
          C(2, 2) = -1;
        }    
Ellipse::Ellipse(const Eigen::Vector2d& axes_,double angle_, const Eigen::Vector2d& center_)
          //counterclockwise if angle >0
         {
          Eigen::Matrix3d A;
        A << 1.0 / std::pow(axes_[0], 2), 0.0, 0.0,
            0.0, 1.0 / std::pow(axes_[1], 2), 0.0,
            0.0, 0.0, -1.0;
        Eigen::Matrix3d R;
        R << std::cos(angle_), -std::sin(angle_), 0.0,
            std::sin(angle_), std::cos(angle_), 0.0,
            0.0, 0.0, 1.0;
        Eigen::Matrix3d T = Eigen::Matrix3d::Identity();
        T(0, 2) = -center_[0];
        T(1, 2) = -center_[1];
          C = T.transpose() * R * A * R.transpose() * T;
          
          this->axes= axes_;
          this->angle = angle_;
          this->center = center_;  
         
         }
Ellipse::Ellipse(const Eigen::Matrix3d& C_) 
         {
            C= 0.5 * (C_ + C_.transpose());
            C/= -C(2, 2);
            
            Eigen::Matrix3d C_star=C.inverse();
            C_star/=-C_star(2,2);
            center = -C_star.col(2).head(2);
            Eigen::Matrix3d T_c = Eigen::Matrix3d::Identity();
            T_c(0, 2) = -center[0];
            T_c(1, 2) = -center[1];
            Eigen::Matrix3d temp = T_c * C_star * T_c.transpose();
            Eigen::Matrix3d C_center = 0.5 * (temp + temp.transpose());
            
            Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigen_solver(C_center.block<2, 2>(0, 0));
            Eigen::Matrix2d eig_vectors = eigen_solver.eigenvectors();
            Eigen::Vector2d eig_values = eigen_solver.eigenvalues();
            
            if(eig_values[0]<eig_values[1])
            {   double tmp=eig_values[1];
                eig_values[1]=eig_values[0];
                eig_values[0]=tmp;

                eig_vectors.col(0).swap(eig_vectors.col(1));
            }
            if (eig_vectors.determinant() < 0.0) {
                eig_vectors.col(1) *= -1;
            }
            if (eig_vectors(0, 0) < 0) {    // force first axis to be positive in x
                eig_vectors *= -1.0;
            }

            axes = eig_values.cwiseAbs().cwiseSqrt();
            angle = std::atan2(eig_vectors(1, 0), eig_vectors(0, 0));
           
         }
BoundingBox Ellipse::compute_box() 
{
         
        double c = std::cos(angle);
        double s = std::sin(angle);
        double xmax = std::sqrt(std::pow(axes[0]*c, 2) + std::pow(-axes[1]*s, 2));
        double ymax = std::sqrt(std::pow(axes[0]*s, 2) + std::pow( axes[1]*c, 2));

         
        BoundingBox box(center[0],center[1],xmax*2,ymax*2,-1,-1.0);
        return box;
}         
std::pair<Eigen::Vector2d, Eigen::Matrix2d> Ellipse::GetGAussian()const
{
     Eigen::Matrix2d A_dual;
        A_dual << std::pow(axes[0], 2), 0.0,
                  0.0, std::pow(axes[1], 2);
        Eigen::Matrix2d R;
        R << std::cos(angle), -std::sin(angle),
             std::sin(angle), std::cos(angle);
        Eigen::Matrix2d cov = R * A_dual * R.transpose();
        return {center, cov};
}

Ellipse Ellipse::compute_ellipse(BoundingBox box,double angle)
{  
 Eigen::Vector2d axes_={box.w,box.h};
 Eigen::Vector2d center={box.x,box.y};
  
 return Ellipse(axes_,angle, center);
} 




};
