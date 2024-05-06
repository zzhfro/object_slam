#include"Ellipse.h"
#include <Eigen/Dense>
#include <iostream>
#include <math.h>
namespace ORB_SLAM2
{
Ellipse::Ellipse(const Eigen::Vector2d& axes_,double angle_, const Eigen::Vector2d& center_)
          //counterclockwise if angle >0
         {
          Eigen::Matrix3d Two;
          Eigen::Matrix3d C_center = Eigen::Matrix3d(Eigen::Vector3d(1/std::pow(axes_[0], 2), 
                                                                 1/std::pow(axes_[1], 2),
                                                                 -1.0).asDiagonal());
          Two << std::cos(angle_), -std::sin(angle_), center_[0],
                std::sin(angle_),  std::cos(angle_), center_[1],
                0.0, 0.0, 1.0;
          
          Eigen::Matrix3d Two_inverse=Two.inverse();
          C=Two_inverse.transpose()*C_center*Two_inverse;
         
          C = 0.5 * (C + C.transpose());
          C /= -C(2, 2);
          
          this->axes= axes_;
          this->angle = angle_;
          this->center = center_;  
         
         }
Ellipse::Ellipse(const Eigen::Matrix3d& C_) 
         {
            if ((C.transpose() - C).cwiseAbs().sum() > 1e-3) 
            {
                std::cout << "Warning: Matrix should be symmetric" << "\n";
            }
            C= C_;
            C/= -C(2, 2);
            
            /*  RuRT -RuRTt
            *   -tTRuRT tTRuRTt-1  
            *   
            */
           Eigen::Matrix2d RuRT=C.block<2,2>(0,0);
           Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(RuRT);
           if (eigensolver.info() != Eigen::Success) 
           {
               std::cerr << "Eigen decomposition failed!" << std::endl;
               return;
           }
           Eigen::Matrix2d Q = eigensolver.eigenvectors(); // 旋转矩阵
           Eigen::Vector2d lambda = eigensolver.eigenvalues(); // 特征值

           std::sort(lambda.data(), lambda.data() + lambda.size());
           Eigen::Matrix2d U = Eigen::DiagonalMatrix<double, 2>(lambda);
           axes[0]=std::sqrt(1/U(0,0));
           axes[1]=std::sqrt(1/U(1,1));
              
           Eigen::Matrix2d R=Q;   
           
           angle = std::atan(R(1,0)/R(0,0));
           Eigen::Vector2d tmp=C.block<2,1>(0,2);
            
           center=-RuRT.inverse()*tmp;
           
           


        
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

};
