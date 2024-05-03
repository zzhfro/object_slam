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
Ellipse::Ellipse(const Eigen::Matrix3d& C_) 
         {
            if ((C.transpose() - C).cwiseAbs().sum() > 1e-3) 
            {
                std::cout << "Warning: Matrix should be symmetric" << "\n";
            }
            C= C;
            C/= -C(2, 2);
            Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(C);
            if (eigensolver.info() != Eigen::Success) {
                std::cerr << "Eigen decomposition failed!" << std::endl;
                return;
            }

            // 获取特征值和特征向量
            Eigen::VectorXd eigenvalues = eigensolver.eigenvalues();
            Eigen::MatrixXd eigenvectors = eigensolver.eigenvectors();

            // 椭圆的中心位置
            center = -eigenvectors.col(2).head(2);

            // 椭圆的轴长
            axes = eigenvalues.head(2).array().sqrt();

            // 计算角度
            angle = atan2(eigenvectors(1, 0), eigenvectors(0, 0));

        
         }
BoundingBox Ellipse::ComputeBbox() 
{
         
        double c = std::cos(angle);
        double s = std::sin(angle);
        double xmax = std::sqrt(std::pow(axes[0]*c, 2) + std::pow(-axes[1]*s, 2));
        double ymax = std::sqrt(std::pow(axes[0]*s, 2) + std::pow( axes[1]*c, 2));

         
        BoundingBox box(center[0],center[1],xmax*2,ymax*2,-1,-1);
        return box;
}         

};
