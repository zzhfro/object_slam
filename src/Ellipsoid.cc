#include "Ellipsoid.h"
#include <iostream>
#include <Eigen/Dense>
#include "BoundingBox3d.h"

#define TO_RAD(x) 0.01745329251 * (x)
#define TO_DEG(x) 57.2957795131 * (x)
namespace ORB_SLAM2{

Ellipsoid::Ellipsoid(const Eigen::Matrix4d& Q_)
{    
     Q=0.5 * (Q_ + Q_.transpose());
     Q /= -Q(3, 3);

     Eigen::Matrix4d Q_star=Q.inverse();
     Q_star/=-Q_star(3,3);

     center = -Q_star.col(3).head(3);

    Eigen::Matrix4d T_c = Eigen::Matrix4d::Identity();
    T_c(0, 3) = -center[0];
    T_c(1, 3) = -center[1];
    T_c(2, 3) = -center[2];
    Eigen::Matrix4d temp = T_c *Q_star * T_c.transpose();
    Eigen::Matrix4d Q_center = 0.5 * (temp + temp.transpose());

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(Q_center.block<3, 3>(0, 0));
    Eigen::Matrix3d eig_vectors = eigen_solver.eigenvectors();
    Eigen::Vector3d eig_values = eigen_solver.eigenvalues();
    

    /**
     * 
     * sort the values and vectors
    */

    if(eig_values[0]<eig_values[1])
    {
        double tmp=eig_values[1];
        eig_values[1]=eig_values[0];
        eig_values[0]=tmp;
        eig_vectors.col(0).swap(eig_vectors.col(1)); 
    }
    if(eig_values[0]<eig_values[2])
    {
        double tmp=eig_values[2];
        eig_values[2]=eig_values[0];
        eig_values[0]=tmp;
        eig_vectors.col(0).swap(eig_vectors.col(2)); 
    }
    if(eig_values[1]<eig_values[2])
    {
        double tmp=eig_values[2];
        eig_values[2]=eig_values[1];
        eig_values[1]=tmp;
        eig_vectors.col(1).swap(eig_vectors.col(2)); 
    }
    if (eig_vectors.determinant() < 0.0) {
                eig_vectors.col(2) *= -1;
            }
    if (eig_vectors(0, 0) < 0) {    // force first axis to be positive in x
        eig_vectors *= -1.0;
    }

    
    axes = eig_values.cwiseAbs().cwiseSqrt();
    R = eig_vectors;




}
Ellipsoid::Ellipsoid(const Eigen::Vector3d& axes,const Eigen::Matrix3d& R,const Eigen::Vector3d& center)
{

          Eigen::Matrix4d Q_star = Eigen::Matrix4d(Eigen::Vector4d(std::pow(axes[0], 2),
                                                                std::pow(axes[1], 2),
                                                                std::pow(axes[2], 2),
                                                                -1.0).asDiagonal());
          Eigen::Matrix4d T_center = Eigen::Matrix4d::Identity();
          T_center(0, 3) = center[0];
          T_center(1, 3) = center[1];
          T_center(2, 3) = center[2];
          Eigen::Matrix4d Rw_e = Eigen::Matrix4d::Identity();
          Rw_e.block<3, 3>(0, 0) = R;

          Eigen::Matrix4d transf = T_center * Rw_e;
          Q_star = transf * Q_star * transf.transpose();

          Eigen::Matrix4d Q_ = 0.5 * (Q_star + Q_star.transpose());
          Q_ /= -Q_(3, 3);

          Q=Q_.inverse();
          Q/=-Q(3,3);


          this->axes= axes;
          this->R = R;
          this->center = center; 



}


Eigen::Matrix<double, Eigen::Dynamic, 3> Ellipsoid::generate_ellipsoid_points(int azimuths, int elevations, int sampling) {
    int n = (azimuths + elevations) * (sampling);
    Eigen::Matrix<double, Eigen::Dynamic, 3> pts(n, 3);
    int k = 0;

    double a_step = 360.0 / azimuths;

    for (int i = 0; i < azimuths; ++i) {
        double az = TO_RAD(a_step * i);
        double e_step = 180.0 / (sampling-1);
        for (int j = 0; j < sampling; ++j) {
            double elev = TO_RAD(-90.0 + e_step * j);
            pts(k, 0) = std::cos(az) * std::cos(elev);
            pts(k, 1) = std::sin(az) * std::cos(elev);
            pts(k, 2) = std::sin(elev);
            ++k;
        }
    }

    double e_step = 180.0 / elevations;
    for (int i = 0; i < elevations; ++i) {
        double el = TO_RAD(-90.0 + e_step * i);
        double a_step = 360.0 / (sampling-1);
        for (int j = 0; j < sampling; ++j) {
            double az = TO_RAD(a_step * j);
            pts(k, 0) = std::cos(az) * std::cos(el);
            pts(k, 1) = std::sin(az) * std::cos(el);
            pts(k, 2) = std::sin(el);
            ++k;
        }
    }
    return pts;
}
Eigen::Matrix<double, Eigen::Dynamic, 3> Ellipsoid::generate_point_cloud(int sampling) 
{
    Eigen::Matrix<double, Eigen::Dynamic, 3> pts;
    
    pts =  this->generate_ellipsoid_points(8, 8, sampling);   

    pts.col(0) *= axes[0];
    pts.col(1) *= axes[1];
    pts.col(2) *= axes[2];

    Eigen::Matrix<double, 3, Eigen::Dynamic> pts_transf =get_R()*pts.transpose();
    pts_transf.colwise() += get_center();
   
    return pts_transf.transpose();
}
Eigen::Vector3d TriangulatePoints(const std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>& points,
                                  const std::vector<Eigen::Matrix<double,3,4>, Eigen::aligned_allocator<Eigen::Matrix<double,3,4>>>& projections)
{
    const int n = projections.size();
    Eigen::Matrix<double, Eigen::Dynamic, 4> A(2*n, 4);
    for (int i = 0; i < n; ++i) {
        A.row(i*2) = points[i][0] * projections[i].row(2) - projections[i].row(0);
        A.row(i*2+1) = points[i][1] * projections[i].row(2) - projections[i].row(1);
    }
    Eigen::JacobiSVD<Eigen::Matrix<double, Eigen::Dynamic, 4>> svd(A, Eigen::ComputeFullV);
    Eigen::MatrixXd V = svd.matrixV();
    Eigen::Vector4d X = V.col(3);
    Eigen::Vector3d center = X.head(3) / X[3];
    return center;
}

std::pair<bool, Ellipsoid>
   Ellipsoid::reconstruct_ellipsoid_from_center(std::vector<BoundingBox>& boxes,
                             std::vector<Eigen::Matrix<double,3,4>>& Rts, 
                             Eigen::Matrix3d& K)
   { 
    /**
     * r=1/n*(sum(w/fx+h/fy)*0.5)
     * 
     * 
    */
     int n=boxes.size();
     std::vector<double> sizes(n);
     std::vector<Eigen::Matrix<double,3,4>, Eigen::aligned_allocator<Eigen::Matrix<double,3,4>>> projections(n);
     std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> points2d(n);
    for (size_t i = 0; i < n; ++i) {
        const auto& bb = boxes[i];
        points2d[i] = BoundingBox::box_center(bb);
        projections[i] = K * Rts[i];
        sizes[i] = 0.5 * (bb.w / K(0, 0) + bb.h / K(1, 1));
    }

    Eigen::Vector3d center = TriangulatePoints(points2d, projections);

    double mean_3d_size=0;
    for(int i=0;i<n;++i)
    {
        Eigen::Vector3d X_cam = Rts[i] * center.homogeneous();
        if (X_cam.z() < 0) {
            //std::cerr << "Reconstruction failed: z is negative" << std::endl;
            return {false, Ellipsoid()};
        }
        Eigen::Vector3d X_img = K * X_cam;
        double u = X_img[0] / X_img[2];
        double v = X_img[1] / X_img[2];

        if ((points2d[i] - Eigen::Vector2d(u, v)).norm() > 100) 
        {
            //std::cerr << "Reconstruction failed: reconstructed center is too far from a detection" << std::endl;
            return {false, Ellipsoid()};
        }

        mean_3d_size += X_cam.z() * sizes[i];

    }
    mean_3d_size /= sizes.size();
    return {true, Ellipsoid(Eigen::Vector3d::Ones() * mean_3d_size * 0.5, Eigen::Matrix3d::Identity(), center)};
   }


BoundingBox3d Ellipsoid::compute3d_box() 
{
    
    Eigen::Matrix3d M = R * Eigen::DiagonalMatrix<double, 3>(axes);
    double dx = M.row(0).norm();
    double dy = M.row(1).norm();
    double dz = M.row(2).norm();

    BoundingBox3d box_3d(center[0] , center[1], center[2],dx*2,dy*2,dz*2,0);
    return box_3d;
}   
bool Ellipsoid::is_inside(const Eigen::Vector3d& pt) const {
  
    Eigen::Vector3d pt_e = R.transpose() * (pt - center);
    return std::pow(pt_e[0] / axes[0], 2) + std::pow(pt_e[1] / axes[1], 2) + std::pow(pt_e[2] / axes[2], 2) <= 1.0;
}

};
