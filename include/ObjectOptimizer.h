#ifndef OBJECT_OPTIMIZER_H
#define OBJECT_OPTIMIZER_H
#include <iostream>
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include <Eigen/Core>
#include <opencv2/core/core.hpp>
#include <cmath>
#include <chrono>
#include "Ellipse.h"
#include "Ellipsoid.h"
#include "Distance.h"
namespace ORB_SLAM2{


class VertexEllipsoidNoRotate: public g2o::BaseVertex<6, Eigen::Matrix<double, 6, 1>>
{
 public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override {
        _estimate.head(3) = Eigen::Vector3d::Zero();
        _estimate.tail(3) = Eigen::Vector3d::Zero();
    }

    virtual void oplusImpl(const double *update) override { 
       
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> u(update);
        _estimate += u;
    }

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}



}; 


class EdgeEllipsoidProjection: public g2o::BaseUnaryEdge<1, double, VertexEllipsoidNoRotate>
{

  public:
     EdgeEllipsoidProjection(const Eigen::Matrix<double, 3, 4>& P_, const Ellipse& det_ell_, const Eigen::Matrix3d& ellipsoid_R_)
        : P(P_), det_ell(det_ell_), ellipsoid_R(ellipsoid_R_) {}

     virtual void computeError() override
    {
        const VertexEllipsoidNoRotate* v = static_cast<VertexEllipsoidNoRotate*>(_vertices[0]);
        Eigen::Matrix<double, 6, 1> ell = v->estimate();
        Ellipsoid ellipsoid(ell.head(3), Eigen::Matrix<double, 3, 3>::Identity(), ell.tail(3));

        Ellipse proj = ellipsoid.project(P);

        _error[0] = gaussian_wasserstein_2d(det_ell, proj);
        
        /*
        std::cout<<"dEBUG IN G20"<<std::endl;
        std::cout<<P<<std::endl;
        std::cout<<det_ell.get_axes()<<std::endl;
        std::cout<<"dEBUG IN G20"<<std::endl;
        */
    }

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}   

  private:
    Eigen::Matrix<double, 3, 4> P;
    Ellipse det_ell;
    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> ellipsoid_R;



};
struct EllipsoidQuat {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EllipsoidQuat() {}
    explicit EllipsoidQuat(double *data) {
        // storage [ax ay az tx ty tz rx ry rz]
        // qw is computed such that q(qx, qy, qz, qw) is normalized
        axes = Eigen::Vector3d(data);
        se3 = g2o::SE3Quat(Eigen::Matrix<double, 6, 1>(data+3));
    }

    void SetTo(double *data) const {
        for (int i = 0; i < 3; ++i)
            data[i] = axes[i];
        auto rt = se3.log();
        for (int i = 0; i < 6; ++i)
            data[i+3] = rt[i];
    }

    Ellipsoid ToEllipsoid() const {
        return Ellipsoid(axes.cwiseAbs(), se3.rotation().toRotationMatrix(), se3.translation());
    }
    static EllipsoidQuat FromEllipsoid( Ellipsoid& ell) {
        EllipsoidQuat eq;
        eq.axes = ell.get_axes();
        eq.se3 = g2o::SE3Quat(Eigen::Quaterniond(ell.get_R()), ell.get_center());
        return eq;
    }

    Eigen::Vector3d axes = Eigen::Vector3d::Zero();
    g2o::SE3Quat se3 = g2o::SE3Quat();
};
class VertexEllipsoidQuat: public g2o::BaseVertex<9, EllipsoidQuat>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    virtual void setToOriginImpl() override {
        _estimate = EllipsoidQuat();
    }

    virtual void oplusImpl(const double *update) override {
        _estimate.axes += Eigen::Map<const Eigen::Vector3d>(update);
        _estimate.se3 = g2o::SE3Quat::exp(Eigen::Map<const Eigen::Matrix<double, 6, 1>>(update + 3)) * _estimate.se3;
    }

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}
};

class EdgeEllipsoidProjectionQuat: public g2o::BaseUnaryEdge<1, double, VertexEllipsoidQuat>
{
    public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeEllipsoidProjectionQuat(const Eigen::Matrix<double, 3, 4>& P, const Ellipse& det_ell, const Eigen::Matrix3d& ellipsoid_R)
        : P_(P), det_ell_(det_ell), ellipsoid_R_(ellipsoid_R) {}

    virtual void computeError() override
    {
        const VertexEllipsoidQuat* v = static_cast<VertexEllipsoidQuat*>(_vertices[0]);
        Ellipsoid ellipsoid = v->estimate().ToEllipsoid();

        Ellipse proj = ellipsoid.project(P_);
        _error[0] = gaussian_wasserstein_2d(det_ell_, proj);
    }

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}

    private:
    Eigen::Matrix<double, 3, 4> P_;
    Ellipse det_ell_;
    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> ellipsoid_R_;
};





}
#endif 