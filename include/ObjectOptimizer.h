#ifndef OBJECT_OPTIMIZER_H
#define OBJECT_OPTIMIZER_H
#include <iostream>
#include <g2o/core/g2o_core_api.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
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
        : P(P), det_ell(det_ell_), ellipsoid_R(ellipsoid_R_) {}

     virtual void computeError() override
    {
        const VertexEllipsoidNoRotate* v = static_cast<VertexEllipsoidNoRotate*>(_vertices[0]);
        Eigen::Matrix<double, 6, 1> ell = v->estimate();
        Ellipsoid ellipsoid(ell.head(3), Eigen::Matrix<double, 3, 3>::Identity(), ell.tail(3));

        Ellipse proj = ellipsoid.project(P);

        _error[0] = gaussian_wasserstein_2d(det_ell, proj);
    }

    virtual bool read(std::istream&) override {return true;}
    virtual bool write(std::ostream&) const override {return true;}   

  private:
    Eigen::Matrix<double, 3, 4> P;
    Ellipse det_ell;
    Eigen::Matrix<double, 3, 3, Eigen::DontAlign> ellipsoid_R;



};
}
#endif 