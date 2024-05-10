#ifndef DISTANCE_H
#define DISTANCE_H
#include "Ellipse.h"
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
namespace ORB_SLAM2{


double gaussian_wasserstein_2d(const Ellipse& ell1, const Ellipse& ell2);




};

#endif