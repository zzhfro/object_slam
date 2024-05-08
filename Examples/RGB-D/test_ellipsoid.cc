/**
 * 
 * 
 * to test 
*/
#include "Ellipse.h"
#include "Ellipsoid.h"
#include "BoundingBox.h"
#include <Eigen/Dense>
#include <iostream>
int main()
{

double a=3;
double b=2;
double c=1;


Eigen::Vector3d axes_ellipsoid={a,b,c};
Eigen::Matrix3d R;
double angle_ellipsoid=3.14/3;
R<<1,0,0,
   0,std::cos(angle_ellipsoid),-std::sin(angle_ellipsoid),
   0,std::sin(angle_ellipsoid),std::cos(angle_ellipsoid);
Eigen::Vector3d center={1,2,3};



ORB_SLAM2::Ellipsoid ell_init(axes_ellipsoid,R,center);
ORB_SLAM2::Ellipsoid ell_new(ell_init.get_Q());

std::cout<<"axes init"<<std::endl;
std::cout<<ell_new.get_axes()<<std::endl;
std::cout<<"center init"<<std::endl;
std::cout<<ell_new.get_center()<<std::endl;
std::cout<<"R init"<<std::endl;
std::cout<<ell_new.get_R()<<std::endl;;
std::cout<<"Q new"<<std::endl;
std::cout<<ell_new.Q<<std::endl;


Eigen::Vector2d axes_ellipse={a,b};
Eigen::Vector2d center_ellipse={1,3};
double angle=0;


ORB_SLAM2::Ellipse eliipse_init(axes_ellipse,angle,center_ellipse); 
ORB_SLAM2::Ellipse ellipse_new(eliipse_init.get_C());
std::cout<<"axes ellipse init"<<std::endl;
std::cout<<ellipse_new.get_axes()<<std::endl;
std::cout<<"cente rellipse init"<<std::endl;
std::cout<<ellipse_new.get_center()<<std::endl;
std::cout<<"angle ellipse init"<<std::endl;
std::cout<<ellipse_new.get_angle()<<std::endl;
std::cout<<"C ellipse init"<<std::endl;
std::cout<<ellipse_new.get_C()<<std::endl;


std::cout<<"TEST BOX FROM ELLISPE"<<std::endl;
ORB_SLAM2::BoundingBox box=eliipse_init.compute_box();
std::cout<<box.x<<" "<<box.y<<" "<<box.w<<" "<<box.h<<" "<<std::endl;

std::cout<<"TEST ELLISPE FROM  BOX"<<std::endl;
ORB_SLAM2::Ellipse ell_box=ORB_SLAM2::Ellipse::compute_ellipse(box);
std::cout<<"from box axes"<<std::endl;
std::cout<<ell_box.get_axes()<<std::endl;
std::cout<<"^^^^^^^^^^^^"<<std::endl;
std::cout<<ell_box.get_center()<<std::endl;
std::cout<<"^^^^^^^^^^^^"<<std::endl;
std::cout<<ell_box.get_angle()<<std::endl;




}