#ifndef BOUNDING_BOX_3d_H
#define BOUNDING_BOX_3d_H
#include <iostream>
#include <algorithm>
namespace ORB_SLAM2{


class BoundingBox3d
{
 public: 
        double x,y,z,w,l,h;  //the central and the width and higth
        int ObjectCategory;
   BoundingBox3d(double _x, double _y,double _z, double _w,double _l, double _h, int _ObjectCategory)
        : x(_x), y(_y),z(_z), w(_w), l(_l),h(_h),  ObjectCategory(_ObjectCategory)
   {
     
   }
   BoundingBox3d()
   {
     
   }
   double volume()
   {
    return w*l*h;
   }
static double intersection_volume(const BoundingBox3d& box1, const BoundingBox3d& box2) ;
static double union_volume(const BoundingBox3d& box1, const BoundingBox3d& box2);
static double calculate_iou(const BoundingBox3d& box1, const BoundingBox3d& box2);
};

}

#endif