#include"BoundingBox3d.h"

namespace ORB_SLAM2
{   
 

    double BoundingBox3d::intersection_volume(const BoundingBox3d& box1, const BoundingBox3d& box2) 
   {
    double dx = std::min(box1.x + box1.w, box2.x + box2.w) - std::max(box1.x, box2.x);
    double dy = std::min(box1.y + box1.l, box2.y + box2.l) - std::max(box1.y, box2.y);
    double dz = std::min(box1.z + box1.h, box2.z + box2.h) - std::max(box1.z, box2.z);
    if (dx <= 0 || dy <= 0 || dz <= 0) {
        return 0.0; // No intersection
    } else {
        return dx * dy * dz;
    }
}

    double BoundingBox3d::union_volume(const BoundingBox3d& box1, const BoundingBox3d& box2) 
    {
      double vol1 = box1.w * box1.l * box1.h;
      double vol2 = box2.w * box2.l * box2.h;
      double intersectionVol = intersection_volume(box1, box2);
      return vol1 + vol2 - intersectionVol;
    }

// Calculate Intersection over Union (IoU) of two 3D bounding boxes
    double BoundingBox3d::calculate_iou(const BoundingBox3d& box1, const BoundingBox3d& box2) 
    {
       double intersectionVol = intersection_volume(box1, box2);
       double unionVol = union_volume(box1, box2);
       if (unionVol == 0) {
           return 0.0; // Prevent division by zero
       } else {
           return intersectionVol / unionVol;
       }
    }

}