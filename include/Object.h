#ifndef OBJECT_H 
#define OBJECT_H 
#include <iostream>
#include <memory>
#include <unordered_map>
#include <list>
#include "Ellipsoid.h"
#include "MapPoint.h"
#include<opencv2/core/core.hpp>
namespace ORB_SLAM2
{
enum class ObjectTrackStatus {
    ONLY_2D,    
    INITIALIZED,
    IN_MAP,
    BAD
};   
class Object{
public:
   Object():category_id(-1),object_id(-1),ellipsoid()
   {
    
   }
   
   Ellipsoid ellipsoid;
   int category_id;
   int object_id;
   cv::Scalar color; //color is bounding to the category_id
   std::unordered_map<MapPoint*, int> associated_map_points_;
   ObjectTrackStatus status = ObjectTrackStatus::ONLY_2D;

};



}

#endif