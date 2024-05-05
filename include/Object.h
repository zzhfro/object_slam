#ifndef OBJECT_H 
#define OBJECT_H 
#include <iostream>
#include <memory>
#include <unordered_map>
#include <list>
#include "Map.h"
#include "Ellipsoid.h"
#include "MapPoint.h"
#include<opencv2/core/core.hpp>
#include "Map.h"
#include <mutex>
#include "Frame.h"
#include <Eigen/Dense>

namespace ORB_SLAM2
{
class Map;
class Tracking;

enum class ObjectTrackStatus {
    ONLY_2D,    
    INITIALIZED,
    IN_MAP,
    BAD
};   


class Object
{
public:
   Object():category_id(-1),object_id(-1),ellipsoid()
   {
    
   }
   
   static Object* creat_new_object(int category,BoundingBox &box,Eigen::Matrix<double,3,4> &Rt,int frame_id,Tracking* track,KeyFrame* kf);

   void insert_Map(Map *pmap);

   ObjectTrackStatus get_status()
   {
    return status;
   }
   Ellipsoid get_ellipsoid()
   {
    return ellipsoid;
   }

   int get_category_id()
   {
    return category_id;
   }
   
   std::unordered_map<MapPoint*, int> get_associate_mappoints()  {
        std::unique_lock<std::mutex> lock(mutex_associated_map_points);
        return associated_map_points;
    }

   Ellipsoid ellipsoid;
   int category_id;
   int object_id;

   static unsigned int object_factory_id;
   cv::Scalar color; //color is bounding to the category_id
   std::vector<BoundingBox> box_observed; //store the box observe the object
   std::vector<int> frame_ids; //the correspondingframe
   std::vector<Eigen::Matrix<double,3,4>> Rts;

   std::vector<double> confs;
   
   std::unordered_map<KeyFrame*,double> keyframe_confs;
   std::unordered_map<KeyFrame*,BoundingBox> box_observed_kf;
   //std::unordered_map<MapPoint*, int> associated_map_points_;
   
   std::mutex mutex_associated_map_points;
   std::unordered_map<MapPoint*, int> associated_map_points;

   ObjectTrackStatus status = ObjectTrackStatus::ONLY_2D;
   
   Tracking* tracker=nullptr; 
};



}

#endif