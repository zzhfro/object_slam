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
#include "ObjectColorManager.h"
#include "KeyFrame.h"

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
   
   void add_detection(BoundingBox& box,Eigen::Matrix<double,3,4>& Rt, int frame_idx, KeyFrame* kf);

   void insert_Map(Map *pmap);
   
   void optimize_reconstruction();
   double get_angled_difference();
   
  
   
   ObjectTrackStatus get_status()
   {
    std::unique_lock<std::mutex> lock(mutex_status);
    return status;
   }
   
  Ellipsoid get_ellipsoid()   
   {
    std::unique_lock<std::mutex> lock(mutex_ellipsoid);
    return ellipsoid;
   }
   std::unordered_map<KeyFrame*, double> get_keyframe_confs()
   {
    std::unique_lock<std::mutex> lock(mutex_add_detection);
    return keyframe_confs;
   }
   std::unordered_map<KeyFrame*, BoundingBox> get_box_observed_kf()
   {
    std::unique_lock<std::mutex> lock(mutex_add_detection);
    return box_observed_kf;
   }
   std::unordered_map<KeyFrame*, Eigen::Matrix<double,3,4>> get_Rts_kf()
   {
    std::unique_lock<std::mutex> lock(mutex_add_detection);
    return Rts_kf;
   }

   void set_bad()
   {
    std::unique_lock<std::mutex> lock(mutex_bad);
    status_bad=true;

   }
   bool if_bad()
   {
    std::unique_lock<std::mutex> lock(mutex_bad);
   return status_bad;
   }
   
   void set_ellipsoid(const Ellipsoid& ell)
   {
     std::unique_lock<std::mutex> lock(mutex_ellipsoid);
     ellipsoid=ell;

   }
   int get_category_id()
   {
    return category_id;
   }
   
   int get_last_obsframe_id()
   {
     unique_lock<mutex> lock(mutex_add_detection);
    return frame_ids.back();
   }
   
   int get_obs_num()
   {
    unique_lock<mutex> lock(mutex_add_detection);
    return box_observed.size();
   }
   
   int get_obs_kf_num()
   {
    return box_observed_kf.size();
   }


   std::unordered_map<MapPoint*, int> get_associate_mappoints()  {
        std::unique_lock<std::mutex> lock(mutex_associated_map_points);
        return associated_map_points;
    }
   std::unordered_map<MapPoint*, int> get_associate_mappoints_filter(int threshold);

   bool reconstruct_from_center();

   bool reconstruct_from_center_kf();
   
   bool check_reprojection_iou(double threshold);

   bool merge(Object * be_merged_obj);
   bool remove_obj(Object * be_merged_obj);

   double check_reprojection_ioU_kf(double iou_threshold);
   int category_id;
   int object_id;
   double uncertainty = 0.0;
   static unsigned int object_factory_id;

   std::vector<int> frame_ids; //the correspondingframe
   std::vector<Eigen::Matrix<double,3,4>> Rts;
   
   std::unordered_map<KeyFrame*, double>keyframe_confs;
   std::unordered_map<KeyFrame*, BoundingBox>box_observed_kf;
   std::unordered_map<KeyFrame*, Eigen::Matrix<double,3,4>>Rts_kf;

   std::mutex mutex_add_detection;
   std::vector<double> confs;
   std::vector<BoundingBox> box_observed; //store the box observe the object
   
   std::mutex mutex_associated_map_points;
   std::unordered_map<MapPoint*, int> associated_map_points;
   
   std::mutex mutex_ellipsoid;
   Ellipsoid ellipsoid;

   std::mutex mutex_status;
   ObjectTrackStatus status = ObjectTrackStatus::ONLY_2D;
   
   std::mutex mutex_bad;
   bool status_bad=false;
   Tracking* tracker=nullptr; 
};



}

#endif