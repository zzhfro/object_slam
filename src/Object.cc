#include "Object.h"
#include "BoundingBox.h"
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include "Tracking.h"
namespace ORB_SLAM2
{
  #define TO_RAD(x) 0.01745329251 * (x)
  unsigned int Object::object_factory_id=0;
  void Object::insert_Map(Map *pmap) 
   {
        
        pmap->AddMapObject(this);
        status = ObjectTrackStatus::IN_MAP;
   }
  Object* Object::creat_new_object(int category,BoundingBox &box,Eigen::Matrix<double,3,4> &Rt,int frame_id,Tracking* track,KeyFrame* kf)
  {  
     Object* new_obj = new Object();
     new_obj->object_id=object_factory_id;
     object_factory_id++;
     new_obj->category_id=category;

     new_obj->box_observed.clear();
     new_obj->box_observed.push_back(box);
     
     new_obj->Rts.clear();
     new_obj->Rts.push_back(Rt);
       
     new_obj->frame_ids.clear();
     new_obj->frame_ids.push_back(frame_id);

     new_obj->confs.clear();
     new_obj->confs.push_back(box.ObjectConf);
     
     new_obj->keyframe_confs.clear();
     new_obj->box_observed_kf.clear();

     if(kf)
     {
      new_obj->keyframe_confs[kf]=box.ObjectConf;
      new_obj->box_observed_kf[kf]=box;
     }

     cv::Scalar col(0, 0, 0);
    if (new_obj->category_id == 56) 
    { 
       col = cv::Scalar(255, 245, 5);
        
    } else if (new_obj->category_id == 26 || new_obj->category_id == 24) { // bag
        col = cv::Scalar(184, 216, 176);
    } else if (new_obj->category_id == 58) { // plant
        col = cv::Scalar(6, 191, 0);
    } else if (new_obj->category_id == 75) { // vase
        col = cv::Scalar(173, 105, 42);
    } else if (new_obj->category_id == 49) { // orange
        col = cv::Scalar(144, 0, 255);
    } else if (new_obj->category_id == 73) { // book
        col = cv::Scalar(0, 110, 255);
    } else if (new_obj->category_id == 41) { // cup
        col = cv::Scalar(255, 0, 0);
    } else if (new_obj->category_id == 39) { // bottle
        col = cv::Scalar(63, 194, 177);
    } else if (new_obj->category_id == 67) { // phone
        col = cv::Scalar(255, 153, 0);
    }
    cv::Scalar bgr(col[2], col[1], col[0]);
     if (!(col[0] == 0 && col[1] == 0 && col[2] == 0))
        new_obj->color =  bgr;
     new_obj->tracker = track;
     new_obj->status = ObjectTrackStatus::ONLY_2D;   
     
     new_obj->uncertainty=0.5;

     return new_obj;
  } 
  void Object::add_detection(BoundingBox& box,Eigen::Matrix<double,3,4>& Rt, int frame_idx, KeyFrame* kf)
  {
    unique_lock<mutex> lock(mutex_add_detection);
    box_observed.push_back(box);
    confs.push_back(box.ObjectConf);
    frame_ids.push_back(frame_idx);
    Rts.push_back(Rt);
    if(kf)
    {
       keyframe_confs[kf]=box.ObjectConf;
       box_observed_kf[kf]=box;
       std::unique_lock<std::mutex> lock(mutex_associated_map_points);
       for(int i=0;i<kf->mvKeys.size();++i)
       {
         MapPoint* point_3d=kf->mvpMapPoints[i];
         if(point_3d)
         {
          cv::KeyPoint kp=kf->mvKeys[i];
          if(box.if_keypoint_inbox(kp))
          {
               if (associated_map_points.find(point_3d) == associated_map_points.end())
                        associated_map_points[point_3d] = 1;
                      else
                        associated_map_points[point_3d]++;
          }

         }

       }

    }

    //update uncertainty
    if(status==ObjectTrackStatus::IN_MAP)
    {
        double k = uncertainty / (uncertainty + std::exp(box.ObjectConf));
        uncertainty = uncertainty * (1.0 - k);
    }

  }
  
  double Object::get_angled_difference()
  {
     Eigen::Vector3d c0=BoundingBox::box_center(box_observed.back()).homogeneous();
     Eigen::Matrix3d K;
     cv::cv2eigen(tracker->GetK(), K);
     Eigen::Matrix3d K_inv = K.inverse();
     Eigen::Vector3d v0 = K_inv * c0;
     v0=Rts.back().block<3,3>(0,0).transpose()*(v0-Rts.back().block<3, 1>(0, 3));
     v0.normalize();
     Eigen::Vector3d c1=BoundingBox::box_center(box_observed[0]).homogeneous();
     Eigen::Vector3d v1 = K_inv * c1;
     v1=Rts[0].block<3,3>(0,0).transpose()*(v0-Rts[0].block<3, 1>(0, 3));
     v1.normalize();

     return std::atan2(v0.cross(v1).norm(), v0.dot(v1));
  

  }

  bool Object::restruct_from_center()
  {
     if(this->get_angled_difference()<TO_RAD(10.0))
     {
        //only above 10 thne begin restruct
        return false;
     }
     Eigen::Matrix3d K;
     cv::cv2eigen(tracker->GetK(), K);
     auto [status_reconstruct, ellipsoid_tmp] = Ellipsoid::reconstruct_ellipsoid_from_center(box_observed, Rts, K);
     if(!status_reconstruct)
     {
        return false;

     }

     ellipsoid=ellipsoid_tmp;
     if (status == ObjectTrackStatus::ONLY_2D)
        status = ObjectTrackStatus::INITIALIZED;

  }







}