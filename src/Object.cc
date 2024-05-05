#include "Object.h"
#include "BoundingBox.h"
#include <Eigen/Dense>
#include "Tracking.h"
namespace ORB_SLAM2
{

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


     return new_obj;
  } 
}