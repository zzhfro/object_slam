#include "Object.h"
#include "BoundingBox.h"
#include <Eigen/Dense>
#include "Tracking.h"
namespace ORB_SLAM2
{

  void Object::insert_Map(Map *pmap) 
   {
        
        pmap->AddMapObject(this);
        status = ObjectTrackStatus::IN_MAP;
   }
  Object* Object::creat_new_object(int category,BoundingBox &box,double conf,Eigen::Matrix<double,3,4> &Rt,int frame_id,Tracking* track,KeyFrame* kf)
  {

  } 
}