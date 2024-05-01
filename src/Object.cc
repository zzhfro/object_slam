#include "Object.h"


namespace ORB_SLAM2
{

  void Object::insert_Map(Map *pmap) 
   {
        
        pmap->AddMapObject(this);
        status = ObjectTrackStatus::IN_MAP;
   }
}