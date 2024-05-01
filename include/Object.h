#ifndef OBJECT_H 
#define OBJECT_H 
#include <iostream>
#include "Ellipsoid.h"
namespace ORB_SLAM2
{
class Object{
public:
   Object():category_id(-1),object_id(-1),ellipsoid()
   {
    
   }

   Ellipsoid ellipsoid;
   int category_id;
   int object_id;

};



}

#endif