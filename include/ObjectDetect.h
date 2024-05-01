/**
 * created by Zehui Zhao 2024.4
*/

#ifndef OBJECT_DETECT_H 
#define OBJECT_DETECT_H 

#include"FrameDrawer.h"
#include <mutex>
#include "BoundingBox.h" 
namespace ORB_SLAM2
{
class BoundingBox;  
class DetectResult
{
public:
  
   string image_num; //in fact it is not necessary but i use it for check 
   int NumObject;
   //std::vector<int> ObjectCategory;
   //std::vector<double> ObjectConf;

   std::vector<BoundingBox> ObjectBoxes;
   std::vector<BoundingBox> StaticBoxes;
   std::vector<BoundingBox> DynamicBoxes;
   
   // return 1 if dynamic return 0 if static
   inline bool if_dynamic(const std::vector<int> &dynamic_category, int object_catrgory) 
   {
     return std::find(dynamic_category.begin(), dynamic_category.end(), object_catrgory) != dynamic_category.end();
   }

   // since most category can be seen as static 
   //another reason that use this way insted of save the static point is for robutness
  void set_dynamicBoxes(const std::vector<int> &dynamic_catrgory)
  {
    for(int i=0;i<NumObject;++i)
    {
      if(!if_dynamic(dynamic_catrgory,ObjectBoxes[i].ObjectCategory))
      {
        StaticBoxes.push_back(ObjectBoxes[i]);
      }
    }
  }

  void set_staticBoxes(std::vector<int> dynamic_catrgory)
  {
    for(int i=0;i<NumObject;++i)
    {
      if(if_dynamic(dynamic_catrgory,ObjectBoxes[i].ObjectCategory))
      {
        DynamicBoxes.push_back(ObjectBoxes[i]);
      }
    }
  }

   void print_detect() //for debug 
   {
    std::cout<<image_num<<std::endl;
    std::cout<<"object_num:"<<NumObject<<std::endl;
    for(int i=0;i<NumObject;++i)
    {
      std::cout<<"object_category:"<<ObjectBoxes[i].ObjectCategory<<std::endl;
      std::cout<<"object_Boxes:"<<ObjectBoxes[i].x<<" "<<ObjectBoxes[i].y<<" "
                                <<ObjectBoxes[i].w<<" "<<ObjectBoxes[i].h<<std::endl;
      std::cout<<"object_conf"<<ObjectBoxes[i].ObjectConf<<std::endl;                          
    }
   }
   
   DetectResult(string name,int num,double category[],double conf[] , double boxes[])
   { 
     image_num=name;
     NumObject=num;
     for(int i=0;i<NumObject;++i)
     {
       
       BoundingBox box(boxes[i*4],boxes[i*4+1],boxes[i*4+2],boxes[i*4+3],int(category[i]),conf[i]);
       ObjectBoxes.push_back(box);
     }
   }
   
   DetectResult(string name,int num,std::vector<double> &category,std::vector<double> &conf , std::vector<double> &boxes)
   {
     image_num=name;
     NumObject=num;
     for(int i=0;i<NumObject;++i)
     {
       BoundingBox box(boxes[i*4],boxes[i*4+1],boxes[i*4+2],boxes[i*4+3],int(category[i]),conf[i]);
       ObjectBoxes.push_back(box);
     }
   }
  
  bool iskeypoint_dynamic(const cv::KeyPoint& kp,std::vector<BoundingBox> &dyanmicBoxes)  ;
  bool iskeypoint_static(const cv::KeyPoint& kp,std::vector<BoundingBox> &staticBoxes) ;
};


}
#endif