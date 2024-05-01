#include "ObjectDetect.h"

namespace ORB_SLAM2
{
 bool DetectResult::iskeypoint_dynamic(const cv::KeyPoint& kp,std::vector<BoundingBox> &dyanmicBoxes) 
 {
        
        float kp_u  = kp.pt.x;
        float kp_v = kp.pt.y;
        bool is_dynamic = false;
        for(int i=0;i<dyanmicBoxes.size();++i)
        {
             
           double left = dyanmicBoxes[i].x-dyanmicBoxes[i].w*0.5;
           double right = dyanmicBoxes[i].x+dyanmicBoxes[i].w*0.5;

           double top = dyanmicBoxes[i].y-dyanmicBoxes[i].h*0.5;
           
           double bottom = dyanmicBoxes[i].y+dyanmicBoxes[i].h*0.5;;
           
           if(kp_u>left-2 && kp_u<right+2 && kp_v>top-2 && kp_v<bottom-2)
           {
              // 如果特征点在动态目标检测框内
              is_dynamic = true;
              return is_dynamic;
           }
            
        }
        return is_dynamic;
        
}

// 判断特征点是否在静态检测框内
bool DetectResult::iskeypoint_static(const cv::KeyPoint& kp,std::vector<BoundingBox> &staticBoxes) 
{
        float kp_u  = kp.pt.x;
        float kp_v = kp.pt.y;
        bool is_static = false;
        for(int i=0;i<staticBoxes.size();++i)
        {
             
           double left = staticBoxes[i].x-staticBoxes[i].w*0.5;
           double right = staticBoxes[i].x+staticBoxes[i].w*0.5;

           double top = staticBoxes[i].y-staticBoxes[i].h*0.5;
           
           double bottom = staticBoxes[i].y+staticBoxes[i].h*0.5;;
           
           if(kp_u>left-2 && kp_u<right+2 && kp_v>top-2 && kp_v<bottom-2)
           {
              // 如果特征点在动态目标检测框内
              is_static = true;
              return is_static;
           }
            
        }
        return is_static;

}
}
// 判断特征点是否在动态检测框内

