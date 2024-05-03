 #ifndef BOUNDING_BOX_H
 #define BOUNDING_BOX_H
 #include <iostream>
 namespace ORB_SLAM2{
 class BoundingBox
   {
    public: 
        double x,y,w,h;  //the central and the width and higth
        int ObjectCategory;
        double ObjectConf;
        
        BoundingBox(double _x, double _y, double _w, double _h, int _ObjectCategory,double _ObjectConf)
           : x(_x), y(_y), w(_w), h(_h),  ObjectCategory(_ObjectCategory),ObjectConf(_ObjectConf)
       {
       }

       double area()
       {
        return w*h;
       }
       static double calculate_iou(BoundingBox &box1,BoundingBox &box2)
       {
        double intersection_x_min = std::max(box1.x - box1.w / 2.0, box2.x - box2.w / 2.0);
        double intersection_x_max = std::min(box1.x + box1.w / 2.0, box2.x + box2.w / 2.0);
        double intersection_y_min = std::max(box1.y - box1.h / 2.0, box2.y - box2.h / 2.0);
        double intersection_y_max = std::min(box1.y + box1.h / 2.0, box2.y + box2.h / 2.0);

        // 计算交集区域的面积
        double intersection_area = std::max(0.0, intersection_x_max - intersection_x_min) *
                                    std::max(0.0, intersection_y_max - intersection_y_min);

        // 计算两个框的面积
        double box1_area = box1.area();
        double box2_area = box2.area();

        // 计算并返回 IoU
        return intersection_area / (box1_area + box2_area - intersection_area);
       }
       static double calculate_intersection_area(BoundingBox &box1,BoundingBox &box2) 
       {
          // 计算交集的区域
          double intersection_x_min = std::max(box1.x - box1.w / 2.0, box2.x - box2.w / 2.0);
          double intersection_x_max = std::min(box1.x + box1.w / 2.0, box2.x + box2.w / 2.0);
          double intersection_y_min = std::max(box1.y - box1.h / 2.0, box2.y - box2.h / 2.0);
          double intersection_y_max = std::min(box1.y + box1.h / 2.0, box2.y + box2.h / 2.0);

          // 计算交集区域的面积
          return std::max(0.0, intersection_x_max - intersection_x_min) *
                std::max(0.0, intersection_y_max - intersection_y_min);
       }
   };

 }
#endif    