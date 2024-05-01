 #ifndef BOUNDING_BOX_H
 #define BOUNDING_BOX_H
 
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
   };

 }
#endif    