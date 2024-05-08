#include "Object.h"
#include "BoundingBox.h"
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include "Tracking.h"
#include "Optimizer.h"
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/base_unary_edge.h"
#include "Thirdparty/g2o/g2o/core/sparse_optimizer.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "ObjectOptimizer.h"
#include "Distance.h"
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
    this->set_ellipsoid(ellipsoid_tmp);
     if (status == ObjectTrackStatus::ONLY_2D)
        status = ObjectTrackStatus::INITIALIZED;
     return true;   

  }

  void Object::optimize_reconstruction()
  {  
     {
      unique_lock<mutex> lock(mutex_status);
      if(status!=ObjectTrackStatus::INITIALIZED)
      {
         std::cout<<"the object should be initialized before optimized"<<std::endl;
         return;
      }
     }
     Ellipsoid ell=get_ellipsoid();
     std::cout<<"ell."<<std::endl;
     std::cout<<ell.get_center()<<std::endl;
     std::cout<<"ell."<<std::endl;
     Eigen::Matrix3d K;
     cv::cv2eigen(tracker->GetK(), K);
     

     typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 1>> BlockSolver_6_1;
     BlockSolver_6_1::LinearSolverType *linear_solver = new g2o::LinearSolverDense<BlockSolver_6_1::PoseMatrixType>();


    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        new BlockSolver_6_1(linear_solver)
    );
     g2o::SparseOptimizer optimizer;     // 图模型
     optimizer.setAlgorithm(solver);   // 设置求解器
     optimizer.setVerbose(true);       // 关闭调试输出

     VertexEllipsoidNoRotate*  v=new VertexEllipsoidNoRotate();
     Eigen::Matrix<double, 6, 1> e;
     
     e << ell.get_axes(), ell.get_center();
     /*
     std::cout<<"#####DEBUG in e####"<<std::endl;
     std::cout<<e<<std::endl;
     std::cout<<"#####DEBUG in e####"<<std::endl;
     */
     v->setEstimate(e);
     v->setId(0);
     optimizer.addVertex(v);
    std::vector<BoundingBox>boxes_;
    std::vector<double>confs_;
    std::vector<Eigen::Matrix<double,3,4>> Rts_;
     //add edge
     {
         unique_lock<mutex> lock(mutex_add_detection);
         boxes_=box_observed;
         confs_=confs;
         Rts_=Rts;
     }
     for(int i=0;i<boxes_.size();++i)
     {
         Eigen::Matrix<double, 3, 4> P = K * Rts_[i];
         BoundingBox box=boxes_[i];
         /*
         std::cout<<"size debug"<<std::endl;
         std::cout<<boxes_.size()<<std::endl;
         std::cout<<"size debug"<<std::endl;
         */
         EdgeEllipsoidProjection *edge = new EdgeEllipsoidProjection(P, Ellipse::compute_ellipse(box), ell.get_R());
         edge->setId(i);
         edge->setVertex(0, v);
         Eigen::Matrix<double, 1, 1> information_matrix = Eigen::Matrix<double, 1, 1>::Identity();
         information_matrix *=confs_[i] ;
         edge->setInformation(information_matrix);
         optimizer.addEdge(edge);
     }   
         /*
         std::cout<<"^^^^^PROJECTTION DEBUG^^^^"<<std::endl;
         Eigen::Matrix<double, 3, 4> P = K * Rts_[1];
         Ellipse ell_proj=ell.project(P );
         Ellipse ell_compute_from_box=Ellipse::compute_ellipse(boxes_[1]);
         std::cout<<"ell_proj info"<<std::endl;
         std::cout<<ell_proj.get_axes()<<std::endl;
         std::cout<<ell_proj.get_center()<<std::endl;
         std::cout<<ell_proj.get_angle()<<std::endl;
         std::cout<<"ell_from_box info"<<std::endl;
         std::cout<<ell_compute_from_box.get_axes()<<std::endl;
         std::cout<<ell_compute_from_box.get_center()<<std::endl;
         std::cout<<ell_compute_from_box.get_angle()<<std::endl;
         std::cout<<"conf"<<std::endl;
         std::cout<<confs_[1]<<std::endl;
         std::cout<<"error"<<std::endl;
         std::cout<<gaussian_wasserstein_2d(ell_compute_from_box, ell_proj)<<std::endl;
         std::cout<<"^^^^^^^"<<std::endl;
        */
 
         optimizer.initializeOptimization();
         optimizer.optimize(8);
         Eigen::Matrix<double, 6, 1> ellipsoid_estimate = v->estimate();
         
         Ellipsoid new_ellipsoid(ellipsoid_estimate.head(3), Eigen::Matrix3d::Identity(), ellipsoid_estimate.tail(3));
         /*
         std::cout<<"###DEBUG###"<<std::endl;
         std::cout<<ell.get_axes()-new_ellipsoid.get_axes()<<std::endl;
         std::cout<<"###DEBUG###"<<std::endl;
         */
         this->set_ellipsoid(new_ellipsoid);
   

  }





}