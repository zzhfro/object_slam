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
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
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
      new_obj->keyframe_confs.clear();
      new_obj->keyframe_confs[kf]=box.ObjectConf;
      new_obj->box_observed_kf.clear();
      new_obj->box_observed_kf[kf]=box;
      new_obj->Rts_kf.clear();
      new_obj->Rts_kf[kf]=Rt;
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
    if(this->get_category_id()==0)
    {
      max_capacity=40;
    }
    if (box_observed.size() > max_capacity) {
            box_observed.erase(box_observed.begin());
            confs.erase(confs.begin());
            frame_ids.erase(frame_ids.begin());
            Rts.erase(Rts.begin());
            
        }

    if(kf)
    {
       keyframe_confs[kf]=box.ObjectConf;
       box_observed_kf[kf]=box;
       Rts_kf[kf]=Rt;
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
  
  bool Object::reconstruct_from_center()
  {
     if(std::abs(this->get_angled_difference())<TO_RAD(10.0))
     {
        
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

  bool  Object::reconstruct_from_center_kf()
  {
     std::cout<<"Debug2.1"<<std::endl;
     Eigen::Matrix3d K;
     cv::cv2eigen(tracker->GetK(), K);
     
     std::unordered_map<KeyFrame*,Eigen::Matrix<double,3,4>> Rts_kf_map=this->get_Rts_kf();
     std::cout<<"Debug2.21"<<std::endl;
     std::unordered_map<KeyFrame*,BoundingBox> box_kf_map=this->get_box_observed_kf();
     std::cout<<"Debug2.2"<<std::endl;
     std::vector<BoundingBox>box_kf;
     std::vector<Eigen::Matrix<double,3,4>> Rts_kf;

     for (auto& it : box_kf_map) {
         
            box_kf.push_back(it.second);
            Rts_kf.push_back(Rts_kf_map[it.first]);
        }
     std::cout<<"Debug2.3"<<std::endl;   
     auto [status_reconstruct, ellipsoid_tmp] = Ellipsoid::reconstruct_ellipsoid_from_center(box_kf, Rts_kf, K);
     if(!status_reconstruct)
     {
        return false;

     }
     std::cout<<"Debug2.4"<<std::endl;
    this->set_ellipsoid(ellipsoid_tmp);
     return true; 
  }
 double Object::check_reprojection_iou(double threshold,bool if_erase)
  { 
    unique_lock<mutex> lock(mutex_add_detection);
    Eigen::Matrix3d K;
    cv::cv2eigen(tracker->GetK(), K);
    double iou=0;
    double valid_count=0;
    //std::vector<BoundingBox> boxes=this->get_boxes();
    //std::vector<Eigen::Matrix<double, 3, 4>> Rts_=this->get_Rts();
    for(int di=0;di<box_observed.size();++di)
    {
      Eigen::Matrix<double, 3, 4> P = K * Rts[di];
      BoundingBox box=box_observed[di];
      Ellipse ell_project=this->get_ellipsoid().project(P);
      BoundingBox box_project=ell_project.compute_box();
      iou=BoundingBox::calculate_iou(box,box_project);
      if(iou>threshold)
      {
         valid_count++;
      }

      else
      {
         if(if_erase&&iou<0.1)
         {
            box_observed.erase(box_observed.begin()+di);
            confs.erase(confs.begin()+di);
            frame_ids.erase(frame_ids.begin()+di);
            Rts.erase(Rts.begin()+di);
         }
    
      }
   
    }

    

    return valid_count/box_observed.size();
  }
  double Object::check_reprojection_iou_single(BoundingBox& box,Eigen::Matrix<double,3,4>& Rt)
  {
    Eigen::Matrix3d K;
    cv::cv2eigen(tracker->GetK(), K);
    double iou=0;
    Eigen::Matrix<double, 3, 4> P = K * Rt;
    Ellipse ell_project=this->get_ellipsoid().project(P);
    BoundingBox box_project=ell_project.compute_box();
    iou=BoundingBox::calculate_iou(box,box_project);
      
    return iou;
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
     /*
     std::cout<<"ell."<<std::endl;
     std::cout<<ell.get_center()<<std::endl;
     std::cout<<"ell."<<std::endl;
     */
     Eigen::Matrix3d K;
     cv::cv2eigen(tracker->GetK(), K);
     

     typedef g2o::BlockSolver<g2o::BlockSolverTraits<6, 1>> BlockSolver_6_1;
    BlockSolver_6_1::LinearSolverType *linear_solver = new g2o::LinearSolverDense<BlockSolver_6_1::PoseMatrixType>();


    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        new BlockSolver_6_1(linear_solver)
    );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(false);

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
         std::cout<<"DEBUG IN EDGE LOOP"<<std::endl;
         std::cout<<P<<std::endl;
         std::cout<<"DEBUG IN EDGE LOOP"<<std::endl;
         
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
         g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
         edge->setRobustKernel(rk);
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
         double error=0;
         for(int i=0;i<boxes_.size();++i)
         {
             P = K * Rts_[i]; 
            Ellipse ell_proj=ell.project(P );
            Ellipse ell_compute_from_box=Ellipse::compute_ellipse(boxes_[i]); 
            std::cout<< P<<std::endl;
           error=error+gaussian_wasserstein_2d(ell_compute_from_box, ell_proj);
         }
         std::cout<<boxes_.size()<<std::endl;
         std::cout<<error<<std::endl;
         std::cout<<"^^^^^^^"<<std::endl;
        */
        
         optimizer.initializeOptimization();
         optimizer.optimize(10);
         Eigen::Matrix<double, 6, 1> ellipsoid_estimate = v->estimate();
         
         Ellipsoid new_ellipsoid(ellipsoid_estimate.head(3), Eigen::Matrix3d::Identity(), ellipsoid_estimate.tail(3));
         /*
         std::cout<<"###DEBUG###"<<std::endl;
         std::cout<<ell.get_axes()-new_ellipsoid.get_axes()<<std::endl;
         std::cout<<"###DEBUG###"<<std::endl;
         */
         this->set_ellipsoid(new_ellipsoid);
   

  }
  std::unordered_map<MapPoint*, int> Object::get_associate_mappoints_filter(int threshold)
  {
   std::unique_lock<std::mutex> lock(mutex_associated_map_points);
   std::unordered_map<MapPoint*, int> filtered;
        for (auto mp_cnt : associated_map_points) {
            if (mp_cnt.second > threshold) {
                filtered[mp_cnt.first] = mp_cnt.second;
            }
        }
   //obs >threshold
        if (this->get_status()==ObjectTrackStatus::IN_MAP) 
        {
            const auto& ellipsoid = this->get_ellipsoid();
            for (auto mp_cnt : associated_map_points) 
            {
                cv::Mat p = mp_cnt.first->GetWorldPos();
                Eigen::Vector3d pos(p.at<float>(0), p.at<float>(1), p.at<float>(2));
                if (ellipsoid.is_inside(pos))
                    filtered[mp_cnt.first] = mp_cnt.second;
            }
        }

        return filtered;
  }
   bool Object::merge(Object * be_merged_obj)
   {
     std::unique_lock<std::mutex> lock(mutex_add_detection);
     std::unordered_map<KeyFrame*, double> be_merged_confs=be_merged_obj->get_keyframe_confs();
     std::unordered_map<KeyFrame*, BoundingBox> be_merged_box=be_merged_obj->get_box_observed_kf();
     std::unordered_map<KeyFrame*, Eigen::Matrix<double,3,4>> be_merged_Rts=be_merged_obj->get_Rts_kf();
    
    std::cout<<"DEBUG1"<<std::endl;
     for (auto kf : be_merged_box) {
        box_observed_kf[kf.first] = kf.second;
        Rts_kf[kf.first]=be_merged_Rts[kf.first];
        keyframe_confs[kf.first] = be_merged_confs[kf.first];
    }
    std::cout<<"DEBUG2"<<std::endl;
    bool status=this->reconstruct_from_center_kf();
    std::cout<<"DEBUG3"<<std::endl;
    return status;
    
   }


      double Object::check_reprojection_ioU_kf(double iou_threshold)
      {
         std::unordered_map<KeyFrame*, BoundingBox> boxes_=this->get_box_observed_kf();
         std::unordered_map<KeyFrame*,Eigen::Matrix<double,3,4>> Rts_=this->get_Rts_kf();

         Ellipsoid ell=this->get_ellipsoid();
         Eigen::Matrix3d K;
         cv::cv2eigen(tracker->GetK(), K);
         double iou=0;
         double error_count=0;
         for(auto it = boxes_.begin(); it != boxes_.end(); ++it)
         {
            BoundingBox box=boxes_[it->first];
            Eigen::Matrix<double, 3, 4> P = K * Rts_[it->first];
            Ellipse ell_project=this->get_ellipsoid().project(P);
            BoundingBox box_project=ell_project.compute_box();
            iou=BoundingBox::calculate_iou(box,box_project);
            if(iou>iou_threshold);
            {
               error_count++;
            }

         }

         return error_count/(boxes_.size()+1);


     }
     bool Object::remove_obj(Object * be_merged_obj)
     {
      std::unordered_map<KeyFrame*, BoundingBox> boxes_=be_merged_obj->get_box_observed_kf();
      std::unique_lock<std::mutex> lock(mutex_add_detection);
      for (auto kf : boxes_) {
        if (box_observed_kf.find(kf.first) != box_observed_kf.end())
        {
            box_observed_kf.erase(kf.first);
            keyframe_confs.erase(kf.first);
            Rts_kf.erase(kf.first);
        }
      }


     }
    

    
  





}