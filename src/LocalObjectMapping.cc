#include "LocalObjectMapping.h"
#include "Object.h"
#include "BoundingBox.h"
#include <opencv2/core/eigen.hpp>
#include <Eigen/Dense>
#include "Tracking.h"
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
#include "BoundingBox3d.h"
#include "Utils.h"
#include "KeyFrame.h"
namespace ORB_SLAM2{

LocalObjectMapping::LocalObjectMapping(Map *pMap):
    mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
     mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true)
{
}


void LocalObjectMapping::SetTracker(Tracking *pTracker)
{
    mpTracker=pTracker;
}
bool LocalObjectMapping::CheckModifiedObjects() 
{
    unique_lock<mutex> lock(mMutexNewKFs);
    return !modified_objects.empty();
}
void LocalObjectMapping::InsertModifiedObject(Object* obj)
{
    unique_lock<mutex> lock(mMutexNewKFs);
    if (!obj->if_bad() && modified_objects_set.find(obj) == modified_objects_set.end()) {
        modified_objects.push(obj);
        modified_objects_set.insert(obj);
    }
}
void LocalObjectMapping::Run()
{

    mbFinished = false;

    while(1)
    {
        // Tracking will see that Local Mapping is busy
        // SetAcceptKeyFrames(false);
        // For now LocalObjectMapping always accept newly modified objects

        // while ... do reconstruction
        while (CheckModifiedObjects() ) 
        {

            Object *obj = modified_objects.front();

            // obj->GetTrack()->TryResetEllipsoidFromMaPoints();
            // obj->GetTrack()->AssociatePointsInsideEllipsoid(mpMap);

            optimize_reconstruction(obj);
             
             //the fusion process

             Ellipsoid ell = obj->get_ellipsoid();
             const Eigen::Vector3d& center = ell.get_center();
             BoundingBox3d bb = ell.compute3d_box();
             int cat = obj->get_category_id();

            // Check objects fusion
            std::vector<Object*> map_objects = mpMap->GetAllObjects();
            auto pts = obj->get_associate_mappoints_filter(10);
            for (auto* obj2 : map_objects)
            {
                if (obj == obj2 || obj2->get_category_id() != cat)
                    continue;

                Ellipsoid ell2 = obj2->get_ellipsoid();
                BoundingBox3d bb2 = ell2.compute3d_box();

                double iou = BoundingBox3d::calculate_iou(bb2,bb);
                auto inter = BoundingBox3d::intersection_volume(bb, bb2);
                double rel_inter = inter / bb.volume();
                double rel_inter2 = inter / bb2.volume();

                auto pts_2 = obj2->get_associate_mappoints_filter(10);
                auto nb_common_points = count_map_intersection(pts, pts_2);

               
                if (ell.is_inside(ell2.get_center()) || ell2.is_inside(center) || iou > 0.15 || rel_inter > 0.2 || rel_inter2 > 0.2 || nb_common_points >= 10) 
                { 
                    std::cout<<"success1"<<std::endl;
            
                    auto ellipsoid_save = obj2->get_ellipsoid();
                    auto merge_ok = obj2->merge(obj);
                    std::cout<<"Merge test"<<std::endl;
                    std::cout<<merge_ok<<std::endl;
                    std::cout<<obj2->check_reprojection_ioU_kf(0.2)<<std::endl;
                    //if (merge_ok && obj2->check_reprojection_ioU_kf(0.2) > 0.5) 
                    if(1)
                    {   
                        
                        std::cout<<"success2"<<std::endl;
                        obj->set_bad(); // object will be removed from the map
                        break;
                    } else {
                        // undo fusion and reset ellipsoid
                        obj2->remove_obj(obj);
                        obj2->set_ellipsoid(ellipsoid_save);
                    }
                }
            }


            {
                std::unique_lock<std::mutex> lock(mMutexNewKFs);
                modified_objects.pop();
                modified_objects_set.erase(obj);
            }
        }


        if(Stop())
        {
            // Safe area to stop
            while(isStopped() && !CheckFinish())
            {
                usleep(3000);
            }
            if(CheckFinish())
                break;
        }

        ResetIfRequested();

        // Tracking will see that Local Mapping is busy
        // SetAcceptKeyFrames(true);

        if(CheckFinish())
            break;

        usleep(3000);
    }
    SetFinish();
}
void LocalObjectMapping::optimize_reconstruction(Object* obj)
{
    
      ObjectTrackStatus status=obj->get_status();
      if(status!=ObjectTrackStatus::IN_MAP)
      {
         std::cout<<"the object should be in map before optimized"<<std::endl;
         return;
      }
     
     Ellipsoid ell=obj->get_ellipsoid();
     /*
     std::cout<<"ell."<<std::endl;
     std::cout<<ell.get_center()<<std::endl;
     std::cout<<"ell."<<std::endl;
     */
     Eigen::Matrix3d K;
     cv::cv2eigen(mpTracker->GetK(), K);
     

     typedef g2o::BlockSolver<g2o::BlockSolverTraits<9, 1>> BlockSolver_9_1;
    BlockSolver_9_1::LinearSolverType *linear_solver = new g2o::LinearSolverDense<BlockSolver_9_1::PoseMatrixType>();


    auto solver = new g2o::OptimizationAlgorithmLevenberg(
        new BlockSolver_9_1(linear_solver)
    );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

     VertexEllipsoidQuat*  v=new VertexEllipsoidQuat();
     EllipsoidQuat ellipsoid_quat = EllipsoidQuat::FromEllipsoid(ell);
   
     v->setEstimate(ellipsoid_quat);
     v->setId(0);
     optimizer.addVertex(v);
     
    std::unordered_map<KeyFrame*, BoundingBox> boxes_;
    std::unordered_map<KeyFrame*, double>confs_;
    std::unordered_map<KeyFrame*,Eigen::Matrix<double,3,4>> Rts_;
     //add edge
     
    boxes_=obj->get_box_observed_kf();
    confs_=obj->get_keyframe_confs();
    Rts_=obj->get_Rts_kf();
    
    int i=0;
    for (auto it = boxes_.begin(); it != boxes_.end(); ++it)
    {   
        KeyFrame* kf = it->first;
        BoundingBox box = it->second;
        Eigen::Matrix<double, 3, 4> P = K * Rts_[kf];
        EdgeEllipsoidProjectionQuat *edge = new EdgeEllipsoidProjectionQuat(P, Ellipse::compute_ellipse(box), ell.get_R());
        edge->setId(i);
        ++i;
        edge->setVertex(0, v);
        Eigen::Matrix<double, 1, 1> information_matrix = Eigen::Matrix<double, 1, 1>::Identity();
        information_matrix *=confs_[kf];
        edge->setInformation(information_matrix);
        optimizer.addEdge(edge);
    }   
         
         optimizer.initializeOptimization();
         optimizer.optimize(40);
         EllipsoidQuat ellipsoid_estimate = v->estimate();
         
         Ellipsoid new_ellipsoid=ellipsoid_estimate.ToEllipsoid();
                
         obj->set_ellipsoid(new_ellipsoid);
}
void LocalObjectMapping::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopRequested = true;
    unique_lock<mutex> lock2(mMutexNewKFs);
    
}

bool LocalObjectMapping::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(mbStopRequested && !mbNotStop)
    {
        mbStopped = true;
        cout << "Local Mapping STOP" << endl;
        return true;
    }

    return false;
}

bool LocalObjectMapping::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool LocalObjectMapping::stopRequested()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopRequested;
}

void LocalObjectMapping::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);
    if(mbFinished)
        return;
    mbStopped = false;
    mbStopRequested = false;

    cout << "Local Mapping RELEASE" << endl;
}



bool LocalObjectMapping::SetNotStop(bool flag)
{
    unique_lock<mutex> lock(mMutexStop);

    if(flag && mbStopped)
        return false;

    mbNotStop = flag;

    return true;
}


void LocalObjectMapping::RequestReset()
{
    {
        unique_lock<mutex> lock(mMutexReset);
        mbResetRequested = true;
    }

    while(1)
    {
        {
            unique_lock<mutex> lock2(mMutexReset);
            if(!mbResetRequested)
                break;
        }
        usleep(3000);
    }
}

void LocalObjectMapping::ResetIfRequested()
{
    unique_lock<mutex> lock(mMutexReset);
    if(mbResetRequested)
    {
        
        mbResetRequested=false;
    }
}

void LocalObjectMapping::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool LocalObjectMapping::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void LocalObjectMapping::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;    
    unique_lock<mutex> lock2(mMutexStop);
    mbStopped = true;
}

bool LocalObjectMapping::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}







};

