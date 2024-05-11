#ifndef LOCAL_OBJECT_MAPPING_H
#define LOCAL_OBJECT_MAPPING_H
#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "KeyFrameDatabase.h"
#include "Object.h"
#include "Utils.h"
#include <unordered_set>
#include <mutex>

namespace ORB_SLAM2{

class LocalObjectMapping
{
public:
    LocalObjectMapping(Map* pMap);

    

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    // Thread Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool SetNotStop(bool flag);
   
   bool CheckModifiedObjects(); 
   void  InsertModifiedObject(Object* obj);
   void optimize_reconstruction(Object* obj);


    void RequestFinish();
    bool isFinished();


protected:


    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;

    Tracking* mpTracker;
    
    std::mutex mMutexNewKFs;

    std::queue<Object*> modified_objects;
    std::unordered_set<Object*> modified_objects_set;//to check 

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;





};





}




#endif