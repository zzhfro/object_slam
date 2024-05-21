/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include"ObjectColorManager.h"
#include"BoundingBox.h"
#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"
#include "ObjectDetect.h"
#include "Frame.h"
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<mutex>


namespace ORB_SLAM2
{

class Tracking;
class Viewer;
class Map;
class DetectResult;

class FrameDrawer
{
public:
    FrameDrawer(Map* pMap,const std::string &color_path);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);
     std::string color_path;
    // Draw last processed frame.
    cv::Mat DrawFrame();
    cv::Mat DrawDetect(cv::Mat &img);
protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm;
    int N;
    vector<cv::KeyPoint> mvCurrentKeys;
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    std::vector<BoundingBox> boxes_drawer;
    DetectResult* detect_drawer;
    int mState;
    ObjectColorManager object_color_manager;
    
    Map* mpMap;

    std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
