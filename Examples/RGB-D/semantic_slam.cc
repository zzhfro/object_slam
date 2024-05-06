
/**
 * created by zzh 2024.4
 * currently it is an offline mode
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<System.h>
#include<ObjectDetect.h>
#include"BoundingBox.h"
using namespace std;

std::vector<int> dynamic_object={0}; 
//only person are considered dynamic object 

void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

void LoadDetection(const std::string& filename, std::vector<ORB_SLAM2::DetectResult> &DetectResults);
bool matchDetection( std::string image_name, std::string detection_name);

int main(int argc, char **argv)
{   
    if(argc != 6)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association path to detection" << endl;
        return 1;
    }

    // Retrieve paths to images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
    
    // Check consistency in the number of images and depthmaps
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }
      
    // load detection for the convenience of validation currently offline mode
    std::vector<ORB_SLAM2::DetectResult> DetectResults;
    LoadDetection(string(argv[5]),DetectResults);
    
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    std::cout << endl << "-------" << endl;
    std::cout << "Start processing sequence ..." << endl;
    std::cout << "Images in the sequence: " << nImages << endl << endl;
    

    // Main loop
    cv::Mat imRGB, imD;
    assert(nImages==DetectResults.size());
     int ti=0;
    for(int ni=0; ni<nImages; ni++)
    {  
        // Read image and depthmap from file
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        auto t1 = std::chrono::steady_clock::now();
#endif
         
        // Pass the image to the SLAM system
        ORB_SLAM2::DetectResult detect =DetectResults[ti];
        ti++;
        string args=argv[3];
        string tmp=args+"/"+vstrImageFilenamesRGB[ni];
        bool aligned;
        
        while(1)
        {   

            if(matchDetection(tmp,detect.image_name))
            {
                break;
            }
            else
            {
                detect =DetectResults[ti];
                ti++;
            }

        }
        
        SLAM.TrackRGBD(imRGB,imD,tframe,detect);
         
        /*
        std::cout<<"######trackdebug######"<<std::endl;
        std::cout<<vstrImageFilenamesRGB[ni]<<std::endl;
        std::cout<<detect.image_name<<std::endl;
        detect.print_detect();
        std::cout<<"######trackdebug######"<<std::endl;
        */
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        auto t2 = std::chrono::steady_clock::now();
#endif

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
            usleep((T-ttrack)*1e6);
    }

    // Stop all threads
    SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");   

    return 0;
}
/*aligned*/
bool matchDetection( std::string image_name, std::string detection_name) {
    
        if (detection_name == image_name) 
            {
                return true;
            }
        else
           {
              return false;
           }
    
    // 如果没有匹配的检测结果，则返回 false
    
}

void LoadDetection(const std::string& filename, std::vector<ORB_SLAM2::DetectResult> &DetectResults)
{
   std::ifstream file(filename);
   if (!file.is_open()) 
   {
        std::cerr << "Error: Unable to open file " << filename << std::endl;
        
   }
   std::string line;
   std::vector<double> num1;
   std::vector<double> num2;
   std::vector<double> num3;

    while (std::getline(file, line)) {
        // 提取图片名字
        //ORB_SLAM2::DetectResult detect; 
        num1.clear();
        num2.clear();
        num3.clear();
        string image_name = line;
        
        // 读取下三行数字
        std::getline(file, line);
        std::istringstream iss1(line);
        double number;
        while (iss1 >> number) {
            num1.push_back(number);
        }

        std::getline(file, line);
        std::istringstream iss2(line);
        while (iss2 >> number) {
            num2.push_back(number);
        }
        
        std::getline(file, line);
        std::istringstream iss3(line);
        while (iss3 >> number) {
            num3.push_back(number);
        }
        int object_num=num2.size();
        ORB_SLAM2::DetectResult detect(image_name,object_num,num2,num3,num1);
        
        detect.set_dynamicBoxes(dynamic_object);
        detect.set_staticBoxes(dynamic_object);
        DetectResults.push_back(detect);
    }

    file.close();

}
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    ifstream fAssociation;
    fAssociation.open(strAssociationFilename.c_str());
    while(!fAssociation.eof())
    {
        string s;
        getline(fAssociation,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}

