/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <list>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;

void log(const string& name,
         const vector<double> &log_detection_time,
         const vector<int> &log_detection_points,
         const vector<double> &log_extraction_time,
         const vector<int> &log_matched_points,
         ofstream& logfile

    )
{

    double avg_detection_time = std::accumulate(log_detection_time.begin(),log_detection_time.end(),0) / log_detection_time.size();

    int avg_detection_points = std::accumulate(log_detection_points.begin(),log_detection_points.end(),0) / log_detection_points.size();

    double avg_extraction_time = std::accumulate(log_extraction_time.begin(),log_extraction_time.end(),0) / log_extraction_time.size();

    int avg_matched_points = std::accumulate(log_matched_points.begin(),log_matched_points.end(),0) / log_matched_points.size();

    logfile << name << ": \t\t"
            << avg_detection_time << "ms\t"
            << avg_detection_points << "\t"
            << avg_extraction_time << "ms\t"
            << avg_matched_points << "\t"
            << endl;
}

/* MAIN PROGRAM */
int run(const string& detectorType,const string& descriptorType,
        const string& matcherType,ofstream &logfile)
{
    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 3;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    /* MAIN LOOP OVER ALL IMAGES */

    vector<double> log_detection_time;
    vector<double> log_extraction_time;
    vector<int> log_filterd_points;
    vector<int> log_detection_points;
    vector<int> log_matched_points;

    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        if(dataBuffer.size() == dataBufferSize){
            //dataBuffer.pop_front();
            dataBuffer.erase(dataBuffer.begin());
        }
        dataBuffer.push_back(frame);

        //// EOF STUDENT ASSIGNMENT
        //cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints_origin; // create empty feature list for current image
        vector<cv::KeyPoint> keypoints; // keypoints in front of the vehicle
        //string detectorType = "SHITOMASI";
        //string detectorType = "BRISK";
        //string detectorType = "HARRIS";

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        double t = (double)cv::getTickCount();

        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints_origin, imgGray, false);
        }else if (detectorType.compare("HARRIS") == 0){
            detKeypointsHarris(keypoints_origin, imgGray, false);
        }else{
            detKeypointsModern(keypoints_origin, imgGray,detectorType, false);
        }

        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        log_detection_time.push_back(1000 * t / 1.0);
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);

        if (bFocusOnVehicle)
        {
            for(auto p:keypoints_origin){
                if(vehicleRect.contains(p.pt)){
                    keypoints.push_back(p);
                }
                ///if(p.pt.x > vehicleRect.x
                ///   && p.pt.x < vehicleRect.x+vehicleRect.width
                ///   && p.pt.y > vehicleRect.y
                ///   && p.pt.y < vehicleRect.y+vehicleRect.height){
                ///    keypoints.push_back(p);
                ///}
            }
        }
        log_detection_points.push_back(keypoints.size());
        //log_detection_points.push_back(keypoints_origin.size());
        ////log_filterd_points.push_back(keypoints.size());
        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        bool bLimitKpts = false;
        if (bLimitKpts)
        {
            int maxKeypoints = 50;

            if (detectorType.compare("SHITOMASI") == 0)
            { // there is no response info, so keep the first 50 as they are sorted in descending quality order
                keypoints.erase(keypoints.begin() + maxKeypoints, keypoints.end());
            }
            cv::KeyPointsFilter::retainBest(keypoints, maxKeypoints);
            //cout << " NOTE: Keypoints have been limited!" << endl;
        }

        // push keypoints and descriptor for current frame to end of data buffer
        //(dataBuffer.end() - 1)->keypoints = keypoints;
        dataBuffer.rbegin()->keypoints = keypoints;
        //cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;

        t = (double)cv::getTickCount();
        descKeypoints(dataBuffer.rbegin()->keypoints, dataBuffer.rbegin()->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (--dataBuffer.end())->descriptors = descriptors;

        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        log_extraction_time.push_back(1000 * t / 1.0);

        //cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN

            string distanceType = "DES_BINARY"; // DES_BINARY, DES_HOG

            if (descriptorType.compare("SIFT") == 0){
                distanceType = "DES_HOG";
            }

            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filteristng with t=0.8 in file matching2D.cpp

            matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                             (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                             matches, descriptorType, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            log_matched_points.push_back(matches.size());

            //cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            //bVis = false;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                cout << "Press key to continue to next image" << endl;
                cv::waitKey(0); // wait for key to be pressed
            }
            //bVis = false;
        }

    } // eof loop over all images

    //string log_str = "detectorType: " + detectorType + ", descriptorType: " + descriptorType;
    string log_str = detectorType + "+" + descriptorType;
    log(log_str,
        log_detection_time,
        log_detection_points,
        log_extraction_time,
        log_matched_points,
        logfile);
    return 0;
}

int main(int argc, const char *argv[])
{
    ofstream logfile;
    logfile.open ("log.txt");
    //vector<string> detectorTypes{ "HARRIS", "FAST", "BRISK", "ORB", "AKAZE", "SIFT"};
    //vector<string> detectorTypes{"HARRIS", "FAST", "BRISK", "ORB", "SIFT"};
    vector<string> detectorTypes{"SHITOMASI","HARRIS", "FAST", "BRISK", "ORB", "SIFT"};
    vector<string> descriptorTypes{"BRISK","BRIEF", "ORB", "FREAK", "SIFT"};
    // FLANN, KNN
    vector<string> matcherType = {"BF","FLANN"};
    for(auto detectorType:detectorTypes){
        for(auto descriptorType:descriptorTypes){
            cout << detectorType << ", " << descriptorType << endl;
            if(detectorType.compare("SIFT") == 0
               &&descriptorType.compare("ORB") == 0){
                continue;
            }
            run(detectorType,descriptorType,"BF",logfile);
        }
    }

    //detectorTypes = {"AKAZE"};
    //descriptorTypes = {"BRISK","BRIEF", "ORB", "FREAK", "AKAZE", "SIFT"};
    //for(auto detectorType:detectorTypes){
    //    for(auto descriptorType:descriptorTypes){
    //        cout << detectorType << ", " << descriptorType << endl;
    //        run(detectorType,descriptorType,"BF",logfile);
    //    }
    //}


    //run(detectorType,descriptorType);

    logfile.close();
}