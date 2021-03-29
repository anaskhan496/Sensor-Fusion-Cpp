#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"

using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
    // Convert current Lidar point into homogeneous coordinates and store it in the 4D variable X.
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        //map X onto the image plane of the camera. 
        // Store the result in Y.
        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;

    // transform Y back into Euclidean coordinates and store the result in the variable pt.
        cv::Point pt;
        pt.x = Y.at<double>(0, 0) / Y.at<double>(0, 2); // pixel coordinates // x/w
        pt.y = Y.at<double>(1, 0) / Y.at<double>(0, 2); // y/w

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box (Avoids grouping errors)
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }

        } // eof loop over all bounding boxes

        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        { 
          //Only one bbox to which we want to include the lidar point. Thus, enclosingBoxes[0]  
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }

    } // eof loop over all Lidar points
}


void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID); //RNG - Random Number Generator
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150)); // 0 - black, 150 - shade of gray

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0; 
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw; // distance to closest point in x-direction
            ywmin = ywmin<yw ? ywmin : yw; // object width min
            ywmax = ywmax>yw ? ywmax : yw; // object width max

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle to draw around the perimeter of the lidar point cloud
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);  
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}


// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    double dist = 0.0;
    std::vector<cv::DMatch> kptMatches_bbox;

    // double shrinkFactor = 0.15;
    // cv::Rect smallerBox;

    // // shrink current bounding box slightly to avoid having too many outlier points around the edges
    // smallerBox.x = boundingBox.roi.x + shrinkFactor * boundingBox.roi.width / 2.0;
    // smallerBox.y = boundingBox.roi.y + shrinkFactor * boundingBox.roi.height / 2.0;
    // smallerBox.width = boundingBox.roi.width * (1 - shrinkFactor);
    // smallerBox.height = boundingBox.roi.height * (1 - shrinkFactor);

    for(auto match = kptMatches.begin(); match!=kptMatches.end(); match++)
    {
        //extract keypoints between current and previous frame
        cv::KeyPoint query = kptsPrev[match->queryIdx]; 
        cv::KeyPoint train = kptsCurr[match->trainIdx];   

        if(boundingBox.roi.contains(cv::Point(query.pt.x,query.pt.y)) && boundingBox.roi.contains(cv::Point(train.pt.x,train.pt.y)))
        {
            kptMatches_bbox.push_back(*match);
        }     
    }

    for(auto it=kptMatches_bbox.begin(); it!=kptMatches_bbox.end(); it++)
    {
        dist+= cv::norm(kptsCurr.at(it->trainIdx).pt - kptsPrev.at(it->queryIdx).pt); 
    }
    double dist_mean = 0.0;
    if(kptMatches_bbox.size()>0)
    {
        dist_mean = dist/kptMatches_bbox.size();
    }
    else
    {
        return;
    }

    for(auto it=kptMatches_bbox.begin(); it!=kptMatches_bbox.end(); it++)
    {
        double d = cv::norm(kptsCurr.at(it->trainIdx).pt - kptsPrev.at(it->queryIdx).pt); 
        if(d<1.3*dist_mean)
        {
            boundingBox.kptMatches.push_back(*it);
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
// - kptsPrev and kptsCurr are the input keypoint sets, kptMatches are the matches between the two sets,
//   frameRate is required to compute the delta time between frames and TTC will hold the result of the computation
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, 
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = it1 + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        } // eof inner loop over all matched kpts
    }     // eof outer loop over all matched kpts

    // only continue if list of distance ratios is not empty
    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

   // compute camera-based TTC from distance ratios
    // Computing the mean distance ratio as in the function we just discussed would presumably lead to a faulty calculation of the TTC
    /* accumulate(first, last, sum);
    first, last : first and last elements of range whose elements are to be added
    sum :  initial value of the sum*/

    /*
    accumulate(first, last, sum, myfun); 
    myfun : a function for performing any 
        specific task. For example, we can
        find product of elements between
        first and last.
    */

    // double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();

    // double dT = 1 / frameRate;
    // TTC = -dT / (1 - meanDistRatio);

    double MedianDistRatio = 0.0;
    std::sort(distRatios.begin(), distRatios.end());
    if(distRatios.size()%2==0)
    {
        MedianDistRatio = (distRatios[(distRatios.size()/2)] + distRatios[distRatios.size()/2])/2.0;
    }
    else if(distRatios.size()%2!=0)
    {
        MedianDistRatio = distRatios[(distRatios.size()/2)-1];        
    }
    double dT = 1 / frameRate;
    TTC = -dT / (1 - MedianDistRatio);

}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{
    double laneWidth = 4.0;

    vector<double> prevX;
    vector<double> currX;

    //store distance to Lidar points within ego lane    
    for(auto it1=lidarPointsPrev.begin(); it1<lidarPointsPrev.end(); it1++)
    {
        if(abs(it1->y)<= laneWidth/2.0) //considering yth point because it points to the left of the lidar sensor
        {
            prevX.push_back(it1->x);
        }
    }

    for(auto it2=lidarPointsCurr.begin(); it2<lidarPointsCurr.end(); it2++)
    {
        if(abs(it2->y)<= laneWidth/2.0)
        {
            currX.push_back(it2->x);
        }
    }

    //Deal with outlier Lidar points in a statistically robust way to avoid severe estimation errors
    std::sort(prevX.begin(), prevX.end());
    std::sort(currX.begin(), currX.end());

    double medianXCurr = 0.0;
    double medianXPrev = 0.0;

    if(prevX.size()%2==0 && currX.size()%2==0)
    {
       medianXPrev = (prevX[(prevX.size()/2)-1] + prevX[prevX.size()/2])/2.0;
       medianXCurr = (currX[(currX.size()/2)-1] + currX[currX.size()/2])/2.0;
    }
    else if(prevX.size()%2!=0 && currX.size()%2!=0)
    {
        medianXPrev = prevX[(prevX.size()/2)-1];
        medianXCurr = currX[(currX.size()/2)-1];       
    }
    else if(prevX.size()%2==0 && currX.size()%2!=0)
    {
        medianXPrev = (prevX[(prevX.size()/2)-1] + prevX[prevX.size()/2])/2.0;
        medianXCurr = currX[(currX.size()/2)-1];        
    }
    else if(prevX.size()%2!=0 && currX.size()%2==0)
    {
        medianXPrev = prevX[(prevX.size()/2)-1];
        medianXCurr = (currX[(currX.size()/2)-1] + currX[currX.size()/2])/2.0;
    }
    else if(prevX.size()==0 || currX.size()==0)
    {
        TTC = NAN;
        return;
    }
    // compute TTC from both measurements
    TTC = medianXCurr * (1/frameRate) / (medianXPrev - medianXCurr);
}


void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // for(auto it = matches.begin(); it!= matches.end(); it++)
    // {
    //     cout<<it->queryIdx<<" "<<it->trainIdx<<" "<<it->distance<<"\n";
    // }
    // for(auto it = prevFrame.kptMatches.begin(); it!= prevFrame.kptMatches.end(); it++)
    // {
    //     cout<<it->queryIdx<<" "<<it->trainIdx<<" "<<it->distance<<"\n";
    // }
    // cout<<prevFrame.kptMatches.size()<<" "<<prevFrame.keypoints.size()<<"\n";

    vector<vector<int>> pts(prevFrame.boundingBoxes.size(),vector<int>(currFrame.boundingBoxes.size()));
    for(auto match = matches.begin(); match != matches.end(); match++)
    {
        //extract keypoints between current and previous frame
        cv::KeyPoint query = prevFrame.keypoints[match->queryIdx]; 
        cv::KeyPoint train = currFrame.keypoints[match->trainIdx];
        bool query_found = false;
        bool train_found = false;

    //find the bboxes that enclose kpts in prev and curr frame
    //The matched candidates whose box id will be stroed
        std::vector<int> query_id;
        for(int p=0; p<prevFrame.boundingBoxes.size(); p++)
        {   
            if(prevFrame.boundingBoxes[p].roi.contains(cv::Point(query.pt.x,query.pt.y)))
            {
                query_found = true;
                query_id.push_back(p);
            }
        }

        std::vector<int> train_id;
        for(int c=0; c<currFrame.boundingBoxes.size(); c++)
        {   
            if(currFrame.boundingBoxes[c].roi.contains(cv::Point(train.pt.x,train.pt.y)))
            {
                train_found = true;
                train_id.push_back(c);
            }
        }

   //Once stored, find the matched candidates in the array which share the 
   //id in the prev and curr frame and count them
        if(query_found && train_found)
        {
            for(auto q:query_id)
            {
                for(auto t:train_id)
                {
                    pts[q][t] += 1;
  
                }
            }
        }
    }

    // associate bboxes with highest occurences
    for(int p=0; p<prevFrame.boundingBoxes.size(); p++)
    {
        int max = 0;
        for(int c=0; c<currFrame.boundingBoxes.size(); c++)
        {
            if(pts[p][c]>max)
            {
                max = pts[p][c];
                bbBestMatches[prevFrame.boundingBoxes[p].boxID] = currFrame.boundingBoxes[c].boxID;
                // bbBestMatches[p] = c;
            }
        }        
    }
}
