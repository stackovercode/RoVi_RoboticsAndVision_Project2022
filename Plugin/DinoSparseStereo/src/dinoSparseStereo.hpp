#ifndef DINOSPARSESTEREO_HPP
#define DINOSPARSESTEREO_HPP

#include <rw/rw.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <string>
#include <utility> // std::pair

#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/features2d.hpp>


// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rw/invkin.hpp>


class dinoSparseStereo {
public:
    dinoSparseStereo(int hough_dp=1, 
            int hough_upperThres=100,
            int hough_centerThres=1, 
            int hough_minRadius=10,
            int hough_maxRadius=30);
    
    void printProjectionMatrix(std::string frameName, rw::models::WorkCell::Ptr wc, cv::Mat &proj_mat, cv::Mat &cam_mat);
    cv::Mat addGaussianNoise(double mean, double stdDev, cv::Mat img);

    void SIFTDetector(cv::Mat &img_marker);
    std::vector<cv::Point2f> GetCornersOfMarkerInScene(cv::Mat &img_scene);


    void evaluatePerformance(float mean, std::vector<float> stdDev, int numberOfImages, cv::Mat projLeftMat, cv::Mat projRightMat);
    void moveBall(rw::models::WorkCell::Ptr workcell, rw::kinematics::State state, const std::string frameName);

    cv::Mat findPose(cv::Mat leftImage, cv::Mat rightImage, cv::Mat projLeftMat, cv::Mat projRightMat);
    std::pair<cv::Point2d, cv::Mat> locateBall(cv::Mat undistortedImage);

    //cv::Mat colorFiltering(const cv::Mat &input); 

    int getHough_dp() const;
    int getHoughUpperThres() const;
    int getHoughCenterThres() const;
    int getHoughMinRadius() const;
    int getHoughMaxRadius() const;

    std::string WC_FILE = "../../../WorkCell/Scene.wc.xml";

private:
    cv::Point mOpticalCenter;
    cv::Scalar mLowerBoundary, mUpperBoundary;

    int mHough_dp,
        mHoughUpperThres,
        mHoughCenterThres,
        mHoughMinRadius,
        mHoughMaxRadius;
         
    //cv::Ptr<cv::Feature2D> f2d;
    cv::Mat descriptors_marker;
    std::vector<cv::KeyPoint> keypoints_marker;
    cv::Mat img_marker;
};

#endif // DINOSPARSESTEREO_HPP