#ifndef SPARSESTEREO_HPP
#define SPARSESTEREO_HPP

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
#include "opencv2/xfeatures2d.hpp"

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rw/invkin.hpp>


class sparseStereo {
public:
    sparseStereo(int hough_dp=1, 
            int hough_upperThres=100,
            int hough_centerThres=1, 
            int hough_minRadius=22,
            int hough_maxRadius=32);
    
    void printProjectionMatrix(std::string frameName, rw::models::WorkCell::Ptr wc, cv::Mat &proj_mat, cv::Mat &cam_mat);
    cv::Mat addGausNoise(double mean, double stdDev, cv::Mat img);

    std::pair<cv::Point2d, cv::Mat> locateBallV2(cv::Mat undistortedImage);


    void evaluatePerformance(float mean, std::vector<float> stdDev, int numberOfImages, cv::Mat projLeftMat, cv::Mat projRightMat);
    void moveBall(rw::models::WorkCell::Ptr workcell, rw::kinematics::State state, const std::string frameName);

    cv::Mat findPose(cv::Mat leftImage, cv::Mat rightImage, cv::Mat projLeftMat, cv::Mat projRightMat);
    std::pair<cv::Point2d, cv::Mat> locateBall(cv::Mat undistortedImage);

    cv::Mat colorFiltering(const cv::Mat &input); 
    std::pair<cv::Point2d, int> myEdgeDet(const cv::Mat& inputImage, cv::Mat &outputImage);

    int getHough_dp() const;
    int getHoughUpperThres() const;
    int getHoughCenterThres() const;
    int getHoughMinRadius() const;
    int getHoughMaxRadius() const;

    std::string WC_FILE = "../../../WorkCell/Scene.wc.xml";

private:
    cv::Point mOpticalCenter;
    //cv::Mat mMapX, mMapY;
    cv::Scalar mLowerBoundary, mUpperBoundary;

    int mHough_dp,
        mHoughUpperThres,
        mHoughCenterThres,
        mHoughMinRadius,
        mHoughMaxRadius;
};

#endif // SPARSESTEREO_HPP