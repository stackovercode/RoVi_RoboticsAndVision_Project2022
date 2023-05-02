#ifndef SPARSESTEREO_HPP
#define SPARSESTEREO_HPP

#include <rw/rw.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <string>
#include <utility>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

// Point cloud include
#include <pcl-1.12/pcl/point_types.h>
#include <pcl-1.12/pcl/point_cloud.h>
#include <pcl-1.12/pcl/io/io.h>
#include <pcl-1.12/pcl/io/pcd_io.h>


using pclPoint = pcl::PointXYZRGB;
using pclCloud = pcl::PointCloud<pclPoint>;

class denseStereo {
public:
    denseStereo();
    //denseStereo(std::string firstImage, std::string secoundImade);

    cv::Mat computeDisparity(cv::Mat imageOne, cv::Mat imageTwo,int nDips, int bloSiz, int type);
    cv::Mat normaliseDisparity(cv::Mat disparityMap);

    void savePointCloud(std::string filename, cv::Mat points, cv::Mat colors, double max_z);
    
    //Helper functions for showing images
    void showImage(cv::Mat image, std::string nameWin);
    void showImages(cv::Mat firstImage, cv::Mat secoundImage, std::string winName1,std::string winName2);

    cv::Mat qMat(int img_width, int img_height, double Tx, double f);

    cv::Mat reproject3D(cv::Mat disp, cv::Mat Q);

private:
    cv::Mat mImageOne;
    cv::Mat mImageTwo;
    cv::Mat disparityMap;

    pcl::PointXYZRGB p_test;

};

#endif // SPARSESTEREO_HPP