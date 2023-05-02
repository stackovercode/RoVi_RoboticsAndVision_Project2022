#include "denseStereo.hpp"
/*
denseStereo::denseStereo(std::string firstImagePath, std::string secoundImadePath){
    std::cout << "Creating the class" << std::endl;
    
    cv::Mat firstImage = cv::imread(firstImagePath);
    cv::Mat secoundImage = cv::imread(secoundImadePath);

    if (firstImage.empty())
    {
        std::cout << "The first image was empty at path: "<< firstImagePath << std::endl;
    }
    if (secoundImage.empty())
    {
        std::cout << "The secound image was empty at path: "<< secoundImadePath << std::endl;
    }

    cv::cvtColor(firstImage,mImageOne,cv::COLOR_BGR2GRAY);
    cv::cvtColor(secoundImage,mImageTwo,cv::COLOR_BGR2GRAY);
    std::string nameW = "First Image";
    cv::namedWindow(nameW);
    cv::imshow(nameW,mImageOne);
    cv::waitKey(0);
    nameW = "Secound Image";
    cv::namedWindow(nameW);
    cv::imshow(nameW,mImageTwo);
    cv::waitKey(0);

}*/
denseStereo::denseStereo(){
    std::cout << "Made the object" << std::endl;
};

cv::Mat denseStereo::computeDisparity(cv::Mat imageOne, cv::Mat imageTwo,int nDips, int bloSiz, int type)
{
    cv::Mat disparityTemp;
    auto bm = cv::StereoBM::create(nDips,bloSiz);
    auto SGBM = cv::StereoSGBM::create(0,nDips,bloSiz);
    if (type == 1)
    {
        bm -> compute(imageOne,imageTwo,disparityTemp);
    }
    else if (type == 2)
    {
        SGBM -> compute(imageOne,imageTwo,disparityTemp);
    }
    disparityTemp *= 1./16.;
    return disparityTemp;
}

cv::Mat denseStereo::normaliseDisparity(cv::Mat disparityMap)
{
    cv::Mat normTemp;
    cv::normalize(disparityMap,normTemp,0,255,cv::NORM_MINMAX, CV_8UC1);
    return normTemp;
}

void denseStereo::savePointCloud(std::string filename, cv::Mat points, cv::Mat colors, double max_z) {
    pclPoint p_default;
    pclCloud::Ptr dst(new pclCloud(points.rows, points.cols, p_default));
    for (size_t i = 0; i < points.rows; i++) {
        for (size_t j = 0; j < points.cols; j++) {
            cv::Vec3f xyz = points.at<cv::Vec3f>(i, j);
            cv::Vec3b bgr = colors.at<cv::Vec3b>(i, j);
            // Check if points are too far away, if not take them into account
            if (fabs(xyz[2]) < max_z) {
                pclPoint pn;
                pn.x = xyz[0];
                pn.y = xyz[1];
                pn.z = xyz[2];
                pn.r = bgr[2];
                pn.g = bgr[1];
                pn.b = bgr[0];
                dst->at(i, j) = pn;
            }
        }
    }
    pcl::io::savePCDFileASCII(filename, *dst);
}
void denseStereo::showImage(cv::Mat image, std::string nameWin)
{
    cv::namedWindow(nameWin);
    cv::imshow(nameWin,image);
    //cv::waitKey(0);
}
void denseStereo::showImages(cv::Mat firstImage, cv::Mat secoundImage, std::string winName1,std::string winName2)
{   
    //std::string winName = "First Image";
    cv::namedWindow(winName1);
    cv::imshow(winName1,firstImage);
    cv::namedWindow(winName2);
    cv::imshow(winName2,secoundImage);
    cv::waitKey(0);
}

cv::Mat denseStereo::qMat(int img_width, int img_height, double Tx, double f)
{
    double cx = (img_width/2);
    double cy = (img_height/2);
	cv::Mat Q = cv::Mat::zeros(4, 4, CV_64F);
	Q.at<double>(0, 0) = 1;
	Q.at<double>(1, 1) = 1;
	Q.at<double>(0, 3) = -cx;
	Q.at<double>(1, 3) = -cy;
	Q.at<double>(2, 3) = f;
	Q.at<double>(3, 2) = -1/Tx;
    return Q;
}

cv::Mat denseStereo::reproject3D(cv::Mat disp, cv::Mat Q)
{
    cv::Mat points;
    cv::reprojectImageTo3D(disp, points, Q, true);
    return points;
}