#include "dinoSparseStereo.hpp"

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;



// Function to detect features in the image
void detectFeatures(Mat image, vector<KeyPoint>& keypoints){
    // Detect the features in the image using FAST algorithm
    cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    FAST(image, keypoints, 20, false);
    //cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);


}

// Function to compute the descriptors for the detected features
void computeDescriptors(Mat image, vector<KeyPoint>& keypoints, Mat& descriptors){
    // Compute the descriptors for the detected features using ORB algorithm
    Ptr<ORB> orb = ORB::create();
    orb->compute(image, keypoints, descriptors);
}

cv::Mat changeBackground(cv::Mat img){
    // Iterate through all pixels in the image
    for (int row = 0; row < img.rows; row++)
    {
        for (int col = 0; col < img.cols; col++)
        {
            cv::Vec3b pixel = img.at<cv::Vec3b>(row, col);
            // Check if the pixel is black (BGR values of (0, 0, 0))
            if (pixel[0] == 0 && pixel[1] == 0 && pixel[2] == 0)
            {
                // Set the pixel to white (BGR values of (255, 255, 255))
                pixel[0] = 255;
                pixel[1] = 255;
                pixel[2] = 255;
                img.at<cv::Vec3b>(row, col) = pixel;
            }
        }
    }
    return img;
}

// Function to match the descriptors between two images
void matchDescriptors(Mat image1, Mat image2, vector<KeyPoint>& keypoints1, vector<KeyPoint>& keypoints2, Mat& descriptors1, Mat& descriptors2, vector<DMatch>& matches)
{
    // Match the descriptors between the two images using Brute Force Matcher
    BFMatcher matcher(NORM_HAMMING);
    matcher.match(descriptors1, descriptors2, matches);


    // Filter the matches using Lowe's ratio test
    vector<DMatch> goodMatches;

    for (int i = 0; i < matches.size(); i++){
        if( matches[i].distance < 0.4 * matches[i + 1].distance){
            goodMatches.push_back(matches[i]);
        }
    }

        // Compute fundamental matrix from the matches
    std::vector<Point2f> left_points, right_points;
    for (const auto& match : goodMatches)
    {
        left_points.push_back(keypoints1[match.queryIdx].pt);
        right_points.push_back(keypoints2[match.trainIdx].pt);
    }

    cv::Mat img_matches;
    cv::drawMatches( image1, keypoints1, image2, keypoints2,
               goodMatches, img_matches, Scalar::all(-1), Scalar::all(-1),
               vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );//NOT_DRAW_SINGLE_POINTS


    cv::Mat reault1, result2, result3;
    reault1 = changeBackground(image1);
    std::string one = "/home/reventlov/RobCand/RoVi_Project/DinoSparseStereo/img/img_1.png";
     cv::imwrite (one, reault1);
    result2 = changeBackground(image2);
    std::string two = "/home/reventlov/RobCand/RoVi_Project/DinoSparseStereo/img/img_2.png";
     cv::imwrite (two, result2);

    //  cv::imshow("Combined features", img_matches);
    //  cv::waitKey(0);
    result3 = changeBackground(img_matches);
    std::string three = "/home/reventlov/RobCand/RoVi_Project/DinoSparseStereo/img/img_matches.png";
    cv::imwrite (three, result3);


    // Get the keypoints from the matches
    for (int i = 0; i < goodMatches.size(); i++)
    { 
        keypoints1.push_back(keypoints1[goodMatches[i].queryIdx]);
        keypoints2.push_back(keypoints2[goodMatches[i].trainIdx]);
    }

   std::vector<Point2f> obj;
   std::vector<Point2f> scene;

    for( int i = 0; i < goodMatches.size(); i++ ){
    //-- Get the keypoints from the good matches
    obj.push_back( keypoints1[ goodMatches[i].queryIdx ].pt );
    scene.push_back( keypoints2[ goodMatches[i].trainIdx ].pt );
    }


    cv::Point2f center_point;
    center_point += cv::Point2f( image2.cols, 0);

    
}

// Function to estimate the pose of the Trex head
void estimatePose(vector<KeyPoint>& keypoints1, vector<KeyPoint>& keypoints2, Mat& rotation, Mat& translation)
{
    // Convert the keypoints to points
    vector<Point2f> points1, points2;
    KeyPoint::convert(keypoints1, points1);
    KeyPoint::convert(keypoints2, points2);
    
    // Estimate the pose of the Trex head using PnP algorithm
    Mat inliers;
    solvePnP(points1, points2, Mat::eye(3, 3, CV_64F), Mat::zeros(1, 4, CV_64F), rotation, translation, true, cv::SOLVEPNP_EPNP);
}


cv::Mat color_threshold_Dino(cv::Mat img){

    cv::Mat image_HSV, mask, dino;
    cv::cvtColor(img, image_HSV, cv::COLOR_BGR2HSV);
    //cv::inRange(image_HSV, cv::Scalar(0, 190, 70), cv::Scalar(154, 255, 255), mask);
    cv::inRange(image_HSV, cv::Scalar(30, 86, 6), cv::Scalar(65, 255, 255), mask);

    // Opening and closing to get whole duck
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);

    //cv::Mat imageWeighted;
    //cv::addWeighted(mask, 1.0, mask, 1.0, 0.0, dino);

    cv::bitwise_and(img, mask, dino);

    // cv::imshow("nksjdlfn", dino);
    // cv::waitKey(0);

    return dino;
}


int main() {
    dinoSparseStereo v;
    
    std::string wc_path = "../../../WorkCell/Scene.wc.xml";
    const rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wc_path);
    if ( NULL == wc ){
        RW_THROW("Error loading workcell");
        return -1;
    }

    rw::kinematics::State state = wc->getDefaultState();

    cv::Mat projLeftMat, projRightMat, cam_left_mat, cam_right_mat, image_hsv1, image_hsv2;
    v.printProjectionMatrix("Camera_Left", wc, projLeftMat, cam_left_mat);
    v.printProjectionMatrix("Camera_Right", wc, projRightMat, cam_right_mat);

    cv::Mat image1, image2, ballPose;
    image1 = cv::imread("/home/reventlov/RobCand/RoVi_Project/DinoSparseStereo/image/Camera_Left1_3.png");
    image2 = cv::imread("/home/reventlov/RobCand/RoVi_Project/DinoSparseStereo/image/Camera_Right1_3.png");

    image_hsv1 = color_threshold_Dino(image1);
    image_hsv2 = color_threshold_Dino(image2);

    // Detect the features in the two images
    vector<KeyPoint> keypoints1, keypoints2;
    detectFeatures(image_hsv1, keypoints1);
    detectFeatures(image_hsv2, keypoints2);
    
    // Compute the descriptors for the detected features
    Mat descriptors1, descriptors2, descriptors_marker, descriptors_scene;
    computeDescriptors(image_hsv1, keypoints1, descriptors1);
    computeDescriptors(image_hsv2, keypoints2, descriptors2);
    
    // Match the descriptors between the two images
    vector<DMatch> matches;
    matchDescriptors(image_hsv1, image_hsv2, keypoints1, keypoints2, descriptors1, descriptors2, matches);
    

    // Estimate the pose of the Trex head
    Mat rotation, translation, resultPose;

    // Convert the keypoints to points
    vector<Point2f> points1, points2;
    // KeyPoint::convert(keypoints1, points1);
    // KeyPoint::convert(keypoints2, points2);


    for( int i = 0; i < matches.size(); i++ ){
    //-- Get the keypoints from the good matches
    points1.push_back( keypoints1[ matches[i].queryIdx ].pt );
    points2.push_back( keypoints2[ matches[i].trainIdx ].pt );
    }


     cv::Mat triangulatePoint(1, points1.size(), CV_64FC4);
    // cv::Mat leftPoint(points1.size(), 1, CV_64FC2);
    // cv::Mat rightPoint(points1.size(), 1, CV_64FC2);
    //cv::Mat triangulatePoint(1, 4, CV_64FC4);
    cv::Mat leftPoint(1, 4, CV_64FC2);
    cv::Mat rightPoint(1, 4, CV_64FC2);
    std::vector<cv::Point3d> triangulate_points_norm;


    for (int i = 0; i < points1.size(); i++){
        leftPoint.at<cv::Vec2f>(i) = points1[i];
        rightPoint.at<cv::Vec2f>(i) = points2[i];
    }

    // leftPoint.at<cv::Vec2f>(0) = points1[0];
    // rightPoint.at<cv::Vec2f>(0) = points2[0];


    cv::triangulatePoints(projLeftMat, projRightMat, leftPoint, rightPoint, triangulatePoint);
    //triangulatePoint =  triangulatePoint/triangulatePoint.at<double>(0, 3);
    cv::Mat mat_triang_norm;
    for (unsigned int i = 0; i < 4; i++) {
        mat_triang_norm = triangulatePoint.col(i).rowRange(0, 3) / triangulatePoint.at<double>(3, i); // Normalizes the 3d points
        triangulate_points_norm.push_back(cv::Point3d(mat_triang_norm.at<double>(0, 0), mat_triang_norm.at<double>(0, 1), mat_triang_norm.at<double>(0, 2))); // Normalizes the 3d points
    }


    //triangulatePoint =  triangulatePoint/triangulatePoint.at<double>(0, 3);



    cv::Mat transformationTable = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0,
                                                  0, 1, 0, 0,
                                                  0, 0, 1, 0.1,
                                                  0, 0, 0, 1);

    cv::Mat pose = cv::Mat::zeros(4, 4, CV_64F);
    // pose.at<double>(0, 0) = 1;
    // pose.at<double>(1, 1) = 1;
    // pose.at<double>(2, 2) = 1;
    // pose.at<double>(3, 3) = 1;

    //cv::triangulatePoints(projLeftMat,projRightMat,leftPoint,rightPoint,pnts3D);

    //triangulatePoints(projLeftMat,projRightMat,leftPoint,rightPoint,sddd);


    // Create a std::vector of cv::Point3d objects representing a set of 3D points
    //std::vector<cv::Point3d> points = { cv::Point3d(1.0, 2.0, 3.0), cv::Point3d(4.0, 5.0, 6.0), cv::Point3d(7.0, 8.0, 9.0) };

    // Create a 4x4 transformation matrix representing a rotation around the X-axis by 45 degrees
    // cv::Mat transformation = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0,
    //                                                   0, cos(CV_PI / 4), -sin(CV_PI / 4), 0,
    //                                                   0, sin(CV_PI / 4), cos(CV_PI / 4), 0,
    //                                                   0, 0, 0, 1);

    // Apply the transformation to the points
    std::vector<cv::Point3d> transformed_points;

    //cv::transform(triangulate_points_norm, transformed_points, transformationTable);


    // // Extract x, y, and z coordinates
    // double x = pose_3d.at<double>(0, 0);
    // double y = pose_3d.at<double>(1, 0);
    // double z = pose_3d.at<double>(2, 0);



    std::cout << "---------------------------------------------" << std::endl;
    //<Pos> 0 0.45 0.11 </Pos> 
    //std::cout << "[" << x << ", " << y << ", " << z << "]" << std::endl;
    //std::cout << triangulate_points_norm << std::endl;
    std::cout << "---------------------------------------------" << std::endl;
    //std::cout << pose_3d << std::endl;
    std::cout << triangulatePoint*transformationTable << std::endl;
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << triangulatePoint << std::endl;
    std::cout << "---------------------------------------------" << std::endl;
    std::cout << triangulate_points_norm << std::endl;


    
    return 0;
}