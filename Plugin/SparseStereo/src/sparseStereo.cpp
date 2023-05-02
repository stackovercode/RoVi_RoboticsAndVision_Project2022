#include "sparseStereo.hpp"

sparseStereo::sparseStereo( int hough_dp, int hough_upperThres,
                int hough_centerThres, int hough_minRadius,
                int hough_maxRadius)
    :   mHough_dp{hough_dp},
        mHoughUpperThres{hough_upperThres},
        mHoughCenterThres{hough_centerThres},
        mHoughMinRadius{hough_minRadius},
        mHoughMaxRadius{hough_maxRadius}
{}

void sparseStereo::printProjectionMatrix(std::string frameName, rw::models::WorkCell::Ptr wc, cv::Mat &proj_mat, cv::Mat &cam_mat) {
    rw::kinematics::State state = wc->getDefaultState();
    rw::kinematics::Frame* cameraFrame = wc->findFrame(frameName);
    if (cameraFrame != NULL) {
        if (cameraFrame->getPropertyMap().has("Camera")) {
            // Read the dimensions and field of view
            double fovy;
            int width, height;
            std::string camParam = cameraFrame->getPropertyMap().get<std::string>("Camera");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;

            double fovy_pixel = height / 2 / tan(fovy * (2*M_PI) / 360.0 / 2.0 );

            Eigen::Matrix<double, 3, 4> KA;
            KA << fovy_pixel, 0, width / 2.0, 0,
                  0, fovy_pixel, height / 2.0, 0,
                  0, 0, 1, 0;

            // OPENCV //
            cv::Mat KA_opencv = (cv::Mat_<double>(3, 4) << fovy_pixel, 0, width/2.0, 0,
                                                           0, fovy_pixel, height / 2.0, 0,
                                                           0, 0, 1, 0);

            cam_mat = KA_opencv.colRange(0, 3);

            std::cout << "Intrinsic parameters:" << std::endl;
            std::cout << KA << std::endl;

            rw::math::Transform3D<> camPosOGL = cameraFrame->wTf(state); // Transform world to camera
            rw::math::Transform3D<> openGLToVis = rw::math::Transform3D<>(rw::math::RPY<>(-rw::math::Pi, 0, rw::math::Pi).toRotation3D()); // Rotate camera to point towards the table
            rw::math::Transform3D<> H = inverse(camPosOGL * inverse(openGLToVis));

            std::cout << "Extrinsic parameters:" << std::endl;
            std::cout << H.e() << std::endl;

            cv::Mat H_opencv = (cv::Mat_<double>(4, 4) << H.R().getRow(0)[0], H.R().getRow(0)[1], H.R().getRow(0)[2], H.P()[0],
                                                          H.R().getRow(1)[0], H.R().getRow(1)[1], H.R().getRow(1)[2], H.P()[1],
                                                          H.R().getRow(2)[0], H.R().getRow(2)[1], H.R().getRow(2)[2], H.P()[2],
                                                                   0        ,           0       ,          0        ,     1   );
            // Calculates projection matrix opencv
            proj_mat = KA_opencv * H_opencv;
        }
    }
}

cv::Mat sparseStereo::addGausNoise(double mean, double stdDev, cv::Mat img){
    cv::Mat result = img.clone();
    cv::Mat noise(img.size(), CV_32SC3);
    cv::randn(noise, mean, stdDev);
    result += noise;
    cv::convertScaleAbs(result, result);
    return result;
}

void sparseStereo::evaluatePerformance(float mean, std::vector<float> stdDev, int numberOfImages, cv::Mat projLeftMat, cv::Mat projRightMat){
    const std::string imagePath =  "../../data/test30images/";
    
    // Loading images
    std::vector<cv::Mat> testLeftImg;
    std::vector<cv::Mat> testRightImg;

    for(unsigned int i = 1; i < numberOfImages+1; i++){
        testLeftImg.push_back(cv::imread(imagePath + "Camera_Left" + std::to_string(i) + ".png"));
        testRightImg.push_back(cv::imread(imagePath + "Camera_Right" + std::to_string(i) + ".png"));
    }

    cv::Mat left_eval_pic;
    cv::Mat right_eval_pic;
    cv::Mat ball_pose;
    float cur_std_dev;

    std::ofstream myFile;
    std::ofstream myFileTime;
    myFileTime.open("../performaceTime");
    myFile.open("../performanceEstimation");


    for(unsigned int i = 0; i < stdDev.size(); i++){
        cur_std_dev = stdDev[i];
        for(unsigned int j = 0; j < numberOfImages; j++){
            if(j % 5 == 0){
                std::cout << "Testing standard deviation " << cur_std_dev << " Picture " << j << " out of " << numberOfImages << std::endl;
            }
            left_eval_pic = addGausNoise(0, cur_std_dev, testLeftImg[j]);
            right_eval_pic = addGausNoise(0, cur_std_dev, testRightImg[j]);


            auto start = std::chrono::high_resolution_clock::now();
            ball_pose = findPose(left_eval_pic, right_eval_pic, projLeftMat, projRightMat);
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);

            myFile << cur_std_dev << " " << ball_pose.at<double>(0, 0) << " " << ball_pose.at<double>(0, 1) << " " << ball_pose.at<double>(0, 2) << std::endl;
            myFileTime << duration.count() << std::endl;

            // std::string left_eval_pic_filename = "/home/reventlov/RobCand/RoVi_Project/SparseStereo/data/TestImagesNoise/left_eval_pic_std_" + std::to_string(cur_std_dev)  + "_img_" + std::to_string(j) + ".png";  
            // cv::imwrite (left_eval_pic_filename, left_eval_pic);
         }
    }
    myFileTime.close();
    myFile.close();
}

std::pair<cv::Point2d, cv::Mat> sparseStereo::locateBallV2(cv::Mat undistortedImage){
    std::vector<cv::Vec3f> circles;
    cv::Scalar lowerHSVBoundary = {50, 50, 0}, upperHSVBoundary = {75, 255, 255};

	// Blur the image
	cv::Mat image_kernel;
	cv::blur(undistortedImage, image_kernel, cv::Size(3, 3));

	// Convert to HSV colorspace
	cv::Mat image_hsv;
	cv::cvtColor(image_kernel, image_hsv, cv::COLOR_BGR2HSV);

    cv::erode(image_hsv, image_hsv, cv::Mat(), cv::Point(-1, -1), 1, 1, 1);
    cv::dilate(image_hsv, image_hsv, cv::Mat(), cv::Point(-1, -1), 1, 1, 1);

	// Threshold the HSV image, keep only the green pixels
	cv::Mat lower_green_hue_range;
	cv::Mat upper_green_hue_range;
	cv::inRange(image_hsv, cv::Scalar(30, 86, 6), cv::Scalar(65, 255, 255), lower_green_hue_range);
	cv::inRange(image_hsv, cv::Scalar(30, 86, 6), cv::Scalar(65, 255, 255), upper_green_hue_range);

	// Combine the above two images
	cv::Mat green_hue_image;
	cv::addWeighted(lower_green_hue_range, 1.0, upper_green_hue_range, 1.0, 0.0, green_hue_image);

    cv::HoughCircles(green_hue_image, circles, cv::HOUGH_GRADIENT, mHough_dp, green_hue_image.rows, mHoughUpperThres, mHoughCenterThres, mHoughMinRadius, mHoughMaxRadius);
    
    cv::Point2d objectCenter;

	if (circles.size() == 0) std::cout << "No circles were detected." << std::endl;
	else{
		for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle){
			objectCenter.x = round(circles[current_circle][0]);
            objectCenter.y = round(circles[current_circle][1]);
			int radius = round(circles[current_circle][2]);

			cv::circle(undistortedImage, objectCenter, radius, cv::Scalar(0, 0, 255), 2);
		}
	}


    return std::make_pair(objectCenter, undistortedImage);
}

std::pair<cv::Point2d, cv::Mat> sparseStereo::locateBall(cv::Mat undistortedImage){
    std::vector<cv::Vec3f> circles;
    cv::Mat edgeUndistortedImage = undistortedImage;
    cv::Scalar lowerHSVBoundary = {50, 50, 0}, upperHSVBoundary = {75, 255, 255};

	cv::Mat image_kernel;


	cv::Mat image_hsv;
	cv::cvtColor(undistortedImage, image_hsv, cv::COLOR_BGR2HSV);


    cv::erode(image_hsv, image_hsv, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(image_hsv, image_hsv, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(image_hsv, image_hsv, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(image_hsv, image_hsv, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

	 cv::Mat lower_green_hue_range, green_hue_image;

	cv::inRange(image_hsv, lowerHSVBoundary, upperHSVBoundary, green_hue_image);


    cv::GaussianBlur(green_hue_image, green_hue_image, cv::Size(3,3), 0, 0);

    cv::HoughCircles(green_hue_image, circles, cv::HOUGH_GRADIENT, mHough_dp, green_hue_image.rows, mHoughUpperThres, mHoughCenterThres, mHoughMinRadius, mHoughMaxRadius);

    cv::Point2d objectCenter;
    int radius;

	if (circles.size() == 0) std::cout << "No circles were detected." << std::endl;
	else{
		for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle){
			objectCenter.x = round(circles[current_circle][0]);
            objectCenter.y = round(circles[current_circle][1]);
			radius = round(circles[current_circle][2]);

			cv::circle(undistortedImage, objectCenter, radius, cv::Scalar(0, 0, 255), 2);
		}
	} 
    // TESING IMPLEMENTATION FOR SIZE CIRCLE FILTERING 
    //cv::Mat undistortedImageNew;
    //if (circles.size() > 1){
        //std::cout << "Using alternative locater" << std::endl;
        //std::pair<cv::Point2d, int> locate = myEdgeDet(edgeUndistortedImage,edgeUndistortedImage);
            //if (abs(radius - locate.second) < 0.3) {
                //std::cout << "Using egde data" << std::endl;
                //cv::circle(undistortedImage, locate.first, locate.second, cv::Scalar(0, 0, 255), 2);
                //cv::imshow("egde", undistortedImage);
                //cv::waitKey(0);
                //return std::make_pair(locate.first, undistortedImage);
            //}
    //}
    //std::cout << objectCenter << std::endl;
    //  std::pair<cv::Point2d, int> locate = myEdgeDet(edgeUndistortedImage,edgeUndistortedImage);
    
    // cv::imshow("result", undistortedImage);
    // cv::waitKey(0);


    return std::make_pair(objectCenter, undistortedImage);
}


// Key method for locationg the pose
cv::Mat sparseStereo::findPose(cv::Mat left_img, cv::Mat right_img, cv::Mat projLeftMat, cv::Mat projRightMat){
    cv::Mat triangulatePoint(1, 1, CV_64FC4);
    cv::Mat leftPoint(1, 1, CV_64FC2);
    cv::Mat rightPoint(1, 1, CV_64FC2);
    cv::Mat result;

    std::pair<cv::Point2d,cv::Mat> locateCircleLeft = locateBallV2(left_img);
    std::pair<cv::Point2d,cv::Mat> locateCircleRight = locateBallV2(right_img);


    leftPoint.at<cv::Vec2d>(0) = locateCircleLeft.first;
    rightPoint.at<cv::Vec2d>(0) = locateCircleRight.first;

    cv::triangulatePoints(projLeftMat, projRightMat, leftPoint, rightPoint, triangulatePoint);
    triangulatePoint =  triangulatePoint/triangulatePoint.at<double>(0, 3);

    cv::Mat transformationTable = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0,
                                                  0, 1, 0, 0,
                                                  0, 0, 1, 0.1,
                                                  0, 0, 0, 1);

    return (transformationTable*triangulatePoint);
}


int sparseStereo::getHough_dp() const{
    return mHough_dp;
}

int sparseStereo::getHoughUpperThres() const{
    return mHoughUpperThres;
}

int sparseStereo::getHoughCenterThres() const{
    return mHoughCenterThres;
}

int sparseStereo::getHoughMinRadius() const{
    return mHoughMinRadius;
}

int sparseStereo::getHoughMaxRadius() const{
    return mHoughMaxRadius;
}

// Testing method for verifying the correct HSV Tresholds
cv::Mat sparseStereo::colorFiltering(const cv::Mat &input) {
    cv::Mat hsv, mask, result;  
    int lowH = 0, highH = 179, lowS = 0, highS = 255, lowV = 0, highV = 255;
    cv::namedWindow("Control", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("LowH", "Control", &lowH, 179);
    cv::createTrackbar("HighH", "Control", &highH, 179);
    cv::createTrackbar("LowS", "Control", &lowS, 255);
    cv::createTrackbar("HighS", "Control", &highS, 255);
    cv::createTrackbar("LowV", "Control", &lowV, 255);
    cv::createTrackbar("HighV", "Control", &highV, 255);
    while (true) {
        cv::cvtColor(input, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(highH, highS, highV), mask);
        cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
        cv::bitwise_and(input, input, result, mask);
        cv::imshow("Thresholded Image", mask);
        if (cv::waitKey(0) == 27) { break; }
    }
    return result;
}

// Implementation of Sobel edge detection. Developed in Machine Vision Course 2019 (Created by Emil Reventlov Husted)
std::pair<cv::Point2d, int> sparseStereo::myEdgeDet(const cv::Mat& inputImage, cv::Mat &outputImage){
    cv::Mat img(inputImage.rows, inputImage.cols, CV_8U);
    cvtColor(inputImage, img, cv::COLOR_BGR2GRAY);
    threshold(img, img, 82, 255, cv::THRESH_BINARY); // 175, 255,

    //resize(img, img, Size(320, 240));
    outputImage = cv::Mat::zeros(img.size(), CV_16U);

    int sobel_x[3][3];
    int sobel_y[3][3];

    //x-direction mask
    sobel_x[0][0] = -1; sobel_x[0][1] = 0; sobel_x[0][2] =1;
    sobel_x[1][0] = -2; sobel_x[1][1] = 0; sobel_x[1][2] =2;
    sobel_x[2][0] = -1; sobel_x[2][1] = 0; sobel_x[2][2] =1;

    //y-direction mask
    sobel_y[0][0] = 1; sobel_y[0][1] = 2; sobel_y[0][2] = 1;
    sobel_y[1][0] = 0; sobel_y[1][1] = 0; sobel_y[1][2] = 0;
    sobel_y[2][0] = -1; sobel_y[2][1] = -2; sobel_y[2][2] = -1;

    for (int j = 0; j<img.rows-2; j++)
    {
        for (int i = 0; i<img.cols-2; i++)
        {
            int pixval_x =
            (sobel_x[0][0] * static_cast<int>(img.at<uchar>(j-1,i-1))) + (sobel_x[0][1] * static_cast<int>(img.at<uchar>(j,i-1))) + (sobel_x[0][2] * static_cast<int>(img.at<uchar>(j+1,i-1))) +
            (sobel_x[1][0] * static_cast<int>(img.at<uchar>(j-1,i))) + (sobel_x[1][1] * static_cast<int>(img.at<uchar>(j,i))) + (sobel_x[1][2] * static_cast<int>(img.at<uchar>(j+1,i))) +
            (sobel_x[2][0] * static_cast<int>(img.at<uchar>(j-1,i+1))) + (sobel_x[2][1] * static_cast<int>(img.at<uchar>(j,i+1))) + (sobel_x[2][2] * static_cast<int>(img.at<uchar>(j+1,i+1)));

            int pixval_y =
            (sobel_y[0][0] * static_cast<int>(img.at<uchar>(j-1,i-1))) + (sobel_y[0][1] * static_cast<int>(img.at<uchar>(j,i-1))) + (sobel_y[0][2] * static_cast<int>(img.at<uchar>(j+1,i-1))) +
            (sobel_y[1][0] * static_cast<int>(img.at<uchar>(j-1,i))) + (sobel_y[1][1] * static_cast<int>(img.at<uchar>(j,i))) + (sobel_y[1][2] * static_cast<int>(img.at<uchar>(j+1,i))) +
            (sobel_y[2][0] * static_cast<int>(img.at<uchar>(j-1,i+1))) + (sobel_y[2][1] * static_cast<int>(img.at<uchar>(j,i+1))) + (sobel_y[2][2] * static_cast<int>(img.at<uchar>(j+1,i+1)));

            int sum = hypot(pixval_x, pixval_y);
            outputImage.at<ushort>(j,i) = sum;
        }
    }
    outputImage.convertTo(outputImage, CV_8U);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(outputImage, circles, cv::HOUGH_GRADIENT, 1, outputImage.rows, 80, 1, 15, 35);

    cv::Point2d objectCenter;
    int radius;

	if (circles.size() == 0) std::cout << "No circles were detected." << std::endl;
	else{
		for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle){
			objectCenter.x = round(circles[current_circle][0]);
            objectCenter.y = round(circles[current_circle][1]);
			radius = round(circles[current_circle][2]);
		}
	}
    return std::make_pair(objectCenter, radius);
}