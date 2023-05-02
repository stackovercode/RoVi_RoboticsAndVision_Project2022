#include "sparseStereo.hpp"



int main() {
    sparseStereo v;
    
    std::string wc_path = "../../../WorkCell/Scene.wc.xml";
    const rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wc_path);
    if ( NULL == wc ){
        RW_THROW("Error loading workcell");
        return -1;
    }

    //Load ball Frame 
    const std::string target_name = "Ball";
    rw::kinematics::MovableFrame *ballFrame = wc->findFrame<rw::kinematics::MovableFrame>(target_name);
    if (NULL == ballFrame ){
        RW_THROW("COULD NOT FIND BALL FRAME ... check name!");
        return -1;
    }

    rw::kinematics::State state = wc->getDefaultState();

    cv::Mat projLeftMat, projRightMat, cam_left_mat, cam_right_mat;
    v.printProjectionMatrix("Camera_Left", wc, projLeftMat, cam_left_mat);
    v.printProjectionMatrix("Camera_Right", wc, projRightMat, cam_right_mat);

    cv::Mat imgLeftBall, imgRightBall, ballPose;
    imgLeftBall = cv::imread("/home/reventlov/RobCand/RoVi_Project/SparseStereo/data/test30images/Camera_Left27.png");
    imgRightBall = cv::imread("/home/reventlov/RobCand/RoVi_Project/SparseStereo/data/test30images/Camera_Right27.png");

    // cv::Mat test = v.colorFiltering(imgLeftBall);
    // ballPose = v.findPose(imgLeftBall, imgRightBall, projLeftMat, projRightMat);
    // std::cout << ballPose << std::endl;

    
    ballPose = v.findPose(imgLeftBall, imgRightBall, projLeftMat, projRightMat);
    std::cout << ballPose << std::endl;

    // v.moveBall(wc, state, target_name);


// /* THIS IS USED FOR PERFORMANCE OF VISION */
    float mean = 0;
    std::vector<float> stdDev{0, 1, 5, 20, 50, 100, 200, 255};
    //std::vector<float> stdDev{0, 1, 5, 20, 50, 90, 120, 150};
    int numberOfImages = 30;
    v.evaluatePerformance(mean, stdDev, numberOfImages, projLeftMat, projRightMat);
// /* THIS IS USED FOR PERFORMANCE OF VISION */



    return 0;
}

