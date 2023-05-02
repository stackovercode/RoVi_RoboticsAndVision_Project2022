#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rw/invkin.hpp>

#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>

// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

// OpenCV 3
#include <opencv2/opencv.hpp>

//#include "sparseStereo.hpp"
//#include "robotics.hpp"

// Qt
#include <QTimer>
#include "ui_SamplePlugin.h"
#include <rws/RobWorkStudio.hpp>
#include <QPushButton>
#include <QApplication>
#include <QStyleFactory>


#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

// Additional:
#include <functional>
#include <vector>
#include <fstream>

#include <iostream>
#include <string>

#include <rw/invkin.hpp>
#include <rw/math.hpp>

using namespace rw::common;
using namespace rw::graphics;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::sensor;
using namespace rwlibs::opengl;
using namespace rwlibs::simulation;

using namespace std;
using namespace rw::math;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;




using namespace rws;

using namespace cv;

using namespace std::placeholders;


class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin
{
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
public:
    SamplePlugin();
    virtual ~SamplePlugin();

    virtual void open(rw::models::WorkCell* workcell);

    virtual void close();

    virtual void initialize();

    int imgTest = 0;

    rw::models::SerialDevice::Ptr robot_ur5;

private slots:
    void btnPressed();
    void interpolation(Q Q1, Q Q2, int sample);
    void timer();
    void getImage();
    void get25DImage();
    void getCollectImageTest ();

    // Vision
    cv::Mat poseEstimation();
    void printProjectionMatrix(std::string frameName, cv::Mat &proj_mat, cv::Mat &cam_mat);
    void printProjectionMatrix(std::string frameName, rw::models::WorkCell::Ptr wc, cv::Mat &proj_mat, cv::Mat &cam_mat);
    void moveCylinder(int index);
    std::pair<cv::Mat, cv::Mat> findPose(cv::Mat left_img, cv::Mat right_img, cv::Mat projLeftMat, cv::Mat projRightMat);
    std::pair<cv::Point2d, cv::Mat> locateBallV2(cv::Mat undistortedImage);


    // Robotics
    void P2P(cv::Mat pose);
    double constant_vel(double t, double t0, double t1);
    void resetRobotAndObject();
    std::pair<std::vector<rw::math::Q>,std::vector<rw::math::Transform3D<>>> P2P(std::vector<rw::math::Transform3D<>> P, std::vector<float> T, rw::kinematics::MovableFrame* targetFrame, rw::models::SerialDevice::Ptr robot_ur5, rw::models::WorkCell::Ptr wc, rw::kinematics::State state, rw::proximity::CollisionDetector& detector);
    rw::math::Q findCollisionFreeSolution(rw::models::SerialDevice::Ptr robot_ur5, rw::kinematics::State state, rw::proximity::CollisionDetector& detector, std::vector<rw::math::Q> solutions);
    rw::math::Q findCollisionFreeSolution(rw::models::SerialDevice::Ptr robot_ur5, rw::kinematics::State state, rw::proximity::CollisionDetector& detector, std::vector<rw::math::Q> solutions, rw::math::Q prevSolution);
    // std::vector<rw::math::Transform3D<>> pointOrder(rw::math::Transform3D<> pickFrame, rw::math::Transform3D<> homeFrame, rw::math::Transform3D<> placeFrame, rw::math::Transform3D<> nearPickFrame, rw::math::Transform3D<> nearPlaceFrame);

    void stateChangedListener(const rw::kinematics::State& state);

    bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q);
    void createPathRRTConnect(Q from, Q to,  double extend, double maxTime);
	void printProjectionMatrix(std::string frameName);
    double constantVelocity(double t, double t0, double t1);  
    rw::math::Transform3D<double> convertToTransform3D(const Eigen::Vector3d translation, const Eigen::Quaterniond rotation);
    std::vector<rw::math::Q> findConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot_ur5, rw::models::WorkCell::Ptr wc, rw::kinematics::State state);



private:

    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);

    QTimer* _timer;
    QTimer* _timer25D;
    
    
    Device::Ptr _device;
    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;
    rwlibs::simulation::GLFrameGrabber25D* _framegrabber25D;    
    std::vector<std::string> _cameras;
    std::vector<std::string> _cameras25D;
    QPath _path;
    int _step;
    int _stepGrasp;
    int _stepRelease;
    rw::kinematics::MovableFrame* _ball;
    // rw::kinematics::MovableFrame* _cylinder;
    //  rw::kinematics::MovableFrame* _dino;
    // rw::kinematics::MovableFrame* _square;
    // rw::kinematics::MovableFrame* _bottle;
    rw::kinematics::MovableFrame* _base;
    rw::kinematics::Frame* _table;
    rw::kinematics::Frame* _tcp;
    rw::kinematics::MovableFrame *_target;
    rw::math::Q _home;
    rw::math::Transform3D<> _ballPos;
    rw::models::SerialDevice::Ptr _UR5;
    rw::math::Transform3D<> _ballHomePos;


};

#endif /*RINGONHOOKPLUGIN_HPP_*/
