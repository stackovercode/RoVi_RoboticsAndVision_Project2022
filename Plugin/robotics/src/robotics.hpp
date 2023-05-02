#ifndef ROBOTICS_HPP
#define ROBOTICS_HPP

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>
#include <rw/invkin.hpp>
#include <rw/math/EAA.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/math/Transform3D.hpp>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>

// Additional:
#include <functional>
#include <numeric>
#include <rw/rw.hpp>
#include <iostream>
#include <vector>
#include <map>
#include <fstream>
#include <string>
#include <utility>
#include <cstdio>
#include <cstdlib>
#include <Eigen/Dense>
#include <Eigen/Geometry>



class Robotics{
    
public:
    Robotics();

    double constantVelocity(double t, double t0, double t1);

    std::vector<rw::math::Transform3D<>> runLiTime(std::vector<rw::math::Transform3D<>> cartesian, std::vector<float> Time);

 
    rw::trajectory::InterpolatorTrajectory<rw::math::Q> Robotics::linInterp(std::vector<rw::math::Q> q, 
                                                                            double duration, 
                                                                            rw::trajectory::TimedStatePath& timestatepath, 
                                                                            rw::models::SerialDevice::Ptr robot_ur5, 
                                                                            rw::kinematics::State state);


    void performTest(std::vector<rw::math::Q> q,
                                    std::vector<float> rimes, 
                                    std::string f_name,
                                    rw::kinematics::MovableFrame *targetFrame, 
                                    rw::models::SerialDevice::Ptr robot_ur5, 
                                    rw::models::WorkCell::Ptr wc, 
                                    rw::kinematics::State state, 
                                    rw::proximity::CollisionDetector& detector);

    
    std::pair<std::pair<Eigen::Vector3d, Eigen::Matrix3d>,Eigen::Matrix4d> trans2Eigen(rw::math::Transform3D<> P);


    std::vector<rw::math::Q> findConfigurations(const std::string nameGoal, 
                                            const std::string nameTcp, 
                                            rw::models::SerialDevice::Ptr robot_ur5, 
                                            rw::models::WorkCell::Ptr wc, 
                                            rw::kinematics::State state);

    rw::math::Q findCollisionFreeSolution(rw::models::SerialDevice::Ptr robot_ur5, 
                                        rw::kinematics::State state, 
                                        rw::proximity::CollisionDetector& detector, 
                                        std::vector<rw::math::Q> solutions);

    rw::math::Q findCollisionFreeSolution(rw::models::SerialDevice::Ptr robot_ur5, 
                                        rw::kinematics::State state, 
                                        rw::proximity::CollisionDetector& detector, 
                                        std::vector<rw::math::Q> solutions, 
                                        rw::math::Q prevSolution);

    std::pair<std::vector<rw::math::Q>,std::vector<rw::math::Transform3D<>>> CollisionFreeP2P(std::vector<rw::math::Transform3D<>> P, 
                                                                                            std::vector<float> T, 
                                                                                            rw::kinematics::MovableFrame *targetFrame, 
                                                                                            rw::models::SerialDevice::Ptr robot_ur5, 
                                                                                            rw::models::WorkCell::Ptr wc, 
                                                                                            rw::kinematics::State state, 
                                                                                            rw::proximity::CollisionDetector& detector);
                                                                                            

    rw::math::Transform3D<double> convertToTransform3D(const Eigen::Vector3d translation, const Eigen::Quaterniond rotation);



    void createRwplay(std::vector<rw::math::Q> jointPath, 
                        rw::models::SerialDevice::Ptr robot_ur5,
                        rw::models::WorkCell::Ptr wc, 
                        rw::kinematics::State state, 
                        const std::string rwplayPath);

    void createData(std::vector<rw::math::Q> joints, const std::string jointPath, std::vector<rw::math::Transform3D<>> cartesian, const std::string cartesianPath, std::vector<float> Time, const std::string timePath);



    std::string WC_FILE = "../../WorkCell/Scene.wc.xml";

private:

 
};

#endif // ROBOTICS_HPP