#ifndef REACHABILITY_HPP
#define REACHABILITY_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <cmath>

#include <rw/rw.hpp>
#include <rw/invkin.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

USE_ROBWORK_NAMESPACE
using namespace robwork;

class Reachability
{
    //**************************************************************
    //*                        Public methods                      *
    //**************************************************************
public:
    /** Default Constructor */
    Reachability();

    void saveConfirgurations(const std::vector<float> &x,
                const std::vector<float> &y,
                const std::vector<unsigned int> &solutions,
                const std::string &file_path);

    std::vector<rw::math::Q> getConfigurations(const std::string nameGoal,
                                            const std::string nameTcp,
                                            rw::models::SerialDevice::Ptr robot,
                                            rw::models::WorkCell::Ptr wc,
                                            rw::kinematics::State state); 


    std::vector<rw::math::Q> getCollisionFreeSolutions(rw::models::WorkCell::Ptr wc,
                                                    rw::models::SerialDevice::Ptr device,
                                                    rw::kinematics::MovableFrame::Ptr object,
                                                    const std::string target,
                                                    const std::string rwplay_path,
                                                    rw::kinematics::State state);


    void showProgres(double full, int prog);


    std::string WC_FILE = "../../WorkCell/Scene.wc.xml";


    //******************************************************************
    //*                       Private methods                          *
    //******************************************************************

private:
    /** Member vairables */



};
#endif // REACHABILITY_HPP