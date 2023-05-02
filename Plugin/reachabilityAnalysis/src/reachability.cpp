#include "reachability.hpp"


Reachability::Reachability(){}

void Reachability::saveConfirgurations(const std::vector<float> &x,
               const std::vector<float> &y,
               const std::vector<unsigned int> &solutions,
               const std::string &file_path) {
    // check if vectors have the same length
    if (x.size() != y.size() && y.size() != solutions.size()) { return; }

    // //write to file
    // //std::string file_path = "../../data_cylinder_top.txt";
    std::ofstream my_file;
    my_file.open(file_path);
    for (unsigned int i = 0; i < x.size(); i++) {
        std::string x_str = std::to_string(x[i]), y_str = std::to_string(y[i]), s_str = std::to_string(solutions[i]);
        std::string str = x_str + " " + y_str + " " + " " + s_str + "\n";
        my_file << str;
    }
    my_file.close();
}


std::vector<rw::math::Q> Reachability::getConfigurations(const std::string nameGoal,
                                           const std::string nameTcp,
                                           rw::models::SerialDevice::Ptr robot,
                                           rw::models::WorkCell::Ptr wc,
                                           rw::kinematics::State state) {
    const std::string robotName = robot->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    rw::kinematics::Frame* frameGoal = wc->findFrame(nameGoal);
    rw::kinematics::Frame* frameTcp = wc->findFrame(nameTcp);
    rw::kinematics::Frame* frameRobotBase = wc->findFrame(nameRobotBase);
    rw::kinematics::Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==NULL || frameTcp==NULL || frameRobotBase==NULL || frameRobotTcp==NULL)
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (frameGoal==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (frameTcp==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==NULL ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==NULL ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    rw::math::Transform3D<> frameBaseTGoal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state);
    rw::math::Transform3D<> frameTcpTRobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state);

    // get grasp frame in robot tool frame
    rw::math::Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr(new rw::invkin::ClosedFormIKSolverUR(robot, state));
    return closedFormSovler->solve(targetAt, state);
}


std::vector<rw::math::Q> Reachability::getCollisionFreeSolutions(rw::models::WorkCell::Ptr wc,
                                                   rw::models::SerialDevice::Ptr device,
                                                   rw::kinematics::MovableFrame::Ptr object,
                                                   const std::string target,
                                                   const std::string rwplay_path,
                                                   rw::kinematics::State state) {
    // create detector
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    std::vector<rw::math::Q> collisionFreeSolutions;

    // get collision free solutions for every degree around the roll axis
    for (double roll_angle = 0; roll_angle < 360.0; roll_angle += 1.0) {
        // rotate object
        rw::math::RPY<> rot(roll_angle*rw::math::Deg2Rad, 0, 0);
        rw::math::Vector3D<> pos = object->getTransform(state).P();

        /***************************************************************/
        /* Use this to move the object up, when grabbing from the top. */
        /***************************************************************/
    
        Frame* table_frame = wc->findFrame ("Table");
        rw::math::Transform3D<> trans(pos, rot);
        object->moveTo(trans, table_frame,state); //moveTo

        // object->setTransform(Transform3D<> (Vector3D<> (object->getTransform (state).P ()),
        //                                       RPY<> (0, 0, 0)),
        //                        state);

        /****************************************************************************/
        /* Use this to rotate 360 degree aound object, when grabbing from the side. */
        /****************************************************************************/

        //object->setTransform(Transform3D<> (pos, rot), state);

        // get configurations for the GraspTarget == GraspTCP
        std::vector<rw::math::Q> solutions = getConfigurations(target, "GraspTCP", device, wc, state);

        // check all the configurations for collisions
        for (unsigned int i = 0; i < solutions.size(); i++) {
            // set the robot in that configuration and check if it is in collision
            device->setQ(solutions[i], state);
            if (!detector->inCollision(state)) {//, NULL, true)) {
                collisionFreeSolutions.push_back(solutions[i]); // save it
                break; // we only need one
            }
        }
    }

    //visualize them if there are any
   if (collisionFreeSolutions.size() > 0) {
       TimedStatePath tStatePath;
       double time=0;
       for (unsigned int i = 0; i < collisionFreeSolutions.size(); i++) {
           device->setQ(collisionFreeSolutions[i], state);
           tStatePath.push_back(TimedState(time, state));
           time+=0.01;
       }
       // Store at rwplay file
       rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, rwplay_path);
       rw::loaders::PathLoader::storeTimedStatePath(*wc, tStatePath, "../../Experiments/stepsize_006/top/rwplays/complete/visu.rwplay");
  }

    return collisionFreeSolutions;
}

void Reachability::showProgres(double full, int prog){
    std::cout << std::setw(3) << prog;
    std::cout << "%  [";
    for(int k = 0; k < full; ++k){ 
        std::cout << "█";
    }
    std::cout << std::string(15-full, ' ');
    std::cout << "]" << "\n";
}