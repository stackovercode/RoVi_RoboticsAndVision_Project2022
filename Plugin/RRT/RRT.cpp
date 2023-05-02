#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include <chrono>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 900.
/*
RRT code based on Lesson 6 and 7 in robotics.
*/
bool checkCollisions (Device::Ptr device, const State& state, const CollisionDetector& detector,
                      const Q& q)
{
    State testState;
    CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ (q, testState);
    colFrom = detector.inCollision (testState, &data);
    if (colFrom) {
        cerr << "Configuration in collision: " << q << endl;
        cerr << "Colliding frames: " << endl;
        FramePairSet fps = data.collidingFrames;
        for (FramePairSet::iterator it = fps.begin (); it != fps.end (); it++) {
            cerr << (*it).first->getName () << " " << (*it).second->getName () << endl;
        }
        return false;
    }
    return true;
}
/*
struct PathChecker
{
    WorkCell::Ptr wc;
    Device::Ptr dev;
    CollisionDetector::Ptr col;

    bool inCollision (Q from, Q to)
    {
        State state = wc->getDefaultState ();

        LinearInterpolator< Q > p (from, to, (from - to).norm2 ());
        for (double step = STEPSIZE; step < p.duration (); step += STEPSIZE) {
            dev->setQ (p.x (step), state);
            if (col->inCollision (state)) {
                return true;
            }
        }
        return false;
    }

    double pathLength (TimedStatePath p)
    {
        double d = 0;
        Q last   = dev->getQ (p.front ().getValue ());
        for (TimedState ts : p) {
            Q current = dev->getQ (ts.getValue ());
            d += (current - last).norm2 ();
            last = current;
        }
        return d;
    }
};
*/
int main ()
{

    /*
    This code is inspired my lesson 6 in Robotics and lesson 7, but pruning did not help with the RRT path planning
    */
    std::ofstream fileTime("stepSize_vs_generationTime.txt");
    std::ofstream fileSize("stepSize_vs_pathSize.txt");

    for (double step = 0.1; step < 5.0; step=step+0.1)
    {
    auto start = std::chrono::high_resolution_clock::now();
    std::string stepSizeName = "stepSize_"+std::to_string(step);
    const string wcFile     = "/home/rovi2022/projects/RoVi_Project/WorkCellObstacle/Scene.wc.xml";
    const string deviceName = "UR-6-85-5-A";
    //std::ofstream fileTime(stepSizeName+"_tim.txt");
    //std::ofstream fileXYZ(stepSizeName+"_XYZ.txt");
    //std::ofstream fileROT(stepSizeName+"_rot.txt");
    /*
    if (!fileTime.is_open())
    {
        std::cerr << "Error opening time file" << std::endl;
        return 1;
    } else if (!fileXYZ.is_open())
    {
        std::cerr << "Error opening XYZ file" << std::endl;
    }
    */
    WorkCell::Ptr wc    = WorkCellLoader::Factory::load (wcFile);
    Frame* tool_frame   = wc->findFrame ("Tool");

    Device::Ptr device = wc->findDevice (deviceName);
    if (device == NULL) {
        cerr << "Device: " << deviceName << " not found!" << endl;
        return 0;
    }

    State state = wc->getDefaultState ();
    //Q from (-0.959931, -0.959931, 1.0472, -1.57237, -1.57237, 0); //Top Position
    Q from{-2.00713, -1.65806, 2.00713, -1.84883, -1.48353, 0}; //Low Position
    //Q from (-1.48353, -1.5708, 1.5708, -1.5708, -1.5708, 0); //Middle
    Q to (-1.13446, -1.5708, -1.8326, -1.309, 1.65806, 0); //End point
    rw::math::Math::seed(); //Set this to get the same results
    device->setQ (from, state);
    //Kinematics::gripFrame (bottle_frame, tool_frame, state);
    CollisionDetector detector (wc, ProximityStrategyFactory::makeDefaultCollisionStrategy ());
    PlannerConstraint constraint = PlannerConstraint::make (&detector, device, state);

    QSampler::Ptr sampler    = QSampler::makeConstrained (QSampler::makeUniform (device),
                                                               constraint.getQConstraintPtr ());
    QMetric::Ptr metric      = MetricFactory::makeEuclidean< Q > ();

    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner (constraint, sampler, metric, step, RRTPlanner::RRTConnect);
    if (!checkCollisions (device, state, detector, from))
        return 0;
    if (!checkCollisions (device, state, detector, to))
        return 0;
    QPath path;
    Timer t;
    t.resetAndResume ();
    planner->query (from, to, path, MAXTIME);
    t.pause ();
    if (t.getTime () >= MAXTIME) {
        cout << "\nNotice: max time of " << MAXTIME << " seconds reached." << endl;
        }
    TimedStatePath tStatePath;
    if (path.size () >= 2) {
        double distance = 0;
        Q last (path.front ());
        for (Q q : path) {
            distance += (q-last).norm2();
            }

            double time = 0.0;
            for (size_t i = 0; i < path.size (); i += 1) {
                device->setQ (path.at (i), state);
                tStatePath.push_back (TimedState (time, state));
                //auto currentTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count();
                time += 0.01;
                /*
                rw::math::Transform3D<> baseToTcp = device->baseTend(state);
                rw::math::Vector3D<> position = baseToTcp.P();
                rw::math::Rotation3D<> rotation = baseToTcp.R();
                //std::cout << "Position --> " << pos << std::endl;
                //std::cout << "Rotation --> " << rot << std::endl;
                //std::cout << "x -->" << pos[0] << std::endl;
                fileTime << currentTime << std::endl;
                fileXYZ << position[0] <<"," << position[1] << ","  << position[2] << std::endl;
                */
                }
            }

    rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, stepSizeName+"_path.rwplay");
    
    //This part is not used as it was tested if the pruning would help to speed tihngs op
    /*
    PathChecker check = {wc, device, rw::core::ownedPtr (new CollisionDetector (wc, CollisionStrategy::Factory::makeStrategy ("PQP")))};
    std::cout << "PathLength: " << check.pathLength (tStatePath) << std::endl;
    int i = 0;
    while (i < (int) path.size () - 2) {
        Q from = device->getQ (tStatePath[i].getValue ());
        Q to   = device->getQ (tStatePath[i + 2].getValue ());

        if (check.inCollision (from, to)) {
            i++;
        }
        else {
            path.erase (path.begin () + i + 1);
            if (i > 0) {
                i--;
        }
    }
    }
    std::cout << "PathLength Pruned: " << check.pathLength (tStatePath) << std::endl;
    rw::loaders::PathLoader::storeTimedStatePath (*wc, tStatePath, stepSizeName+"_PrunedPath.rwplay");
    */
   /*
    fileTime.close();
    fileXYZ.close();
    */
    auto endTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start).count();
    fileTime << step << " " << endTime << std::endl;
    fileSize << step << " " << tStatePath.size() << std::endl;
    }
    fileTime.close();
    fileSize.close();
    std::cout << "Done" << std::endl;
    return 0;
}
