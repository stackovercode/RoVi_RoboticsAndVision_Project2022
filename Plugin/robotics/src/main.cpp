#include "robotics.hpp"

int main() {
    Robotics p;

    // load workcell;
    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(p.WC_FILE);
    if ( NULL == wc ){
        RW_THROW("COULD NOT LOAD WORKCELL ... check path!");
        return -1;
    }

    // Loading tool frame
    const std::string tool_name = "GraspTCP";
    rw::kinematics::Frame *toolFrame = wc->findFrame<rw::kinematics::Frame>(tool_name);
    if (NULL == toolFrame ){
        RW_THROW("COULD NOT FIND TOOL_FRAME ... check name!");
        return -1;
    }
    //Load Cylinder Frame 
    const std::string target_name = "Cylinder";
    rw::kinematics::MovableFrame *cylinderFrame = wc->findFrame<rw::kinematics::MovableFrame>(target_name);
    if (NULL == cylinderFrame ){
        RW_THROW("COULD NOT FIND TARGET_FRAME ... check name!");
        return -1;
    }

    // Loading target
    const std::string graspTarget_name = "GraspTarget";
    rw::kinematics::MovableFrame *targetFrame = wc->findFrame<rw::kinematics::MovableFrame>(graspTarget_name);
    if (NULL == targetFrame ){
        RW_THROW("COULD NOT FIND GRASPTARGET_FRAME ... check name!");
        return -1;
    }

    // find device
    const std::string device_name = "UR-6-85-5-A";
    rw::models::SerialDevice::Ptr robot_ur5 = wc->findDevice<rw::models::SerialDevice>(device_name);
    if (NULL == robot_ur5) {
        RW_THROW("COULD NOT FIND DEVICE_FRAME ... check name!");
        return -1;
    }

    // find ur5 robot base frame
    const std::string base_name = "URReference";
    rw::kinematics::MovableFrame::Ptr base_frame = wc->findFrame<rw::kinematics::MovableFrame>(base_name);
    if (NULL == base_frame) {
        RW_THROW("Could not load " + base_name + " ... check model");
        return -1;
    }

    // Get state
    rw::kinematics::State state = wc->getDefaultState();
    rw::math::Vector3D<> placePos(0.30, -0.50, 0.15);

    // Create the helper transformations
    rw::kinematics::Frame *table_frame = wc->findFrame ("Table");

        // PICK 1
    std::vector<rw::math::Vector3D<>> cylinder_positions = {rw::math::Vector3D<>(-0.25, 0.45, 0.15)};
        // PICK 2
    cylinder_positions.push_back(rw::math::Vector3D<>(-0.0, 0.45, 0.15));
        // PICK 3
    cylinder_positions.push_back(rw::math::Vector3D<>(0.25, 0.45, 0.15));

    rw::math::Rotation3D<> cylinder_rot = cylinderFrame->getTransform(state).R();
    rw::math::Transform3D<> cylinder_trans(cylinder_positions[0], cylinder_rot);    

    rw::math::Vector3D<> nearPlacePos(-0.16, -0.4775, 0.55);    
    rw::math::Transform3D<> nearPlaceFrame(nearPlacePos, cylinder_rot);

    rw::math::Vector3D<> nearPickPos(-0.40, 0.20, 0.55);  
    rw::math::Transform3D<> nearPickFrame(nearPickPos, cylinder_rot);

    // Add the place position to interpolation point bag
    rw::math::Transform3D<> placeFrame(placePos, cylinder_rot);

    // Move Base  
    rw::math::Rotation3D<> base_rot = base_frame->getTransform(state).R();
    rw::math::Vector3D<> base_positions(0.14, 0.06, 0.0);
    rw::math::Transform3D<> base_trans(base_positions, base_rot);
    base_frame->moveTo(base_trans, state);

    // Move to pick area
    cylinderFrame->moveTo(cylinder_trans, table_frame, state);
    
    std::vector<rw::math::Transform3D<>> Points;
    std::vector<float> Times;
    rw::math::Transform3D<> Point0 = robot_ur5->baseTframe(robot_ur5->getEnd(),state)*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,0.1));
    Points.push_back(Point0);
    rw::math::Transform3D<> Point1 = nearPickFrame;
    Points.push_back(Point1);
    rw::math::Transform3D<> Point2 = cylinderFrame->getTransform(state)*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.25));
    Points.push_back(Point2);
    rw::math::Transform3D<> Point3 = cylinderFrame->getTransform(state);
    Points.push_back(Point3);
    rw::math::Transform3D<> Point4 = cylinderFrame->getTransform(state)*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.25));
    Points.push_back(Point4);
    rw::math::Transform3D<> Point5 = nearPickFrame;
    Points.push_back(Point5);
    rw::math::Transform3D<> Point6 = nearPlaceFrame;
    Points.push_back(Point6);
    rw::math::Transform3D<> Point7 = placeFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.25));
    Points.push_back(Point7);
    rw::math::Transform3D<> Point8 = placeFrame;
    Points.push_back(Point8);
    rw::math::Transform3D<> Point9 = placeFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.25));
    Points.push_back(Point9);
    rw::math::Transform3D<> Point10 = nearPlaceFrame;
    Points.push_back(Point10);
    rw::math::Transform3D<> Point11 = robot_ur5->baseTframe(robot_ur5->getEnd(),state)*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,0.1));
    Points.push_back(Point11);
  
    for (float i = 0; i <= Points.size(); i++){
        Times.push_back(i);
    }

    rw::proximity::CollisionDetector detector (wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());

    rw::math::Transform3D<> worldTranformtion = rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.1));

    std::vector<rw::math::Q> configurationPath;
    std::vector<rw::math::Transform3D<>> tf = Points;
    for ( unsigned int i = 0; i < tf.size(); i++) {
        tf[i] = worldTranformtion*tf[i];
    }
    for ( rw::math::Transform3D<> Tf : tf ){
        targetFrame->moveTo(Tf, state);
        std::vector<rw::math::Q> solutions = p.findConfigurations("GraspTarget", "GraspTCP", robot_ur5, wc, state);
        rw::math::Q configuration = p.findCollisionFreeSolution(robot_ur5, state, detector, solutions);
        configurationPath.push_back(configuration);
    }

    std::pair<std::vector<rw::math::Q>,std::vector<rw::math::Transform3D<>>> paths = p.CollisionFreeP2P(tf, Times, targetFrame, robot_ur5, wc, state, detector);

    //**************************************************************
    //*      Test, visualization and data Collection methods       *
    //**************************************************************

    /*START*/               /*Comment out to test*/
    /**************************************************************/
    std::vector<rw::math::Q> joints = paths.first;
    std::vector<rw::math::Transform3D<>> cartesian = paths.second;
    std::ofstream tfConfigFile, qConfigFile, liTimeFile;
    std::vector<int> liTime;
    std::vector<rw::math::Transform3D<>> path2Run;
    // // PointMap Data collection
    p.createData(joints, "../../JointsData.txt", cartesian,"../../TFData.txt",Times,"../../liTimeData.txt");
    // // Video data collection
    p.createRwplay(joints,robot_ur5, wc, state,"../../visu.rwplay");
    for (unsigned int i = 0; i < 60; i++){
        auto start = std::chrono::high_resolution_clock::now();
        path2Run = p.runLiTime(tf, Times);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
        int dur_ms = duration.count();
        liTime.push_back(dur_ms);
    }
    liTimeFile.open("../../liTimeData.txt");
    for (int Tms : liTime ){
        liTimeFile << Tms << std::endl;
    }
    std::vector<float> T;
    for (double i = 0; i < Points.size(); i++){
        T.push_back(i);
    }
    p.performTest(paths.first, T, "../../testAccuracy.txt", targetFrame, robot_ur5, wc, state, detector);
     /**************************************************************/
    /*STOP*/


    std::cout << "Done! " << std::endl;

	return 0;
}