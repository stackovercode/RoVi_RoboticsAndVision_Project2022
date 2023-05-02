#include "robotics.hpp"

Robotics::Robotics(){}

// Time for interpolation motion
double Robotics::constantVelocity(double t, double t0, double t1) {
    return (t-t0)/(t1-t0);
}

// Derived from Kaspers solution from Robotics Exercise 4
    rw::trajectory::InterpolatorTrajectory<rw::math::Q> Robotics::linInterp(std::vector<rw::math::Q> q, 
                                                                            double duration, 
                                                                            rw::trajectory::TimedStatePath& timestatepath, 
                                                                            rw::models::SerialDevice::Ptr robot_ur5, 
                                                                            rw::kinematics::State state){
    rw::trajectory::TimedStatePath motion;
                                                                                
    timestatepath.clear();
    rw::trajectory::InterpolatorTrajectory<rw::math::Q> interplin;

    for (size_t i = 0; i < q.size(); i++){
        if (i+1 <= q.size()){
             rw::trajectory::LinearInterpolator<rw::math::Q> interp(q[i], q[i+1], duration);
              interplin.add(interp);
        }
    }

    for (double i = 0; i < interplin.duration(); i++){
       robot_ur5->setQ(interplin.x(i),state);
       motion.push_back(TimedState(i,state));
    }
    timestatepath = motion;
    return interplin;
}


// Derived from kasper solutions on exercise 3
std::vector<rw::math::Q> Robotics::findConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot_ur5, rw::models::WorkCell::Ptr wc, rw::kinematics::State state){
    // Get, make and print name of frames
    const std::string robotName = robot_ur5->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    rw::kinematics::Frame* frameGoal = wc->findFrame(nameGoal);
    rw::kinematics::Frame* frameTcp = wc->findFrame(nameTcp);
    rw::kinematics::Frame* frameRobotBase = wc->findFrame(nameRobotBase);
    rw::kinematics::Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==nullptr || frameTcp==nullptr || frameRobotBase==nullptr || frameRobotTcp==nullptr)
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (frameGoal==nullptr ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (frameTcp==nullptr ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==nullptr ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==nullptr ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    rw::math::Transform3D<> frameBaseTGoal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state);
    rw::math::Transform3D<> frameTcpTRobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state);

    // get grasp frame in robot tool frame
    rw::math::Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(robot_ur5, state) );
    return closedFormSovler->solve(targetAt, state);
}

// Derived from kasper solutions on exercise 3 & 4
rw::math::Q Robotics::findCollisionFreeSolution(rw::models::SerialDevice::Ptr robot_ur5, rw::kinematics::State state, rw::proximity::CollisionDetector& detector, std::vector<rw::math::Q> solutions){
    rw::math::Q configuration;
    for ( unsigned int i = 0; i < solutions.size(); i++ ){
        robot_ur5->setQ(solutions[i], state);
        if ( !detector.inCollision(state, NULL, true) )
        {
            configuration = solutions[i];
            break;
        }
    }
    return configuration;
}

// Derived from kasper solutions on exercise 3 & 4
rw::math::Q Robotics::findCollisionFreeSolution(rw::models::SerialDevice::Ptr robot_ur5, rw::kinematics::State state, rw::proximity::CollisionDetector& detector, std::vector<rw::math::Q> solutions, rw::math::Q prevSolution){
    rw::math::Q configuration;
    double dist = std::numeric_limits<double>::max();

    for ( unsigned int i = 0; i < solutions.size(); i++ )
    {
        robot_ur5->setQ(solutions[i], state);
        if ( !detector.inCollision(state, NULL, true) )
        {
            double cdist = rw::math::Q(prevSolution - solutions[i]).norm2();
            if ( cdist < dist )
            {
                configuration = solutions[i];
                dist = cdist;
            }
        }
    }
    return configuration;
}

// Create helper transformation
rw::math::Transform3D<double> Robotics::convertToTransform3D(const Eigen::Vector3d translation, const Eigen::Quaterniond rotation){
  Eigen::Isometry3d iso = Eigen::Translation3d(translation) *rotation;
  return rw::math::Transform3D<double>(iso.matrix());
}


/*Linear interpolation derived from Robotics lecture 4 slides 12-15*/
/*Method from Robotics lecture notes and reconstructed for using quaternions*/
std::pair<std::vector<rw::math::Q>,std::vector<rw::math::Transform3D<>>> Robotics::CollisionFreeP2P(std::vector<rw::math::Transform3D<>> P, std::vector<float> T, rw::kinematics::MovableFrame* targetFrame, rw::models::SerialDevice::Ptr robot_ur5, rw::models::WorkCell::Ptr wc, rw::kinematics::State state, rw::proximity::CollisionDetector& detector){ 
    std::vector<rw::math::Q> path;
    std::vector<rw::math::Transform3D<>> point;
    for ( unsigned int i = 1; i < P.size(); i++ ){
        for ( float t = T[i-1]; t < T[i]; t += 0.01f ){

            // This is the code for calucalate using quaternions.
            Eigen::Vector3d Pi = P[i-1].P() + (P[i].P() - P[i-1].P())*constantVelocity(t,T[i-1],T[i]);
            Eigen::Quaterniond q1(P[i-1].R().e()), q2(P[i].R().e());            
            Eigen::Quaterniond q = q1.slerp(constantVelocity(t,T[i-1],T[i]), q2);
            point.push_back(convertToTransform3D(Pi,q));

            targetFrame->moveTo(point.back(), state);

            std::vector<rw::math::Q> solutions = Robotics::findConfigurations("GraspTarget", "GraspTCP", robot_ur5, wc, state);

            rw::math::Q configuration;
            if ( path.size() > 0 )
                configuration = Robotics::findCollisionFreeSolution(robot_ur5, state, detector, solutions, path.back());
            else
                configuration = Robotics::findCollisionFreeSolution(robot_ur5, state, detector, solutions);

            path.push_back(configuration);
        }
    }
    return std::make_pair(path, point);
}

// Create helper transformation
std::pair<std::pair<Eigen::Vector3d, Eigen::Matrix3d>,Eigen::Matrix4d> Robotics::trans2Eigen(rw::math::Transform3D<> P){
    Eigen::Matrix3d R = P.R().e();
    Eigen::Vector3d T = P.P();
    Eigen::Matrix4d Trans; 
    Trans.setIdentity(); 
    Trans.block<3,3>(0,0) = R;
    Trans.block<3,1>(0,3) = T;
    return std::make_pair(std::make_pair(T,R),Trans);
}

// Create rwplay
void Robotics::createRwplay(std::vector<rw::math::Q> jointPath, rw::models::SerialDevice::Ptr robot_ur5, rw::models::WorkCell::Ptr wc, rw::kinematics::State state, const std::string rwplayPath){
    rw::trajectory::TimedStatePath statePath;
    double time = 0;
    double dur = 0.1;
    for ( rw::math::Q configuration : jointPath ) {
        robot_ur5->setQ(configuration, state);
        statePath.push_back(rw::trajectory::TimedState(time, state));
        time += dur/double(1);
    }
    rw::loaders::PathLoader::storeTimedStatePath(*wc, statePath, rwplayPath);
}

// Create my data output
void Robotics::createData(std::vector<rw::math::Q> joints, const std::string jointPath, std::vector<rw::math::Transform3D<>> cartesian, const std::string cartesianPath, std::vector<float> Time, const std::string timePath){
    std::ofstream tfConfigFile, qConfigFile, liTimeFile;
    qConfigFile.open(jointPath);
    for ( rw::math::Q jointQ : joints ){
        qConfigFile << jointQ[0] << " " << jointQ[1] << " " << jointQ[2] << " " << jointQ[3] << " " << jointQ[4] << " " << jointQ[5] << std::endl;
    }
    qConfigFile.close();

    tfConfigFile.open(cartesianPath);
    for ( rw::math::Transform3D<> tf : cartesian ){
        tfConfigFile << tf.R().getRow(0)[0] << " " << tf.R().getRow(0)[1] << " " << tf.R().getRow(0)[2] << " " << tf.P()[0] << std::endl;
        tfConfigFile << tf.R().getRow(1)[0] << " " << tf.R().getRow(1)[1] << " " << tf.R().getRow(1)[2] << " " << tf.P()[1] << std::endl;
        tfConfigFile << tf.R().getRow(2)[0] << " " << tf.R().getRow(2)[1] << " " << tf.R().getRow(2)[2] << " " << tf.P()[2] << std::endl;
        tfConfigFile <<          0          << " " <<          0          << " " <<          0          << " " <<      1    << std::endl;
    }
    tfConfigFile.close();
}

std::vector<rw::math::Transform3D<>> Robotics::runLiTime(std::vector<rw::math::Transform3D<>> cartesian, std::vector<float> Time){
    std::vector<rw::math::Transform3D<>> path;
    std::vector<Eigen::Vector3d> Pi;
    std::vector<Eigen::Quaterniond> q;
    for (unsigned int i = 1; i < cartesian.size(); i++){
        for (float t = Time[i-1]; t < Time[i]; t += 0.01f){
            Pi.push_back(cartesian[i-1].P() + (cartesian[i].P() - cartesian[i-1].P())*constantVelocity(t,Time[i-1],Time[i]));
            Eigen::Quaterniond q1(cartesian[i-1].R().e()), q2(cartesian[i].R().e());            
            q.push_back(q1.slerp(constantVelocity(t,Time[i-1],Time[i]), q2));
        }
    }

    for (unsigned int i = 0; i < Pi.size(); i++){
        path.push_back(convertToTransform3D(Pi[i], q[i]));
    }
    return path;
}



// Performing the test for accrucy. 
// void Robotics::performTest( std::vector<rw::math::Q> q,
//                                     std::vector<float> times, 
//                                     std::string f_name,
//                                     rw::kinematics::MovableFrame *targetFrame, 
//                                     rw::models::SerialDevice::Ptr robot_ur5, 
//                                     rw::models::WorkCell::Ptr wc, 
//                                     rw::kinematics::State state, 
//                                     rw::proximity::CollisionDetector& detector){
//     std::ofstream testfile;
//     testfile.open(f_name);
//     std::map<int, rw::math::Q> path;
//     float time_step = 0.01;
//     float time = 0;
//     float t = 0;
//     int time_index = 0;
//     std::pair<int, rw::math::Q> interpolation;

//     rw::common::Timer _t;   
//     for ( int k = 0; k < 50; k++) {

        
//         _t.resetAndResume();
//         for (size_t j = 0; j < times.size(); j++) {
//             int interval = int(std::round((time+times[j])-time_step));

//             for (int i = 0; i <= interval/time_step; i++) {
//                 interpolation.first = time_index;
//                 interpolation.second = q[j]+constantVelocity(t, time, time+times[j])*(q[j+1]-q[j]);
//                 t += time_step;
//                 time_index++;
//             }
//             time_index--;
//             time += times[j];
//             t = time;
//         }
    
//         path.insert(std::pair<int, rw::math::Q>(interpolation.first, interpolation.second));
//         _t.pause();
//         testfile << _t.getTime() << "," << std::endl;
//         std::cout << "Planning time: " << _t.getTime() << std::endl;

//         rw::math::EuclideanMetric<rw::math::Q> metric2;
//         float distance = 0;

//         for (size_t i = 0; i < path.size()-1; i++) {
//             distance += metric2.distance(path.find(int(i+1))->second, path.find(int(i))->second);
//             std::cout << "-----------------------------" << std::endl;
//             std::cout << path.find(int(i+1))->second << std::endl;
//             std::cout << path.find(int(i))->second << std::endl;
//             std::cout << "-----------------------------" << std::endl;
//         }
//         testfile << distance << std::endl;
//         std::cout << "Distance: " << distance << std::endl;
//     }
//     testfile.close();
// }