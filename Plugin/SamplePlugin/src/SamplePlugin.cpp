#include "SamplePlugin.hpp"

#include <filesystem>

typedef rw::kinematics::MovableFrame MovFrame;
typedef rw::math::Vector3D<> Vekt;
typedef rw::math::RPY<> Rpy;
typedef rw::math::Transform3D<> Pose;
typedef rw::kinematics::Frame Frame;
typedef rw::math::Rotation3D<> Rotm;

SamplePlugin::SamplePlugin () : RobWorkStudioPlugin ("SamplePluginUI", QIcon ((std::filesystem::path(__FILE__).parent_path()/"pa_icon.png").c_str())){
    setupUi (this);
    


    _timer = new QTimer (this);
    connect (_timer, SIGNAL (timeout ()), this, SLOT (timer ()));

    // now connect stuff from the ui component
    // connect(_btnStart    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	// connect(_btnStop, SIGNAL(pressed()), this, SLOT(btnPressed()) );
    // connect(_btnRestart    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
	// connect(_btn_move    ,SIGNAL(pressed()), this, SLOT(btnPressed()) );
    connect(_btn_move_default, SIGNAL(pressed()),this, SLOT(btnPressed()) );
    connect(_btn_home, SIGNAL(pressed()),this, SLOT(btnPressed()) );
    connect(_btn_Pose, SIGNAL(pressed()),this, SLOT(btnPressed()) );
    

    connect (_btn_im, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn_scan, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn0, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn1, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn2, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn3, SIGNAL (pressed ()), this, SLOT (btnPressed ()));
    connect (_btn4, SIGNAL(pressed()),this, SLOT(btnPressed()) );
    connect (_spinBox, SIGNAL (valueChanged (int)), this, SLOT (btnPressed ()));

    // Image textureImage(300,300,Image::GRAY,Image::Depth8U);
	// _textureRender = new RenderImage(textureImage);
	// Image bgImage(0,0,Image::GRAY,Image::Depth8U);
	// _bgRender = new RenderImage(bgImage,2.5/1000.0);

	_framegrabber = NULL;


    _cameras    = {"Camera_Right", "Camera_Left"};
    _cameras25D = {"Scanner25D"};
}

SamplePlugin::~SamplePlugin ()
{
    delete _textureRender;
    delete _bgRender;
}


void SamplePlugin::initialize ()
{
    log ().info () << "INITALIZE"
                   << "\n";

    getRobWorkStudio ()->stateChangedEvent ().add (
        std::bind (&SamplePlugin::stateChangedListener, this, std::placeholders::_1), this);

    // Auto load workcell

    std::filesystem::path wc_path (__FILE__);
    wc_path          = wc_path.parent_path() / "../../WorkCell/Scene.wc.xml";
	std::cout << "wc path: " << wc_path << std::endl;
    WorkCell::Ptr wc = WorkCellLoader::Factory::load (wc_path.string ());
    getRobWorkStudio ()->setWorkCell (wc);
}

void SamplePlugin::open (WorkCell* workcell)
{
    log ().info () << "OPEN"
                   << "\n";
    _wc    = workcell;
    _state = _wc->getDefaultState ();

    log ().info () << workcell->getFilename () << "\n";

    if (_wc != NULL) {
        // Add the texture render to this workcell if there is a frame for texture
        Frame* textureFrame = _wc->findFrame ("MarkerTexture");
        if (textureFrame != NULL) {
            getRobWorkStudio ()->getWorkCellScene ()->addRender (
                "TextureImage", _textureRender, textureFrame);
        }
        // Add the background render to this workcell if there is a frame for texture
        Frame* bgFrame = _wc->findFrame ("Background");
        if (bgFrame != NULL) {
            getRobWorkStudio ()->getWorkCellScene ()->addRender (
                "BackgroundImage", _bgRender, bgFrame);
        }

        // Create a GLFrameGrabber if there is a camera frame with a Camera property set
        Frame* cameraFrame = _wc->findFrame (_cameras[0]);
        if (cameraFrame != NULL) {
            if (cameraFrame->getPropertyMap ().has ("Camera")) {
                // Read the dimensions and field of view
                double fovy;
                int width, height;
                std::string camParam = cameraFrame->getPropertyMap ().get< std::string > ("Camera");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber             = new GLFrameGrabber (width, height, fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio ()->getView ()->getSceneViewer ();
                _framegrabber->init (gldrawer);
            }
        }

        Frame* cameraFrame25D = _wc->findFrame (_cameras25D[0]);
        if (cameraFrame25D != NULL) {
            if (cameraFrame25D->getPropertyMap ().has ("Scanner25D")) {
                // Read the dimensions and field of view
                double fovy;
                int width, height;
                std::string camParam =
                    cameraFrame25D->getPropertyMap ().get< std::string > ("Scanner25D");
                std::istringstream iss (camParam, std::istringstream::in);
                iss >> fovy >> width >> height;
                // Create a frame grabber
                _framegrabber25D          = new GLFrameGrabber25D (width, height, fovy);
                SceneViewer::Ptr gldrawer = getRobWorkStudio ()->getView ()->getSceneViewer ();
                _framegrabber25D->init (gldrawer);
            }
        }

    _ball  = _wc->findFrame<rw::kinematics::MovableFrame>("Ball");
    if ( _ball == nullptr )
        RW_THROW("Ball frame not found.");

    // _cylinder  = _wc->findFrame<rw::kinematics::MovableFrame>("Cylinder");
    // if ( _cylinder == nullptr )
    // RW_THROW("Ball frame not found.");

    // _dino  = _wc->findFrame<rw::kinematics::MovableFrame>("Trex");
    // if ( _dino == nullptr )
    // RW_THROW("Ball frame not found.");

    // _square  = _wc->findFrame<rw::kinematics::MovableFrame>("Square");
    // if ( _square == nullptr )
    // RW_THROW("Ball frame not found.");

    // _bottle  = _wc->findFrame<rw::kinematics::MovableFrame>("Bottle");
    // if ( _bottle == nullptr )
    //     RW_THROW("Ball frame not found.");
    
    
    _table = _wc->findFrame<rw::kinematics::Frame>("Table");
    if ( _table == nullptr )
        RW_THROW("Table frame not found.");
    
    _tcp   = _wc->findFrame<rw::kinematics::Frame>("UR-6-85-5-A.TCP");
    if ( _tcp == nullptr )
        RW_THROW("TCP frame not found.");

    _UR5 = _wc->findDevice<rw::models::SerialDevice>("UR-6-85-5-A");
    if ( _UR5 == nullptr )
        RW_THROW("Device UR6 not found.");

    _target = _wc->findFrame<rw::kinematics::MovableFrame>("GraspTarget");
    if ( _target == nullptr )
        RW_THROW("Error finding frame: GraspTarget");

    _base = _wc->findFrame<rw::kinematics::MovableFrame>("URReference");
    if (NULL == _base)
        RW_THROW("Could not load URReference ... check model");


    _device = _wc->findDevice ("UR-6-85-5-A");
    _step   = -1;

    _home = _UR5->getQ(_state);
    _ballPos = _ball->getTransform(_state);
    _ballHomePos = _ballPos;

    }
}

void SamplePlugin::close ()
{
    log ().info () << "CLOSE"
                   << "\n";

    // Stop the timer
    _timer->stop ();
    // Remove the texture render
    Frame* textureFrame = _wc->findFrame ("MarkerTexture");
    if (textureFrame != NULL) {
        getRobWorkStudio ()->getWorkCellScene ()->removeDrawable ("TextureImage", textureFrame);
    }
    // Remove the background render
    Frame* bgFrame = _wc->findFrame ("Background");
    if (bgFrame != NULL) {
        getRobWorkStudio ()->getWorkCellScene ()->removeDrawable ("BackgroundImage", bgFrame);
    }
    // Delete the old framegrabber
    if (_framegrabber != NULL) {
        delete _framegrabber;
    }
    _framegrabber = NULL;
    _wc           = NULL;
}

Mat SamplePlugin::toOpenCVImage (const Image& img)
{
    Mat res (img.getHeight (), img.getWidth (), CV_8SC3);
    res.data = (uchar*) img.getImageData ();
    return res;
}

void SamplePlugin::btnPressed ()
{
    QObject* obj = sender ();
    if (obj == _btn0) {
        //		log().info() << "Button 0\n";
        //		// Toggle the timer on and off
        //		if (!_timer25D->isActive())
        //		    _timer25D->start(100); // run 10 Hz
        //		else
        //			_timer25D->stop();
        _timer->stop ();
        rw::math::Math::seed ();
        double extend  = 0.05;
        double maxTime = 60;
        Q from (6, 1.571, -1.572, -1.572, -1.572, 1.571, 0);
        Q to (6, 1.847, -2.465, -1.602, -0.647, 1.571, 0);    // From pose estimation
        createPathRRTConnect (from, to, extend, maxTime);
    } else if (obj == _btn1) {
        resetRobotAndObject();
        log ().info () << "Button 1\n";
        // Toggle the timer on and off
        if (!_timer->isActive ()) {
            _timer->start (100);    // run 10 Hz
            _step = 0;
        }
        else
            _step = 0;
    } else if (obj == _spinBox) {
        log ().info () << "spin value:" << _spinBox->value () << "\n";
    } else if (obj == _btn2) {
        //log ().info () << "spin value:" << _spinBox->value () << "\n";
    }else if (obj == _btn3) {
        _timer->stop ();
        rw::math::Math::seed ();
        double extend  = 0.05;
        double maxTime = 60;
        Q from (6, 1.571, -1.572, -1.572, -1.572, 1.571, 0);
        //Q to (1.016, -1.264, -1.158, -2.444, 2.102, -0.779); 
        // 1. Near pick joint position
        Q to (2.848, -1.203, -1.558, -1.563, 1.571, 0); 
        // 2. Near place joint position
        //Q to (-1.67447, -1.59122, -1.42628, -1.56299, 1.57101, 1.55038); 
        //Q to (-0.212, -1.830, -1.063, -2.02, 0.969, 0.591);
        createPathRRTConnect (from, to, extend, maxTime);
    }

    /* Q[6]{2.848, -1.203, -1.558, -1.563, 1.571, 0}
    * 
    * 
    *
    * 
    */

    else if (obj == _btn4) {
        _timer->stop ();
        rw::math::Math::seed ();
        double extend  = 0.05;
        double maxTime = 60;
        int time = 10;
        Q from (6, 1.571, -1.572, -1.572, -1.572, 1.571, 0);
        //Q to (1.016, -1.264, -1.158, -2.444, 2.102, -0.779); 
        // 1. Near pick joint position
        Q to (2.848, -1.203, -1.558, -1.563, 1.571, 0); 
        // 2. Near place joint position
        //Q to (-1.67447, -1.59122, -1.42628, -1.56299, 1.57101, 1.55038); 
        //Q to (-0.212, -1.830, -1.063, -2.02, 0.969, 0.591);
        interpolation(from, to, time);
    }
    else if (obj == _btn_im) {
        //moveCylinder(1);
        //getImage ();
       getCollectImageTest();
    } else if (obj == _btn_scan) {
        get25DImage ();
    }  else if (obj == _btn_Pose) {
        poseEstimation ();
    } else if (obj == _btn_home) {
        _state = _wc->getDefaultState();
        getRobWorkStudio()->setState(_state);
    } else if (obj == _btn_move_default) {
        resetRobotAndObject();
        cv::Mat pose;
        pose = poseEstimation ();
        std::cout << pose << std::endl;
        P2P(pose);

        if (!_timer->isActive()){
        _timer->start(10); // run 100 Hz
        _step = 0;
        }
        else
            _step = 0;
    }
}

double SamplePlugin::constant_vel(double t, double t0, double t1) {
    return (t-t0)/(t1-t0);
}



void SamplePlugin::P2P(cv::Mat pose){
    //Robotics p;


//     // Convert the cv::Mat to a rw::math::Vector3D
//     rw::math::Vector3D<double> Pose (pose.at<double>(0), pose.at<double>(1), pose.at<double>(2));
//    // std::cout << "Pose: " << vec << std::endl;


     rw::math::Vector3D<> placePos(0.30, -0.50, 0.155);

//     // Create the helper transformations
//     // rw::kinematics::Frame *table_frame = _wc->findFrame ("Table");
//     //std::vector<rw::math::Vector3D<>> cylinder_positions = {rw::math::Vector3D<>(-0.15, 0.45, 0.155)};
//     std::vector<rw::math::Vector3D<>> cylinder_positions = {Pose};


//     rw::math::Rotation3D<> cylinder_rot = _target->getTransform(_state).R();
//     rw::math::Transform3D<> cylinder_trans(cylinder_positions[0], cylinder_rot);    
    
//     rw::math::Vector3D<> nearPlacePos(-0.16, -0.4775, 0.5625);
//     rw::math::Transform3D<> nearPlaceFrame(nearPlacePos, cylinder_rot);

//     rw::math::Vector3D<> nearPickPos(-0.255, 0.19, 0.655);
//     rw::math::Transform3D<> nearPickFrame(nearPickPos, cylinder_rot);

    // // Add the place position to interpolation point bag
    // rw::math::Transform3D<> placeFrame(placePos, cylinder_rot);

//     // Move to pick area
//     _ball->moveTo(cylinder_trans, _table, _state);

    rw::kinematics::Frame *table_frame = _wc->findFrame ("Table");

            // PICK 1
    std::vector<rw::math::Vector3D<>> ball_positions = {rw::math::Vector3D<>(-0.25, 0.45, 0.155)};
        // PICK 2
    ball_positions.push_back(rw::math::Vector3D<>(-0.0, 0.45, 0.155));
        // PICK 3
    ball_positions.push_back(rw::math::Vector3D<>(0.25, 0.45, 0.155));

    rw::math::Rotation3D<> ball_rot = _target->getTransform(_state).R();
    rw::math::Transform3D<> cylinder_trans(ball_positions[0], ball_rot);    

     rw::math::Vector3D<> nearPlacePos(-0.16, -0.4775, 0.5625);
    rw::math::Transform3D<> nearPlaceFrame(nearPlacePos, ball_rot);

    rw::math::Vector3D<> nearPickPos(-0.255, 0.19, 0.655);
    rw::math::Transform3D<> nearPickFrame(nearPickPos, ball_rot);

    // Add the place position to interpolation point bag
    rw::math::Transform3D<> placeFrame(placePos, ball_rot);

    // Move Base  
    rw::math::Rotation3D<> base_rot = _base->getTransform(_state).R();
    rw::math::Vector3D<> base_positions(0.14, 0.06, 0.0);
    rw::math::Transform3D<> base_trans(base_positions, base_rot);
    _base->moveTo(base_trans, _state);

    // Move to pick area
    _ball->moveTo(cylinder_trans, table_frame, _state);

   //std::vector<rw::math::Transform3D<>> Points = p.pointOrder(_ball->getTransform(_state), _UR5->baseTend(_state), placeFrame, nearPickFrame, nearPlaceFrame);
   //std::vector<rw::math::Transform3D<>> Points = pointOrder(_ball->getTransform(_state), _UR5->baseTend(_state), placeFrame, nearPickFrame, nearPlaceFrame);

    std::vector<rw::math::Transform3D<>> Points;
    std::vector<float> Times;
    rw::math::Transform3D<> Point0 = _UR5->baseTframe(_UR5->getEnd(),_state)*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,0.1));
    Points.push_back(Point0);
    rw::math::Transform3D<> Point1 = nearPickFrame;
    Points.push_back(Point1);
    rw::math::Transform3D<> Point2 = _ball->getTransform(_state)*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.25));
    Points.push_back(Point2);
    rw::math::Transform3D<> Point3 = _ball->getTransform(_state);
    Points.push_back(Point3);
    rw::math::Transform3D<> Point4 = _ball->getTransform(_state)*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.25));
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
    rw::math::Transform3D<> Point10 = _UR5->baseTframe(_UR5->getEnd(),_state);
    Points.push_back(Point10);
    // rw::math::Transform3D<> Point11 = _UR5->baseTframe(robot_ur5->getEnd(),_state)*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,0.1));
    // Points.push_back(Point11);

    for (float i = 0; i < Points.size(); i++){
        Times.push_back(i);
    }

    

    rw::proximity::CollisionDetector detector (_wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy ());

    rw::math::Transform3D<> T_World = rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.1));

    std::vector<rw::math::Q> configurationPath;
    std::vector<rw::math::Transform3D<>> Tfs = Points;
    for ( unsigned int i = 0; i < Tfs.size(); i++ )
        Tfs[i] = T_World*Tfs[i];

    for ( rw::math::Transform3D<> Tf : Tfs ){
        _target->moveTo(Tf, _state);
        std::vector<rw::math::Q> solutions = findConfigurations("GraspTarget", "GraspTCP", _UR5, _wc, _state);
        rw::math::Q configuration = findCollisionFreeSolution(_UR5, _state, detector, solutions);
        configurationPath.push_back(configuration);
    }

    std::pair<std::vector<rw::math::Q>,std::vector<rw::math::Transform3D<>>> paths = P2P(Tfs, Times, _target, _UR5, _wc, _state, detector);

    std::vector<rw::math::Q> joints = paths.first;
    std::vector<rw::math::Transform3D<>> cartesian = paths.second;

    for ( unsigned int i = 0; i < cartesian.size(); i++ )
    {
        if ( cartesian[i].P() == Tfs[3].P() )
            _stepGrasp = i;

        if ( cartesian[i].P() == Tfs[8].P() )
            _stepRelease = i;

    }
    std::cout << "Grasp step: " << _stepGrasp << std::endl;
    std::cout << "Release step: " << _stepRelease << std::endl;

    std::cout << "Path length: " << joints.size() << std::endl;

    _path.clear();
    for ( rw::math::Q configuration : joints ) // configurationPath
    {
        if ( configuration.size() == 6 )
            _path.push_back(configuration);
    }
    std::cout << "Collision free path length: " << _path.size() << std::endl;

    if (!_timer->isActive()){
        _timer->start(10); // run 100 Hz
        _step = 0;
    }
    else
        _step = 0;

}

void SamplePlugin::resetRobotAndObject(){
    _device->setQ(_home,_state);
     _ball->moveTo(_ballPos, _table,_state);
    //_ball->moveTo(_ballPos,_state);
    getRobWorkStudio()->setState(_state);
}

void SamplePlugin::get25DImage ()
{
    if (_framegrabber25D != NULL) {
        for (size_t i = 0; i < _cameras25D.size (); i++) {
            // Get the image as a RW image
            Frame* cameraFrame25D = _wc->findFrame (_cameras25D[i]);    // "Camera");
            _framegrabber25D->grab (cameraFrame25D, _state);

            // const Image& image = _framegrabber->getImage();

            const rw::geometry::PointCloud* img = &(_framegrabber25D->getImage ());

            std::ofstream output (_cameras25D[i] + ".pcd");
            output << "# .PCD v.5 - Point Cloud Data file format\n";
            output << "FIELDS x y z\n";
            output << "SIZE 4 4 4\n";
            output << "TYPE F F F\n";
            output << "WIDTH " << img->getWidth () << "\n";
            output << "HEIGHT " << img->getHeight () << "\n";
            output << "POINTS " << img->getData ().size () << "\n";
            output << "DATA ascii\n";
            for (const auto& p_tmp : img->getData ()) {
                rw::math::Vector3D< float > p = p_tmp;
                output << p (0) << " " << p (1) << " " << p (2) << "\n";
            }
            output.close ();
        }
    }
}

void SamplePlugin::getImage ()
{
    if (_framegrabber != NULL) {
        for (size_t i = 0; i < _cameras.size (); i++) {
            // Get the image as a RW image
            Frame* cameraFrame = _wc->findFrame (_cameras[i]);    // "Camera");
            _framegrabber->grab (cameraFrame, _state);

            const rw::sensor::Image* rw_image = &(_framegrabber->getImage ());

            // Convert to OpenCV matrix.
            cv::Mat image = cv::Mat (rw_image->getHeight (),
                                     rw_image->getWidth (),
                                     CV_8UC3,
                                     (rw::sensor::Image*) rw_image->getImageData ());

            // Convert to OpenCV image
            Mat imflip, imflip_mat;
            cv::flip (image, imflip, 1);
            cv::cvtColor (imflip, imflip_mat, COLOR_RGB2BGR);


            cv::imwrite ("/home/reventlov/RobCand/RoVi_Project/SamplePlugin/img/"+_cameras[i] + ".png", imflip_mat);
            //cv::imwrite ("/home/reventlov/RobCand/RoVi_Project/SamplePlugin/image/test.png", imflip_mat);

            // Show in QLabel
            QImage img (imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
            QPixmap p         = QPixmap::fromImage (img);
            unsigned int maxW = 480;
            unsigned int maxH = 640;
            _label->setPixmap (p.scaled (maxW, maxH, Qt::KeepAspectRatio));
        }
    }
    std::cout << "Camera_Right" << std::endl;
    printProjectionMatrix ("Camera_Right");
    std::cout << "Camera_Left" << std::endl;
    printProjectionMatrix ("Camera_Left");
}

void SamplePlugin::moveCylinder(int index){
    //List of x, y, z values
    rw::math::Vector3D<> cylinder_pos;
    // 30 positions
    double x_values[] = {-0.20, -0.15, 0.20, -0.25, 0.25, 0.30, 0.20, 0.20, -0.20, -0.30, 0.30, 
    0.15, -0.15, -0.25, -0.30, 0.20, 0.30, -0.10, 0.20, -0.20, -0.15, 0.20, 0.10, 0.25, -0.30, -0.10, -0.20, -0.25, -0.10, 0.25}; // first number 
    double y_values[] = {0.50, 0.45, 0.45, 0.50, 0.45, 0.45, 0.40, 0.45, 0.40, 0.50, 
    0.50, 0.40, 0.40, 0.50, 0.55, 0.45, 0.50, 0.55, 0.45, 0.55, 0.40, 0.45, 0.50, 0.50, 0.50, 0.45, 0.35, 0.45, 0.50,  0.45};// first number 

    // 50 positions
    // double x_values[] = {-0.15, 0.20, -0.25, 0.25, 0.30, 0.20, 0.20, -0.20, -0.30, 0.30, 
    // 0.15, -0.15, -0.25, -0.30, 0.20, 0.30, -0.10, 0.20, -0.20, -0.15, 0.20, 0.10, 0.25, -0.30, -0.10, -0.20,
    // 0.15, -0.20, -0.35, 0.35, 0.30, 0.35, 0.15, -0.15, -0.30, 0.25, 0.20, 0.15, 0.30, -0.25, -0.20, 0.35, 0.20, 0.25, 0.10, -0.35, 0.30, }; // additional numbers
    // double y_values[] = {0.50, 0.45, 0.45, 0.50, 0.45, 0.45, 0.40, 0.45, 0.40, 0.50, 
    // 0.50, 0.40, 0.40, 0.50, 0.55, 0.45, 0.50, 0.55, 0.45, 0.55, 0.40, 0.45, 0.50, 0.50, 0.50, 0.45, 
    // 0.55, 0.50, 0.50, 0.55, 0.45, 0.40, 0.40, 0.35, 0.50, 0.35, 0.55, 0.30, 0.35, 0.45, 0.55, 0.30, 0.40, 0.35, 0.45, 0.50, 0.55, 0.35, 0.45, 0.50}; // additional numbers

    std::cout << "The x values, y values and 0.165 of the list is: " << std::endl;

    for (int i = 0; i < 30; i++){ 
        std::cout << x_values[i] << "\t" << y_values[i] << "\t" << 0.155 << std::endl; 
    }

    //Get the x, y values from the lists
    double x = x_values[index];
    double y = y_values[index];
    double z {0.155};

    //Set the position of the cylinder
    cylinder_pos = rw::math::Vector3D<>(x, y, z);

    //Get the rotation of the cylinder
    rw::math::Rotation3D<> cylinder_rot = _ball->getTransform(_state).R();

    //Create the transform
    rw::math::Transform3D<> cylinder_trans(cylinder_pos, cylinder_rot);

    //Move the cylinder to the new position
    _ball->moveTo(cylinder_trans, _table, _state);
    getRobWorkStudio()->setState(_state);
}

void SamplePlugin::getCollectImageTest (){
    imgTest++;    
    // const std::string target_name = "Ball";
    // rw::kinematics::MovableFrame::Ptr Ball_frame = _wc->findFrame<rw::kinematics::MovableFrame>(target_name);
    // if (NULL == Ball_frame) {
    //     RW_THROW("Could not find movable frame " + target_name + " ... check model");
    // }
    Frame* table_frame = _wc->findFrame ("Table");
    for (int i = 0; i < 30; i++){
            moveCylinder(i);
            if (!_timer->isActive()){
                _timer->start(10); // run 100 Hz
                _step = 0;
            }
            else
                _step = 0;

        if (_framegrabber != NULL) {
        
            for (size_t j = 0; j < _cameras.size (); j++) {
                // Get the image as a RW image
                Frame* cameraFrame = _wc->findFrame (_cameras[j]);    // "Camera");
                _framegrabber->grab (cameraFrame, _state);

                const rw::sensor::Image* rw_image = &(_framegrabber->getImage ());

                // Convert to OpenCV matrix.
                cv::Mat image = cv::Mat (rw_image->getHeight (),
                                        rw_image->getWidth (),
                                        CV_8UC3,
                                        (rw::sensor::Image*) rw_image->getImageData ());

                // Convert to OpenCV image
                Mat imflip, imflip_mat;
                cv::flip (image, imflip, 1);
                cv::cvtColor (imflip, imflip_mat, COLOR_RGB2BGR);


                cv::imwrite ("/home/reventlov/RobCand/RoVi_Project/SamplePlugin/image/"+_cameras[j] + std::to_string(i+1) +".png", imflip_mat);   
                //cv::imwrite ("/home/reventlov/RobCand/RoVi_Project/SamplePlugin/image/"+_cameras[j] + std::to_string(imgTest)+ "_" + std::to_string(i) +".png", imflip_mat);
                //cv::imwrite ("/home/reventlov/RobCand/RoVi_Project/SamplePlugin/image/test.png", imflip_mat);

                // Show in QLabel
                QImage img (imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
                QPixmap p         = QPixmap::fromImage (img);
                unsigned int maxW = 480;
                unsigned int maxH = 640;
                _label->setPixmap (p.scaled (maxW, maxH, Qt::KeepAspectRatio));
               
            }
        }
            if (i==29)
            {
                break;
            } 
        // rw::math::Vector3D<> cylinder_pos = rw::math::Vector3D<>(0.3, -0.5, 0.15);
        // rw::math::Rotation3D<> ball_rot = Ball_frame->getTransform(_state).R();
        // rw::math::Transform3D<> cylinder_trans(cylinder_pos, ball_rot);
        // Ball_frame->moveTo(cylinder_trans, table_frame, _state);
    }   
    std::cout << "Camera_Right" << std::endl;
    printProjectionMatrix ("Camera_Right");
    std::cout << "Camera_Left" << std::endl;
    printProjectionMatrix ("Camera_Left");
}

cv::Mat SamplePlugin::poseEstimation (){
    std::vector<cv::Mat> images;
    //sparseStereo v;
    cv::Mat leftImg, rightImg, ballPose, returnImg;
    cv::Mat projLeftMat, projRightMat, cam_left_mat, cam_right_mat;
     if (_framegrabber != NULL) {
        for (size_t i = 0; i < _cameras.size (); i++) {
            // Get the image as a RW image
            Frame* cameraFrame = _wc->findFrame (_cameras[i]);    // "Camera");
            _framegrabber->grab (cameraFrame, _state);

            const rw::sensor::Image* rw_image = &(_framegrabber->getImage ());

            // Convert to OpenCV matrix.
            cv::Mat image = cv::Mat (rw_image->getHeight (),
                                     rw_image->getWidth (),
                                     CV_8UC3,
                                     (rw::sensor::Image*) rw_image->getImageData ());

            // Convert to OpenCV image
            cv::Mat imflip, imflip_mat;
            cv::flip (image, imflip, 1);
            cv::cvtColor (imflip, imflip_mat, COLOR_RGB2BGR);

            images.push_back(imflip_mat);
            //cv::imwrite ("/home/reventlov/RobCand/RoVi_Project/SamplePlugin/image/"+_cameras[i] + ".png", imflip_mat);

            // Show in QLabel
            QImage img (imflip.data, imflip.cols, imflip.rows, imflip.step, QImage::Format_RGB888);
            QPixmap p         = QPixmap::fromImage (img);
            unsigned int maxW = 480;
            unsigned int maxH = 640;
            _label->setPixmap (p.scaled (maxW, maxH, Qt::KeepAspectRatio));
        }
    }
    rightImg = images[1];
    leftImg = images[0];
    
    std::cout << "Camera_Right" << std::endl;
    printProjectionMatrix ("Camera_Right",_wc,projLeftMat,cam_left_mat);
    std::cout << "Camera_Left" << std::endl;
    printProjectionMatrix ("Camera_Left",_wc,projRightMat,cam_right_mat);


    std::pair<cv::Mat, cv::Mat> poseImg = findPose(leftImg, rightImg, projLeftMat, projRightMat);

    std::cout << poseImg.first << std::endl;

    // Show in QLabel
    QImage img (poseImg.second.data, poseImg.second.cols, poseImg.second.rows, poseImg.second.step, QImage::Format_RGB888);
    QPixmap p         = QPixmap::fromImage (img);
    unsigned int maxW = 480;
    unsigned int maxH = 640;
    _label->setPixmap (p.scaled (maxW, maxH, Qt::KeepAspectRatio));

    return poseImg.first;

}


void SamplePlugin::interpolation(Q Q1, Q Q2, int sample)
{
	if(_wc->getDevices().empty()){
		RW_THROW("Load failed");
	}
	// calculate difference
	Q dQ = Q2-Q1;
	// calucalte stepsize
	Q step = dQ/sample;
	//run simulation
	for(int i = 0; i < sample; i++){
		//calc next step
		Q QStep = Q1 + (i*step);
		//update state by Q
		_device->setQ(QStep, _state);
		//update RobWorkStudio
		getRobWorkStudio()->setState(_state);
		//print Qstep info
		log().info() << QStep << "\n";
	}
}

// void SamplePlugin::timer ()
// {
//     if (0 <= _step && (size_t) _step < _path.size ()) {
//         _device->setQ (_path.at (_step), _state);
//         getRobWorkStudio ()->setState (_state);
//         _step++;
//     }
// }

void SamplePlugin::stateChangedListener (const State& state){
    _state = state;
}

bool SamplePlugin::checkCollisions (Device::Ptr device, const State& state, const CollisionDetector& detector, const Q& q)
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

void SamplePlugin::createPathRRTConnect (Q from, Q to, double extend, double maxTime)
{
    _device->setQ (from, _state);
    getRobWorkStudio ()->setState (_state);
    CollisionDetector detector (_wc, ProximityStrategyFactory::makeDefaultCollisionStrategy ());
    PlannerConstraint constraint = PlannerConstraint::make (&detector, _device, _state);
    QSampler::Ptr sampler        = QSampler::makeConstrained (QSampler::makeUniform (_device),
                                                       constraint.getQConstraintPtr ());
    QMetric::Ptr metric          = MetricFactory::makeEuclidean< Q > ();
    QToQPlanner::Ptr planner =
        RRTPlanner::makeQToQPlanner (constraint, sampler, metric, extend, RRTPlanner::RRTConnect);

    _path.clear ();
    if (!checkCollisions (_device, _state, detector, from))
        cout << from << " is in colission!" << endl;
    if (!checkCollisions (_device, _state, detector, to))
        cout << to << " is in colission!" << endl;
    ;
    Timer t;
    t.resetAndResume ();
    planner->query (from, to, _path, maxTime);
    t.pause ();

    if (t.getTime () >= maxTime) {
        cout << "Notice: max time of " << maxTime << " seconds reached." << endl;
    }

    const int duration = 10;

    if (_path.size () == 2) {    // The interpolated path between Q start and Q goal is collision
                                 // free. Set the duration with respect to the desired velocity
        LinearInterpolator< Q > linInt (from, to, duration);
        QPath tempQ;
        for (int i = 0; i < duration + 1; i++) {
            tempQ.push_back (linInt.x (i));
        }

        _path = tempQ;
    }
}

void SamplePlugin::printProjectionMatrix(std::string frameName, rw::models::WorkCell::Ptr wc, cv::Mat &proj_mat, cv::Mat &cam_mat) {
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


void SamplePlugin::printProjectionMatrix (std::string frameName,  cv::Mat &proj_mat, cv::Mat &cam_mat){
    Frame* cameraFrame = _wc->findFrame (frameName);
    if (cameraFrame != NULL) {
        if (cameraFrame->getPropertyMap ().has ("Camera")) {
            // Read the dimensions and field of view
            double fovy;
            int width, height;
            std::string camParam = cameraFrame->getPropertyMap ().get< std::string > ("Camera");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;

            double fovy_pixel = height / 2 / tan (fovy * (2 * M_PI) / 360.0 / 2.0);

            Eigen::Matrix< double, 3, 4 > KA;
            KA << fovy_pixel, 0, width / 2.0, 0, 0, fovy_pixel, height / 2.0, 0, 0, 0, 1, 0;

                        // OPENCV //
            cv::Mat KA_opencv = (cv::Mat_<double>(3, 4) << fovy_pixel, 0, width/2.0, 0,
                                                           0, fovy_pixel, height / 2.0, 0,
                                                           0, 0, 1, 0);

            cam_mat = KA_opencv.colRange(0, 3);

            std::cout << "Intrinsic parameters:" << std::endl;
            std::cout << KA << std::endl;

            Transform3D<> camPosOGL   = cameraFrame->wTf (_state);
            Transform3D<> openGLToVis = Transform3D<> (RPY<> (-Pi, 0, Pi).toRotation3D ());
            Transform3D<> H           = inverse (camPosOGL * inverse (openGLToVis));

            std::cout << "Extrinsic parameters:" << std::endl;
            std::cout << H.e () << std::endl;

            cv::Mat H_opencv = (cv::Mat_<double>(4, 4) << H.R().getRow(0)[0], H.R().getRow(0)[1], H.R().getRow(0)[2], H.P()[0],
                                                H.R().getRow(1)[0], H.R().getRow(1)[1], H.R().getRow(1)[2], H.P()[1],
                                                H.R().getRow(2)[0], H.R().getRow(2)[1], H.R().getRow(2)[2], H.P()[2],
                                                        0        ,           0       ,          0        ,     1   );
            // Calculates projection matrix opencv
            proj_mat = KA_opencv * H_opencv;
        }
    }
}


void SamplePlugin::printProjectionMatrix (std::string frameName){
    Frame* cameraFrame = _wc->findFrame (frameName);
    if (cameraFrame != NULL) {
        if (cameraFrame->getPropertyMap ().has ("Camera")) {
            // Read the dimensions and field of view
            double fovy;
            int width, height;
            std::string camParam = cameraFrame->getPropertyMap ().get< std::string > ("Camera");
            std::istringstream iss (camParam, std::istringstream::in);
            iss >> fovy >> width >> height;

            double fovy_pixel = height / 2 / tan (fovy * (2 * M_PI) / 360.0 / 2.0);

            Eigen::Matrix< double, 3, 4 > KA;
            KA << fovy_pixel, 0, width / 2.0, 0, 0, fovy_pixel, height / 2.0, 0, 0, 0, 1, 0;

            std::cout << "Intrinsic parameters:" << std::endl;
            std::cout << KA << std::endl;

            Transform3D<> camPosOGL   = cameraFrame->wTf (_state);
            Transform3D<> openGLToVis = Transform3D<> (RPY<> (-Pi, 0, Pi).toRotation3D ());
            Transform3D<> H           = inverse (camPosOGL * inverse (openGLToVis));

            std::cout << "Extrinsic parameters:" << std::endl;
            std::cout << H.e () << std::endl;
        }
    }
}

void SamplePlugin::timer() {
    if(0 <= _step && _step < _path.size()){
        
        
        _device->setQ(_path.at(_step),_state);
        _step++;
        if (_step == _stepGrasp)
            rw::kinematics::Kinematics::gripFrame(_ball, _tcp, _state);

        if (_step == _stepRelease)
            rw::kinematics::Kinematics::gripFrame(_ball, _table, _state);
        
        getRobWorkStudio()->setState(_state);
        
        if ( _step >= _path.size() )
            std::cout << "Ball final (place) location: " << _ball->getTransform(_state).P() << std::endl;
    }
}

std::pair<std::vector<rw::math::Q>,std::vector<rw::math::Transform3D<>>> SamplePlugin::P2P(std::vector<rw::math::Transform3D<>> P, std::vector<float> T, rw::kinematics::MovableFrame *targetFrame, rw::models::SerialDevice::Ptr robot_ur5, rw::models::WorkCell::Ptr wc, rw::kinematics::State state, rw::proximity::CollisionDetector& detector){ 
    std::vector<rw::math::Q> path;
    std::vector<rw::math::Transform3D<>> point;
    //Robotics p;
    for ( unsigned int i = 1; i < P.size(); i++ ){
        for ( float t = T[i-1]; t < T[i]; t += 0.01f ){

            Eigen::Vector3d Pi = P[i-1].P() + (P[i].P() - P[i-1].P())*constantVelocity(t,T[i-1],T[i]);
            Eigen::Quaterniond q1(P[i-1].R().e()), q2(P[i].R().e());            
            Eigen::Quaterniond q = q1.slerp(constantVelocity(t,T[i-1],T[i]), q2);
            point.push_back(convertToTransform3D(Pi,q));

            targetFrame->moveTo(point.back(), state);

            std::vector<rw::math::Q> solutions = findConfigurations("GraspTarget", "GraspTCP", robot_ur5, wc, state);

            rw::math::Q configuration;
            if ( path.size() > 0 ) {
                configuration = findCollisionFreeSolution(robot_ur5, state, detector, solutions, path.back());
            }else
                configuration = findCollisionFreeSolution(robot_ur5, state, detector, solutions);

            path.push_back(configuration);
        }
    }
    return std::make_pair(path, point);
}

rw::math::Q SamplePlugin::findCollisionFreeSolution(rw::models::SerialDevice::Ptr robot_ur5, rw::kinematics::State state, rw::proximity::CollisionDetector& detector, std::vector<rw::math::Q> solutions){
    rw::math::Q configuration;
    for ( unsigned int i = 0; i < solutions.size(); i++ )
    {
        robot_ur5->setQ(solutions[i], state);
        if ( !detector.inCollision(state, NULL, true) )
        {
            configuration = solutions[i];
            break;
        }
    }
    return configuration;
}

rw::math::Q SamplePlugin::findCollisionFreeSolution(rw::models::SerialDevice::Ptr robot_ur5, rw::kinematics::State state, rw::proximity::CollisionDetector& detector, std::vector<rw::math::Q> solutions, rw::math::Q prevSolution){
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

std::vector<rw::math::Q> SamplePlugin::findConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot_ur5, rw::models::WorkCell::Ptr wc, rw::kinematics::State state){
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


double SamplePlugin::constantVelocity(double t, double t0, double t1) {
    return (t-t0)/(t1-t0);
}

rw::math::Transform3D<double> SamplePlugin::convertToTransform3D(const Eigen::Vector3d translation, const Eigen::Quaterniond rotation){
  // Create an Eigen::Isometry3d with the translation and rotation
  Eigen::Isometry3d iso = Eigen::Translation3d(translation) * rotation;

  // Convert the Eigen::Isometry3d to a rw::math::Transform3D
  return rw::math::Transform3D<double>(iso.matrix());
}


std::pair<cv::Point2d, cv::Mat> SamplePlugin::locateBallV2(cv::Mat undistortedImage){
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

    // cv::imshow("1", image_hsv);
    // cv::waitKey(0);

	// Threshold the HSV image, keep only the green pixels
	cv::Mat lower_green_hue_range;
	cv::Mat upper_green_hue_range;
	cv::inRange(image_hsv, cv::Scalar(30, 86, 6), cv::Scalar(65, 255, 255), lower_green_hue_range);
	cv::inRange(image_hsv, cv::Scalar(30, 86, 6), cv::Scalar(65, 255, 255), upper_green_hue_range);

	// Combine the above two images
	cv::Mat green_hue_image;
	cv::addWeighted(lower_green_hue_range, 1.0, upper_green_hue_range, 1.0, 0.0, green_hue_image);


    cv::HoughCircles(green_hue_image, circles, cv::HOUGH_GRADIENT, 1, green_hue_image.rows, 100, 1, 20, 35);
    
    cv::Point2d objectCenter;

    // cv::imshow("2", green_hue_image);
    // cv::waitKey(0);

	if (circles.size() == 0) std::cout << "No circles were detected." << std::endl;
	else{
		for (size_t current_circle = 0; current_circle < circles.size(); ++current_circle){
			objectCenter.x = round(circles[current_circle][0]);
            objectCenter.y = round(circles[current_circle][1]);
			int radius = round(circles[current_circle][2]);

			cv::circle(undistortedImage, objectCenter, radius, cv::Scalar(0, 0, 255), 2);
		}
	}

    // cv::imshow("3", undistortedImage);
    // cv::waitKey(0);

    return std::make_pair(objectCenter, undistortedImage);
}


    std::pair<cv::Mat, cv::Mat>  SamplePlugin::findPose(cv::Mat left_img, cv::Mat right_img, cv::Mat projLeftMat, cv::Mat projRightMat){
    cv::Mat triangulatePoint(1, 1, CV_64FC4);
    cv::Mat leftPoint(1, 1, CV_64FC2);
    cv::Mat rightPoint(1, 1, CV_64FC2);
    cv::Mat result;

    std::pair<cv::Point2d,cv::Mat> locateCircleLeft = locateBallV2(left_img);
    std::pair<cv::Point2d,cv::Mat> locateCircleRight = locateBallV2(right_img);
    
   // cv::hconcat(locateCircleRight.second,locateCircleRight.second, result);
    // cv::imshow("result", locateCircleLeft.second);
    // cv::waitKey(0);

    leftPoint.at<cv::Vec2d>(0) = locateCircleLeft.first;
    rightPoint.at<cv::Vec2d>(0) = locateCircleRight.first;

    cv::triangulatePoints(projLeftMat, projRightMat, leftPoint, rightPoint, triangulatePoint);
    triangulatePoint =  triangulatePoint/triangulatePoint.at<double>(0, 3);

    cv::Mat transformationTable = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0,
                                                  0, 1, 0, 0,
                                                  0, 0, 1, 0.1,
                                                  0, 0, 0, 1);

    return std::make_pair((transformationTable*triangulatePoint), locateCircleLeft.second);
}