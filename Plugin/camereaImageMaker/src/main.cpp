#include <rw/core/PropertyMap.hpp>
#include <rw/core/Ptr.hpp>
#include <rw/kinematics/State.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/SimulatedCamera.hpp>
#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

using namespace rw::core;
using namespace rw::common;
using rw::graphics::SceneViewer;
using namespace rw::kinematics;
using rw::loaders::WorkCellLoader;
using rw::models::WorkCell;
using rw::sensor::Image;
using namespace rwlibs::simulation;
using namespace rws;
/*
void makeImage(std::string pathWC, std::string nameCamera){

}*/
int main (int argc, char** argv)
{
    
    static const std::string WC_FILE = "/home/rovi2022/projects/RoVi_Project/WorkCellMOD/Scene.wc.xml";
    const WorkCell::Ptr wc = WorkCellLoader::Factory::load (WC_FILE);
    if (wc.isNull ())
        RW_THROW ("WorkCell could not be loaded.");
    std::string nameCamL = "Camera_Left";
    std::string nameCamR = "Camera_Right";
    Frame* const cameraL = wc->findFrame (nameCamL);
    Frame* const cameraR = wc->findFrame (nameCamR);
    if (cameraL == nullptr or cameraR == nullptr)
        RW_THROW ("Camera frame could not be found.");
    const PropertyMap& propertiesL = cameraL->getPropertyMap ();
    const PropertyMap& propertiesR = cameraR->getPropertyMap ();
    if (!propertiesL.has ("Camera"))
        RW_THROW ("Camera frame does not have Camera property.");
    const std::string parametersL = propertiesL.get< std::string > ("Camera");
    const std::string parametersR = propertiesR.get< std::string > ("Camera");
    std::istringstream issL (parametersL, std::istringstream::in);
    std::istringstream issR (parametersR, std::istringstream::in);
    double fovyL, fovyR;
    int widthL, widthR;
    int heightL, heightR;
    issL >> fovyL >> widthL >> heightL;
    issR >> fovyR >> widthR >> heightR;
    std::cout << "Camera properties: fov " << fovyL << " width " << widthL << " height " << heightL << std::endl;
    std::cout << "Camera properties: fov " << fovyR << " width " << widthR << " height " << heightR << std::endl;

    RobWorkStudioApp app ("");
    RWS_START (app)
    {
        RobWorkStudio* const rwstudio = app.getRobWorkStudio ();
        rwstudio->postOpenWorkCell (WC_FILE);
        TimerUtil::sleepMs (5000);

        const SceneViewer::Ptr gldrawerL = rwstudio->getView ()->getSceneViewer ();
        const SceneViewer::Ptr gldrawerR = rwstudio->getView ()->getSceneViewer ();
        const GLFrameGrabber::Ptr framegrabberL =
            ownedPtr (new GLFrameGrabber (widthL, heightL, fovyL));
        framegrabberL->init (gldrawerL);
        const GLFrameGrabber::Ptr framegrabberR =
            ownedPtr (new GLFrameGrabber (widthR, heightR, fovyR));
        framegrabberR->init (gldrawerR);
        SimulatedCamera::Ptr simcamL =
            ownedPtr (new SimulatedCamera ("SimulatedCamera", fovyL, cameraL, framegrabberL));
        SimulatedCamera::Ptr simcamR =
            ownedPtr (new SimulatedCamera ("SimulatedCamera", fovyR, cameraR, framegrabberR));
        
        simcamL->setFrameRate (100);
        simcamL->initialize ();
        simcamL->start ();
        simcamL->acquire ();

        simcamR->setFrameRate (100);
        simcamR->initialize ();
        simcamR->start ();
        simcamR->acquire ();

        static const double DT = 0.001;
        const Simulator::UpdateInfo info (DT);
        State state = wc->getDefaultState ();
        int cnt     = 0;
        const Image* imgL;
        const Image* imgR;
        while (!simcamL->isImageReady ()) {
            std::cout << "Image is not ready yet. Iteration " << cnt << std::endl;
            simcamL->update (info, state);
            simcamR->update (info, state);
            cnt++;
        }
        imgL = simcamL->getImage ();
        imgL->saveAsPPM (nameCamL+".ppm");
        simcamL->acquire ();
        simcamL->stop ();
        imgR = simcamR->getImage ();
        imgR->saveAsPPM (nameCamR+".ppm");
        simcamR->acquire ();
        simcamR->stop ();
        app.close ();
    }
    RWS_END ()

    cv::Mat imgL = cv::imread(nameCamL+".ppm",cv::IMREAD_COLOR);
    cv::Mat imgR = cv::imread(nameCamR+".ppm",cv::IMREAD_COLOR);
    //cv::flip(imgL,imgL,1);
    //cv::flip(imgR,imgR,1);
    cv::imwrite(nameCamL+".png",imgL);
    cv::imwrite(nameCamR+".png",imgR);
    //std::cout << "All done with writing the images and fliping the CAM" << std::endl;
    std::cout << "All done with writing the images" << std::endl;
}