#include "denseStereo.hpp"

int main(int argc, char** argv) {

    if (argc != 3) {
        std::cout << "Usage:" << '\n';
        std::cout << argv[0] << " firstImage.png secoundImage.png\n";
        return -1;
    }
    
    denseStereo m1Dense;
    //cv::Mat imageL = cv::imread("/home/rovi2022/projects/RoVi_Project/DenseStereo/image/left3.png");
    //cv::Mat imageR = cv::imread("/home/rovi2022/projects/RoVi_Project/DenseStereo/image/right3.png");
    //cv::Mat imageL = cv::imread("/home/rovi2022/projects/RoVi_Project/DenseStereo/image/tsukuba_l.png");
    //cv::Mat imageR = cv::imread("/home/rovi2022/projects/RoVi_Project/DenseStereo/image/tsukuba_r.png");
    cv::Mat imageL = cv::imread(argv[1]);
    cv::Mat imageR = cv::imread(argv[2]);
    //cv::Mat imageR = cv::imread("/home/rovi2022/projects/RoVi_Project/DenseStereo/image/leftCam.ppm");
    //cv::Mat imageL = cv::imread("/home/rovi2022/projects/RoVi_Project/DenseStereo/image/rightCam.ppm");
    //cv::Mat imageL = cv::imread("/home/rovi2022/projects/RoVi_Project/DenseStereo/image/Scene_Left.png");
    //cv::Mat imageR = cv::imread("/home/rovi2022/projects/RoVi_Project/DenseStereo/image/Scene_Right.png");
    if (imageR.empty() || imageL.empty()) {
        std::cout << "Image where empty so returning" << std::endl;
        return -1;
    }
    
    cv::Mat colors = imageL;
    cv::cvtColor(imageL,imageL,cv::COLOR_BGR2GRAY);
    cv::cvtColor(imageR,imageR,cv::COLOR_BGR2GRAY);
    //m1Dense.showImages(imageL,imageR);
    // nscene, bottle_offset: disp = 9, blocksize = 9 
    // rabbit_offset: disp = 5, blocksize = 5 
    
    //Used in the Q matrix
    double Tx = 200;
    double f = 514;
    bool exit = false;
    while (!exit)
    {
        std::string methodeSelection;
        std::cout << "BM or SBM: ";
        while (true)
        {
            std::cin >> methodeSelection;
            if (methodeSelection == "bm" or methodeSelection == "BM" or methodeSelection == "SBM" or methodeSelection == "sbm"){
            std::cout << "Performing: " << methodeSelection << std::endl;
            break;
            } else {
            std::cout << "Wrong selection: " << methodeSelection << " is not valied" << std::endl;
            }
        }
        cv::Mat oldNormDisp;
        while(true){
            int nDis;
            int windowVal;
            //This are used to display data for comparison
            int nDisOld;
            int windowOld;
            cv::Mat disp;
            std::cout << "Select nDisp: ";
            std::cin >> nDis;
            std::cout << "Select window: ";
            std::cin >> windowVal;
            if (methodeSelection == "BM" || methodeSelection == "bm")
            {
                disp = m1Dense.computeDisparity(imageL,imageR,16*nDis,windowVal,1);
            }else if (methodeSelection == "SBM" or methodeSelection == "sbm")
            {
                disp = m1Dense.computeDisparity(imageL,imageR,16*nDis,windowVal,2);
            }

            cv::Mat dispNorm = m1Dense.normaliseDisparity(disp);
            cv::Mat points;
            std::string winName1 = "Values ndib 9*"+std::to_string(nDis)+" windows "+std::to_string(windowVal);
            auto qMat = m1Dense.qMat(imageL.cols,imageL.rows,Tx,f);
            cv::reprojectImageTo3D(disp,points,qMat,true);
            m1Dense.savePointCloud("cloud.pcd",points,colors,500);


            if (!oldNormDisp.empty())
            {
                std::string winName2 = "Values ndib 9*"+std::to_string(nDisOld)+" windows "+std::to_string(windowOld);
                m1Dense.showImages(dispNorm,oldNormDisp,winName1,winName2);
            } else {
                m1Dense.showImage(dispNorm,winName1);
            }
            cv::waitKey(0);

            char cho;
            
            std::cout << "[R]estart All, Refine [V]alues or [Q]uit ";
            std::cin >> cho;
            

            if (toupper(cho) == 'R')
            {
                std::cout << "Restarting the process" << std::endl;
                break;
            } else if (toupper(cho) == 'Q')
            {
                std::cout << "Exiting the program" << std::endl;
                exit = true;
                break;
            }
            else{
                std::cout << "New Values old values where. nDip: 16*" << nDis << " window " << windowVal << std::endl;
                oldNormDisp = dispNorm;
                nDisOld = nDis;
                windowOld = windowVal;
            }

        }       
    }

    std::cout << "DONE" << std::endl;
    return 0;
}

