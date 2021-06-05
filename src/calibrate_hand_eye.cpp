#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>



using namespace std;

string strHomePath;
string strHandeyeCarrier;

//function: 从calibration_result.yamll加载相机参数
bool readCalibrateResultYAML(const string& filename, cv::Mat& K, cv::Mat& distCoef, cv::Size& boardSize, float& squareSize)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ | cv::FileStorage::FORMAT_YAML);
    if(!fs.isOpened())
    {
        std::cout << " [ " + filename + " ] 文件打开失败"<< endl;
        return false; 
    }

    fs["camera_matrix"]>>K;
    fs["distortion_coefficients"]>>distCoef;

    int board_width = fs["board_width"];
    int board_height = fs["board_height"];
    boardSize = cv::Size(board_width, board_height);

    squareSize = fs["square_size"];
    std::cout << "棋盘格宽数： "<< boardSize.width << "， 高数： " << boardSize.height << endl;
    std::cout << "棋盘格尺寸: " << squareSize << " mm" << endl;
    std::cout << "内参矩阵： " << endl << K << endl;
    std::cout << "畸变系数： " << endl << distCoef << endl;
    fs.release();

    return true;

}




//function: 从marker_poses.xml加载位姿
bool readPosesYAML(const string& filename, vector<cv::Mat>& vR_wm, vector<cv::Mat>& vt_wm)
{
    vR_wm.resize(0);
    vt_wm.resize(0);
    cv::FileStorage fs(filename, cv::FileStorage::READ|cv::FileStorage::FORMAT_XML);
    if(!fs.isOpened())
    {
        std::cout << "[ " + filename +" ] 文件打开失败！" << endl;
        return false;
    }

    int num = fs["Num"];

    for(int i=0; i<num; i++)
    {
        cv::Mat Rwm, twm;
        fs["Twm"][i][0] >> Rwm;
        fs["Twm"][i][1] >> twm;

        vR_wm.push_back(Rwm);
        vt_wm.push_back(twm);
        //cout << "Rwm(1:3, 1:3, " << i <<")="  << Rwm << ";"<<endl;
        //cout << "twm(1:3, 1, " << i << ")=" <<twm << ";" <<endl;
    }

    fs.release();

    return true;

}




// function: 从image_list.xml加载手眼标定图像
bool readImageList( const string& filename, vector<cv::Mat>& vImages )
{
    vImages.resize(0);
    cv::FileStorage fs(filename, cv::FileStorage::READ|cv::FileStorage::FORMAT_XML);
    if( !fs.isOpened() )
        return false;

    int num = fs["Num"];

    for(int i=0; i<num; i++)
    {
        string imageFile;
        fs["images"][i] >> imageFile;
        //cout << imageFile << endl;
        cv::Mat imageMat = cv::imread(imageFile);
        if(imageMat.empty())
        {
            std::cout << "[ " + imageFile + " ] 图像读取失败" << endl;
            std::cout << false; 
        }
        vImages.push_back(imageMat);
    }
    
    return true;
}



bool EstimateCameraPose(const vector<cv::Mat>& vImages, const string& strCalibrationResult, vector<cv::Mat>& vR_cb, vector<cv::Mat>& vt_cb)
{

    cv::Mat K(3, 3, CV_64FC1);
    cv::Mat distCoef(1, 5, CV_64FC1);
    cv::Size boardSize;
    float squareSize;
    readCalibrateResultYAML(strCalibrationResult, K, distCoef, boardSize, squareSize);

    vR_cb.resize(0);
    vt_cb.resize(0);

    for(size_t i=0; i<vImages.size(); i++)
    {
        cv::Mat img = vImages[i];
        vector<cv::Point2f> vImgPoints;
        vector<cv::Point3f> vPoints;

        // 获取角点图像坐标
        if(0 == findChessboardCorners(img, boardSize, vImgPoints))
        {
            std::cout << "图像 " << i << "提取不到角点" << endl;
            return false;
        }
        else
        {
            cv::Mat imgGray;
            cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);
            //亚像素精细化 
            cv::cornerSubPix(imgGray, vImgPoints, cv::Size(11,11), cv::Size(-1,-1), cv::TermCriteria( cv::TermCriteria::EPS+cv::TermCriteria::COUNT, 30, 0.1 ));
            //绘制图像
        }
        
        // 获取角点三维坐标
        for (int i=0; i<boardSize.height; i++) 
        {
            for (int j=0; j<boardSize.width; j++) 
            {
                cv::Point3d tmpPoint;
                tmpPoint.x = j*squareSize;
                tmpPoint.y = i*squareSize;
                tmpPoint.z = 0;
                vPoints.push_back(tmpPoint);
            }
        }
        

        cv::Mat rvec, tvec, rMat;

        cv::solvePnP(vPoints, vImgPoints, K, distCoef, rvec, tvec, false);
        cv::Rodrigues(rvec, rMat);

        vR_cb.push_back(rMat);
        vt_cb.push_back(tvec);

    }

    return true;
}

int main(int argc, char** argv)
{
    // 路径: Settings.yaml
    string strSettingFile = "/home/birl/climbing_ws/src/calibration/Settings.yaml";


    // 1. 加载Setting.yaml中参数
    cv::FileStorage fSettings(strSettingFile, cv::FileStorage::READ|cv::FileStorage::FORMAT_YAML);
    if(!fSettings.isOpened())
    {
        std::cout << "无法打开[ " << strSettingFile.c_str() <<" ]" << endl;
        return -1;
    }
    string strCalibrateResultFile, strfs, strImageListYAML, strHandeyeResultFile;
    strHomePath = string(fSettings["Calibration.Home_path"]);
    strHandeyeCarrier = string(fSettings["Hand_Eye_Calibration_Carrier"]);
    
    strCalibrateResultFile = strHomePath + string(fSettings["Calibrate_Hand_Eye.Input_Calibrate_Result"]);

    string strHandEyeInputPath = strHomePath + string(fSettings["Collect_Hand_Eye_Data.Input_Path"]) + strHandeyeCarrier;
    strfs = strHandEyeInputPath + string(fSettings["Calibrate_Hand_Eye.Input_Tool_Poses_File"]);
    strImageListYAML = strHandEyeInputPath + string(fSettings["Calibrate_Hand_Eye.Input_Image_List_File"]);
    strHandeyeResultFile = strHandEyeInputPath + string(fSettings["Calibrate_Hand_Eye.Output_Handeye_Result"]);
    fSettings.release();


    // 2. 读取marker位姿， vR_wm, vt_wm, marker to world
    vector<cv::Mat> vR_wm, vt_wm;
    readPosesYAML(strfs, vR_wm,  vt_wm);


    // 3. 读取hand_eye图像
    vector<cv::Mat> vImages;
    readImageList(strImageListYAML, vImages);


    // 4. 获取相机相对标定板位姿，vR_cb, vt_cb 
    vector<cv::Mat> vR_cb, vt_cb;
    EstimateCameraPose(vImages, strCalibrateResultFile, vR_cb, vt_cb);
    std::cout << "完成R_wm, t_wm, R_cb, t_cb的获取，开始解AX=XB方程" << endl;


    // 5.  开始求解AX=XB方程 
    cv::HandEyeCalibrationMethod method;

    // CALIB_HAND_EYE_TSAI         = 0, //!< A New Technique for Fully Autonomous and Efficient 3D Robotics Hand/Eye Calibration @cite Tsai89
    // CALIB_HAND_EYE_PARK         = 1, //!< Robot Sensor Calibration: Solving AX = XB on the Euclidean Group @cite Park94
    // CALIB_HAND_EYE_HORAUD       = 2, //!< Hand-eye Calibration @cite Horaud95
    // CALIB_HAND_EYE_ANDREFF      = 3, //!< On-line Hand-Eye Calibration @cite Andreff99
    // CALIB_HAND_EYE_DANIILIDIS   = 4 

    cv::FileStorage handeye_result=cv::FileStorage(strHandeyeResultFile, cv::FileStorage::WRITE|cv::FileStorage::FORMAT_YAML|cv::FileStorage::APPEND);

    for(int method_idx=0; method_idx<5; method_idx++)
    {
        switch (method_idx)
        {
        case 0:
            method = cv::CALIB_HAND_EYE_TSAI;
            handeye_result<< "method" << "CALIB_HAND_EYE_TSAI";
            std::cout << "CALIB_HAND_EYE_TSAI---------------------" << endl;
            break;
        case 1:
            method = cv::CALIB_HAND_EYE_PARK;
            handeye_result<< "method" << "ALIB_HAND_EYE_PARK";
            std::cout << "ALIB_HAND_EYE_PARK----------------------" << endl;
            break;
        case 2:
            method = cv::CALIB_HAND_EYE_HORAUD;
            handeye_result<< "method" << "CALIB_HAND_EYE_HORAUD";
            std::cout << "CALIB_HAND_EYE_HORAUD-------------------" << endl;
            break;
        case 3:
            method = cv::CALIB_HAND_EYE_ANDREFF;
            handeye_result<< "method" << "CALIB_HAND_EYE_ANDREFF";
            std::cout << "CALIB_HAND_EYE_ANDREFF------------------" << endl;
            break;
        case 4:
            method = cv::CALIB_HAND_EYE_DANIILIDIS;
            handeye_result<< "method" << "CALIB_HAND_EYE_DANIILIDIS";
            std::cout << "CALIB_HAND_EYE_DANIILIDIS--------------" << endl;
            break;
 
        default:
            method = cv::CALIB_HAND_EYE_TSAI;
            handeye_result<< "method" << "CALIB_HAND_EYE_TSAI";
            std::cout << "CALIB_HAND_EYE_TSAI----------------------" << endl;
            break;
        }
    


        cv::Mat R_mc, t_mc;
        cv::calibrateHandEye(vR_wm, vt_wm, vR_cb, vt_cb, R_mc, t_mc, method);


        cv::Mat Tmc = (cv::Mat_<double>(4, 4) << R_mc.at<double>(0, 0), R_mc.at<double>(0, 1), R_mc.at<double>(0, 2), t_mc.at<double>(0), 
                                                R_mc.at<double>(1, 0), R_mc.at<double>(1, 1), R_mc.at<double>(1, 2), t_mc.at<double>(1), 
                                                R_mc.at<double>(2, 0), R_mc.at<double>(2, 1), R_mc.at<double>(2, 2), t_mc.at<double>(2), 
                                                    0                   , 0                   ,  0                  ,  1             );
        cout <<"Tmc" << endl << Tmc << endl;


        int num = vR_wm.size();
        vector<cv::Mat> vT_wb;
        for(int i=0; i<num; i++)
        {
            cv::Mat Twm = (cv::Mat_<double>(4, 4) << vR_wm[i].at<float>(0, 0), vR_wm[i].at<float>(0, 1), vR_wm[i].at<float>(0, 2), vt_wm[i].at<float>(0),
                                                    vR_wm[i].at<float>(1, 0), vR_wm[i].at<float>(1, 1), vR_wm[i].at<float>(1, 2), vt_wm[i].at<float>(1),
                                                    vR_wm[i].at<float>(2, 0), vR_wm[i].at<float>(2, 1), vR_wm[i].at<float>(2, 2), vt_wm[i].at<float>(2),
                                                    0                   , 0                   ,  0                  ,  1             );

            cv::Mat Tcb = (cv::Mat_<double>(4, 4) << vR_cb[i].at<double>(0, 0), vR_cb[i].at<double>(0, 1), vR_cb[i].at<double>(0, 2), vt_cb[i].at<double>(0),
                                                    vR_cb[i].at<double>(1, 0), vR_cb[i].at<double>(1, 1), vR_cb[i].at<double>(1, 2), vt_cb[i].at<double>(1),
                                                    vR_cb[i].at<double>(2, 0), vR_cb[i].at<double>(2, 1), vR_cb[i].at<double>(2, 2), vt_cb[i].at<double>(2),
                                                    0                   , 0                   ,  0                  ,  1             );

            cv::Mat Twb_tmp, Twb;
            Twb =  Twm*Tmc*Tcb;
            vT_wb.push_back(Twb);
        }

        vector<double> r1, r2, r3, r4, r5, r6, r7, r8, r9;
        vector<double> t1, t2, t3;

        for(int i=0; i<num; i++)
        {
            r1.push_back(vT_wb[i].at<double>(0, 0));
            r2.push_back(vT_wb[i].at<double>(0, 1));
            r3.push_back(vT_wb[i].at<double>(0, 2));
            r4.push_back(vT_wb[i].at<double>(1, 0));
            r5.push_back(vT_wb[i].at<double>(1, 1));
            r6.push_back(vT_wb[i].at<double>(1, 2));
            r7.push_back(vT_wb[i].at<double>(2, 0));
            r8.push_back(vT_wb[i].at<double>(2, 1));
            r9.push_back(vT_wb[i].at<double>(2, 2));

            t1.push_back(vT_wb[i].at<double>(0, 3));
            t2.push_back(vT_wb[i].at<double>(1, 3));
            t3.push_back(vT_wb[i].at<double>(2, 3));
        }


        cv::Mat mat_r1 = cv::Mat(r1);
        cv::Mat mat_r2 = cv::Mat(r2);
        cv::Mat mat_r3 = cv::Mat(r3);
        cv::Mat mat_r4 = cv::Mat(r4);
        cv::Mat mat_r5 = cv::Mat(r5);
        cv::Mat mat_r6 = cv::Mat(r6);
        cv::Mat mat_r7 = cv::Mat(r7);
        cv::Mat mat_r8 = cv::Mat(r8);
        cv::Mat mat_r9 = cv::Mat(r9);

        cv::Mat mat_t1 = cv::Mat(t1);
        cv::Mat mat_t2 = cv::Mat(t2);
        cv::Mat mat_t3 = cv::Mat(t3);
        
        cv::Mat mr1, mr2, mr3, mr4, mr5, mr6, mr7, mr8, mr9, mt1, mt2, mt3;
        cv::Mat stdr1, stdr2, stdr3, stdr4, stdr5, stdr6, stdr7, stdr8, stdr9, stdt1, stdt2, stdt3;

        cv::meanStdDev(mat_r1, mr1, stdr1);
        cv::meanStdDev(mat_r2, mr2, stdr2);
        cv::meanStdDev(mat_r3, mr3, stdr3);
        cv::meanStdDev(mat_r4, mr4, stdr4);
        cv::meanStdDev(mat_r5, mr5, stdr5);
        cv::meanStdDev(mat_r6, mr6, stdr6);
        cv::meanStdDev(mat_r7, mr7, stdr7);
        cv::meanStdDev(mat_r8, mr8, stdr8);
        cv::meanStdDev(mat_r9, mr9, stdr9);
        cv::meanStdDev(mat_t1, mt1, stdt1);
        cv::meanStdDev(mat_t2, mt2, stdt2);
        cv::meanStdDev(mat_t3, mt3, stdt3);


        cv::Mat mat_dev = (cv::Mat_<double>(3, 4) << stdr1.at<double>(0), stdr2.at<double>(0), stdr3.at<double>(0), stdt1.at<double>(0), 
                                                    stdr4.at<double>(0), stdr5.at<double>(0), stdr6.at<double>(0), stdt2.at<double>(0), 
                                                    stdr7.at<double>(0), stdr8.at<double>(0), stdr9.at<double>(0), stdt3.at<double>(0));

        
        handeye_result << "Tmc" << Tmc;
        handeye_result << "Standard_Deviation" << mat_dev;

        std::cout << "standard_deviation" << endl << mat_dev << endl;
        std::cout << "-----------------------------------------------------" << endl << endl << endl;

        
    }

    handeye_result.release();
    return 0;
}