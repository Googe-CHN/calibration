#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <iostream>
#include <vector>

using namespace std;


vector<cv::Mat> vImages;
vector<cv::Mat> vRwm;
vector<cv::Mat> vtwm;


int IMAGE_COUNT=0;



string strOutputPath;
cv::FileStorage marker_poses_yaml;
cv::FileStorage image_list_yaml;


// function: 不阻塞的按键检测
bool kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL,0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if(ch != EOF)
    {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}


// function: 按s键保存图像,并保存位姿
void callback(const sensor_msgs::ImageConstPtr& image_msg, const geometry_msgs::PoseStampedConstPtr& pose_msg)
{  
    setlocale(LC_ALL, ""); // 设置编码格式，解决ROS_INFO中文乱码问题
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
    }

    cv::imshow("kinect", cv_ptr->image);
    cv::waitKey(1);

    if(kbhit())
    {
        int key = getchar();
        if(key=='s' | key=='S')
        {
            string image_name = strOutputPath+std::to_string(IMAGE_COUNT)+".png";
            cv::imwrite(strOutputPath+std::to_string(IMAGE_COUNT)+".png", cv_ptr->image);
            image_list_yaml << image_name.c_str();

            //保存maker位姿, Twm
            float x = pose_msg->pose.orientation.x;
            float y = pose_msg->pose.orientation.y;
            float z = pose_msg->pose.orientation.z;
            float w = pose_msg->pose.orientation.w;
            float xx = x*x;
            float yy = y*y;
            float zz = z*z;
            float xy = x*y;
            float wz = w*z;
            float wy = w*y;
            float xz = x*z;
            float yz = y*z;
            float wx = w*x;
            cv::Mat Rwm = (cv::Mat_<float>(3,3) << 1.0f-2*(yy+zz),  2*(xy-wz),      2*(wy+xz),
                                                   2*(xy+wz),       1.0f-2*(xx+zz), 2*(yz-wx),
                                                   2*(xz-wy),       2*(yz+wx),      1.0f-2*(xx+yy));
        
            cv::Mat twm = (cv::Mat_<float>(3,1) << pose_msg->pose.position.x*1000, pose_msg->pose.position.y*1000, pose_msg->pose.position.z*1000);


            string MP_tag = "marker_poses" + std::to_string(IMAGE_COUNT);

            marker_poses_yaml << "[";
            marker_poses_yaml << Rwm;
            marker_poses_yaml << twm;
            marker_poses_yaml << "]";

            IMAGE_COUNT++;
            ROS_INFO("保存图像%d张，保存Rwm, twm完毕", IMAGE_COUNT);

        }
   }

}




int main(int argc, char** argv)
{

    if(argc !=2)
    {
        cerr << endl << "Usage: rosrun calibration collect_hand_eye_data /path_to_settings.yaml"<< endl;
        return 1;
    }

    string strSettingFile = string(argv[1]);

    // 1. 加载Setting.yaml中参数
    cv::FileStorage fSettings(strSettingFile, cv::FileStorage::READ|cv::FileStorage::FORMAT_YAML);
    if(!fSettings.isOpened())
    {
        cout << "无法打开[ " << strSettingFile.c_str() <<" ]" << endl;
        return -1;
    }
    string image_topic = fSettings["Collect_Hand_Eye_Data.Image_Topic"];// /kinect2/qhd/image_color
    string marker_pose_topic = fSettings["Collect_Hand_Eye_Data.Marker_Pose_Topic"]; // /vrpn_client_node/kinect/pose
    strOutputPath = string(fSettings["Collect_Hand_Eye_Data.Output_Path"]);
    string strImageListFile = strOutputPath + string(fSettings["Collect_Hand_Eye_Data.Output_Image_List_File"]);
    string strMarkerPosesFile = strOutputPath + string(fSettings["Collect_Hand_Eye_Data.Output_Marker_Poses_File"]);
    fSettings.release();
    cout << "进入Hand Eye Data Collection程序" << endl;
    cout << "图片保存路径为: " << strOutputPath << endl;

    // 如果输出文件夹不存在则创建
    if(access(strOutputPath.c_str(), F_OK) == -1)
    {
        cout << "[" + strOutputPath + "] 文件夹不存在，现在创建！" << endl;
        if(0 == mkdir(strOutputPath.c_str(), 0755))
        {
            cout << "创建成功！" << endl;
        }
        else
        {
            cout << "文件夹创建失败！" << endl;
            return -1;
        }
    }
    
    // 创建 marker_poses.yaml
    marker_poses_yaml = cv::FileStorage(strMarkerPosesFile, cv::FileStorage::WRITE|cv::FileStorage::APPEND|cv::FileStorage::FORMAT_YAML);
    if(!marker_poses_yaml.isOpened())
    {
        cout << "无法打开[ " << strMarkerPosesFile.c_str() <<" ]" << endl;
        return -1;     
    } 
    
    //创建 image_list.yaml
    image_list_yaml = cv::FileStorage(strImageListFile, cv::FileStorage::WRITE | cv::FileStorage::APPEND | cv::FileStorage::FORMAT_YAML);
    if(!image_list_yaml.isOpened())
    {
        cout << "无法打开[ " << strImageListFile.c_str() <<" ]" << endl;
        return -1;     
    }

    marker_poses_yaml << "Twm" << "[";
    image_list_yaml << "images" << "[";

    // 2. ROS初始化及话题订阅 

    ros::init(argc, argv, "Collect_Hand_Eye_Data");
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, image_topic.c_str(), 10);
    message_filters::Subscriber<geometry_msgs::PoseStamped> pose_sub(nh, marker_pose_topic.c_str(), 10);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, geometry_msgs::PoseStamped> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), image_sub, pose_sub);
  
    
    sync.registerCallback(boost::bind(&callback, _1, _2));
    ros::spin();


    marker_poses_yaml << "]";
    marker_poses_yaml << "Num" << IMAGE_COUNT;
    marker_poses_yaml.release();

    image_list_yaml << "]";
    image_list_yaml << "Num" << IMAGE_COUNT;
    image_list_yaml.release();
    
    cout << "图片、ｍarker位姿搜集完成，总共收集: "<<IMAGE_COUNT<<"张，保存于："<<endl << strOutputPath.c_str() << endl;

    return 0;
}



