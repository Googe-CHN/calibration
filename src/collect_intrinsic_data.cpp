#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <iostream>
#include <vector>
#include <string>

using namespace std;

string strHomePath;
string strOutputPath;
cv::FileStorage image_list_yaml;
int IMAGE_COUNT = 0;


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


//function: 回调函数，用于显示实时图像，按s键保存图像
void callback(const sensor_msgs::ImageConstPtr& image_msg)
{
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
            //string image_tag = "image"+std::to_string(IMAGE_COUNT);
            string image_name = strOutputPath + "/" + std::to_string(IMAGE_COUNT)+".png";
            cv::imwrite(image_name, cv_ptr->image);
            image_list_yaml << image_name.c_str();
            IMAGE_COUNT++;
            ROS_INFO("保存图像%d张.", IMAGE_COUNT);
        }
    }
}


// functions: main

int main(int argc, char** argv)
{
    // 路径: Settings.yaml
    string strSettingFile = "/home/birl/climbing_ws/src/calibration/Settings.yaml";

    // 1. 加载Setting.yaml中参数
    cv::FileStorage fSettings(strSettingFile, cv::FileStorage::READ|cv::FileStorage::FORMAT_YAML);
    if(!fSettings.isOpened())
    {
        cout << "无法打开[ " << strSettingFile.c_str() <<" ]" << endl;
        return -1;
    }

    string strImageTopic = fSettings["Collect_Intrinsic_Data.Image_Topic"];
    strHomePath = string(fSettings["Calibration.Home_path"]);

    strOutputPath = strHomePath + string(fSettings["Collect_Intrinsic_Data.Output_Path"]);
    string strImageListFile = strOutputPath + string(fSettings["Collect_Intrinsic_Data.Output_Image_List_File"]);
    fSettings.release();


    // 2. 创建image_list.yaml写入文件
    if(access(strOutputPath.c_str(), F_OK) == -1)
    {
        cout << "[" + strOutputPath + "]文件夹不存在，现在创建！" << endl;
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
    
    image_list_yaml = cv::FileStorage(strImageListFile, cv::FileStorage::WRITE | cv::FileStorage::APPEND | cv::FileStorage::FORMAT_YAML);
    if(!image_list_yaml.isOpened())
    {
        cout << "无法打开[ " << strImageListFile.c_str() <<" ]" << endl;
        return -1;     
    }
    image_list_yaml << "images" << "[";


    // 3. 启动ros节点收取图像，并按s键确认保存
    ros::init(argc, argv, "collect_intrinsic_data");
    ros::NodeHandle nh;
    ros::Subscriber image_sub = nh.subscribe(strImageTopic.c_str(), 10, callback);
    ros::spin();


    // 4. 结束采集，完成imageList.yaml保存
    image_list_yaml << "]";
    image_list_yaml << "Num" << IMAGE_COUNT;
    image_list_yaml.release();
    
    cout << "图片搜集完成，总共收集: "<<IMAGE_COUNT<<"张，保存于："<<endl << strOutputPath.c_str() << endl;

    return 0;

}




