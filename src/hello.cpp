#include <ros/ros.h>
#include <std_msgs/String.h>
#include <json/json.h>
#include <std_msgs/Bool.h>

#include <iostream>

//语音检测的开关
std_msgs::Bool speak_switch;

//发布语音检测反馈信息的发布器
//握手信号发布器
ros::Publisher shake_info_pub;
//抓娃娃信号发布器
ros::Publisher grasp_info_pub;

//抓娃娃服务客户端

//能哥语音信号发布齐器
ros::Publisher voice_feedback_info_pub;

//接收语音回调处理函数
void shakehand_chatterCallback(const std_msgs::String::ConstPtr& msg)
{
    //检测到相关语音信息后要发布的不二类型信息
    std_msgs::Bool isShake;
    std_msgs::Bool isGrasp;
    isShake.data = false;
    isGrasp.data = false;

    //打印出接收到的语音信息
    std::cout << "接收语音信息成功!" << std::endl; 
    std::cout << "接收到的语音信息为: " << msg->data.c_str() << std::endl;

    std_msgs::String voice_msg;
    std::string ss;
    
    Json::Reader Json_reader;
    Json::Value Json_data;

    std::string data = msg->data;
    //读取数据
    Json_reader.parse(data, Json_data);
    std::string intent = Json_data["intent"].asString();

    //检测到相关语音信息后发布握手信号
    if (intent == "SHAKEHAND")
    {
        std::cout << "接收握手信号成功" << std::endl;
        isShake.data = true;
        shake_info_pub.publish(isShake);

        ss = "你好!";
        voice_msg.data = ss.c_str();
        voice_feedback_info_pub.publish(voice_msg);

    }

    if (intent == "grasp")
    {
        std::cout << "接收抓娃娃信号成功" << std::endl;
        isGrasp.data = true;
        grasp_info_pub.publish(isGrasp);

        ss = "抓娃娃!";
        voice_msg.data = ss.c_str();
        voice_feedback_info_pub.publish(voice_msg);
    }

    
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv,  "shakehand_listener");

    ros::NodeHandle nh;

    shake_info_pub = nh.advertise<std_msgs::Bool>("handgesture_detection", 10);
    grasp_info_pub = nh.advertise<std_msgs::Bool>("GraspToys", 10);

    voice_feedback_info_pub = nh.advertise<std_msgs::String>("voiceSolve_res", 10);

    ros::Subscriber shakehand_sub = nh.subscribe("/user_intent", 1000, shakehand_chatterCallback);

    ros::spin();

    ros::shutdown();
    return 0;
}
