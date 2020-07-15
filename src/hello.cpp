#include <ros/ros.h>
#include <std_msgs/String.h>
#include <jsoncpp/json/json.h>
#include <std_msgs/Bool.h>
#include <rb_msgAndSrv/rb_DoubleBool.h>
#include <hirop_msgs/StartListen.h>
#include <hirop_msgs/StopListen.h>

#include <iostream>

//语音检测的开关
//std_msgs::Bool speak_switch;

//发布语音检测反馈信息的发布器
//握手信号发布器
//ros::Publisher shake_info_pub;
//抓娃娃信号发布器
//ros::Publisher grasp_info_pub;

//语音信号发布齐器
ros::Publisher voice_feedback_info_pub;

//语音启停客户端
ros::ServiceClient stop_listen_client;
ros::ServiceClient start_listen_client;

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
        ss = "你好";
        voice_msg.data = ss.c_str();
        voice_feedback_info_pub.publish(voice_msg);

        // std::cout << "接收握手信号成功" << std::endl;
        // isShake.data = true;
        // shake_info_pub.publish(isShake);
        std::cout << "接收握手信号成功" << std::endl;
    }

    if (intent == "GRASPTOY")
    {
        ss = "抓娃娃";
        voice_msg.data = ss.c_str();
        voice_feedback_info_pub.publish(voice_msg);

        std::cout << "接收抓娃娃信号成功" << std::endl;
        //isGrasp.data = true;
        //grasp_info_pub.publish(isGrasp);
        // rb_msgAndSrv::rb_DoubleBool grasp_client_srv;
        // grasp_client.call(grasp_client_srv); 
    }

    if (intent == "ENABLE")
    {
        ss = "上使能";
        voice_msg.data = ss.c_str();
        voice_feedback_info_pub.publish(voice_msg);

        std::cout << "接收机器人上使能信号成功" << std::endl;

    }

    if (intent == "DISABLE")
    {
        ss = "下使能";
        voice_msg.data = ss.c_str();
        voice_feedback_info_pub.publish(voice_msg);

        std::cout << "接收机器人下使能信号成功" << std::endl;
    }

    if (intent == "BACKHOME")
    {
        ss = "回原点";
        voice_msg.data = ss.c_str();
        voice_feedback_info_pub.publish(voice_msg);

        std::cout << "接收机器人回原点信号成功" << std::endl;
    }

    if (intent == "STOP")
    {
        ss = "停止";
        voice_msg.data = ss.c_str();
        voice_feedback_info_pub.publish(voice_msg);

        std::cout << "接收机器人停止信号成功" << std::endl;
    }

    if (intent == "OPENVISION")
    {
        ss = "启动行人检测";
        voice_msg.data = ss.c_str();
        voice_feedback_info_pub.publish(voice_msg);

        std::cout << "接收启动行人检测信号成功" << std::endl;
    }

    if (intent == "CLOSEVISION")
    {
        ss = "关闭行人检测";
        voice_msg.data = ss.c_str();
        voice_feedback_info_pub.publish(voice_msg);

        std::cout << "接收关闭行人检测信号成功" << std::endl;
    }

    if (intent == "TIME")
    {
        ss = "现在几点";
        voice_msg.data = ss.c_str();
        voice_feedback_info_pub.publish(voice_msg);

        std::cout << "接收报道时间信号成功" << std::endl;
    }

    if (intent == "INTRODUCE")
    {
        ss = "你是谁";
        voice_msg.data = ss.c_str();
        voice_feedback_info_pub.publish(voice_msg);

        std::cout << "接收机器人自我介绍信号成功" << std::endl;
    }

    if (intent == "ROBOTSTATUS")
    {
        ss = "报告机器人状态";
        voice_msg.data = ss.c_str();
        voice_feedback_info_pub.publish(voice_msg);

        std::cout << "接收机器人报告状态信号成功" << std::endl;
    }

    if (intent == "OK")
    {
        ss = "OK";
        voice_msg.data = ss.c_str();
        voice_feedback_info_pub.publish(voice_msg);

        std::cout << "接收机器人OK手势信号成功" << std::endl;
    }
}

bool voice_detect_switch_Callback(rb_msgAndSrv::rb_DoubleBool::Request &req, rb_msgAndSrv::rb_DoubleBool::Response &res)
{

    if (req.request == true)
    {
         hirop_msgs::StartListen start_srv;
         if (start_listen_client.call(start_srv))
         {
            ROS_INFO("voice has been enabled");
            res.respond = true;
         }
         else
         {
            ROS_INFO("please check voice service");
            res.respond = false;
         }
    }

    if (req.request == false)
    {
        hirop_msgs::StopListen stop_srv;
        if (stop_listen_client.call(stop_srv))
        {
            ROS_INFO("voice has been diasbled");
            res.respond = true;
        }
        else
        {
            ROS_INFO("please check voice service");
            res.respond = false;
        }

    }
         
    return true;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv,  "shakehand_listener");

    ros::NodeHandle nh;

    // shake_info_pub = nh.advertise<std_msgs::Bool>("handgesture_detection", 10);
    // //grasp_info_pub = nh.advertise<std_msgs::Bool>("GraspToys", 10);
    // grasp_client = nh.serviceClient<rb_msgAndSrv::rb_DoubleBool>("handClaw_grabDoll");

    voice_feedback_info_pub = nh.advertise<std_msgs::String>("voiceSolve_res", 10);

    ros::Subscriber shakehand_sub = nh.subscribe("/user_intent", 1000, shakehand_chatterCallback);

    ros::ServiceServer stop_voice_service  = nh.advertiseService("switch_voiceDetect", voice_detect_switch_Callback);

    stop_listen_client = nh.serviceClient<hirop_msgs::StopListen>("/stop_listen");
    start_listen_client = nh.serviceClient<hirop_msgs::StartListen>("/start_listen");

    ros::spin();

    ros::shutdown();
    return 0;
}
