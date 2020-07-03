// ROS Dependencies
#include <ros/ros.h>
#include <ros/package.h>
#include <kdl/frames.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// C++ Dependencies
#include <iostream>
#include <stdio.h>
#include <sstream>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>

// NITE Dependencies
#include "NiTE.h"
#include "NiteSampleUtilities.h"

//Tracker Messages
#include <openni2/OpenNI.h>
#include <openni2_tracker/TrackerUser.h>
#include <openni2_tracker/TrackerUserArray.h>

//OpenCV Dependencies
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

//发布手势检测反馈信息的发布器
ros::Publisher HandGestures_info_pub;

//发布行人检测反馈信息的发布器
ros::Publisher Pedestrians_info_pub;

//检测效果图片发布器
image_transport::Publisher videphoto_feedback_pub;

//行人检测的开关
std_msgs::Bool people_detection_switch;

//手部检测的开关
std_msgs::Bool handgesture_detection_switch;

//发送行人信息标志位
int people_judge = 0 ;

//接收开关信息回调函数
void switch_detection_Callback(const std_msgs::Bool::ConstPtr & msg)
{
    people_detection_switch.data = msg->data;
}


int main(int argc, char *argv[])
{
    //ROS初始化
    ros::init(argc, argv, "vision_detect_bridge");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    
    //开启多线程
    ros::AsyncSpinner spinner(4);
    spinner.start();
    

    //定义ROS节点话题发布器和订阅器
    Pedestrians_info_pub = nh.advertise<std_msgs::Bool>("pedestrian_detection", 10);
    HandGestures_info_pub = nh.advertise<std_msgs::Bool>("handgesture_detection", 10);
    videphoto_feedback_pub = it.advertise("videphoto_feedback", 1);
    ros::Subscriber pedestrains_detection_sub  = nh.subscribe("switch_of_vision_detect", 1, switch_detection_Callback);

    //OpenNI初始化,打开Kinect,设置彩色和深度模式视频流
    openni::OpenNI::initialize();
    openni::Device mDevice;
    mDevice.open(openni::ANY_DEVICE);

    // //Get the depthframe
    // openni::VideoStream mDepthStream;
    // mDepthStream.create(mDevice, openni::SENSOR_DEPTH);

    // //Set VideoMode of Depth
    // openni::VideoMode mDepthMode;
    // mDepthMode.setResolution(1920,1080);
    // mDepthMode.setFps(10);
    // mDepthMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
    // mDepthStream.setVideoMode(mDepthMode);

    //Get the colorframe
    openni::VideoStream mColorStream;
    mColorStream.create(mDevice, openni::SENSOR_COLOR);

    //Set VideoMode of Color
    openni::VideoMode mColorMode;
    mColorMode.setResolution(640,480);
    mColorMode.setFps(5);
    mColorMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888); 
    mColorStream.setVideoMode(mColorMode);

    //NiTE初始化,创建用户和手部跟踪器
    nite::NiTE::initialize();

    nite::UserTracker userTracker;
    userTracker.create(&mDevice);
    
    nite::HandTracker handTracker;
    handTracker.create(&mDevice);

    //初始化要识别的手势
    handTracker.startGestureDetection( nite::GESTURE_WAVE );
    handTracker.startGestureDetection( nite::GESTURE_HAND_RAISE);

    //设置灵敏度参数
    userTracker.setSkeletonSmoothingFactor(0.3f);
    handTracker.setSmoothingFactor(0.1f);


    //打开图像数据流开关
    //mDepthStream.start();
    mColorStream.start();

    //初始化行人检测和手势检测的信号反馈
    std_msgs::Bool pedestrians_info;
    std_msgs::Bool handgestures_info;
    
    //默认打开行人检测和关闭手势检测
    people_detection_switch.data = true;
    handgesture_detection_switch.data = false;

    while (ros::ok() && people_detection_switch.data == true)
    {
        cv::Mat cImageBGR;

        //得到彩色图像
        openni::VideoFrameRef mColorFrame;
        mColorStream.readFrame(&mColorFrame);

        //转换为OpenCV格式
        const cv::Mat mImageRGB(mColorFrame.getHeight(), mColorFrame.getWidth(), CV_8UC3, (void*)mColorFrame.getData());
        //std::cout << mImageRGB.size() << std::endl;

        //RGB TO BGR
        cv::cvtColor(mImageRGB, cImageBGR, CV_RGB2BGR);
        cv::resize(cImageBGR, cImageBGR, cv::Size(512,424), 0, 0, cv::INTER_AREA);

        //从图像中得到用户信息
        nite::UserTrackerFrameRef userTrackerFrame;
        userTracker.readFrame(&userTrackerFrame);
        const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
        
        if (users.getSize() == 0) 
        {
             std::cout<< " no people approach, detection service waiting for call!" << std::endl;
             sensor_msgs::ImagePtr Imagemsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cImageBGR).toImageMsg();
             videphoto_feedback_pub.publish(Imagemsg);
             continue;
             
        }

        for (int i = 0; i < users.getSize(); ++i)
        {
            const nite::UserData& user = users[i];
            //检测用户状态
            if (user.isNew())
            {
                userTracker.startSkeletonTracking(user.getId());   
            }

            if (user.isVisible())
            {
                const nite::BoundingBox& bdb = user.getBoundingBox(); 
                cv::Point p1(bdb.min.x, bdb.min.y);
                cv::Point p2(bdb.max.x, bdb.max.y);
                int area = (bdb.max.x - bdb.min.x)*(bdb.max.y - bdb.min.y);
                //std::cout << "矩形面积为:" << area << std::endl;

                if(area <= 75000)
                {
                    people_judge = 1;
                    cv::rectangle(cImageBGR, p1, p2, cv::Scalar(0, 0, 255), 5, 8, 0);
                }

                else if (area > 75000)
                {
                    if(users.isEmpty())
                        people_judge = 0;
                    else 
                        people_judge = 1;
                }
                
                
            }

            if( user.isLost())
            {
                people_judge = 0;
                userTracker.stopSkeletonTracking(user.getId());
            }
        }

        if ( people_judge == 0 )
        {
            std::cout<< " No people detected, safe!" << std::endl;
            pedestrians_info.data = false;
            Pedestrians_info_pub.publish(pedestrians_info);
            handgesture_detection_switch.data = false;
        }
        else if ( people_judge == 1)
        {
            std::cout << " People detected, please slow down!" << std::endl;
            pedestrians_info.data = true;
            Pedestrians_info_pub.publish(pedestrians_info);
            people_judge = 0;
            handgesture_detection_switch.data = true;
        }

        if ( handgesture_detection_switch.data == true )
        {
            //从图像中得到手部信息
            nite::HandTrackerFrameRef HandFrame;
            handTracker.readFrame( &HandFrame );
            const nite::Array<nite::GestureData>& Gestures = HandFrame.getGestures();

            for ( int i = 0; i <  Gestures.getSize(); i++ )
            {
                const nite::GestureData& Gesture = Gestures[i];

                //get the coordinate of the current gesture
                const nite::Point3f& Pos = Gesture.getCurrentPosition();
                
                //检测到手势
                if (Gesture.isComplete())
                {
                    //检测到手势便可开始手部跟踪
                    nite::HandId HandID;
                    handTracker.startHandTracking (Pos, &HandID);
                    ////圈出手部位置
                    cv::Point2f Point;
                    handTracker.convertHandCoordinatesToDepth( Pos.x, Pos.y, Pos.z, &Point.x, &Point.y) ;

                    cv::Point2f PTL(Point.x - 30, Point.y - 30);
                    cv::Point2f PDR(Point.x + 30, Point.y + 30);
                    cv::rectangle(cImageBGR, PTL, PDR, cv::Scalar(255, 0, 0), 2, 8, 0);

                    handgestures_info.data = true;
                    HandGestures_info_pub.publish(handgestures_info);

                    //handgestures_info.data = false;
                }


            }

            //const nite::Array<nite::HandData>& Hands = HandFrame.getHands();

            // for ( int i = 0; i < Hands.getSize(); ++i )
            // {
            //     const nite::HandData& Hand = Hands[i];

            //     if ( Hand.isTracking() )
            //     {
            //         const nite::Point3f& Pos = Hand.getPosition();
            //         cv::Point2f Point;
            //         handTracker.convertHandCoordinatesToDepth( Pos.x, Pos.y, Pos.z, &Point.x, &Point.y) ;

            //         cv::circle ( cImageBGR, Point, 5, cv::Scalar(255, 0, 0), 1 );
            //     }

            //     if ( Hand.isLost() || Hand.isTouchingFov() )
            //     {
            //         handTracker.stopHandTracking(Hand.getId());
            //         handgestures_info.data = false;
            //         HandGestures_info_pub.publish(handgestures_info);
            //     }
            // }

        }
    
    sensor_msgs::ImagePtr Imagemsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cImageBGR).toImageMsg();
    videphoto_feedback_pub.publish(Imagemsg);
    
          
    }
    
    //close everything
    //userTrackerFrame.release();
    handTracker.destroy();
    userTracker.destroy();
    mColorStream.destroy();
    //mDepthStream.destroy();
    //mDevice.close();
    nite::NiTE::shutdown();
    openni::OpenNI::shutdown();
    ros::shutdown();
    std::cout << "vision Detection is shutting down!" << std::endl;;
    
    return 0;
}  