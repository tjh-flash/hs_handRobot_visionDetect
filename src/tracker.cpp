// ROS Dependencies
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <kdl/frames.hpp>

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

//发布行人检测反馈信息的发布器
ros::Publisher pedestrians_info_pub;

//行人检测的开关
std_msgs::Bool detection_switch;

//接收开关信息回调函数
void switch_detection_Callback(const std_msgs::Bool::ConstPtr & msg)
{
    detection_switch.data = msg->data;
}
     

int main(int argc, char *argv[])
{
    //ROS Stuff
    ros::init(argc, argv, "pedstrians_color_tracker");
    ros::NodeHandle nh;
    //ros::NodeHandle pnh("~"); //private node handler

    ros::AsyncSpinner spinner(4);
    spinner.start();

    std_msgs::Bool judge_msg;
    pedestrians_info_pub = nh.advertise<std_msgs::Bool>("pedestrian_detection", 10);
    ros::Subscriber pedestrains_detection_sub  = nh.subscribe("switch_of_peds_detect", 1, switch_detection_Callback);

    //OpenNI Stuff
    openni::OpenNI::initialize();

    //Open Kinect
    openni::Device mDevice;
    mDevice.open(openni::ANY_DEVICE);

    //Get the depthframe
    openni::VideoStream mDepthStream;
    mDepthStream.create(mDevice, openni::SENSOR_DEPTH);

    //Set VideoMode of Depth
    openni::VideoMode mDepthMode;
    mDepthMode.setResolution(640,480);
    mDepthMode.setFps(10);
    mDepthMode.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
    mDepthStream.setVideoMode(mDepthMode);

    //Get the colorframe
    openni::VideoStream mColorStream;
    mColorStream.create(mDevice, openni::SENSOR_COLOR);

    //Set VideoMode of Color
    openni::VideoMode mColorMode;
    mColorMode.setResolution(640,480);
    mColorMode.setFps(10);
    mColorMode.setPixelFormat(openni::PIXEL_FORMAT_RGB888); 
    mColorStream.setVideoMode(mColorMode);

    //set image registration depth to color
    //mDevice.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    
    //NiTE Stuff
    nite::NiTE::initialize();
    nite::UserTracker userTracker;
    userTracker.create(&mDevice);

    //Control the smoothing factor of the skeleton joints.Factor should between 0 and 1
    userTracker.setSkeletonSmoothingFactor( 0.3f );

    //create opencv color window
    //cv::namedWindow("View", CV_WINDOW_AUTOSIZE);

    //After being initialized, start to get the frame stream
    mDepthStream.start();
    mColorStream.start();

    //发送行人信息标志位
    int people_judge = 0 ;
    //图像显示窗口标志位
    int window_switch = 0;

    detection_switch.data = true;
    
    //行人数量
    //int count = 0;

    while (ros::ok() && detection_switch.data == true)
    {
        cv::Mat cImageBGR;

        //get the color frame
        openni::VideoFrameRef mColorFrame;
        mColorStream.readFrame(&mColorFrame);

        //Transform to the OpenCV data
        const cv::Mat mImageRGB(mColorFrame.getHeight(), mColorFrame.getWidth(), CV_8UC3, (void*)mColorFrame.getData());
        //std::cout << mImageRGB.size() << std::endl;

        //RGB TO BGR
        cv::cvtColor(mImageRGB, cImageBGR, CV_RGB2BGR);
        cv::resize(cImageBGR, cImageBGR, cv::Size(512,424), 0, 0, cv::INTER_AREA);

        //Get the frame data of the User
        nite::UserTrackerFrameRef userTrackerFrame;
        userTracker.readFrame(&userTrackerFrame);
        const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
        
        //send the pedestrian detect info
        if (users.getSize() == 0) 
        {
             if(window_switch == 1)
             {
                 cv::destroyWindow("View");
                 window_switch = 0;
             }
             std::cout<< " no people approach, detection service waiting for call!" << std::endl;
             continue;
        }
        
        window_switch = 1;
        cv::namedWindow("View", CV_WINDOW_AUTOSIZE);

        for (int i = 0; i < users.getSize(); ++i)
        {
            const nite::UserData& user = users[i];
            //check the status of the users
            if (user.isNew())
            {
                //start the skeleton track
                //count += 1;
                //std::cout << count << " user found!" << std::endl;
                userTracker.startSkeletonTracking(user.getId());
                
            }

            if (user.isVisible())
            {
                people_judge = 1 ;

                const nite::BoundingBox& bdb = user.getBoundingBox(); 
                cv::Point p1(bdb.min.x, bdb.min.y);
                cv::Point p2(bdb.max.x, bdb.max.y);
                cv::rectangle(cImageBGR, p1, p2, cv::Scalar(0, 0, 255), 1, 8, 0);
                std::cout << cImageBGR.size() << std::endl;
            }

            //if(!user.isVisible() || user.isLost())
            if( user.isLost())
            {
                people_judge = 0;
                userTracker.stopSkeletonTracking(user.getId());
                //count -= 1;
            }
        }

        if( people_judge == 0 )
        {
            std::cout<< " No people detected, safe!" << std::endl;
            judge_msg.data = false;
            pedestrians_info_pub.publish(judge_msg);
        }
        else if ( people_judge == 1)
        {
            std::cout << " People detected, please slow down!" << std::endl;
            //std::cout << " Real-time user number: " << count << std::endl;
            judge_msg.data = true;
            pedestrians_info_pub.publish(judge_msg);
            people_judge = 0;
        }

        //show the image
        cv::imshow("View", cImageBGR);
        cv::waitKey(3);
        //press q to exit the loop
        if(cv::waitKey(2) == 'q')
        {
            userTrackerFrame.release();
            break;
        }
          
    }
    
    //close everything
    //userTrackerFrame.release();
    userTracker.destroy();
    mColorStream.destroy();
    mDepthStream.destroy();
    mDevice.close();
    nite::NiTE::shutdown();
    openni::OpenNI::shutdown();
    ros::shutdown();
    std::cout << "Pedstrian Detection is shutting down!" << std::endl;;
    
    return 0;
}