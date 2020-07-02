// ROS Dependencies
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <boost/thread/thread.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

// C++ Dependencies
#include <iostream>
#include <stdio.h>

// NITE Dependencies
#include "NiTE.h"
#include "NiteSampleUtilities.h"

//Openni Dependencies
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

//手部检测的开关
std_msgs::Bool detection_switch;

//发布手势检测反馈信息的发布器
ros::Publisher HandGestures_info_pub;

//接收开关信息回调函数
void switch_detection_Callback(const std_msgs::Bool::ConstPtr & msg)
{
    detection_switch.data = msg->data;
}

int main(int argc, char *argv[])
{
	//ROS Stuff
    ros::init(argc, argv, "hand_tracker");
    ros::NodeHandle nh;
    
    //Activate multi threads
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    // Rostopic subscriber & publisher
    HandGestures_info_pub = nh.advertise<std_msgs::Bool>("HandGestures_detection", 10);
    ros::Subscriber pedestrains_detection_sub  = nh.subscribe("switch_of_hand_detect", 1, switch_detection_Callback);
    
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
    nite::HandTracker handTracker;
    //handTracker.create(&mDevice);
    if( handTracker.create() != nite::STATUS_OK )
    {
        std::cout << "Couldn't create hand tracker!" << std::endl;
        return -1;
    }

    //set the tracked pose of the hand
    handTracker.startGestureDetection( nite::GESTURE_WAVE );
    handTracker.startGestureDetection( nite::GESTURE_CLICK );
    handTracker.startGestureDetection( nite::GESTURE_HAND_RAISE);

    handTracker.setSmoothingFactor(0.5f);

    //cv::namedWindow("Depth Image", CV_WINDOW_AUTOSIZE);
    cv::namedWindow("Color Image", CV_WINDOW_AUTOSIZE);

	//After being initialized, start to get the frame stream
    mDepthStream.start();
    mColorStream.start();

    //the max depth value we get
	int MaxDepth = mDepthStream.getMaxPixelValue();

    std_msgs::Bool judge_msg;
    detection_switch.data = true;

	while( ros::ok() && detection_switch.data == true )
	{
		//create an opencv::Mat variable to show the color img
		Mat ImageBGR, ImageDEPTH;

		//get the color frame
        openni::VideoFrameRef mColorFrame;
        mColorStream.readFrame(&mColorFrame);

        //get the depth frame
        //openni::VideoFrameRef mDepthFrame;
        //mColorStream.readFrame(&mDepthFrame);

        //Transform to the OpenCV data
        const cv::Mat mImageRGB(mColorFrame.getHeight(), mColorFrame.getWidth(), CV_8UC3, (void*)mColorFrame.getData());
       // const cv::Mat mImageDepth(mDepthFrame.getHeight(), mColorFrame.getWidth(), CV_16UC1, (void*)mDepthFrame.getData());

        //RGB TO BGR
        cv::cvtColor(mImageRGB, ImageBGR, CV_RGB2BGR);
        cv::resize(ImageBGR, ImageBGR, cv::Size(512,424), 0, 0, cv::INTER_AREA);
        //mImageDepth.convertTo(ImageDEPTH, CV_8U, 255/MaxDepth);

		//read the frame to get it's information
        nite::HandTrackerFrameRef HandFrame;
        handTracker.readFrame( &HandFrame );

        //analysis the frame to find the defined gesture
        const nite::Array<nite::GestureData>& Gestures = HandFrame.getGestures();
        //cout << Gestures.getSize() << endl;
        //cout << "Do not find out any gesture yet!" << endl;

        for ( int i = 0; i <  Gestures.getSize(); i++ )
        {
            const nite::GestureData& Gesture = Gestures[i];
            cout << "Found the set gesture" << endl;

            //Juede the case of the gestures
            switch ( Gesture.getType() )
            {
            case nite::GESTURE_WAVE:
                cout << "摇手手势---wave" << endl;
                break;
            
            case nite::GESTURE_CLICK:
                cout << "前推并收回---click" << endl;
                break;
            
            case nite::GESTURE_HAND_RAISE:
                cout << "举手手势---hand_raise" << endl;
                break;
            }

            //print out the coordinate of the current gesture
            const nite::Point3f& Pos = Gesture.getCurrentPosition();
            //cout << "The position of the gesture is : (" << Pos.x << ", " << Pos.y << ", " << Pos.z << ")" << endl;
            
            if (Gesture.isComplete())
            {
                //Once we get the gesture we can start yo track
                nite::HandId HandID;
                handTracker.startHandTracking (Pos, &HandID);

                cout << "Start Hand Tracking...." << endl; 
                //cout << Gesture.getType() << endl;

                judge_msg.data = true;
                HandGestures_info_pub.publish(judge_msg);

            }
        }

        const nite::Array<nite::HandData>& Hands = HandFrame.getHands();

        for ( int i = 0; i < Hands.getSize(); ++i )
        {
            const nite::HandData& Hand = Hands[i];
            
            if ( Hand.isNew() )
              cout << "Target tracked!" << endl;

            else if ( Hand.isLost() )
            {
                cout << "Target lost!" << endl;
                judge_msg.data = false;
                HandGestures_info_pub.publish(judge_msg);
            }
              
            
            //check the tarcking state
            if ( Hand.isTracking() )
            {
                //get the coordinate of the palm
                const nite::Point3f& Pos = Hand.getPosition();
                cout << "The position of the palm is: " << Pos.x << ", " << Pos.y << ", " << Pos.z << endl;
                
                //convert the coordi to the depth image
                cv::Point2f Point;
                handTracker.convertHandCoordinatesToDepth( Pos.x, Pos.y, Pos.z, &Point.x, &Point.y) ;

                cv::circle ( ImageBGR, Point, 3, cv::Scalar(0, 0, 255), 4 );

                //retangle the hand
                cv::Point2f PTL(Point.x - 50, Point.y - 50);
                cv::Point2f PDR(Point.x + 50, Point.y + 50);

                cv::rectangle(ImageBGR, PTL, PDR, cv::Scalar(0, 0, 255), 2, 8, 0);
                //cv::rectangle(ImageDEPTH, PTL, PDR, cv::Scalar(0, 0, 255), 1, 8, 0);
                cout << "Hello" << endl;
 
            }
        }

        cv::imshow("Color Image", ImageBGR);
        cv::waitKey(2);
        //cv::imshow("Depth Image", ImageDEPTH);

	}

    //Release everything
    handTracker.destroy();
    mColorStream.destroy();
    mDepthStream.destroy();
    mDevice.close();

    nite::NiTE::shutdown();
    openni::OpenNI::shutdown();
    ros::shutdown();

    cout << "Hand Dedection had already shut down!" << endl;

	return 0;
}
