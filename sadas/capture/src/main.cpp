#include <iostream>
#include <string>
#include <random>
#include <cstdlib>
#include <deque>
#include <numeric>
#include <ros/ros.h>
#include <ros/time.h>
#include <cv_bridge/cv_bridge.h>
// #include <opencv2/opencv.hpp>
#include "opencv4/opencv2/opencv.hpp"
#include <xycar_msgs/xycar_motor.h>
#include <sensor_msgs/Image.h>

void imageCallback(const sensor_msgs::Image& msg);
cv_bridge::CvImagePtr cv_ptr;
cv::Mat frame;           

int main(int argc, char** argv) {

    ros::init(argc, argv, "test");      // ROS init function
    ros::NodeHandle nh;                 // ROS nodehandle funtion 
    ros::Subscriber sub =               // ROS Subscriber Node
        nh.subscribe("/usb_cam/image_raw/", 1, imageCallback);
    
    int count = 0;
    // Driving Start (Get Camera Image)    
    while (ros::ok()) {
        ros::spinOnce();                                    // Get Camera frame once
        if (frame.empty()) {                            // Wait for camera image to be obtained
            continue;
        }
        cv::imshow("frame", frame);
        char c = cv::waitKey(1);
        if(c == 'q')
        {
            cv::imwrite("/home/nvidia/project4_learning/data_" + std::to_string(ros::Time::now().toSec()) + ".jpg" , frame);
        }
        
    }

    return 0;
}

//------------------------------------------
//      Image Callback Function
//------------------------------------------
void imageCallback(const sensor_msgs::Image& msg)
{
   cv::Mat src = cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<uint8_t*>(&msg.data[0]), msg.step);
   cv::cvtColor(src, frame, cv::COLOR_RGB2BGR);
}
