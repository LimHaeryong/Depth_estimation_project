#ifndef TSTL_DEPTH_H_
#define TSTL_DEPTH_H_

#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <unistd.h>
#include <cstdio>
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "sensor_msgs/LaserScan.h"

#include "yolov3_trt_ros/BoundingBox.h"
#include "yolov3_trt_ros/BoundingBoxes.h"

namespace xycar
{
  class DepthEstimator final
  {
  public:
    DepthEstimator();
    virtual ~DepthEstimator();
    void setParams(const YAML::Node &config);
    void run();

  private:
    void imageCallback(const sensor_msgs::Image& msg);
    void detectionCallback(const yolov3_trt_ros::BoundingBoxes& msg);
    void lidarCallback(const sensor_msgs::LaserScan& msg);

  private:
    bool FISH_EYE;

    cv::Size image_size;
    int yolo_image_size;

    std::vector<double> camera_matrix_;
    std::vector<double> dist_coeffs_;
    std::vector<double> homography_matrix_;
    std::vector<double> rvec_;
    std::vector<double> tvec_;

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat homography_matrix;
    cv::Mat new_camera_matrix;
    cv::Mat rvec;
    cv::Mat tvec;

    cv::Mat map_x;
    cv::Mat map_y;

    double pitch;
    double climbing_angle;

    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Subscriber detection_sub_;
    ros::Subscriber lidar_sub_;

    cv::Mat frame_;
    std::vector<yolov3_trt_ros::BoundingBox> bboxes;

    sensor_msgs::LaserScan lidar_data;

  };
} /* namespace xycar */

#endif
