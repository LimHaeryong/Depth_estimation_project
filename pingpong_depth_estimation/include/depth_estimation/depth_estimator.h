#ifndef HOMOGRAPHY_H_
#define HOMOGRAPHY_H_

#include <opencv2/opencv.hpp>

#include <yaml-cpp/yaml.h>

#include <fstream>
#include <iostream>
#include <vector>
#include <algorithm>
#include <unistd.h>
#include <cstdio>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

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

  private:
    bool FISH_EYE;

    cv::Size image_size;
    double yolo_image_size;

    std::vector<double> camera_matrix_;
    std::vector<double> dist_coeffs_;
    std::vector<double> homography_matrix_;

    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat homography_matrix;
    cv::Mat new_camera_matrix;

    cv::Mat map_x;
    cv::Mat map_y;

    ros::NodeHandle nh_;
    ros::Subscriber image_sub_;
    ros::Subscriber detection_sub_;

    cv::Mat frame_;
    std::vector<yolov3_trt_ros::BoundingBox> balls;
  };
} /* namespace xycar */

#endif
