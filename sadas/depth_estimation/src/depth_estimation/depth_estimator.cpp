#include "depth_estimation/depth_estimator.h"

static bool compare_point(const yolov3_trt_ros::BoundingBox& box1, const yolov3_trt_ros::BoundingBox& box2)
{
  return box1.xmin < box2.xmin;
}

namespace xycar
{
DepthEstimator::DepthEstimator()
{
  std::string config_calibration_path;
  nh_.getParam("config_calibration_path", config_calibration_path);
  YAML::Node config_calibration = YAML::LoadFile(config_calibration_path);

  setParams(config_calibration);
  image_sub_ = nh_.subscribe("/usb_cam/image_raw/", 1, &DepthEstimator::imageCallback, this);
  detection_sub_ = nh_.subscribe("/yolov3_trt_ros/detections", 1, &DepthEstimator::detectionCallback, this);
}

void DepthEstimator::setParams(const YAML::Node &config_calibration) {
  IMAGE_WIDTH = config_calibration["image_size"]["width"].as<int>();
  IMAGE_HEIGHT = config_calibration["image_size"]["height"].as<int>();
  FISH_EYE = config_calibration["fish_eye"].as<bool>();

  image_size.width = IMAGE_WIDTH;
  image_size.height = IMAGE_HEIGHT;

  camera_matrix_ = config_calibration["camera_matrix"].as<std::vector<double>>();
  dist_coeffs_ = config_calibration["dist_coeffs"].as<std::vector<double>>();
  homography_matrix_ = config_calibration["homography"].as<std::vector<double>>();

  camera_matrix = cv::Mat(3, 3, CV_64F, camera_matrix_.data()).clone();
  dist_coeffs = cv::Mat(dist_coeffs_.size(), 1, CV_64F, dist_coeffs_.data()).clone();
  homography_matrix = cv::Mat(3, 3, CV_64F, homography_matrix_.data()).clone();

  if (FISH_EYE == true)
  {
    cv::fisheye::estimateNewCameraMatrixForUndistortRectify(camera_matrix, dist_coeffs, image_size, cv::Mat(),
                                                            new_camera_matrix, 0.0);
    cv::fisheye::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), new_camera_matrix, image_size, CV_32FC1,
                                        map_x, map_y);
  }
  else
  {
    new_camera_matrix = cv::getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, image_size, 0.0).clone();
    cv::initUndistortRectifyMap(camera_matrix, dist_coeffs, cv::Mat(), new_camera_matrix, image_size, CV_32FC1, map_x,
                                map_y);
  }

  //findchessboard_flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK;
  //calibration_fisheye_flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC + cv::fisheye::CALIB_CHECK_COND + cv::fisheye::CALIB_FIX_SKEW;
  
}

DepthEstimator::~DepthEstimator()
{ }

void DepthEstimator::run()
{
  while (detection_sub_.getNumPublishers() == 0 || image_sub_.getNumPublishers() == 0)
  {}

  while (ros::ok())
  {
    ros::spinOnce();
    if (frame_.empty() || balls.empty())
    {
      continue;
    }

    std::cout << "new_camera_matrix : \n" << new_camera_matrix << std::endl;

    for (auto& ball : balls)
    {
      cv::Mat image_point = (cv::Mat_<double>(3, 1) << static_cast<double>((ball.xmin + ball.xmax) / 2), static_cast<double>(ball.ymax), 1.0);
      cv::Mat object_point = homography_matrix * image_point;
      object_point.at<double>(0) /= object_point.at<double>(2);
      object_point.at<double>(1) /= object_point.at<double>(2);
      object_point.at<double>(2) = 1.0;

      std::cout << "location : \n" << object_point << "\n\n";
    }
  }
}

void DepthEstimator::imageCallback(const sensor_msgs::Image& msg)
{
  cv::Mat src = cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<uint8_t*>(&msg.data[0]), msg.step);
  cv::cvtColor(src, frame_, cv::COLOR_RGB2BGR);
}

void DepthEstimator::detectionCallback(const yolov3_trt_ros::BoundingBoxes& msg) 
{
  balls.clear();
  for (auto& box : msg.bounding_boxes) {
    yolov3_trt_ros::BoundingBox cur_box;
    cur_box.probability = box.probability;
    cur_box.id = box.id;
    cur_box.xmin = box.xmin * 640 / 416;
    cur_box.ymin = box.ymin * 480 / 416;
    cur_box.xmax = box.xmax * 640 / 416;
    cur_box.ymax = box.ymax * 480 / 416;
    
    if (cur_box.xmin < 0 || cur_box.xmin > 639 ||
        cur_box.ymin < 0 || cur_box.ymin > 479 ||
        cur_box.xmax < 0 || cur_box.xmax > 639 ||
        cur_box.ymax < 0 || cur_box.ymax > 479)
    {
      continue;
    }
    
    balls.push_back(cur_box);
  }

  std::sort(balls.begin(), balls.end(), compare_point);
}

} /* namespace xycar */
