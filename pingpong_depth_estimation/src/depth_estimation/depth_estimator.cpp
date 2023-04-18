#include "depth_estimation/depth_estimator.h"

static bool compare_point(const yolov3_trt_ros::BoundingBox &box1, const yolov3_trt_ros::BoundingBox &box2)
{
  return box1.xmin < box2.xmin;
}

namespace xycar
{
  DepthEstimator::DepthEstimator()
  {
    std::string config_path;
    nh_.getParam("config_path", config_path);
    YAML::Node config = YAML::LoadFile(config_path);

    setParams(config);
    image_sub_ = nh_.subscribe("/usb_cam/image_raw/", 1, &DepthEstimator::imageCallback, this);
    detection_sub_ = nh_.subscribe("/yolov3_trt_ros/detections", 1, &DepthEstimator::detectionCallback, this);
  }

  void DepthEstimator::setParams(const YAML::Node &config)
  {
    int IMAGE_WIDTH = config["image_size"]["width"].as<int>();
    int IMAGE_HEIGHT = config["image_size"]["height"].as<int>();
    FISH_EYE = config["fish_eye"].as<bool>();

    image_size.width = IMAGE_WIDTH;
    image_size.height = IMAGE_HEIGHT;
    yolo_image_size = config["yolo_image_size"].as<int>();

    camera_matrix_ = config["camera_matrix"].as<std::vector<double>>();
    dist_coeffs_ = config["dist_coeffs"].as<std::vector<double>>();
    homography_matrix_ = config["homography"].as<std::vector<double>>();

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
  }

  DepthEstimator::~DepthEstimator()
  {
  }

  void DepthEstimator::run()
  {
    while (detection_sub_.getNumPublishers() == 0)
    {
    }

    while (ros::ok())
    {
      ros::spinOnce();
      if (balls.empty())
      {
        continue;
      }

      // std::cout << "new_camera_matrix : \n" << new_camera_matrix << std::endl;

      std::vector<int> xindex;
      xindex.reserve(3);
      std::vector<std::pair<double, double>> points;
      points.reserve(3);

      for (auto &ball : balls)
      {
        double dx = (ball.xmax - ball.xmin) / 2;
        cv::Mat image_point = (cv::Mat_<double>(3, 1) << static_cast<double>((ball.xmin + ball.xmax) / 2), static_cast<double>((ball.ymin + ball.ymax) / 2 + dx), 1.0);
        cv::Mat object_point = homography_matrix * image_point;

        object_point.at<double>(0) /= object_point.at<double>(2);
        object_point.at<double>(1) /= object_point.at<double>(2);
        object_point.at<double>(2) = 1.0;

        xindex.push_back(static_cast<int>((object_point.at<double>(0) + 22.5) / 45.0));
        points.push_back(std::make_pair(object_point.at<double>(0), object_point.at<double>(1)));
      }

      for (int i = 0; i < points.size(); ++i)
      {
        std::cout << static_cast<int>(points[i].first) << ',' << static_cast<int>(points[i].second);
        if (i < points.size() - 1)
        {
          std::cout << ", ";
        }
      }
      std::cout << std::endl;

      cv::imshow("frame", frame_);
      // cv::imwrite("/home/nvidia/contest_image/data_" + std::to_string(ros::Time::now().toSec()) + ".png", frame_);

      // while (getchar() != '\n')
      // {
      // }
    }
  }

  void DepthEstimator::imageCallback(const sensor_msgs::Image &msg)
  {
    cv::Mat src = cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(&msg.data[0]), msg.step);
    cv::cvtColor(src, frame_, cv::COLOR_RGB2BGR);
  }

  void DepthEstimator::detectionCallback(const yolov3_trt_ros::BoundingBoxes &msg)
  {
    balls.clear();
    for (auto &box : msg.bounding_boxes)
    {
      yolov3_trt_ros::BoundingBox cur_box;
      cur_box.probability = box.probability;
      cur_box.id = box.id;
      cur_box.xmin = box.xmin * image_size.width / yolo_image_size;
      cur_box.ymin = box.ymin * image_size.height / yolo_image_size;
      cur_box.xmax = box.xmax * image_size.width / yolo_image_size;
      cur_box.ymax = box.ymax * image_size.height / yolo_image_size;

      if (cur_box.xmin < 0 || cur_box.xmin >= image_size.width ||
          cur_box.ymin < 0 || cur_box.ymin >= image_size.height ||
          cur_box.xmax < 0 || cur_box.xmax >= image_size.width ||
          cur_box.ymax < 0 || cur_box.ymax >= image_size.height)
      {
        continue;
      }

      balls.push_back(cur_box);
    }

    std::sort(balls.begin(), balls.end(), compare_point);
  }

} /* namespace xycar */
