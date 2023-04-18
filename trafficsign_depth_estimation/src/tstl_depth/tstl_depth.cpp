#include "tstl_depth/tstl_depth.h"

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
    lidar_sub_ = nh_.subscribe("/scan", 1, &DepthEstimator::lidarCallback, this);
  }

  void DepthEstimator::setParams(const YAML::Node &config)
  {
    int IMAGE_WIDTH = config["image_size"]["width"].as<int>();
    int IMAGE_HEIGHT = config["image_size"]["height"].as<int>();
    FISH_EYE = config["fish_eye"].as<bool>();

    image_size.width = IMAGE_WIDTH;
    image_size.height = IMAGE_HEIGHT;
    yolo_image_size = config["yolo_image_size"].as<int>();

    pitch = config["pitch"].as<double>();
    climbing_angle = config["calimbing_angle"].as<double>();

    camera_matrix_ = config["camera_matrix"].as<std::vector<double>>();
    dist_coeffs_ = config["dist_coeffs"].as<std::vector<double>>();
    homography_matrix_ = config["homography"].as<std::vector<double>>();
    rvec_ = config["rvec"].as<std::vector<double>>();
    tvec_ = config["tvec"].as<std::vector<double>>();

    camera_matrix = cv::Mat(3, 3, CV_64F, camera_matrix_.data()).clone();
    dist_coeffs = cv::Mat(dist_coeffs_.size(), 1, CV_64F, dist_coeffs_.data()).clone();
    homography_matrix = cv::Mat(3, 3, CV_64F, homography_matrix_.data()).clone();
    rvec = cv::Mat(3, 1, CV_64F, rvec_.data()).clone();
    tvec = cv::Mat(3, 1, CV_64F, tvec_.data()).clone();

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
      if (frame_.empty())
      {
        continue;
      }
      cv::Mat undist;
      cv::remap(frame_, undist, map_x, map_y, cv::INTER_LINEAR);
      std::vector<cv::Point3f> lidar_xyz;
      lidar_xyz.reserve(505);
      for (int i = 0; i < 505; ++i)
      {
        float dist = lidar_data.ranges[i];

        if (dist < lidar_data.range_min || dist > 3.0)
        {
          continue;
        }

        dist *= 100;
        float theta = lidar_data.angle_min + i * lidar_data.angle_increment;

        if(theta > -1 * std::M_PI_2 && theta < std::M_PI_2 )
        {
          continue;
        }

        float x = std::cos(theta) * dist;
        float y = std::sin(theta) * dist;

        lidar_xyz.push_back(cv::Point3f(x, y, 0));
      }
      std::vector<cv::Point2f> proj_points;

      cv::projectPoints(lidar_xyz, rvec, tvec, new_camera_matrix, cv::Mat(), proj_points);
      for (auto point : proj_points)
      {
        cv::Point2i point_i(point);
        if (point_i.x < 0 || point_i.x >= image_size.width || point_i.y < 0 || point_i.y >= image_size.height)
        {
          continue;
        }
        cv::drawMarker(undist, point_i, cv::Scalar(0, 255, 0), cv::MARKER_DIAMOND, 5, 1);
      }
      std::vector<std::tuple<float, float, int>> xy_sum(bboxes.size());
      for (int i = 0; i < proj_points.size(); ++i)
      {
        for (int j = 0; j < bboxes.size(); ++j)
        {
          if (bboxes[j].xmin <= proj_points[i].x &&
              proj_points[i].x < bboxes[j].xmax &&
              bboxes[j].ymin <= proj_points[i].y &&
              proj_points[i].y < bboxes[j].ymax)
          {
            std::get<0>(xy_sum[j]) += lidar_xyz[i].x;
            std::get<1>(xy_sum[j]) += lidar_xyz[i].y;
            ++std::get<2>(xy_sum[j]);
            break;
          }
        }
      }
      for (int i = 0; i < bboxes.size(); ++i)
      {
        if (std::get<2>(xy_sum[i]) == 0)
        {
          std::cout << "no corresponding points" << std::endl;
          continue;
        }
        float mean_x = std::get<0>(xy_sum[i]) / static_cast<float>(std::get<2>(xy_sum[i]));
        float mean_y = std::get<1>(xy_sum[i]) / static_cast<float>(std::get<2>(xy_sum[i]));

        float xycar_x = -1 * std::cos(pitch) * mean_x - 4;
        float xycar_z = mean_x * std::sin(pitch) - 7;

        float answer_x = xycar_x + xycar_z * std::tan(climbing_angle);
        float answer_y = mean_y;

        std::cout << "id: " << bboxes[i].id << " / location : " << answer_x << ", " << answer_y << ", num points : " << std::get<2>(xy_sum[i]) << std::endl;
      }
      std::cout << std::endl;
    }
  }

  void DepthEstimator::imageCallback(const sensor_msgs::Image &msg)
  {
    cv::Mat src = cv::Mat(msg.height, msg.width, CV_8UC3, const_cast<uint8_t *>(&msg.data[0]), msg.step);
    cv::cvtColor(src, frame_, cv::COLOR_RGB2BGR);
  }

  void DepthEstimator::detectionCallback(const yolov3_trt_ros::BoundingBoxes &msg)
  {
    bboxes.clear();
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
          cur_box.ymin < 0 || cur_box.ymin > image_size.height ||
          cur_box.xmax < 0 || cur_box.xmax > image_size.width ||
          cur_box.ymax < 0 || cur_box.ymax > image_size.height)
      {
        continue;
      }
      bboxes.push_back(cur_box);
    }
    std::sort(bboxes.begin(), bboxes.end(), compare_point);
  }
  void DepthEstimator::lidarCallback(const sensor_msgs::LaserScan &msg)
  {
    lidar_data = msg;
  }
}
