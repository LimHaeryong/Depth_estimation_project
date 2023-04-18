#include "depth_estimation/depth_estimator.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "Depth Estimator");
  xycar::DepthEstimator depth_estimator;
  depth_estimator.run();

  return 0;
}
