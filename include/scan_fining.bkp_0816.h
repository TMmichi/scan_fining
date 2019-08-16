#pragma once

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>

#include "../include/point.h"

namespace scan_fining
{

class ScanFiner
{
public:
  ScanFiner();

private:
  void updateParams();
  void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan);
  void globalcostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap);
  void processPoints(const sensor_msgs::LaserScan::ConstPtr& scan);
  void maskingScan();
  void transformToWorld();
  void transformToScan();
  void publishFinedScan(const sensor_msgs::LaserScan::ConstPtr& scan);

  // ROS handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_local_;

  //Subscribers, Publishers
  ros::Subscriber scan_sub_;
  ros::Subscriber costmap_sub_;
  ros::Publisher  fined_scan_pub_;

  //Transform class
  tf::TransformListener tf_listener_;

  // Parameters
  std::string p_world_frame_;     // Name of the world coordinate frame
  std::string p_scanner_frame_;   // Name of the scanner coordinate frame

  std::string p_scan_topic_name_;
  std::string p_costmap_topic_name_;
  std::string p_output_topic_name_;

  double p_max_scanner_range_;    // Restrictions on laser scanner

  bool debugging_ = false;

  // Variables
  std::vector<scan_fining::Point> initial_points_;
  std::vector<std::vector<int>> costmap_;
  int global_height_;
  int global_width_;
  float global_resolution_;
  std::vector<scan_fining::Point> temp_points_;
  //std::vector<scan_fining::Point> costmap_points_;
  geometry_msgs::PointStamped scanner_origin_to_global_;
};

} // namespace obstacle_detector
