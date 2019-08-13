#include "../include/scan_fining.h"

using namespace std;
using namespace scan_fining;

ScanFiner::ScanFiner() : nh_(""), nh_local_("~") {
  updateParams();

  scan_sub_ = nh_.subscribe(p_scan_topic_name_, 10, &ScanFiner::scanCallback, this);
  costmap_sub_ = nh_.subscribe(p_costmap_topic_name_, 10, &ScanFiner::globalcostmapCallback, this);
  fined_scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>(p_output_topic_name_, 10);

  ROS_INFO("Scan Finer [OK]");
  ros::spin();
}


void ScanFiner::updateParams() {
  nh_local_.param<std::string>("world_frame", p_world_frame_, "map");
  nh_local_.param<std::string>("scanner_frame", p_scanner_frame_, "scannerframe");

  nh_local_.param<std::string>("scan_name", p_scan_topic_name_, "scan");
  nh_local_.param<std::string>("costmap_name", p_costmap_topic_name_, "costmap_node/costmap/costmap");
  nh_local_.param<std::string>("output_name", p_output_topic_name_, "fined_scan");

  nh_local_.param<double>("max_scanner_range", p_max_scanner_range_, 20.0);
}


void ScanFiner::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
  initial_points_.clear();

  double phi = scan->angle_min - scan->angle_increment;

  for (const float r : scan->ranges) {
    phi += scan->angle_increment;

    if (r >= scan->range_min && r <= scan->range_max && r <= p_max_scanner_range_)
      initial_points_.push_back(Point::fromPolarCoords(r, phi));
  }

  if (!costmap_.empty())
    processPoints(scan);
}


void ScanFiner::globalcostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& costmap_input){
  costmap_.clear();
  global_height_ = costmap_input->info.height;
  global_width_ = costmap_input->info.width;
  global_resolution_ = costmap_input->info.resolution;

  for (int j=0; j<global_height_; j++){
    vector<int> costmap_width_temp_; //should be confirmed - temporally allocated vetor from for loop used as a data
    for (int i=0; i<global_width_; i++){
      costmap_width_temp_.push_back(costmap_input->data[i+j*global_width_]);
      //costmap[j][i] = costmap_points_[i+j*width];
    }
    costmap_.push_back(costmap_width_temp_);
  }

  if (debugging_){
    for (vector<int>& vector_ : costmap_){
      for (int i : vector_){
        cout << i << endl;
      }
    }
  }
}


void ScanFiner::processPoints(const sensor_msgs::LaserScan::ConstPtr& scan) {
  transformToWorld();
  maskingScan();
  transformToScan();

  publishFinedScan(scan);
}


void ScanFiner::transformToWorld() {
  temp_points_.clear();

  geometry_msgs::PointStamped point_l;  // Point in local (scanner) coordinate frame
  geometry_msgs::PointStamped point_w;  // Point in global (world) coordinate frame

  point_l.header.stamp = ros::Time::now();
  point_l.header.frame_id = p_scanner_frame_;

  point_w.header.stamp = ros::Time::now();
  point_w.header.frame_id = p_world_frame_;

  try {
    tf_listener_.waitForTransform(p_world_frame_, p_scanner_frame_, ros::Time::now(), ros::Duration(3.0));
    for (const Point& point_ : initial_points_){
      point_l.point.x = point_.x;
      point_l.point.y = point_.y;
      tf_listener_.transformPoint(p_world_frame_, point_l, point_w);
      if (debugging_){
        cout << "local point : " << point_l << endl;
        cout << "global point : " << point_w << endl;
      }
      temp_points_.push_back(Point(point_w.point.x,point_w.point.y));
    }
    if (debugging_){
      cout << "temp_points_ : " << endl;
      for (const Point& point_ : temp_points_){
        cout << point_ << endl;
      }
    }
    geometry_msgs::PointStamped origin_;
    origin_.header.stamp = ros::Time::now();
    origin_.header.frame_id = p_scanner_frame_;
    origin_.point.x = 0;
    origin_.point.y = 0;
    tf_listener_.transformPoint(p_world_frame_, origin_, scanner_origin_to_global_);
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    cout << "not transformed" << endl;
    ros::Duration(0.2).sleep();
  }
}


void ScanFiner::maskingScan(){
  for (Point& point_ : temp_points_){
    int mask_i = point_.x/global_resolution_ + global_width_/2 + 10; //Need to be fixed
    int mask_j = point_.y/global_resolution_ + global_height_/2 + 10; //Need to be fixed
    try{
      if (costmap_[mask_j][mask_i] > 0){
        //cout << costmap_[mask_j][mask_i] << endl;
        //cout << "x: " << point_.x << ", " << "y: " << point_.y << endl;
        //cout << "i: " << mask_i << ", " << "j: " << mask_j << endl;
        point_.x = scanner_origin_to_global_.point.x;
        point_.y = scanner_origin_to_global_.point.y;
      }
    }
    catch(out_of_range& e){}
  }
}


void ScanFiner::transformToScan(){
  initial_points_.clear();

  geometry_msgs::PointStamped point_l;  // Point in local (scanner) coordinate frame
  geometry_msgs::PointStamped point_w;  // Point in global (world) coordinate frame

  point_l.header.stamp = ros::Time::now();
  point_l.header.frame_id = p_scanner_frame_;

  point_w.header.stamp = ros::Time::now();
  point_w.header.frame_id = p_world_frame_;

  try {
    tf_listener_.waitForTransform(p_scanner_frame_, p_world_frame_, ros::Time::now(), ros::Duration(3.0));

    for (const Point& point_ : temp_points_) {
        point_w.point.x = point_.x;
        point_w.point.y = point_.y;
        tf_listener_.transformPoint(p_scanner_frame_, point_w, point_l);
        initial_points_.push_back(Point(point_l.point.x,point_l.point.y));
    }
  }
  catch (tf::TransformException ex) {
    ROS_ERROR("%s",ex.what());
    ros::Duration(0.2).sleep();
  }
}


void ScanFiner::publishFinedScan(const sensor_msgs::LaserScan::ConstPtr& scan) {
  sensor_msgs::LaserScan fined_scan_;
  //std::vector<float> new_arr_;
  fined_scan_.header.stamp = ros::Time::now();
  fined_scan_.header.frame_id = p_scanner_frame_;
  fined_scan_.angle_min = scan->angle_min;
  fined_scan_.angle_max = scan->angle_max;
  fined_scan_.angle_increment = scan->angle_increment;
  fined_scan_.range_min = scan->range_min;
  fined_scan_.range_max = scan->range_max;

  for (const Point& point_ : initial_points_){
    //new_arr_.push_back(point_.length());
    fined_scan_.ranges.push_back(point_.length());
  }

  fined_scan_pub_.publish(fined_scan_);
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "scan_finer");
  ScanFiner sf;
  return 0;
}
