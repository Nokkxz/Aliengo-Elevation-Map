
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <lcm/lcm-cpp.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.h>

#include "perception/height_map_t.hpp"
#include "perception/robot_location_t.hpp"
#include "perception/leg_control_data_lcmt.hpp"
#include "csignal"

#include <thread>


class Communication
{
private:
  ros::NodeHandle nh_;
  ros::Timer timer_;
  ros::Timer timer2_;
  ros::Subscriber pose_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber pc_sub_;
  ros::Publisher pose_pub_;
  tf::TransformBroadcaster tf_bc_;

  geometry_msgs::PoseWithCovarianceStamped pose_;
  grid_map::GridMap map_;

  lcm::LCM lcm_;
  robot_location_t lcm_loc_;
  height_map_t lcm_map_;
  leg_control_data_lcmt lcm_control_data_;

  int map_cnt_;
  int pc_cnt_;
  int loc_cnt_;

  std::string robot_desc_string_;
  KDL::Tree kdl_tree_;
  robot_state_publisher::RobotStatePublisher* rs_pub_p_;

public:
  Communication()
  {
    timer_ = nh_.createTimer(ros::Duration(1), &Communication::handle_timer, this);
    timer2_ = nh_.createTimer(ros::Duration(0.01), &Communication::handle_timer2, this);
    pose_sub_ = nh_.subscribe("/t265/odom/sample", 10, &Communication::handle_pose_sub, this);
    map_sub_ = nh_.subscribe("/elevation_mapping/elevation_map_raw", 10, &Communication::handle_map_sub, this);
    pc_sub_ = nh_.subscribe("/d400/depth/color/points", 10, &Communication::handle_pc_sub, this);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/sensor_pose", 10);
    map_cnt_ = 0;
    pc_cnt_ = 0;
    loc_cnt_ = 0;

    nh_.param("robot_description", robot_desc_string_, std::string());
    if (!kdl_parser::treeFromString(robot_desc_string_, kdl_tree_)){
      ROS_ERROR("Failed to construct kdl tree");
    }
    rs_pub_p_ = new robot_state_publisher::RobotStatePublisher(kdl_tree_);

  }

  void handle_pose_sub(const nav_msgs::Odometry::ConstPtr& msg)
  {
    pose_.header = msg->header;
    pose_.header.frame_id = "sensor_pose";
    pose_.pose = msg->pose;
    pose_pub_.publish(pose_);

    lcm_loc_.pos_vo[0] = msg->pose.pose.position.x;
    lcm_loc_.pos_vo[1] = msg->pose.pose.position.y;
    lcm_loc_.pos_vo[2] = msg->pose.pose.position.z;
    lcm_loc_.quat[0] = msg->pose.pose.orientation.w;
    lcm_loc_.quat[1] = msg->pose.pose.orientation.x;
    lcm_loc_.quat[2] = msg->pose.pose.orientation.y;
    lcm_loc_.quat[3] = msg->pose.pose.orientation.z;
    lcm_.publish("robot_location", &lcm_loc_);
  }

  void handle_map_sub(const grid_map_msgs::GridMap msg)
  {
    grid_map::GridMapRosConverter::fromMessage(msg, map_);

    lcm_map_.map_center[0] = map_.getPosition()(0);
    lcm_map_.map_center[1] = map_.getPosition()(1);
    lcm_map_.resolution = map_.getResolution();
    memcpy(&lcm_map_.ele_map[0][0], map_.get("elevation_inpainted").data(), sizeof(lcm_map_.ele_map));
    memcpy(&lcm_map_.cost_map[0][0], map_.get("traversability").data(), sizeof(lcm_map_.cost_map));

    lcm_.publish("height_map", &lcm_map_);
  }

  void handle_pc_sub(const sensor_msgs::PointCloud2)
  {
    pc_cnt_++;
  }

  void handle_timer(const ros::TimerEvent&)
  {
    ROS_INFO("FPS\tpc:%d\tmap:%d\tloc:%d", pc_cnt_,map_cnt_,loc_cnt_);
    map_cnt_ = 0;
    pc_cnt_ = 0;
    loc_cnt_ = 0;
  }

  void handle_timer2(const ros::TimerEvent&)
  {
    float thigh = -0.6, calf = 1.15;
    lcm_control_data_.q[0] = 0;
    lcm_control_data_.q[1] = thigh;
    lcm_control_data_.q[2] = calf;
    lcm_control_data_.q[3] = 0;
    lcm_control_data_.q[4] = thigh;
    lcm_control_data_.q[5] = calf;
    lcm_control_data_.q[6] = 0;
    lcm_control_data_.q[7] = thigh;
    lcm_control_data_.q[8] = calf;
    lcm_control_data_.q[9] = 0;
    lcm_control_data_.q[10] = thigh;
    lcm_control_data_.q[11] = calf;
    lcm_.publish("leg_control_data", &lcm_control_data_);
  }

  void handle_joint_positions(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const leg_control_data_lcmt* msg)
  {
    std::map<std::string, double> joint_pos;
    joint_pos.insert(std::make_pair("FR_hip_joint", msg->q[0]));
    joint_pos.insert(std::make_pair("FR_thigh_joint", -msg->q[1]));
    joint_pos.insert(std::make_pair("FR_calf_joint", -msg->q[2]));
    joint_pos.insert(std::make_pair("FL_hip_joint", msg->q[3]));
    joint_pos.insert(std::make_pair("FL_thigh_joint", -msg->q[4]));
    joint_pos.insert(std::make_pair("FL_calf_joint", -msg->q[5]));
    joint_pos.insert(std::make_pair("RR_hip_joint", msg->q[6]));
    joint_pos.insert(std::make_pair("RR_thigh_joint", -msg->q[7]));
    joint_pos.insert(std::make_pair("RR_calf_joint", -msg->q[8]));
    joint_pos.insert(std::make_pair("RL_hip_joint", msg->q[9]));
    joint_pos.insert(std::make_pair("RL_thigh_joint", -msg->q[10]));
    joint_pos.insert(std::make_pair("RL_calf_joint", -msg->q[11]));
    rs_pub_p_->publishTransforms(joint_pos, ros::Time::now(), "");
  }

  void handle_robot_location(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const robot_location_t* msg)
  {
    loc_cnt_++;
  }

  void handle_height_map(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const height_map_t* msg)
  {
    map_cnt_++;
  }

  ~Communication()
  {
    rs_pub_p_-> ~RobotStatePublisher();
  }
};

void handle_signal(int signal){
  ros::shutdown();
  exit(0);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "communication_node");
  Communication communication;
  signal(SIGINT, handle_signal);
  signal(SIGTERM, handle_signal);
  lcm::LCM lcm;
  lcm.subscribe("leg_control_data", &Communication::handle_joint_positions, &communication);
  lcm.subscribe("robot_location", &Communication::handle_robot_location, &communication);
  lcm.subscribe("height_map", &Communication::handle_height_map, &communication);

  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  while(ros::ok() && 0==lcm.handle());
  return 0;
}
