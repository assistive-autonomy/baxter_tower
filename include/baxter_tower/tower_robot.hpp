/*
 * @file      tower_robot.hpp
 * @brief     Robot code for tower building
 * @author    Alejandro Bordallo <alex.bordallo@ed.ac.uk>
 * @date      2016-03-10
 * @copyright (MIT) 2016 RAD-UoE Informatics
*/

#ifndef TOWER_ROBOT_HPP
#define TOWER_ROBOT_HPP

#include <baxter_cpp/baxter_robot.hpp>
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include <std_msgs/UInt8.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/Image.h>
#include <baxter_core_msgs/DigitalIOState.h>
#include <apriltags_ros/AprilTagDetectionArray.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

class TowerRobot {
 public:
  enum FindState {find, search, approach, closein, find_end};
  enum PickState {grip, pickup, check, pick_end};
  enum PlaceState {above, place, retract, reabove, place_end};

  explicit TowerRobot(ros::NodeHandle* nh);
  ~TowerRobot();

  void LoadParams();
  void Init();
  void RosSetup();

  void InitRobot();
  void RunDemo();

  bool FindCube();
  bool PickCube();
  bool PlaceCube();

  void UpdateFace(const std::string& face_name);

  void TagDetectionsCB(
    const apriltags_ros::AprilTagDetectionArray::ConstPtr& msg);
  void ResetDemoCB(const baxter_core_msgs::DigitalIOState::ConstPtr& msg) {
    if (msg->state == 1) {
      reset_ = true;
      blurr_.SetLED("torso_" + side_ + "_outer_light", true);
    }
  }
  void PauseDemoCB(const baxter_core_msgs::DigitalIOState::ConstPtr& msg) {
    (msg->state == 1) ? pause_ = true : pause_ = false;
  }
  void CuffStateCB(const baxter_core_msgs::DigitalIOState::ConstPtr& msg) {
    // if (msg->state == 1) 
    // {
    //   ROS_INFO("HEY!");
    //   UpdateFace("confused");
    //   ros::spinOnce();
    //   rate_.sleep();
    // }
  }

  // void RightRangeCB(const sensor_msgs::Range::ConstPtr& msg);

 private:
  // ROS
  ros::NodeHandle* nh_;
  ros::Rate rate_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Subscriber tag_detection_sub_;
  ros::Subscriber reset_demo_sub_;
  ros::Subscriber pause_demo_sub_;
  ros::Subscriber cuff_state_sub_;
  // ros::Subscriber right_range_sub_;]
  ros::Publisher face_image_pub_;

  // Parameters
  std::vector<int> cubes_;
  std::string arm_name_;
  int exposure_;
  int gain_;
  int explore_poses_;
  double search_time_;
  double pick_x_offset_, pick_y_offset_, pick_z_offset_;
  double place_x_offset_, place_y_offset_, place_z_offset_;

  // Default Parameters
  std::string tower_frame_;
  double sleep_time_;
  double cube_size_;
  double find_z_offset_;
  double approach_z_offset_;
  double pull_z_offset_;

  // Robot
  BaxterRobot blurr_;
  BaxterArm* arm_;
  BaxterRobot::ArmSide arm_side_;
  std::map<std::string, int> cam_settings_;
  std::string gripper_frame_;
  std::string side_;

  // sensor_msgs::Range right_arm_range_;

  // Flags
  bool reset_, pause_, paused_;

  bool cube_selected_, grip_calculated_, pickup_calculated_;

  FindState find_state_;
  PickState pick_state_;
  PlaceState place_state_;
  bool seen_tower_;

  // Variables
  std::stringstream target_frame_;
  geometry_msgs::TransformStamped cube_tf_;
  tf2::Transform approach_offset_;
  baxter_core_msgs::JointCommand approach_pose_;

  tf2::Transform grip_offset_;
  baxter_core_msgs::JointCommand grip_pose_;
  tf2::Transform pickup_offset_;
  baxter_core_msgs::JointCommand pickup_pose_;

  tf2::Transform place_offset_;
  tf2::Transform tower_offset_;
  baxter_core_msgs::JointCommand tower_pose_;
  geometry_msgs::TransformStamped tower_tf_;

  cv_bridge::CvImagePtr cv_ptr_;



  // Other
  int target_cube_;
  int stacked_cubes_;
  apriltags_ros::AprilTagDetectionArray tag_array_;
  std::map<int, int> cube_status_; // Cube, Stacked/Not Stacked (1/0)
};

#endif /* TOWER_ROBOT_HPP */
