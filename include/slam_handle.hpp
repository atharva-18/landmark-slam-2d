/*
 * Copyright (C) 2021 Atharva Pusalkar
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef LANDMARK_SLAM_2D_SLAM_HANDLE_HPP
#define LANDMARK_SLAM_2D_SLAM_HANDLE_HPP

#include <deque>
#include <string>
#include <vector>

#include <eigen3/Eigen/Core>

#include "geometry_msgs/msg/polygon.hpp"
#include <geometry_msgs/msg/pose2_d.h>
#include <geometry_msgs/msg/twist.h>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "slam.hpp"

namespace ns_slam {

class Command {
  public:
    double vx, vy, dtheta;
};

class SlamHandle : public rclcpp::Node
{

 public:
  // Constructor
  SlamHandle();

  // Getters
  int getNodeRate() const;

  // Methods
  void loadParameters();
  void subscribeToTopics();
  void publishToTopics();
  void sendMap();
  void sendState();
  void sendVisualization();

 private:  
  rclcpp::Subscription<geometry_msgs::msg::Polygon>::SharedPtr landmarkDetectionsSubscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr stateEstimateSubscriber_;

  rclcpp::Publisher<geometry_msgs::msg::Polygon>::SharedPtr slamMapPublisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr slamMapRvizPublisher_;
  rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr slamStatePublisher_;

  void landmarkDetectionsCallback(const geometry_msgs::msg::Polygon::SharedPtr landmarks);
  void stateEstimateCallback(const geometry_msgs::msg::Twist::SharedPtr state_estimate);

  std::string state_estimation_topic_name_;
  std::string landmark_detections_topic_name_;
  std::string slam_map_topic_name_;
  std::string slam_map_rviz_topic_name_;
  std::string slam_pose_topic_name_;
  int node_rate_;

  Slam slam_;
  std::vector<geometry_msgs::msg::Point32> landmarks_;
  std::vector<geometry_msgs::msg::Point32> received_landmarks_;
  std::vector<geometry_msgs::msg::Point32> corrected_landmarks_;
  geometry_msgs::msg::Twist state_estimate_;
  geometry_msgs::msg::Polygon slam_map_;
  geometry_msgs::msg::Pose2D slam_state_;
  double previous_time_stamp_ = 0.0;
  double current_time_stamp_;

  int n_particles_;

  Eigen::Vector3d z;
  Eigen::Matrix3d R;
  Eigen::Vector2d p;
  std::deque<Command> us;

  double vx_ = 0;
  double vy_ = 0;
  double theta_dt_ = 0;
  double steps_ = 0;
};
}

#endif //ESTIMATION_SLAM_SLAM_HANDLE_HPP
