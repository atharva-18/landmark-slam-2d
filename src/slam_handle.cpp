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

#include <rclcpp/rclcpp.hpp>
#include <slam_handle.hpp>

namespace ns_slam {

// Constructor
SlamHandle::SlamHandle() : Node("landmark_slam_2d")
{
  loadParameters();
  slam_.setParameters(n_particles_);
  subscribeToTopics();
  publishToTopics();
}

// Getters
int SlamHandle::getNodeRate() const { return node_rate_; }

// Methods
void SlamHandle::loadParameters() {
  this->declare_parameter("landmark_detections_topic_name");
  this->declare_parameter("slam_map_topic_name");
  this->declare_parameter("node_rate");
  this->declare_parameter("slam_map_rviz_topic_name");
  this->declare_parameter("state_estimation_topic_name");
  this->declare_parameter("particles_topic_name");
  this->declare_parameter("observations_topic_name");
  this->declare_parameter("n_particles");
  this->declare_parameter("mh_threshold");

  this->get_parameter_or("landmark_detections_topic_name", landmark_detections_topic_name_, std::string("/perception/landmark_detections"));
  this->get_parameter_or("slam_map_topic_name", slam_map_topic_name_, std::string("/estimation/slam/map"));
  this->get_parameter_or("node_rate", node_rate_, 10);
  this->get_parameter_or("slam_map_rviz_topic_name", slam_map_rviz_topic_name_, std::string("/estimation/visualization/map"));
  this->get_parameter_or("state_estimation_topic_name", state_estimation_topic_name_, std::string("/estimation/slam/state"));
  this->get_parameter_or("slam_pose_topic_name", slam_pose_topic_name_, std::string("/estimation/slam/pose"));
  this->get_parameter_or("n_particles", n_particles_, 100);

  // Velocity covariance
  R << 0.01, 0, 0,
       0, 0.01, 0,
       0, 0, 0.01;
}

void SlamHandle::subscribeToTopics() {
  auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());
  landmarkDetectionsSubscriber_ = this->create_subscription<geometry_msgs::msg::Polygon>(landmark_detections_topic_name_, default_qos, std::bind(&SlamHandle::landmarkDetectionsCallback, this, std::placeholders::_1));
  stateEstimateSubscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(state_estimation_topic_name_, default_qos, std::bind(&SlamHandle::stateEstimateCallback, this, std::placeholders::_1));
}

void SlamHandle::publishToTopics() {
  slamMapPublisher_ = this->create_publisher<geometry_msgs::msg::Polygon>(slam_map_topic_name_, 1);
  slamMapRvizPublisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(slam_map_rviz_topic_name_, 1);
  slamStatePublisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>(slam_pose_topic_name_, 1);
}

void SlamHandle::sendMap() {
  slam_state_.x = slam_.getState().x;
  slam_state_.y = slam_.getState().y;
  slam_state_.theta = slam_.getState().theta;

  slam_map_.points = std::move(slam_.getMap());
  slamMapPublisher_->publish(slam_map_);
  slamStatePublisher_->publish(slam_state_);
}

void SlamHandle::landmarkDetectionsCallback(const geometry_msgs::msg::Polygon::SharedPtr landmarks) {
  landmarks_ = std::move(landmarks->points);

  z << vx_ / (steps_ * 1.0), vy_ / (steps_ * 1.0), theta_dt_ / (steps_ * 1.0);

  auto current_time_stamp_ = this->get_clock()->now().seconds();
  const double dt = current_time_stamp_ - previous_time_stamp_;
  previous_time_stamp_ = current_time_stamp_;

  slam_.predictParticles(z, R, dt);
  slam_.updateLandmarks(landmarks_, false);
  slam_.resample();
  slam_.calcFinalState();
  slam_.createMap();
  
  sendMap();
  sendVisualization();

  steps_ = 0;
}

void SlamHandle::stateEstimateCallback(const geometry_msgs::msg::Twist::SharedPtr state_estimate) {
  state_estimate_ = std::move(*state_estimate);

  Command u;
  u.vx = state_estimate_.linear.x;
  u.vy = state_estimate_.linear.y;
  u.dtheta = state_estimate_.angular.y;
  us.push_back(u);

  double c = std::cos(slam_state_.theta);
  double s = std::sin(slam_state_.theta);

  u.vx = u.vx * c - u.vy * s;
  u.vy = u.vx * s + u.vy * c;

  if(steps_ == 0) {
    vx_ = u.vx;
    vy_ = u.vy;
    theta_dt_ = u.dtheta;
  }else {
    vx_ += u.vx;
    vy_ += u.vy;
    theta_dt_ += u.dtheta;
  }

  steps_++;

  if(us.size() > 100)
    us.pop_front();

}

void SlamHandle::sendVisualization() {
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker;

  marker.pose.orientation.w = 1.0;
  marker.type               = visualization_msgs::msg::Marker::SPHERE;
  marker.action             = visualization_msgs::msg::Marker::ADD;
  marker.id                 = 1;
  marker.scale.x            = 0.5;
  marker.scale.y            = 0.5;
  marker.scale.z            = 0.5;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.header.stamp       = this->get_clock()->now();
  marker.header.frame_id    = "map";
  marker.color.a            = 1.0;
  
  int id_cnt = 2;

  marker.color.r       = 0.0;
  marker.color.g       = 0.0;
  marker.color.b       = 1.0;
  for(long unsigned int i = 0; i < slam_map_.points.size();i++) {
    marker.id = id_cnt;
    marker.pose.position.x = slam_map_.points[i].x;
    marker.pose.position.y = slam_map_.points[i].y;
    markers.markers.push_back(marker);
    id_cnt++;
  }

  slamMapRvizPublisher_->publish(markers);
}

}
