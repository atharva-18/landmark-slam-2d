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

#ifndef LANDMARK_SLAM_2D_SLAM_HPP
#define LANDMARK_SLAM_2D_SLAM_HPP

#include <algorithm>
#include <vector>

#include <eigen3/Eigen/Core>

#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

namespace ns_slam {

class Jacobians;
class Particle;
class Landmark;

class Jacobians {
  public:
    Eigen::Vector2d zp;
    Eigen::MatrixXd Hv;
    Eigen::Matrix2d Hf;
    Eigen::Matrix2d Sf;

};

class Landmark {
  public:
    // Constructor
    Landmark(Eigen::Vector2d x_init, Eigen::Matrix2d Q, Particle* particle);

    // Methods
    void update(Eigen::Vector2d z, Eigen::Matrix2d Q, Particle* particle);
    double pdf(const Eigen::Vector2d& z) const;
    double mahalanobis(const Eigen::Vector2d& z) const;

    Eigen::Vector2d x;
    Eigen::Matrix2d P;

    int n_observed;
};

class Particle {
  public:
    // Constructor
    Particle(int n_particles);
    
    void predict(Eigen::Vector3d u, double dt);
    double computeWeight(int id, Eigen::Vector2d z, Eigen::Matrix2d Q);
    void proposalSampling(int id, Eigen::Vector2d z, Eigen::Matrix2d Q);

    double w;
    double x;
    double y;
    double yaw;

    Eigen::Matrix3d P;

    std::vector<Landmark> landmarks;
};

class Slam {

 public:
  // Constructor
  Slam();

  // Getters
  std::vector<geometry_msgs::msg::Point32> getMap() const;
  geometry_msgs::msg::Pose2D getState() const;

  /**
   *  initializes the cone map
   */
  void initializeMap();

  /**
   *  initializes the state
   */
  void initializeState();

  /**
   *  updates the cone map
   */
  void updateMap(const std::vector<geometry_msgs::msg::Point32> &cones);

  /**
   *  updates the landmark map
   */
  void updateLandmarks(const std::vector<geometry_msgs::msg::Point32> &cones, bool frozen_update);

  /**
   *  Creates cone map based on landmarks
   */
  void createMap();

  /**
   *  calculates the new car state
   */
  void calculateState(const geometry_msgs::msg::Twist &velocity);

  /**
   *  calls the other functions in the right order
   */
  void runAlgorithm();

  /*
   * sets fastSLAM parameters
   */
  void setParameters(int n_particles, double mh_threshold);

  /*
   * Predict particles using motion model
   */
  void predictParticles(Eigen::Vector3d u, Eigen::Matrix3d R, double dt);

  /*
   * Normalizes particle weights
   */
  void normalizeWeight();

  /*
   * Resample using SIR filter
   */
  void resample();

  /*
   * Calculates final state of the particles
   */
  void calcFinalState();

  std::vector<Landmark> landmark_map_;
  std::vector<Particle> particles_;
  std::vector<geometry_msgs::msg::Point32> cone_map_;
  geometry_msgs::msg::Pose2D slam_state_;
  int max_map_size_;

  int n_particles_;
  int n_resample_;
  double mh_threshold_;
  int best_id_ = 0;
};

double pi_2_pi(double angle);

Jacobians computeJacobians(Particle* particle, Eigen::Vector2d xf, Eigen::Matrix2d Pf, Eigen::Matrix2d Q_cov);
}
#endif //ESTIMATION_SLAM_SLAM_HPP
