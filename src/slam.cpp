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

#include <slam.hpp>
#include <slam_handle.hpp>

#include <sstream>
#include <random>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Cholesky>

namespace ns_slam {

double pi_2_pi(double angle) {
  return std::fmod(std::fmod((angle)+M_PI, 2*M_PI)-2*M_PI, 2*M_PI)+M_PI;
}

Jacobians computeJacobians(Particle *particle, Eigen::Vector2d xf, Eigen::Matrix2d Pf, Eigen::Matrix2d Q_cov) {
  Jacobians j;
  j.Hv = Eigen::MatrixXd(2,3);

  double dx = xf(0) - particle->x;
  double dy = xf(1) - particle->y;
  double d2 = std::pow(dx, 2) + std::pow(dy, 2);
  double d = std::sqrt(d2);

  j.zp << d, pi_2_pi(std::atan2(dy, dx) - particle->yaw);

  j.Hv << -dx / d, -dy / d, 0.0,
           dy / d2, -dx / d2, -1.0;

  j.Hf << dx / d, dy / d,
          -dy / d2, dx / d2;

  j.Sf = j.Hf * Pf * j.Hf.transpose() + Q_cov;
  return j;
}

// Constructor
Landmark::Landmark(Eigen::Vector2d z, Eigen::Matrix2d Q, Particle *particle) {
  double r = z(0);
  double b = z(1);
  
  double c = std::cos(pi_2_pi(particle->yaw + b));
  double s = std::sin(pi_2_pi(particle->yaw + b));
  this->x[0] = particle->x + r * c;
  this->x[1] = particle->y + r * s;
  
  double dx = r * c;
  double dy = r * s;
  double d2 = std::pow(dx, 2) + std::pow(dy, 2);
  double d = std::sqrt(d2);

  Eigen::Matrix2d Gz;
  Gz << dx / d, dy / d,
        -dy / d2, dx / d2;
  
  this->P = Gz.inverse() * Q * Gz.transpose().inverse();

  this->n_observed = 0;
}

// Multivariate Gaussian Update from measurements
void Landmark::update(Eigen::Vector2d z, Eigen::Matrix2d Q, Particle *particle) {  
  Jacobians j = computeJacobians(particle, this->x, this->P, Q);

  Eigen::Vector2d dz;
  dz << z(0) - j.zp(0), pi_2_pi(z(1) - j.zp(1));

  auto Pht = this->P * j.Hf.transpose();
  auto Sf = j.Hf * Pht + Q;
  auto Sfa = (Sf + Sf.transpose()) * 0.5;
  Eigen::Matrix2d Schol(Sfa.llt().matrixL().transpose());
  auto Sinv = Schol.inverse();
  auto W1 = Pht * Sinv;
  auto W = W1 * Sinv.transpose();

  this->x = this->x + W * dz;
  this->x = this->x.transpose();
  this->P = this->P - W1 * W1.transpose();

  this->n_observed++;
}

// Get PDF for a given z value from MVN
double Landmark::pdf(const Eigen::Vector2d &z) const {
  double n = z.rows();
  double sqrt2pi = std::sqrt(2 * M_PI);
  double quadform  = (z - this->x).transpose() * P.inverse() * (z - this->x);
  double norm = std::pow(sqrt2pi, - n) *
                std::pow(this->P.determinant(), -0.5);

  return norm * exp(-0.5 * quadform);
}

// Compute Mahalanobis distance between a point and a multivariate distribution
double Landmark::mahalanobis(const Eigen::Vector2d &z) const {
  return std::sqrt((this->x - z).transpose() * this->P * (this->x - z));
}

// Constructor
Particle::Particle(int n_particles) {
  this->w = 1.0 / n_particles;
  this->x = 0.0;
  this->y = 0.0;
  this->yaw = 0.0;
  this->P << 0.1, 0, 0,
             0, 0.1, 0,
             0, 0, 0.1;
}

void Particle::predict(Eigen::Vector3d u, double dt) {
  this->x = this->x + u(0) * dt;
  this->y = this->y + u(1) * dt;
  this->yaw = this->yaw + u(2) * dt;

  this->yaw = pi_2_pi(this->yaw);
}

double Particle::computeWeight(int id, Eigen::Vector2d z, Eigen::Matrix2d Q) {
  double weight;
  
  auto xf = this->landmarks[id].x;
  auto Pf = this->landmarks[id].P;

  Jacobians j = computeJacobians(this, xf, Pf, Q);

  Eigen::Vector2d dz;
  dz << z(0) - j.zp(0), pi_2_pi(z(1) - j.zp(1));
  
  Eigen::Matrix2d invS;
  
  try {
    invS = j.Sf.inverse();
  }catch(...) {
    return 1.0;
  }

  double num = std::exp(-0.5 * dz.transpose() * invS * dz);
  double den = 2.0 * M_PI * std::sqrt(j.Sf.determinant());

  if(std::isinf(num))
    return 1.0;

  weight = num / den;

  return weight;
}

void Particle::proposalSampling(int id, Eigen::Vector2d z, Eigen::Matrix2d Q) {
  auto xf = this->landmarks[id].x;
  auto Pf = this->landmarks[id].P;

  Jacobians j = computeJacobians(this, xf, Pf, Q);

  Eigen::Matrix2d Sinv = j.Sf.inverse();

  Eigen::Vector2d dz;
  dz << z(0) - j.zp(0), pi_2_pi(z(1) - j.zp(1));

  auto Pi = this->P.inverse();
  
  this->P = (j.Hv.transpose() * Sinv * j.Hv + Pi).inverse();
  auto dx = this->P * j.Hv.transpose() * Sinv * dz;

  this->x += dx[0];
  this->y += dx[1];
  this->yaw += dx[2];

}  

// Constructor
Slam::Slam() {
  initializeState();
  best_id_ = 0;
}

// Getters
std::vector<geometry_msgs::msg::Point32> Slam::getMap() const { return cone_map_; }
geometry_msgs::msg::Pose2D Slam::getState() const { return slam_state_; }

void Slam::setParameters(int n_particles, double mh_threshold) {
  n_particles_ = n_particles;
  n_resample_ = n_particles_ / 1.5;
  mh_threshold_ = mh_threshold;

  for(int i = 0; i < n_particles_;i++) {
    particles_.push_back(Particle(n_particles_));
  }
}

void Slam::initializeState() {
  slam_state_.x = 0;
  slam_state_.y = 0;
  slam_state_.theta = 0;

}

void Slam::predictParticles(Eigen::Vector3d u, Eigen::Matrix3d R, double dt) {
  std::random_device rd;
  std::mt19937 e2(rd());
  std::normal_distribution<> dist(0, 1.0);

  for(auto &particle:particles_) {
    Eigen::MatrixXd noise(1, 3);
    noise(0,0) = dist(e2);
    noise(0,1) = dist(e2);
    noise(0,2) = dist(e2);
    // Eigen::Vector3d ud = u;
    Eigen::Vector3d ud = u + (noise * R.cwiseSqrt()).transpose();
  
    // ROS_WARN_STREAM("Particle #" << counts);
    particle.predict(ud, dt);
  }
  
}

void Slam::normalizeWeight() {
  double sum_w = 0.0;
  
  for(auto &particle:particles_)
    sum_w += particle.w;

  try {
    for(auto &particle:particles_)
      particle.w /= sum_w;
  }catch(...) { // Zero division error
    for(auto &particle:particles_)
      particle.w = 1.0 / n_particles_;
  }
}

void Slam::resample() {
  normalizeWeight();

  Eigen::VectorXd pw(particles_.size());

  for(unsigned int i = 0; i < particles_.size(); i++) {
    pw(i) = particles_[i].w;
  }
  
  double n_eff = 1.0 / pw.dot(pw.transpose());

  if(n_eff < n_resample_) {
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<double> weights(n_particles_);
    
    double sum = 0.0;
    for(int i = 0; i < n_particles_; i++) {
      weights[i] = particles_[i].w;
      sum += weights[i];
    }

    for(int i = 0; i < n_particles_; i++) {
      particles_[i].w = particles_[i].w/sum;
    }

    std::discrete_distribution<> d(weights.begin(), weights.end());
    std::vector<Particle> resampParticle;
  
    for (int i = 0; i < n_particles_; i++) {
      int idx = d(gen);
      resampParticle.push_back(particles_[idx]);
    }

    particles_ = resampParticle;

  }
  
}

void Slam::updateLandmarks(const std::vector<geometry_msgs::msg::Point32> &cones, bool frozen_update) {
  std::vector<std::vector<int>> particle_lm;
  for (int i = 0; i < n_particles_; i++) {
    std::vector<int> init = {};
    particle_lm.push_back(init);
  }
  for(long unsigned int i = 0; i < cones.size(); i++) {
    int particle_id = 0;
    for(auto &particle:particles_) {
      int observed_idx;
      Eigen::Vector2d z;

      const double s = std::sin(particle.yaw); 
      const double c = std::cos(particle.yaw);

      // Transform from robot to world frame
      z << (cones[i].x * c - cones[i].y * s) + particle.x, // Calculate Landmark coordinates 
          (cones[i].x * s + cones[i].y * c) + particle.y;

      Eigen::Matrix2d Q;
      Q << 0.001, 0,
           0, 0.0025;

      // Find most probable Landmark
      const auto p_it = std::min_element(particle.landmarks.begin(), particle.landmarks.end(), 
                                        [&](const Landmark &a, const Landmark &b) {
                                            double da = std::hypot(a.x[0] - z(0), a.x[1] - z(1));
                                            double db = std::hypot(b.x[0] - z(0), b.x[1] - z(1));
                                            return da < db;
                                        });
      // Index
      const auto p_i = std::distance(particle.landmarks.begin(), p_it);

      if(p_i == static_cast<long int>(particle.landmarks.size())) {
        // Add new cone to empty map
        z(0) = std::hypot(cones[i].x, cones[i].y);
        z(1) = pi_2_pi(std::atan2(cones[i].y, cones[i].x));

        Landmark new_landmark(z, Q, &particle);
        particle.landmarks.push_back(new_landmark);
        observed_idx = p_i - 1;
      }else if(std::hypot(particle.landmarks[p_i].x[0] - z(0), particle.landmarks[p_i].x[1] - z(1)) > 0.5 && !frozen_update) {
        // Probablity is too low
        z(0) = std::hypot(cones[i].x, cones[i].y);
        z(1) = pi_2_pi(std::atan2(cones[i].y, cones[i].x));

        // Add new cone to map
        Landmark new_landmark(z, Q, &particle);
        particle.landmarks.push_back(new_landmark);
        observed_idx = particle.landmarks.size() - 1;
      }else {
        // Update cone position
        z(0) = std::hypot(cones[i].x, cones[i].y);
        z(1) = pi_2_pi(std::atan2(cones[i].y, cones[i].x));

        particle.w *= particle.computeWeight(p_i, z, Q);
        particle.landmarks[p_i].update(z, Q, &particle);
        particle.proposalSampling(p_i, z, Q);
        observed_idx = p_i;
      }
      particle_lm[particle_id].push_back(observed_idx);
      particle_id++;
    }
  }

  for (int i = 0; i < n_particles_; i++) {
    std::vector<Landmark> new_lm;

    for (long unsigned int j = 0; j < particles_[i].landmarks.size(); j++) {
      if (std::find(particle_lm[i].begin(), particle_lm[i].end(), j)
          != particle_lm[i].end() ||
          particles_[i].landmarks[j].n_observed > 5) {
        new_lm.push_back(particles_[i].landmarks[j]);
      }
    }
    particles_[i].landmarks = new_lm;
  }
}

void Slam::calcFinalState() {
  normalizeWeight();
  slam_state_.x = 0.0;
  slam_state_.y = 0.0;
  slam_state_.theta = 0.0;

  for(long unsigned int i = 0; i < particles_.size(); i++) {
    slam_state_.x += (particles_[i].x * particles_[i].w);
    slam_state_.y += (particles_[i].y * particles_[i].w);
    slam_state_.theta += (particles_[i].yaw * particles_[i].w);
  }

  slam_state_.theta = pi_2_pi(slam_state_.theta);
}

void Slam::createMap() {
  cone_map_.clear();

  double max_w = std::numeric_limits<double>::min();
  int id_cnt = 0;

  for(auto &particle:particles_) {
    if(particle.w > max_w) {
      max_w = particle.w;
      best_id_ = id_cnt; 
    }
    id_cnt++;
  }
  
  for(size_t i = 0;i < particles_[best_id_].landmarks.size(); i++) {
    geometry_msgs::msg::Point32 cone_;
    cone_.x = particles_[best_id_].landmarks[i].x[0];
    cone_.y = particles_[best_id_].landmarks[i].x[1];

    cone_map_.push_back(cone_);
  }

  // ROS_WARN_STREAM("Map size is " <<  (int) (cone_map_.cone_blue.size()
                                    // + cone_map_.cone_yellow.size() 
                                    // + cone_map_.cone_orange.size()));
}

void Slam::calculateState(const geometry_msgs::msg::Twist &velocity) {
  slam_state_.x += velocity.linear.x;
  slam_state_.y += velocity.linear.y;
  slam_state_.theta += velocity.angular.z;
}
}
