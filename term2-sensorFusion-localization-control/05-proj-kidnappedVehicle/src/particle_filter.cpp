/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h>
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

void ParticleFilter::init(const double x, const double y, const double theta,
                          const double std[]) {
  // Set the number of particles.
  num_particles = PF_PAR_INIT_NB_PARTICLES;

  // Initialize all particles to first position (based on estimates of
  // x, y, theta and their uncertainties from GPS)
  std::normal_distribution<double> gd_x_gps(x, std[PF_IDX_X]);
  std::normal_distribution<double> gd_y_gps(y, std[PF_IDX_Y]);
  std::normal_distribution<double> gd_theta_gps(theta, std[PF_IDX_THETA]);

  // Add random Gaussian noise to each particle.
  std::default_random_engine noise_gen;

  for (int i = 0; i < num_particles; i++) {
    Particle p;
    p.id = i;
    p.x = gd_x_gps(noise_gen);
    p.y = gd_y_gps(noise_gen);
    p.theta = gd_theta_gps(noise_gen);
    // Initialize all particles' weight to 1.
    p.weight = 1;
    particles.push_back(p);
    //weights.push_back(1);
  }

  // Set internal initialization flag
  is_initialized = true;
}

void ParticleFilter::prediction(const double delta_t, const double std_pos[],
                                const double velocity, const double yaw_rate) {
  std::normal_distribution<double> gd_x_pos(PF_PAR_POS_MEAN, std_pos[PF_IDX_X]);
  std::normal_distribution<double> gd_y_pos(PF_PAR_POS_MEAN, std_pos[PF_IDX_Y]);
  std::normal_distribution<double> gd_theta_pos(PF_PAR_POS_MEAN,
                                                std_pos[PF_IDX_THETA]);
  std::default_random_engine noise_gen;
  double delta_theta;
  double theta;

  for (int i = 0; i < particles.size(); i++) {
    theta = particles[i].theta;

    // Prediction if yaw rate is greater than minimal rate
    if (fabs(yaw_rate) > PF_PAR_MIN_YAW_RATE) {
      delta_theta = theta + (yaw_rate * delta_t);
      particles[i].x += (velocity / yaw_rate) * (sin(delta_theta) - sin(theta));
      particles[i].y += (velocity / yaw_rate) * (cos(theta) - cos(delta_theta));
      //particles[i].theta = yaw_rate * delta_t;
      particles[i].theta = delta_theta;
    }
    // Prediction calculation if yaw rate is smaller than minimal rate
    else {
      particles[i].x += velocity * delta_t * cos(theta);
      particles[i].y += velocity * delta_t * sin(theta);
    }

    // Add pose noise
    // Gaussain distribution - mean = PF_PAR_POS_MEAN
    //                       - std_deviation = std_pos[PF_IDX_...]
    particles[i].x += gd_x_pos(noise_gen);
    particles[i].y += gd_y_pos(noise_gen);
    particles[i].theta += gd_theta_pos(noise_gen);
  }
}

void ParticleFilter::dataAssociation(const std::vector<LandmarkObs> predicted,
                                     std::vector<LandmarkObs>& observations) {
  double delta_min;
  double delta_pred_obs;

  for (int i = 0; i < observations.size(); i++) {
    for (int j = 0; j < predicted.size(); j++) {
      delta_pred_obs = dist(observations[i].x, observations[i].y,
                            predicted[j].x, predicted[j].y);
      delta_min = (j == 0) ? delta_pred_obs : delta_min;

      if (delta_pred_obs <= delta_min) {
        delta_min = delta_pred_obs;
        observations[i].id = predicted[j].id;
      }
    }
  }
}

void ParticleFilter::updateWeights(const double sensor_range,
  const double std_landmark[], const std::vector<LandmarkObs> observations,
  const Map map_landmarks) {

  double cov_x = std_landmark[PF_IDX_X] * std_landmark[PF_IDX_X];
  double cov_y = std_landmark[PF_IDX_Y] * std_landmark[PF_IDX_Y];
  double normalizer = 2.0 * M_PI * std_landmark[PF_IDX_THETA] * std_landmark[PF_IDX_THETA];

  for (int i = 0; i < particles.size(); i++) {

    // The following routine selects which landmarks are within the sensor_range
    // of a given particle for furhter processing.

    // Vector of landmakrs for a given particle
    std::vector<LandmarkObs> particle_landmarks;
    // Parse all landmarks
    for (auto landmark : map_landmarks.landmark_list) {
      // If the selected landmark is within sensor_range for the particle
      // pushes the landmark to the particle_landmarks list
      if (dist(landmark.x_f, landmark.y_f, particles[i].x, particles[i].y) <= sensor_range) {
        LandmarkObs get_landmark;
        get_landmark.x = landmark.x_f;
        get_landmark.y = landmark.y_f;
        get_landmark.id = landmark.id_i;
        particle_landmarks.push_back(get_landmark);
      }
    }

    // The observations are given in the VEHICLE'S coordinate system.
    // The particles are located according to the MAP'S coordinate system.
    // The following routine transforms the observations (measeurements) to the
    // MAP coordinate system, based on the particle location.

    // Vector of observations (on MAP coord system) for the given particle
    std::vector<LandmarkObs> observations_map;
    for (auto obs_vehicle : observations) {
      LandmarkObs obs_map;
      obs_map.id = obs_vehicle.id;
      obs_map.x = obs_vehicle.x * cos(particles[i].theta) - obs_vehicle.y * sin(particles[i].theta) + particles[i].x;
      obs_map.y = obs_vehicle.x * sin(particles[i].theta) + obs_vehicle.y * cos(particles[i].theta) + particles[i].y;
      observations_map.push_back(obs_map);
    }

    // Helper to associate observations (sensor measurement) to landmarks
    dataAssociation(particle_landmarks, observations_map);
    particles[i].weight = 1.0;

    // Update particle weight based on the associated landmark and measurement
    for (int j = 0; j < observations_map.size(); j++) {
      double pred_x, pred_y, obs_x, obs_y;
      auto transformed_ob = observations_map[j];

      for (int k = 0; k < particle_landmarks.size(); k++) {
        if (particle_landmarks[k].id == observations_map[j].id) {
          pred_x = particle_landmarks[k].x;
          pred_y = particle_landmarks[k].y;
        }
      }
      // Update the weight of each particle using a mult-variate Gaussian distribution
      double dist_x = (pred_x - observations_map[j].x);
      double dist_y = (pred_y - observations_map[j].y);
      // Compute probability for a single pair measurement - landmark
      double pdf = exp(-(dist_x * dist_x/(2*cov_x) + dist_y* dist_y/(2*cov_y))) / normalizer;
      // Update particle weight by acumulating (product) the probabilities
      particles[i].weight *= pdf;
    }
  }
}

void ParticleFilter::resample() {
  // New list of particles based on weight sampling
  std::vector<Particle> new_particles;
  double weight_max = 0;
  double select_random_weight = 0.0;
  std::default_random_engine random_gen;

  for (int i = 0; i < particles.size(); i++) {
    weight_max = (particles[i].weight > weight_max) ? particles[i].weight : weight_max;
  }

  std::uniform_int_distribution<int> int_distribution(0, num_particles - 1);
  std::uniform_real_distribution<double> real_distribution(0.0, weight_max);

  int weighted_index = int_distribution(random_gen);
  for (int i = 0; i < particles.size(); i++) {
    select_random_weight += real_distribution(random_gen) * 2.0;

    while (select_random_weight > particles[weighted_index].weight) {
      select_random_weight -= particles[weighted_index].weight;
      weighted_index = (weighted_index + 1) % num_particles;
    }
    new_particles.push_back(particles[weighted_index]);
  }

  particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
  //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  //Clear the previous associations
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;

  return particle;
}

std::string ParticleFilter::getAssociations(Particle best)
{
  std::vector<int> v = best.associations;
  std::stringstream ss;
    copy( v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    std::string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
std::string ParticleFilter::getSenseX(Particle best)
{
  std::vector<double> v = best.sense_x;
  std::stringstream ss;
    copy( v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    std::string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
std::string ParticleFilter::getSenseY(Particle best)
{
  std::vector<double> v = best.sense_y;
  std::stringstream ss;
    copy( v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    std::string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
