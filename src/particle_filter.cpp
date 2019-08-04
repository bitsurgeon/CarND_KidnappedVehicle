/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
  num_particles = 100;  // TODO: Set the number of particles

  std::default_random_engine gen;
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);

  // initialize particles with noisy GPS location
  for (int i = 0; i < num_particles; ++i) {
    Particle aParticle;
    aParticle.id = i;
    aParticle.x = dist_x(gen);
    aParticle.y = dist_y(gen);
    aParticle.theta = dist_theta(gen);
    aParticle.weight = 1.0;

    particles.push_back(aParticle);
  }

  weights = vector<double>(num_particles, 1.0);

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], 
                                double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution 
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  std::default_random_engine gen;
  for (int i = 0; i < num_particles; ++i) {
    double x_0 = particles[i].x;
    double y_0 = particles[i].y;
    double theta_0 = particles[i].theta;

    double x_f, y_f, theta_f;

    // bicycle motion model
    if (yaw_rate == 0.0) {
      double velocity_change = velocity * delta_t;
      x_f = x_0 + velocity_change * cos(theta_0);
      y_f = y_0 + velocity_change * sin(theta_0);
      theta_f = theta_0;
    } else {
      double scale = velocity / yaw_rate;
      double yaw_change = yaw_rate * delta_t;
      x_f = x_0 + scale * (sin(theta_0 + yaw_change) - sin(theta_0));
      y_f = y_0 + scale * (cos(theta_0) - cos(theta_0 + yaw_change));
      theta_f = theta_0 + yaw_change;
    }

    // add Gaussian motion noise
    std::normal_distribution<double> dist_x(x_f, std_pos[0]);
    std::normal_distribution<double> dist_y(y_f, std_pos[1]);
    std::normal_distribution<double> dist_theta(theta_f, std_pos[2]);

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  // multivariate Gaussian normalizer
  double gauss_norm;
  gauss_norm = 1.0 / (2.0 * M_PI * std_landmark[0] * std_landmark[1]);

  // update weights based on the observations for each particle
  for (int i = 0; i < num_particles; ++i) {

    // convert observations from vehicle coordinate to map coordinate
    double x_part, y_part, theta_part;
    vector<LandmarkObs> predictions;

    x_part = particles[i].x;
    y_part = particles[i].y;
    theta_part = particles[i].theta;

    for (unsigned j = 0; j < observations.size(); ++j) {
      double x_obs, y_obs;
      x_obs = observations[j].x;
      y_obs = observations[j].y;

      LandmarkObs aPred;

      aPred.x = x_part + cos(theta_part) * x_obs - sin(theta_part) * y_obs;
      aPred.y = y_part + sin(theta_part) * x_obs + cos(theta_part) * y_obs;

      predictions.push_back(aPred);
    }

    // associate each observation to it's closest landmark
    vector<int> landmarks;
    vector<double> tobs_x;
    vector<double> tobs_y;
    for (unsigned p = 0; p < predictions.size(); ++p) {
      double min_range = sensor_range;
      int landmark_id;
      for (unsigned m = 0; m < map_landmarks.landmark_list.size(); ++m) {
        double range = dist(predictions[p].x, predictions[p].y, map_landmarks.landmark_list[m].x_f, map_landmarks.landmark_list[m].y_f);
        if (range < min_range) {
          min_range = range;
          landmark_id = map_landmarks.landmark_list[m].id_i;
        }
      }

      landmarks.push_back(landmark_id);
      tobs_x.push_back(predictions[p].x);
      tobs_y.push_back(predictions[p].y);
    }

    particles[i].associations = landmarks;
    particles[i].sense_x = tobs_x;
    particles[i].sense_y = tobs_y;

    // update weight
    double weight_p = 1.0;
    for (unsigned s = 0; s < particles[i].associations.size(); ++s) {
      double x_obs = particles[i].sense_x[s];
      double y_obs = particles[i].sense_y[s];
      int landmark_id = particles[i].associations[s];
      double mu_x = map_landmarks.landmark_list[landmark_id - 1].x_f;
      double mu_y = map_landmarks.landmark_list[landmark_id - 1].y_f;

      double exponent = (pow(x_obs - mu_x, 2) / (2 * pow(std_landmark[0], 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(std_landmark[1], 2)));
    
      weight_p *= gauss_norm * exp(-exponent);
    }
    particles[i].weight = weight_p;
    weights[i] = weight_p;
  }
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  vector<Particle> newParticles;

  std::default_random_engine gen;
  std::uniform_int_distribution<int> dist_ind(0, num_particles);
  std::uniform_real_distribution<double> dist_0_1(0.0, 1.0);

  int index = dist_ind(gen);
  double weight_max = *std::max_element(weights.begin(), weights.end());
  double beta = 0.0;

  for (int i = 0; i < num_particles; ++i) {
    beta += dist_0_1(gen) * 2.0 * weight_max;
    while(beta > weights[index]) {
      beta -= weights[index];
      index = (index + 1) % num_particles;
    }
    newParticles.push_back(particles[index]);
  }

  particles.swap(newParticles);
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}