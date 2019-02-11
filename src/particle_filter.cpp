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
#include <cmath>
// #include <list>

#include "helper_functions.h"

using std::string;
using std::vector;
using std::normal_distribution;
using std::iota;
using std::sort;

// argsort 
// https://stackoverflow.com/questions/1577475/c-sorting-and-keeping-track-of-indexes
template <typename T>
vector<size_t> sort_indexes(const vector<T> &v) {

  // initialize original index locations
  vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}

double multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y);

// calculate multi-variable Guassian probabilities
double multiv_prob(double sig_x, double sig_y, double x_obs, double y_obs,
                   double mu_x, double mu_y) {
  // calculate normalization term
  double gauss_norm;
  gauss_norm = 1 / (2 * M_PI * sig_x * sig_y);

  // calculate exponent
  double exponent;
  exponent = (pow(x_obs - mu_x, 2) / (2 * pow(sig_x, 2)))
               + (pow(y_obs - mu_y, 2) / (2 * pow(sig_y, 2)));
  
  // std::cout << "m_pi: " << M_PI << std::endl;

  // calculate weight using normalization terms and exponent
  double weight;
  weight = gauss_norm * exp(-exponent);
    
  return weight;
}

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
  // set std of variables
  double std_x = std[0];
  double std_y = std[1];
  double std_theta = std[2];
  
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);

  for (int i=0; i<num_particles; i++) {
    //define particle based on the struct
    Particle particle_;
    
    particle_.id = i;
    particle_.x = dist_x(gen);
    particle_.y = dist_y(gen);
    particle_.theta = dist_theta(gen);
    particle_.weight = 1;


    particles.push_back(particle_);
  }

  is_initialized = true;
  vector<double> set_weights(num_particles,0.0);
  weights = set_weights;
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

  double std_pos_x = std_pos[0];
  double std_pose_y = std_pos[1];
  double std_pose_theta = std_pos[2];

  normal_distribution<double> dist_x(0, std_pos_x);
  normal_distribution<double> dist_y(0, std_pose_y);
  normal_distribution<double> dist_theta(0,std_pose_theta);
  // pay attention to there are two cases: one is without yaw rate, one is with yaw, the motion model is different.
  for (int i=0; i<particles.size(); i++) {

    if (fabs(yaw_rate)==0) {

      particles[i].x += cos(particles[i].theta) * velocity * delta_t + dist_x(gen);
      particles[i].y += sin(particles[i].theta) * velocity * delta_t + dist_y(gen);
      particles[i].theta += dist_theta(gen);

    }
    else
    {
      particles[i].x += velocity/yaw_rate * (sin(particles[i].theta + yaw_rate*delta_t)-sin(particles[i].theta)) + dist_x(gen);
      particles[i].y += velocity/yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta+yaw_rate*delta_t)) + dist_y(gen);
      particles[i].theta += yaw_rate*delta_t + dist_theta(gen);
    }
    // std::cout << "prediction particle theta: " << particles[i].theta << std::endl;

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

  // associate the observation to the landmark within the sensor range

  for (int i=0; i<observations.size(); i++) {

    vector<double> dist_vec;
    vector<size_t> sort_idx;

    for (int j=0; j<predicted.size(); j++) {

      dist_vec.push_back(dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y));
      
    }
    sort_idx = sort_indexes(dist_vec);
    // observations[i].id = predicted[sort_idx[0]].id;
    observations[i].id = sort_idx[0];

    // std::cout << "associated observation landmark id: " << observations[i].id << std::endl;
    // std::cout << "x_obs: " << observations[i].x;
    // std::cout << " y_obs: " << observations[i].y << std::endl;

    // std::cout << "landmark_x: " << predicted[sort_idx[0]].x;
    // std::cout << "landmark_y: " << predicted[sort_idx[0]].y << std::endl;

  }

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
  // double whole_weights=0.0;

  for (int i=0; i<particles.size(); i++) {
    //transform the observations in vehicle coordinate system to map coordinate system
    vector<LandmarkObs> transformed_observations;

    for (int k=0; k<observations.size(); k++) {

      LandmarkObs transformed_observation;
      
      transformed_observation.x = particles[i].x + (cos(particles[i].theta) * observations[k].x) - (sin(particles[i].theta) * observations[k].y);
      transformed_observation.y = particles[i].y + (sin(particles[i].theta) * observations[k].x) + (cos(particles[i].theta) * observations[k].y);
      transformed_observation.id = observations[k].id;

      transformed_observations.push_back(transformed_observation);

    }
    //keep the landmarks in the map within the sensor range of the vehicle
    vector<LandmarkObs> predictions;

    for (int j=0; j<map_landmarks.landmark_list.size();j++) {

      double dist_ = dist(particles[i].x,particles[i].y,map_landmarks.landmark_list[j].x_f, map_landmarks.landmark_list[j].y_f);

      if (dist_ <= sensor_range) {
        LandmarkObs prediction;

        prediction.x = map_landmarks.landmark_list[j].x_f;
        prediction.y =  map_landmarks.landmark_list[j].y_f;
        prediction.id = map_landmarks.landmark_list[j].id_i;
        
        predictions.push_back(prediction);

        // std::cout << "prediction particle dist: " << dist_ << std::endl;
      }
    }
    
    dataAssociation(predictions, transformed_observations);
    
    double final_weight=1.0;
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
    //calculate the weight for each particle
    for (int k=0; k<transformed_observations.size(); k++) {

      int predict_idx = int(transformed_observations[k].id);
      double sig_x = std_landmark[0];
      double sig_y = std_landmark[1];
      double x_obs = transformed_observations[k].x;
      double y_obs = transformed_observations[k].y;
      double mu_x = predictions[predict_idx].x;
      double mu_y = predictions[predict_idx].y;
      // double mu_x = map_landmarks.landmark_list[land_idx].x_f;
      // double mu_y = map_landmarks.landmark_list[land_idx].y_f;
      double weight_;
      

      // particles[i].associations.push_back(predictions[predict_idx].id);
      // particles[i].sense_x.push_back(x_obs);
      // particles[i].sense_y.push_back(y_obs);

      associations.push_back(predictions[predict_idx].id);
      sense_x.push_back(x_obs);
      sense_y.push_back(y_obs);

      // std::cout << "x_obs: " << x_obs;
      // std::cout << " y_obs: " << x_obs << std::endl;

      // std::cout << "mu_x: " << mu_x;
      // std::cout << " mu_y: " << mu_y << std::endl;

      // std::cout << "multi prob: " << multiv_prob(sig_x, sig_y, x_obs, y_obs, mu_x, mu_y) << std::endl;
      weight_ = multiv_prob(sig_x, sig_y, x_obs, y_obs, mu_x, mu_y);

      if (weight_>0) {
        final_weight *= weight_;
      }
      // std::cout << "final_weight: " << final_weight << std::endl;
    }

    particles[i].weight = final_weight;
    weights[i] = particles[i].weight;
    // whole_weights += particles[i].weight;
    // for visualization in the output video
    SetAssociations(particles[i], associations, sense_x, sense_y);
  }

  // for (int i=0; i<particles.size(); i++) {

    // particles[i].weight = particles[i].weight/whole_weights;

    // weights[i] = particles[i].weight/whole_weights;

  // }


}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
  

  std::default_random_engine gen;

  vector<Particle> resample_particles;

  std::discrete_distribution<int> resample_dist(weights.begin(), weights.end());

  for (int i=0; i<particles.size(); i++) {

    int idx = resample_dist(gen);

    resample_particles.push_back(particles[idx]);
  }

  particles = resample_particles;

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