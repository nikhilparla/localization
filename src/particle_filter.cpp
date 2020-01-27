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
#include "multi_gauss.h"

using std::string;
using std::vector;
using std::normal_distribution;

std::default_random_engine gen;
vector<LandmarkObs> predicted;
double small_value = 0.000001;  // tweak and check

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */
   
  num_particles = 50;  // TODO: Set the number of particles
  particles.resize(num_particles);
  double std_x, std_y, std_theta;  // Standard deviations for x, y, and theta
  
  // Set standard deviations for x, y, and theta
  std_x 	= std[0];
  std_y 	= std[1];
  std_theta = std[2]; 
  
  // creates a normal (Gaussian) distribution for x,y,theta
  normal_distribution<double> dist_x(x, std_x);
  normal_distribution<double> dist_y(y, std_y);
  normal_distribution<double> dist_theta(theta, std_theta);

  // initialize all weights to 1
  for(int i=0; i< num_particles; i++){
	  particles[i].x 		= dist_x(gen);
	  particles[i].y 		= dist_y(gen);
	  particles[i].theta 	= dist_theta(gen);
	  particles[i].weight 	= 1.0;
	  
	  weights.push_back(particles[i].weight);
  }
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

    // creates a normal (Gaussian) distribution for x,y,theta
    normal_distribution<double> predict_x(0, std_pos[0]);
    normal_distribution<double> predict_y(0, std_pos[1]);
    normal_distribution<double> predict_theta(0, std_pos[2]);

  // First get the measurement
  for (uint i = 0; i < particles.size(); i++)
  {
    particles[i].x = particles[i].x + (velocity / yaw_rate) *
                                          (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
    particles[i].y = particles[i].y + (velocity / yaw_rate) *
                                          (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
    particles[i].theta = particles[i].theta + yaw_rate * delta_t;


    // add the noise and sample from it
    particles[i].x += predict_x(gen);
    particles[i].y +=predict_y(gen);
    particles[i].theta += predict_theta(gen);
  }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark. Observations is actual landmark measurements observed from the lidar.
   *  TODO: predicted vector is the predicted measurements b/w one particular particle and 
   *    all of the map landmarks within sensor range. Perfrom nearest data association and assign 
   *    each sensor observation a map landmark id associated with it
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */
  // for all observations, calculate the least distance and assign that id to that observation
  double min_distance = 50;   // initializing to max range ie sensor range
  for (uint i = 0; i < observations.size(); i++)
  {
    for (uint j = 0; j < predicted.size(); j++)
    {
      double distance = dist(predicted[j].x, predicted[j].y, observations[i].x, observations[i].y);
      if(distance < min_distance){
        min_distance = distance;
        observations[i].id = predicted[j].id;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{
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

  // 1. populate the predicted vector to send to data association function
  //  1.a. you have to do map trnasformations for this
  // 2. call the data asssociation fucn with predicted and observations
  // 3. implement the data association function and get the updated ids for landmarks
  // 4. continue updatedweights function

  vector<LandmarkObs> predicted;
  double distance;
  LandmarkObs obs;

  // copying the observations for passing it to dataAssociation func.
  // observations is a const so cannot send it directly
  vector<LandmarkObs> observs;
  observs.resize(observations.size());

  // paritcles and landmarks are in map coordinates.
  // No trnasofmration required here
  for (uint i = 0; i < particles.size(); i++)
  {
    for (uint k = 0; k < map_landmarks.landmark_list.size(); k++)
    {
      distance = (dist(particles[i].x, particles[i].y,
                       map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f));
      if (distance <= sensor_range)
      {
        obs.id = map_landmarks.landmark_list[k].id_i;
        obs.x = (double)map_landmarks.landmark_list[k].x_f;
        obs.y = (double)map_landmarks.landmark_list[k].y_f;
        predicted.push_back(obs);
      }
    } // all landmarks done

    // convert observation coords to map coords for data association
    for (uint j = 0; j < observations.size(); j++)
    {
      double xm = particles[i].x + cos(particles[i].theta * observations[j].x) -
                 sin(particles[i].theta * observations[j].y);
      double ym = particles[i].y + sin(particles[i].theta * observations[j].x) +
                 cos(particles[i].theta * observations[j].y);
      observs[j].x = xm;
      observs[j].y = ym;
    }
    dataAssociation(predicted, observs);

    // reset particle's weight before calculating new one
    particles[i].weight = 1.0;

    vector<double> weights_obs;
    weights_obs.resize(observations.size());
    double mu_x, mu_y;  // making them global
    //double final_weight = 1;
    for (uint j = 0; j < observs.size(); j++)
    {
      //#if 0
      for (uint k = 0; k < predicted.size(); k++)
      {
        if (predicted[k].id == observs[j].id)
        {
          mu_x = predicted[k].x;
          mu_y = predicted[k].y;
          break;  // break the loop if the id found
        }
      }
      //#endif
      weights_obs[j] = multiv_prob(std_landmark[0], std_landmark[1], observs[j].x, observs[j].y,
                                   mu_x, mu_y);
      if (weights_obs[j] < small_value) // catch zero multiplication
        particles[i].weight *= small_value;
      else
        particles[i].weight *= weights_obs[j];
    }
   
    weights[i] = particles[i].weight;

  } //all particles done
}

void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */
    std::random_device rd;
    std::mt19937 gen(rd());
    std::vector<Particle> particles_2;  // hold the particles that got picked
    particles_2.resize(particles.size());
    std::discrete_distribution<> d(weights.begin(), weights.end());
    for(uint i=0; i < particles.size(); i++){
      particles_2[i] = particles[d(gen)];
    }
    particles = particles_2;
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