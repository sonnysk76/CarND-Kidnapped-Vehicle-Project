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

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
// Define number of particles and initial weight.
	Particle p_Temp;
	num_particles = 10;
	float initial_weight = 1.0;

// Set noise with normal distribution for X, Y and theta
	std::default_random_engine generador;
	std::normal_distribution<double> dist_X(0, std[0]);
	std::normal_distribution<double> dist_Y(0, std[1]);
	std::normal_distribution<double> dist_Theta(0, std[2]);

// Assign initial values to each particle and populate particles and weights vectors
	for(int i = 0; i < num_particles; ++i ){
		p_Temp.id = i;
		p_Temp.x = x + dist_X(generador);
		p_Temp.y = y + dist_Y(generador);
		p_Temp.theta = theta + dist_Theta(generador);
		p_Temp.weight = initial_weight;
		p_Temp.associations.clear();
		p_Temp.sense_x.clear();
		p_Temp.sense_y.clear();

		particles.push_back(p_Temp);
		weights.push_back(initial_weight);
	}
// Flag initialization var as true.
  is_initialized = true;
	return;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
std::default_random_engine gene;

	for(int i = 0; i < particles.size(); ++i){
		double xf;
		double yf;
		double thetaF;
// Calculate predition based on motion model
		if(yaw_rate == 0){
			xf = particles[i].x + velocity * delta_t * cos(particles[i].theta);
			yf = particles[i].y + velocity * delta_t * sin(particles[i].theta);
			thetaF = particles[i].theta;
		}
		else {
			xf = particles[i].x + (velocity / yaw_rate) * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
			yf = particles[i].y + (velocity / yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
			thetaF = particles[i].theta + yaw_rate * delta_t;
		}
//Define and add noise to predictions
		std::normal_distribution<double> noise_X(xf, std_pos[0]);
		std::normal_distribution<double> noise_Y(yf, std_pos[1]);
		std::normal_distribution<double> noise_Theta(thetaF, std_pos[2]);

		particles[i].x = noise_X(gene);
		particles[i].y = noise_Y(gene);
		particles[i].theta = noise_Theta(gene);

	}
	return;
}


void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to

}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

	for(int i = 0; i < particles.size();++i){
	// clear particle transformed observations
		particles[i].sense_x.clear();
		particles[i].sense_y.clear();
	// Perform tranformation.
		for(int j = 0; j < observations.size();++j){
				double Xp, Yp;
				Xp = particles[i].x + ((cos(particles[i].theta) * observations[j].x)
						- (sin(particles[i].theta) * observations[j].y));
				particles[i].sense_x.push_back(Xp);
				Yp = particles[i].y + ((sin(particles[i].theta) * observations[j].x)
						+ (cos(particles[i].theta) * observations[j].y));
				particles[i].sense_y.push_back(Yp);
			}
	// initialize particle associations
			particles[i].associations.clear();
			//LandmarkObs LM_Asso;
	// Perform observations vs landmarks associations.
			double min_dist;
			int id = 0;
			for(int j = 0; j < observations.size(); ++j) {
	      min_dist = 99999.99;
	      for(int k = 0; k < map_landmarks.landmark_list.size(); ++k){
	          double distancia = 0;
	          distancia = dist(particles[i].sense_x[j], particles[i].sense_y[j],
	                      map_landmarks.landmark_list[k].x_f, map_landmarks.landmark_list[k].y_f);
	          if(distancia < sensor_range and distancia < min_dist){
	            min_dist = distancia;
	            id = map_landmarks.landmark_list[k].id_i;
					}
	        }
	      if(id != 0) {
	        particles[i].associations.push_back(id);
					//LM_Asso.push_back {id, }
			  }
	    }

			std::cout << "Particula " << i << " Asociaciones" << '\n';
			for(int j = 0; j < particles[i].associations.size(); ++j){
					std::cout <<j<< " Landmark: " <<particles[i].associations[j]<< '\n';
			}
// weights update.
			particles[i].weight = 1;


			for(int j = 0; j < particles[i].associations.size(); ++j ){
				int position = (particles[i].associations[j]) - 1;
				double sig_lan_x = std_landmark[0];
				double sig_lan_y = std_landmark[1];
				double obs_mc_x = particles[i].sense_x[j];
				double obs_mc_y = particles[i].sense_y[j];
				double lm_x = map_landmarks.landmark_list[position].x_f;
				double lm_y = map_landmarks.landmark_list[position].y_f;

				// calculate normalization term.
				double gauss_norm = (1/(2 * M_PI * sig_lan_x * sig_lan_y));
				// calculate exponent
				double exponent = ((pow(obs_mc_x - lm_x, 2))/(2 *pow(sig_lan_x,2)))
                            + ((pow(obs_mc_y - lm_y,2))/(2 * pow(sig_lan_y,2)));
	      // calculate weight using normalization terms and exponent
				particles[i].weight *=  gauss_norm * exp(-(exponent));
			}
			weights[i] = particles[i].weight;
	} // For particula.
  return;
} // Function
void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight.
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
	default_random_engine gen;
	discrete_distribution<int> distribution(weights.begin(), weights.end());
	std::vector<Particle> resample_particles;

	for(int i = 0; i < num_particles; i++){
		resample_particles.push_back(particles[distribution(gen)]);
	}
	particles = resample_particles;
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

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
