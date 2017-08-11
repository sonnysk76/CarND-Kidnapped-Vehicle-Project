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

  ParticleFilter::num_particles = 10;
  ParticleFilter::particles[ParticleFilter::num_particles];
  Particle particula;
  //double std_x = std[0];
  //double std_y = std[1];
  //double std_theta = std[2];

  std::default_random_engine generador;
  std::normal_distribution<double> distributionx(x, std[0]); // std x
  std::normal_distribution<double> distributiony(y, std[1]); // std y
  std::normal_distribution<double> distributiont(theta, std[2]); // std theta

  for(int i = 0; i < num_particles; i++)
  {
    particula.id = i;
    particula.x = distributionx(generador);
    particula.y = distributiony(generador);
    particula.theta = distributiont(generador);
    particula.weight = 1;
    particles.push_back(particula);

    weights.assign(i, particula.weight);
  }
  //std::cout<< particulas[1].x<<std::endl;
  is_initialized = true;

}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  Particle particula;

  double xf, yf, Tf, x0, y0, T0;

  for (int i = 0; i < num_particles; i++)
  {
    x0 = particles[i].x;
    y0 = particles[i].y;
    T0 = particles[i].theta;
    if(yaw_rate == 0)
    {
      xf = x0 + velocity * delta_t * cos(T0);
      yf = y0 + velocity * delta_t * sin(T0);
      Tf = T0;

    } else {

      xf = x0 + ((velocity / yaw_rate) * (sin(T0 + yaw_rate * delta_t) - sin(T0)));
      yf = y0 + ((velocity / yaw_rate) * (cos(T0) - cos(T0 + yaw_rate * delta_t)));
      Tf = T0 + (yaw_rate * delta_t);
    }
    std::default_random_engine generador;
    std::normal_distribution<double> distributionx(xf, std_pos[0]); // std x
    std::normal_distribution<double> distributiony(yf, std_pos[1]); // std y
    std::normal_distribution<double> distributiont(Tf, std_pos[2]); // std theta
    particula.x = distributionx(generador);
    particula.y = distributiony(generador);
    particula.theta = distributiont(generador);

    particles.assign(i, particula);
    //cout << particles[i].x << " \n" << particles[i].y << "\n" << particles[i].theta << "\n" << endl;

  }


}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

  std::vector<LandmarkObs> Predicted;
  for(int i =0; i < particles.size(); i++)
  {
    Predicted[i].id = particles[i].id;
    Predicted[i].x = particles[i].x;
    Predicted[i].y = particles[i].y;
  }











}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {



  int n_lmark = map_landmarks.landmark_list.size();
  int n_obs = observations.size();
  int n_part = particles.size();
  double theta, Xant, Yant,Xobs, Yobs, Xt, Yt;

  for(int i = 0; i < n_part; i++)
  {
    for(int j = 0; j < n_obs; j++)
    {
      theta = particles[i].theta;
      Xobs = observations[j].x;
      Yobs = observations[j].y;
      Xant = particles[i].x;
      Yant = particles[i].y;

      Xt = cos(theta) * Xobs + sin(theta) * Yobs + Xant;
      Yt = cos(theta) * Yobs - sin(theta) * Xobs + Yant;

      particles[i].sense_x.push_back(Xt);
      particles[i].sense_y.push_back(Yt);
    }
    int n_sense = particles[i].sense_x.size();
    for(int k = 0; k < n_lmark; k++)
    {
      if((fabs(particles[i].x - map_landmarks.landmark_list[k].x_f)) < 35 and
         (fabs(particles[i].y - map_landmarks.landmark_list[k].y_f)) < 35)
      {
        double minD = 999.0;
        int minID;
        double distancia = 0;
        for(int l = 0; l < n_sense; l++)
        {
          double xLM = map_landmarks.landmark_list[k].x_f;
          double yLM = map_landmarks.landmark_list[k].y_f;
          double xS = particles[i].sense_x[l];
          double yS = particles[i].sense_y[l];

          distancia = sqrt(pow(xLM - xS, 2) + pow(yLM - yS, 2)); //dist(xLM, yLM, xS, yS);  //
          if(distancia < minD)
          {
            minD = distancia;
            minID = k;

          }
        }
        particles[i].associations.push_back(minID);
        //std::cout<<"Particula "<< i <<" Landmark "<< k <<"  Distancia Minima "<< minD << "  Vector  " << minID << std::endl;
      }

    }
    for(int m = 0; m < particles[i].associations.size();m++)
    {
      std::cout << "Particula "<<i<<" associacion "<<m<<" ID "<<particles[i].associations[m]<<std::endl;
    }

  }


}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution




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
