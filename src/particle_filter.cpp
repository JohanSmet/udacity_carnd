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
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	num_particles = 100;
	particles.resize(num_particles);

	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	default_random_engine gen;

	for (int i = 0; i < num_particles; ++i) {
		auto &p = particles[i];
		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1.0;
	}

	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

	normal_distribution<double> noise_x(0, std_pos[0]);
	normal_distribution<double> noise_y(0, std_pos[1]);
	normal_distribution<double> noise_theta(0, std_pos[2]);
	default_random_engine gen;

	if (fabs(yaw_rate) > 0.00001) {
		for (auto &p : particles) {
			auto new_theta = fmod(p.theta + (yaw_rate * delta_t), 2 * M_PI);

			p.x = p.x + (velocity / yaw_rate) * (sin(new_theta) - sin(p.theta)) + noise_x(gen);
			p.y = p.y + (velocity / yaw_rate) * (cos(p.theta) - cos(new_theta)) + noise_y(gen);
			p.theta = fmod(new_theta + noise_theta(gen), 2 * M_PI);
		}
	} else {
		for (auto &p : particles) {
			p.x = p.x + (velocity * delta_t) + noise_x(gen);
			p.y = p.y + (velocity * delta_t) + noise_y(gen);
		}
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

	for (auto &obs : observations) {
		auto min_distance = dist(obs.x, obs.y, predicted[0].x, predicted[0].y);
		obs.id = 0;	

		for (int i = 1; i < predicted.size(); ++i) {
			auto distance = dist(obs.x, obs.y, predicted[i].x, predicted[i].y);

			if (distance < min_distance) {
				min_distance = distance;
				obs.id = i;
			}
		}
	}
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

	for (auto &p : particles) {
		// transform observations from particle coordinates to map coordinates
		vector<LandmarkObs>	p_observations;

		const	auto cos_theta = cos(p.theta);
		const auto sin_theta = sin(p.theta);

		for (const auto &obs : observations) {
			p_observations.push_back({
				0, 
				p.x + (cos_theta * obs.x) - (sin_theta * obs.y),
				p.y + (sin_theta * obs.x) + (cos_theta * obs.y)
			});
		}

		// find landmarks in range of the particle
		vector<LandmarkObs> p_landmarks;

		for (const auto &lm : map_landmarks.landmark_list) {
			if (dist(lm.x_f, lm.y_f, p.x, p.y) < sensor_range) {
				p_landmarks.push_back({lm.id_i, lm.x_f, lm.y_f});
			}
		}

		// perform nearest neighbour data association
		dataAssociation(p_landmarks, p_observations);

		// recompute weights
		auto gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
		auto sig_2x2 = 2 * std_landmark[0] * std_landmark[0];
		auto sig_2y2 = 2 * std_landmark[1] * std_landmark[1];

		p.weight = 1;

		for (const auto &obs : p_observations) {
			const auto &lm = p_landmarks[obs.id];
			double exponent = (pow(obs.x - lm.x, 2) / sig_2x2) + (pow(obs.y - lm.y, 2) / sig_2y2);
			p.weight *= gauss_norm * exp(-exponent);
		}
	}
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

	// gather particle weights into 1 vector
	weights.clear();
	for (const auto &p : particles) {
		weights.push_back(p.weight);
	}

	discrete_distribution<> d(begin(weights), end(weights));
	vector<Particle> new_particles;
	default_random_engine gen;

	for (int i = 0; i < num_particles; ++i) {
		new_particles.push_back(particles[d(gen)]);
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
