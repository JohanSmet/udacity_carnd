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

	// set the desired number of particles
	num_particles = 150;

	// initialize gaussian distributions to add random noise to each particle
	normal_distribution<double> dist_x(x, std[0]);
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
	default_random_engine gen;

	// create and initialize all the particles
	particles.resize(num_particles);

	for (int i = 0; i < num_particles; ++i) {
		auto &p = particles[i];

		p.id = i;
		p.x = dist_x(gen);
		p.y = dist_y(gen);
		p.theta = dist_theta(gen);
		p.weight = 1.0;
	}

	// indicate that everything is initialized
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {

	// initialize gaussian distributions to add random noise the the movement
	normal_distribution<double> noise_x(0, std_pos[0]);
	normal_distribution<double> noise_y(0, std_pos[1]);
	normal_distribution<double> noise_theta(0, std_pos[2]);
	default_random_engine gen;

	// move the particles, avoiding division by zero when yaw_rate is close to zero
	if (fabs(yaw_rate) > 0.00001) {
		const auto voy = velocity / yaw_rate;

		for (auto &p : particles) {
			auto new_theta = p.theta + (yaw_rate * delta_t);

			p.x = p.x + voy * (sin(new_theta) - sin(p.theta)) + noise_x(gen);
			p.y = p.y + voy * (cos(p.theta) - cos(new_theta)) + noise_y(gen);
			p.theta = new_theta + noise_theta(gen);
		}
	} else {
		const auto vdt = velocity * delta_t;

		for (auto &p : particles) {
			p.x = p.x + vdt + noise_x(gen);
			p.y = p.y + vdt + noise_y(gen);
		}
	}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> landmarks, std::vector<LandmarkObs>& observations) {
	// NOTE: I used the index into the landmarks vector as the id of the associated landmark of an observation.
	// 	This makes it easier to retrieve the associated landmark in the next step. Adding an unordered_map would
	//  also do the trick but I feel this to be beyond the scope of this project.

	for (auto &obs : observations) {
		auto min_distance = dist(obs.x, obs.y, landmarks[0].x, landmarks[0].y);
		obs.id = 0;	

		for (int i = 1; i < landmarks.size(); ++i) {
			auto distance = dist(obs.x, obs.y, landmarks[i].x, landmarks[i].y);

			if (distance < min_distance) {
				min_distance = distance;
				obs.id = i;
			}
		}
	}
}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {

	// precompute constant values
	const auto gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);
	const auto sig_2x2 = 2 * std_landmark[0] * std_landmark[0];
	const auto sig_2y2 = 2 * std_landmark[1] * std_landmark[1];

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
		p.weight = 1;

		for (const auto &obs : p_observations) {
			const auto &lm = p_landmarks[obs.id];
			const auto exponent = (pow(obs.x - lm.x, 2) / sig_2x2) + (pow(obs.y - lm.y, 2) / sig_2y2);
			p.weight *= gauss_norm * exp(-exponent);
		}
	}
}

void ParticleFilter::resample() {
	// gather particle weights into 1 vector
	weights.clear();
	for (const auto &p : particles) {
		weights.push_back(p.weight);
	}

	// setup a discrete distribution to sample according to the particles weight
	discrete_distribution<> d(begin(weights), end(weights));
	default_random_engine gen;

	// resample the particles
	vector<Particle> new_particles;

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
