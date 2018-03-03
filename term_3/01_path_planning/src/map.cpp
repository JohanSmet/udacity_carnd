#include "map.h"

#include <fstream>
#include <iostream>
#include <sstream>

#include "spline.h"

using namespace std;

struct MapPrivate {
	tk::spline	m_waypoints_spline_x;
	tk::spline	m_waypoints_spline_y;
	tk::spline	m_waypoints_spline_dx;
	tk::spline	m_waypoints_spline_dy;
};

Map::Map() : m_prv(new MapPrivate()) {
}

Map::~Map() {
	delete m_prv;
}

void Map::load_from_file(const char *p_filename) {
  ifstream in_map_(p_filename, ifstream::in);
	string line;

  std::vector<double> waypoints_x;
  std::vector<double> waypoints_y;
  std::vector<double> waypoints_s;
  std::vector<double> waypoints_dx;
  std::vector<double> waypoints_dy;

  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	waypoints_x.push_back(x);
  	waypoints_y.push_back(y);
  	waypoints_s.push_back(s);
  	waypoints_dx.push_back(d_x);
  	waypoints_dy.push_back(d_y);
	}
	
	// make sure to close the splines
	waypoints_x.push_back(waypoints_x[0]);
	waypoints_y.push_back(waypoints_y[0]);
	waypoints_s.push_back(Map::max_s * 1);
	waypoints_dx.push_back(waypoints_dx[0]);
	waypoints_dy.push_back(waypoints_dy[0]);

	// create the splines
	m_prv->m_waypoints_spline_x.set_points(waypoints_s, waypoints_x);
	m_prv->m_waypoints_spline_y.set_points(waypoints_s, waypoints_y);
	m_prv->m_waypoints_spline_dx.set_points(waypoints_s, waypoints_dx);
	m_prv->m_waypoints_spline_dy.set_points(waypoints_s, waypoints_dy);
}


/*
int Map::ClosestWaypoint(double x, double y) const {

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < m_waypoints_x.size(); i++) {
		double map_x = m_waypoints_x[i];
		double map_y = m_waypoints_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen) {
			closestLen = dist;
			closestWaypoint = i;
		}
	}

	return closestWaypoint;
}

int Map::NextWaypoint(double x, double y, double theta) const {

	int closestWaypoint = ClosestWaypoint(x,y);

	double map_x = m_waypoints_x[closestWaypoint];
	double map_y = m_waypoints_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2* M_PI - angle, angle);

  if(angle > M_PI/4) {
    closestWaypoint++;
    if (closestWaypoint == m_waypoints_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> Map::getFrenet(double x, double y, double theta) const {
	int next_wp = NextWaypoint(x,y, theta);

	int prev_wp = next_wp-1;
	if (next_wp == 0) {
		prev_wp  = m_waypoints_x.size()-1;
	}

	double n_x = m_waypoints_x[next_wp]-m_waypoints_x[prev_wp];
	double n_y = m_waypoints_y[next_wp]-m_waypoints_y[prev_wp];
	double x_x = x - m_waypoints_x[prev_wp];
	double x_y = y - m_waypoints_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-m_waypoints_x[prev_wp];
	double center_y = 2000-m_waypoints_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef) {
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++) {
		frenet_s += distance(m_waypoints_x[i],m_waypoints_y[i],m_waypoints_x[i+1],m_waypoints_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};
}*/

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> Map::getXY(double s, double d) const {

	double clamped_s = fmod(s, Map::max_s);

	double x = m_prv->m_waypoints_spline_x(clamped_s) + (d * m_prv->m_waypoints_spline_dx(clamped_s));
	double y = m_prv->m_waypoints_spline_y(clamped_s) + (d * m_prv->m_waypoints_spline_dy(clamped_s));
	return {x, y};

	/*
	int prev_wp = -1;

	while (s > m_waypoints_s[prev_wp+1] && (prev_wp < (int)(m_waypoints_s.size()-1) )) {
		prev_wp++;
	}

	int wp2 = (prev_wp+1) % m_waypoints_x.size();

	double heading = atan2((m_waypoints_y[wp2]-m_waypoints_y[prev_wp]),
                         (m_waypoints_x[wp2]-m_waypoints_x[prev_wp]));

	// the x,y,s along the segment
	double seg_s = (s-m_waypoints_s[prev_wp]);

	double seg_x = m_waypoints_x[prev_wp]+seg_s*cos(heading);
	double seg_y = m_waypoints_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading- M_PI_2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y}; */
}

int Map::lane_from_d(double d) const {
  return static_cast<int>(d / LANE_WIDTH);
}

double Map::lane_center(int lane) const {
	return LANE_HALF_WIDTH + (lane * LANE_WIDTH);
}