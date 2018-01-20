#ifndef CARND_PLANNER_H
#define CARND_PLANNER_H

#include "vehicle.h"
#include "map.h"
#include <vector>


inline double mph_to_mps(float vel_mph) {
	// conversion from miles/hour to meter/second
	//	1 mile per hour = 1.60934 km per hour = 0.447039 meter per second
	return vel_mph * 0.447039;
}

struct FrenetPoint {
  double m_s;
  double m_d;
};

class Planner {

  // interface functions
  public:
    // construction
    Planner(const Map &map);

    // sensor fusion data
    void vehicles_reset();
    void vehicles_add(Vehicle vehicle);

    // trajectory
    void create_trajectory(Vehicle ego, std::vector<std::vector<double>> &trajectory);

  // configuration 'parameters'
  private:
    const int TRAJECTORY_POINTS = 50;
    const double TIMESTEP = 0.02;
    const double SPEED_LIMIT = mph_to_mps(50);
    const double ACCELERATION = 5;      // m/s² 

  // helper functions
  private:
    void reset_simulation(Vehicle ego, int prev_trajectory_len);
    void predict_vehicles(double delta_t);

    void generate_keep_lane_targets();
    bool generate_trajectory(double delta_t);

    double distance_in_lane(double ego_s, double veh_s);
    double check_nearest_vehicle_up_front(double ego_s, int lane, Vehicle &nearest);


  // member variables
  private:
    const Map &m_map;
    std::vector<Vehicle>  m_lane_vehicles[Map::NUM_LANES];

    Vehicle m_last_ego;

    std::vector<FrenetPoint>  m_targets;
    int m_current_lane;
    int m_desired_lane;
    double m_desired_speed;

    std::vector<FrenetPoint> m_frenet_path;
    std::vector<double>      m_speeds;

};


#endif // CARND_PLANNER_H