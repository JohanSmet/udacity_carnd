#include "planner.h"
#include "spline.h"

#include <algorithm>

namespace {

const int STATE_STARTUP = 0;
const int STATE_KEEP_LANE = 1;
const int STATE_CHANGING_LANE = 2;

const char *STATE_NAMES[] = {
  "startup",
  "keep_lane",
  "changing_lane"
};

double transform_s_map_to_car(double car_s, double map_s) {
  double delta_s = map_s - car_s;

  if (delta_s > (Map::max_s / 2.0))
    delta_s -= Map::max_s;
  else if (delta_s < -(Map::max_s / 2.0))
    delta_s += Map::max_s;

  return delta_s;
}

double transform_s_car_to_map(double car_s, double delta_s) {
  double map_s = fmod(delta_s + car_s, Map::max_s);
  return map_s;
}

} // unnamed namespace

Planner::Planner(const Map &map) : m_map(map),
                                   m_lane_vehicles(Map::NUM_LANES) {
  m_state = STATE_STARTUP;
  m_state_time = 0;
  m_current_lane = 1;
  m_desired_lane = 1;
  m_desired_speed = TARGET_SPEED;
}

void Planner::vehicles_reset() {
  for (int lane=0; lane < Map::NUM_LANES; ++lane) {
    m_lane_vehicles[lane].clear();
  }
}

void Planner::vehicles_add(Vehicle vehicle) {
  auto lane = m_map.lane_from_d(vehicle.d());

  if (lane >= 0 && lane < Map::NUM_LANES) {
    m_lane_vehicles[lane].push_back(vehicle);
  }
}

void Planner::create_trajectory(Vehicle ego, std::vector<std::vector<double>> &trajectory) {

  // clear previous trajectory when it becomes too short to use
  if (trajectory[0].size() < 2) {
    trajectory[0].clear();
    trajectory[1].clear();
  }

  // early out if nothing to do
  if (trajectory[0].size() >= TRAJECTORY_POINTS)
    return;

  // reset ego state if trajectory is empty
  if (trajectory[0].empty()) {
    m_last_ego = ego;
    m_last_target = {m_last_ego.s() - 1, m_last_ego.d()};
  }

  // advance simulation of detected vehicles to the end of the previous path
  predict_vehicles(trajectory[0].size() * TIMESTEP);

  // generate new target points if necessary
  if (m_targets.size() < 2) {
    process_state();
  }

  if (m_targets.size() < 2) {
    generate_keep_lane_targets();
  }

  // generate the final trajectory
  int np = TRAJECTORY_POINTS - trajectory[0].size();

  generate_trajectory(np * TIMESTEP, false);

  for (int idx = 0; idx < np; ++idx) {
    auto xy = m_map.getXY(m_frenet_path[idx].m_s, m_frenet_path[idx].m_d);
    trajectory[0].push_back(xy[0]);
    trajectory[1].push_back(xy[1]);

    m_state_time += TIMESTEP;
  }

  // save state of ego at the end of the trajectory
  m_last_ego = m_sim_ego;

  // remove targets that are reached in this trajectory
  std::vector<FrenetPoint> m_old_targets = m_targets;
  m_targets.clear();

  for (auto target : m_old_targets) {
    if (distance_in_lane(m_last_ego.s(), target.m_s) > 0) {
      m_targets.push_back(target);
    } else {
      m_last_target = target;
    }
  }
}

void Planner::predict_vehicles(double delta_t) {
  for (int lane = 0; lane < Map::NUM_LANES; ++lane) {
    for (auto &veh : m_lane_vehicles[lane]) {
      veh.predict(delta_t);
    }
  }
}

void Planner::process_state() {

  switch (m_state) {
    case STATE_STARTUP :
      if (m_state_time >= STARTUP_TIME) {
        change_state(STATE_KEEP_LANE);
      }
      break;
    
    case STATE_KEEP_LANE :
      if (m_state_time >= CHANGE_LANE_COOLDOWN) {
        int ideal_lane = check_for_ideal_lane();

        if (ideal_lane != m_current_lane && try_changing_lane(ideal_lane)) {
          change_state(STATE_CHANGING_LANE);
        }
      }
      break;

    case STATE_CHANGING_LANE :
      if (fabs(m_last_ego.d() - m_map.lane_center(m_desired_lane)) < 0.1) {
        m_current_lane = m_desired_lane;
        change_state(STATE_KEEP_LANE);
      }
      break;
  }
}

void Planner::change_state(int p_new_state) {
  m_state = p_new_state;
  m_state_time = 0;
}

bool Planner::try_changing_lane(int lane) {

  // only try to change to adjactent lanes
  int target_lane = (lane > m_current_lane) ? m_current_lane + 1 : m_current_lane - 1;

  // save some state
  auto f_saved_vehicles = m_lane_vehicles;
  auto f_saved_targets  = m_targets;
  auto f_saved_speed    = m_desired_speed;

  bool sw_ok = false;
  m_desired_speed = std::min(mph_to_mps(40.0), m_desired_speed);

  // generate path and check validity
  generate_change_lane_targets(target_lane);

  if (generate_trajectory(5.0, true)) {
    m_desired_lane = target_lane;
    sw_ok = true;
  }

  // always restore vehicle state
  m_lane_vehicles = f_saved_vehicles;

  // restore targets if there was no valid trajectory
  if (!sw_ok) {
    m_targets = f_saved_targets;
    m_desired_speed = f_saved_speed;
  }

  return sw_ok;
}

void Planner::generate_keep_lane_targets() {

  if (m_targets.size() < 1) {
    m_targets.push_back({m_last_ego.s() + 15, m_map.lane_center(m_desired_lane)});
  }

  if (m_targets.size() < 2) {
    m_targets.push_back({m_targets[0].m_s + 15, m_targets[0].m_d});
  }
}

void Planner::generate_change_lane_targets(int desired_lane) {

  m_targets.clear();

  // sharpness of turn depends on the current speed
  double delta = (m_last_ego.speed() > mph_to_mps(40)) ? 20 : 15;

  auto target_d = m_map.lane_center(desired_lane);
  m_targets.push_back({m_last_ego.s() + delta, (target_d + m_last_ego.d()) / 2.0});
  m_targets.push_back({m_last_ego.s() + (delta * 2), target_d});
}

bool Planner::generate_trajectory(double delta_t, bool check_collision) {

  m_sim_ego = m_last_ego;
  m_frenet_path.clear();

  // create a list of control points for the spline
  std::vector<double> control_s = {
    m_last_target.m_s,
    m_sim_ego.s(),
    m_targets[0].m_s,
    m_targets[1].m_s,
    m_targets[1].m_s + 30,
  };

  std::vector<double> control_d = {
    m_last_target.m_d,
    m_sim_ego.d(),
    m_targets[0].m_d,
    m_targets[1].m_d,
    m_targets[1].m_d,
  };

  // transfrom the control points to be relative to the car (to avoid wrapping problems)
  for (auto &c : control_s) {
    c = transform_s_map_to_car(m_last_ego.s(), c);
  }

  // fit a spline to the control points
  tk::spline  path;
  path.set_points(control_s, control_d);

  // generate the new path from the spline
  double target_s = 30;
	double target_d = path(target_s);
	double target_dist = sqrt((target_s * target_s) + (target_d * target_d));
	double car_s = 0.0;
  double map_s = m_last_ego.s();

  double cur_speed = m_last_ego.speed();

  for (double t=0.0; t < delta_t; t += TIMESTEP) {

      // update predictions for the detected vehicles
      predict_vehicles(TIMESTEP);

      // adjust desired speed depending on the traffic in front
      Vehicle nearest_veh;

      double nearest_front = check_nearest_vehicle_up_front(map_s, m_desired_lane, nearest_veh);

      if (nearest_front < 10) {
        m_desired_speed = nearest_veh.speed() * 0.75;
      } else if (nearest_front < 16) {
        m_desired_speed = nearest_veh.speed();
      } else {
        m_desired_speed = TARGET_SPEED;
      }

      // update speed
      if (cur_speed <= m_desired_speed) {
        cur_speed += ACCELERATION * TIMESTEP;
      } else if (cur_speed >= m_desired_speed) {
        cur_speed -= ACCELERATION * TIMESTEP;
      }

      // calculate position after this timestep
      double n = target_dist / (TIMESTEP * cur_speed);
			car_s = car_s + (target_s / n);
      map_s = transform_s_car_to_map(m_last_ego.s(), car_s);
			double new_d = path(car_s);

      // check for collisions
      if (check_collision && collision_with_vehicle(map_s, new_d)) {
        return false;
      }

      // store path
      m_frenet_path.push_back({map_s, new_d});
      m_sim_ego.update(map_s, new_d, cur_speed);
  }

  return true;
}

double Planner::distance_in_lane(double ego_s, double veh_s) {
  return transform_s_map_to_car(ego_s, veh_s);
}

double Planner::check_nearest_vehicle_up_front(double ego_s, int lane, Vehicle &nearest) {

  double min_dist = Map::max_s;

  for (const auto &veh : m_lane_vehicles[lane]) {
    double dist = distance_in_lane(ego_s, veh.s());
    if (dist > 0 && dist < min_dist) {
      min_dist = dist;
      nearest = veh;
    }
  }

  return min_dist;
}

bool Planner::collision_with_vehicle(double ego_s, double ego_d) {
  const double D_RADIUS = 3.9;
  const double S_RADIUS = 8;

  for (int lane = 0; lane < Map::NUM_LANES; ++lane) {
    for (auto &veh : m_lane_vehicles[lane]) {
      if (fabs(distance_in_lane(ego_s, veh.s())) < S_RADIUS &&
          fabs(ego_d - veh.d()) < D_RADIUS) {
        return true;
      }
    }
  }

  return false;
}

int Planner::check_for_ideal_lane() {
  Vehicle             lane_vehicle;
  std::vector<int>    empty_lanes;
  std::vector<double> lane_speeds;

  // check for vehicles in the lanes
  for (int idx = 0; idx < Map::NUM_LANES; ++idx) {
    double nearest = check_nearest_vehicle_up_front(m_last_ego.s(), idx, lane_vehicle);

    if (nearest < LANE_LOOKAHEAD) {
      lane_speeds.push_back(lane_vehicle.speed());
    } else {
      lane_speeds.push_back(SPEED_LIMIT);
      empty_lanes.push_back(idx);
    }
  }

  // prefer empty lanes
  if (!empty_lanes.empty()) {
    // prefer center lane if available
    if (std::find(empty_lanes.begin(), empty_lanes.end(), 1) != empty_lanes.end())
      return 1;
    else if (std::find(empty_lanes.begin(), empty_lanes.end(), m_current_lane) != empty_lanes.end())
      return m_current_lane;
    else
      return empty_lanes[0];
  }

  // return lane with highest possible speed
  auto fastest = std::max_element(lane_speeds.begin(), lane_speeds.end());
  return std::distance(lane_speeds.begin(), fastest);
}

