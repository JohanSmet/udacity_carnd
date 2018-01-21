#include "planner.h"
#include "spline.h"

#include <iostream>

namespace {

const int STATE_STARTUP = 0;
const int STATE_KEEP_LANE = 1;
const int STATE_CHANGING_LANE = 2;

const char *STATE_NAMES[] = {
  "startup",
  "keep_lane",
  "changing_lane"
};

double transform_s_map_to_car(const Vehicle &car, double map_s) {
  double car_s = map_s - car.s();

  if (car_s > (Map::max_s / 2.0))
    car_s -= Map::max_s;
  else if (car_s < -(Map::max_s / 2.0))
    car_s += Map::max_s;

  return car_s;
}

double transform_s_car_to_map(const Vehicle &car, double car_s) {
  double map_s = fmod(car_s + car.s(), Map::max_s);
  return map_s;
}

/*void transform_map_to_car(const Vehicle &car,
                          double map_x, double map_y,
                          double &car_x, double &car_y) {
  auto cos_yaw = cos(-car.yaw());
  auto sin_yaw = sin(-car.yaw());
  auto dx = map_x - car.x();
  auto dy = map_y - car.y();

  car_x = (dx * cos_yaw) - (dy * sin_yaw);
  car_y = (dx * sin_yaw) + (dy * cos_yaw);
}

void transform_car_to_map(const Vehicle &car,
                          double car_x, double car_y,
                          double &map_x, double &map_y) {
	auto cos_yaw = cos(car.yaw());
	auto sin_yaw = sin(car.yaw());

  map_x = (car_x * cos_yaw) - (car_y * sin_yaw) + car.x();
	map_y = (car_x * sin_yaw) + (car_y * cos_yaw) + car.y();
}
 */
} // unnamed namespace

Planner::Planner(const Map &map) : m_map(map) {
  m_state = STATE_STARTUP;
  m_state_time = 0;
  m_current_lane = 1;
  m_desired_lane = 1;
  m_desired_speed = SPEED_LIMIT * 0.9;
}

void Planner::vehicles_reset() {
  for (int lane=0; lane < Map::NUM_LANES; ++lane) {
    m_lane_vehicles[lane].clear();
   m_lane_vehicles_org[lane].clear();
  }
}

void Planner::vehicles_add(Vehicle vehicle) {
  auto lane = m_map.lane_from_d(vehicle.d());

  if (lane >= 0 && lane < Map::NUM_LANES) {
    m_lane_vehicles[lane].push_back(vehicle);
    m_lane_vehicles_org[lane].push_back(vehicle);
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
  
  m_last_delta = trajectory[0].size() * TIMESTEP;

  // generate new target points if necessary
/*  bool trajectory_ok = m_targets.size() >= 2;

  if (!trajectory_ok && 
      m_last_ego.speed() < mph_to_mps(40) && m_last_ego.speed() > mph_to_mps(30)) {
      trajectory_ok = try_changing_lane();
  }

  if (!trajectory_ok) {
    // reset simulation to the end of the previous trajectory
    generate_keep_lane_targets();
  } */

  if (m_targets.size() < 2) {
    process_state();
  }

  if (m_targets.size() < 2) {
    generate_keep_lane_targets();
  }

  reset_simulation();
  generate_trajectory(1.0, false);

  // copy the needed part of the trajectory to the output
  int np = TRAJECTORY_POINTS - trajectory[0].size();

  for (int idx = 0; idx < np; ++idx) {
    auto xy = m_map.getXY(m_frenet_path[idx].m_s, m_frenet_path[idx].m_d);
    trajectory[0].push_back(xy[0]);
    trajectory[1].push_back(xy[1]);

    m_state_time += TIMESTEP;
  }

  // save state of ego at the end of the trajectory
  int p1 = trajectory[0].size() - 1;
  int p2 = p1 - 1;

  m_last_ego = Vehicle(m_frenet_path[np-1].m_s, m_frenet_path[np-1].m_d, m_speeds[np-1]);

  // remove targets that are reached in this trajectory
  std::vector<FrenetPoint> m_old_targets = m_targets;
  m_targets.clear();

  for (auto target : m_old_targets) {
    if (transform_s_map_to_car(m_last_ego, target.m_s) > 0) {
      m_targets.push_back(target);
    } else {
      m_last_target = target;
    }
  }

  std::cout << "Ego-s = " << m_last_ego.s()
            << " target[0].s = " << m_targets[0].m_s
            << " target[1].s = " << m_targets[1].m_s
            << std::endl;
}

void Planner::reset_simulation() {

  // restore original sensor fusion data
  for (int lane = 0; lane < Map::NUM_LANES; ++lane) {
    m_lane_vehicles[lane] = m_lane_vehicles_org[lane];
  }

  // simulate the movement of the detected vehicles up to the end of the current trajectory
  predict_vehicles(m_last_delta);

  m_frenet_path.clear();
  m_speeds.clear();
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
      if (m_state_time >= CHANGE_LANE_COOLDOWN && m_last_ego.speed() < mph_to_mps(42)) {
        if (try_changing_lane()) {
          change_state(STATE_CHANGING_LANE);
        }
      }
      break;

    case STATE_CHANGING_LANE :
      if (fabs(m_last_ego.d() - m_map.lane_center(m_desired_lane)) < 0.1) {
        m_current_lane = m_desired_lane;
        change_state(STATE_KEEP_LANE);
      }

  }

}

void Planner::change_state(int p_new_state) {
  std::cout << "STATE CHANGE from " << STATE_NAMES[m_state]
            << " to " << STATE_NAMES[p_new_state]
            << std::endl;

  m_state = p_new_state;
  m_state_time = 0;

}

bool Planner::try_changing_lane() {

  // build a list of potential lanes
  std::vector<int> potential_lanes = {1};
  if (m_current_lane == 1) {
    potential_lanes = {0, 2};
  }

  // try each potential lane
  for (int lane : potential_lanes) {
    reset_simulation();
    generate_change_lane_targets(lane);

    if (generate_trajectory(5.0, true)) {
      m_desired_lane = lane;
      return true;
    }
  }

  return false;
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

  auto target_d = m_map.lane_center(desired_lane);
  m_targets.push_back({m_last_ego.s() + 15, (target_d + m_last_ego.d()) / 2.0});
  m_targets.push_back({m_last_ego.s() + 30, target_d});
}

bool Planner::generate_trajectory(double delta_t, bool check_collision) {

  Vehicle sim_ego = m_last_ego;

  // create a list of control points for the spline
  std::vector<double> control_s = {
    m_last_target.m_s,
    sim_ego.s(),
    m_targets[0].m_s,
    m_targets[1].m_s,
    m_targets[1].m_s + 30,
  };

  std::vector<double> control_d = {
    m_last_target.m_d,
    sim_ego.d(),
    m_targets[0].m_d,
    m_targets[1].m_d,
    m_targets[1].m_d,
  };

  // transfrom the control points to be relative to the car (to avoid wrapping problems)
  for (auto &c : control_s) {
    c = transform_s_map_to_car(m_last_ego, c);
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

  double cur_speed = sim_ego.speed();

  for (double t=0.0; t < delta_t; t += TIMESTEP) {

      // update predictions for the detected vehicles
      predict_vehicles(TIMESTEP);

      // adjust desired speed depending on the traffic in front
      Vehicle nearest_veh;

      double nearest_front = check_nearest_vehicle_up_front(map_s, m_current_lane, nearest_veh);

      if (nearest_front < 10) {
        m_desired_speed = nearest_veh.speed() * 0.75;
      } else if (nearest_front < 16) {
        m_desired_speed = nearest_veh.speed();
      } else {
        m_desired_speed = SPEED_LIMIT * 0.95;
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
      map_s = transform_s_car_to_map(m_last_ego, car_s);
			double new_d = path(car_s);

      // check for collisions
      if (check_collision && collision_with_vehicle(map_s, new_d)) {
        return false;
      }

      // store path
      m_frenet_path.push_back({map_s, new_d});
      m_speeds.push_back(cur_speed);
  }

  return true;

}

double Planner::distance_in_lane(double ego_s, double veh_s) {
  return veh_s - ego_s;
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
  const double D_RADIUS = 1.5;
  const double S_RADIUS = 8;

  for (int lane = 0; lane < Map::NUM_LANES; ++lane) {
    for (auto &veh : m_lane_vehicles[lane]) {
      if (fabs(distance_in_lane(ego_s, veh.s())) < S_RADIUS &&
          fabs(ego_d - veh.d()) < D_RADIUS) {
        return true;
      }
    }
  }
}
