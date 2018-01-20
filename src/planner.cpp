#include "planner.h"
#include "spline.h"

#include <iostream>

namespace {

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
  m_current_lane = 1;
  m_desired_lane = 1;
  m_desired_speed = SPEED_LIMIT * 0.9;
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

  // reset simulation to the end of the previous trajectory
  reset_simulation(ego, trajectory[0].size());
  
  // generate new target points if necessary
  if (m_targets.size() < 2) {
    generate_keep_lane_targets();
  }

  // generate new trajectory, look ahead far enough to avoid bad trajectories
  generate_trajectory(2.0);

  // copy the needed part of the trajectory to the output
  int np = TRAJECTORY_POINTS - trajectory[0].size();

  for (int idx = 0; idx < np; ++idx) {
    auto xy = m_map.getXY(m_frenet_path[idx].m_s, m_frenet_path[idx].m_d);
    trajectory[0].push_back(xy[0]);
    trajectory[1].push_back(xy[1]);
  }

  // save state of ego at the end of the trajectory
  int p1 = trajectory[0].size() - 1;
  int p2 = p1 - 1;

  m_last_ego = Vehicle(m_frenet_path[np-1].m_s, m_frenet_path[np-1].m_d, m_speeds[np-1]);

  // remove targets that are reached in this trajectory
  std::vector<FrenetPoint> m_old_targets = m_targets;
  m_targets.clear();

  for (auto target : m_old_targets) {
    if (target.m_s > m_last_ego.s()) {
      m_targets.push_back(target);
    }
  }
}

void Planner::reset_simulation(Vehicle ego, int prev_trajectory_len) {

  // simulate the movement of the detected vehicles up to the end of the current trajectory
  predict_vehicles(prev_trajectory_len * TIMESTEP);

  // reset ego state if trajectory is empty
  if (prev_trajectory_len == 0) {
    m_last_ego = ego;
  }

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

void Planner::generate_keep_lane_targets() {

  if (m_targets.size() < 1) {
    m_targets.push_back({m_last_ego.s() + 15, m_last_ego.d()});
  }

  if (m_targets.size() < 2) {
    m_targets.push_back({m_targets[0].m_s + 15, m_targets[0].m_d});
  }
}

bool Planner::generate_trajectory(double delta_t) {

  Vehicle sim_ego = m_last_ego;

  // create a list of control points for the spline
  std::vector<double> control_s = {
    sim_ego.s(),
    m_targets[0].m_s,
    m_targets[1].m_s,
    m_targets[1].m_s + 30,
    m_targets[1].m_s + 60,
  };

  std::vector<double> control_d = {
    sim_ego.d(),
    m_targets[0].m_d,
    m_targets[1].m_d,
    m_targets[1].m_d,
    m_targets[1].m_d
  };

  // fit a spline to the control points
  tk::spline  path;
  path.set_points(control_s, control_d);

  // generate the new path from the spline
  double target_s = sim_ego.s() + 30;
	double target_d = path(target_s);
	double target_dist = distance(sim_ego.s(), sim_ego.d(), target_s, target_d);
	double new_s = sim_ego.s(); 
  double new_d = sim_ego.d();

  double cur_speed = sim_ego.speed();

  for (double t=0.0; t < delta_t; t += TIMESTEP) {

      // update predictions for the detected vehicles
      predict_vehicles(TIMESTEP);

      // adjust desired speed depending on the traffic in front
      Vehicle nearest_veh;

      double nearest_front = check_nearest_vehicle_up_front(new_s, m_current_lane, nearest_veh);

      if (nearest_front < 10) {
        m_desired_speed = nearest_veh.speed() * 0.75;
      } else if (nearest_front < 16) {
        m_desired_speed = nearest_veh.speed();
      } else {
        m_desired_speed = SPEED_LIMIT * 0.95;
      }

      std::cout << "nearest up front = " << nearest_front
                << " desired speed = " << m_desired_speed
                << std::endl;

      // update speed
      if (cur_speed <= m_desired_speed) {
        cur_speed += ACCELERATION * TIMESTEP;
      } else if (cur_speed >= m_desired_speed) {
        cur_speed -= ACCELERATION * TIMESTEP;
      }

      std::cout << " speed = " << cur_speed << std::endl;

      // calculate position after this timestep
      double n = target_dist / (TIMESTEP * cur_speed);
			new_s = new_s + (30 / n);
			new_d = path(new_d);

      // XXX check for collisions

      // store path
      m_frenet_path.push_back({new_s, new_d});
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

