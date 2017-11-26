#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <limits>

using CppAD::AD;

// TODO: Set the timestep length and duration
const size_t N = 10;
const double dt = 0.075;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// reference speed
const double ref_v = 60;

// ranges of variables in the arrays
const size_t X_FST = 0;
const size_t X_LST = X_FST + N - 1;
const size_t Y_FST = X_LST + 1;
const size_t Y_LST = Y_FST + N - 1;
const size_t PSI_FST = Y_LST + 1;
const size_t PSI_LST = PSI_FST + N - 1;
const size_t V_FST = PSI_LST + 1;
const size_t V_LST = V_FST + N - 1;
const size_t CTE_FST = V_LST + 1;
const size_t CTE_LST = CTE_FST + N - 1;
const size_t EPSI_FST = CTE_LST + 1;
const size_t EPSI_LST = EPSI_FST + N - 1;
const size_t DELTA_FST = EPSI_LST + 1;
const size_t DELTA_LST = DELTA_FST + N - 2;
const size_t A_FST = DELTA_LST + 1;
const size_t A_LST = A_FST + N - 2;

const size_t STATE_FST = X_FST;
const size_t STATE_LST = EPSI_LST;
const size_t ACTUATOR_FST = DELTA_FST;
const size_t ACTUATOR_LST = A_LST;


class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // TODO: implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;

    // the cost based on the reference state 
    for (auto t = 0ul; t < N; ++t) {
      fg[0] += 10000 * CppAD::pow(vars[CTE_FST + t], 2);    
      fg[0] += 10000 * CppAD::pow(vars[EPSI_FST + t], 2);
      fg[0] += 10 * CppAD::pow(vars[V_FST + t] - ref_v, 2);
    }

    // minimize the use of actuators.
    for (auto t = 0ul; t < N - 1; t++) {
      fg[0] += 50 * CppAD::pow(vars[DELTA_FST + t], 2);
      fg[0] += 50 * CppAD::pow(vars[A_FST + t], 2);
    }

    // minimize the value gap between sequential actuations.
    for (auto t = 0ul; t < N - 2; t++) {
      fg[0] += 12500 * CppAD::pow(vars[DELTA_FST + t + 1] - vars[DELTA_FST + t], 2);
      fg[0] += 5000 * CppAD::pow(vars[A_FST + t + 1] - vars[A_FST + t], 2);
    }

    // setup constraints

    // >> set initial constraints
    fg[1 + X_FST] = vars[X_FST];
    fg[1 + Y_FST] = vars[Y_FST];
    fg[1 + PSI_FST] = vars[PSI_FST];
    fg[1 + V_FST] = vars[V_FST];
    fg[1 + CTE_FST] = vars[CTE_FST];
    fg[1 + EPSI_FST] = vars[EPSI_FST];

    std::cout << vars[A_FST] << std::endl;

    // >> set constraints for all the steps of the predicted path
    for (auto t= 1ul; t < N; ++t) {

      // the state at the next timestep
      auto x1 = vars[X_FST + t];
      auto y1 = vars[Y_FST + t];
      auto psi1 = vars[PSI_FST + t];
      auto v1 = vars[V_FST + t];
      auto cte1 = vars[CTE_FST + t];
      auto epsi1 = vars[EPSI_FST + t];

      // the state at the current timestep
      auto x0 = vars[X_FST + t - 1];
      auto y0 = vars[Y_FST + t - 1];
      auto psi0 = vars[PSI_FST + t - 1];
      auto v0 = vars[V_FST + t - 1];
      auto cte0 = vars[CTE_FST + t - 1];
      auto epsi0 = vars[EPSI_FST + t - 1];

      auto delta0 = vars[DELTA_FST + t - 1];
      auto a0 = vars[A_FST + t - 1];

      // compute trajectory and derivative at current timestep
      auto f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0, 2) + coeffs[3] * CppAD::pow(x0, 3);
      auto psides0 = CppAD::atan(coeffs[1] + 2 * coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0, 2));

      // apply the vehicle model to predict the state at the next timestemp
      fg[1 + X_FST + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + Y_FST + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + PSI_FST + t] = psi1 - (psi0 - v0/Lf * delta0 * dt);
      fg[1 + V_FST + t] = v1 - (v0 + a0 * dt);
      fg[1 + CTE_FST + t] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[1 + EPSI_FST + t] = epsi1 - ((psi0 - psides0) - v0/Lf * delta0 * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  const auto x = state[0];
  const auto y = state[1];
  const auto psi = state[2];
  const auto v = state[3];
  const auto cte = state[4];
  const auto epsi = state[5];
  const auto delta = state[6];
  const auto a = state[7];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = (6 * N) + (2 * (N-1));
  // TODO: Set the number of constraints
  size_t n_constraints = 6 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  vars[X_FST] = x;
  vars[Y_FST] = y;
  vars[PSI_FST] = psi;
  vars[V_FST] = v;
  vars[CTE_FST] = cte;
  vars[EPSI_FST] = epsi;
  vars[DELTA_FST] = delta;
  vars[A_FST] = a;

  // TODO: Set lower and upper limits for variables.
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // set all non-actuators upper and lowerlimits to min & max values
  for (size_t i = STATE_FST; i <= STATE_LST; ++i) {
    vars_lowerbound[i] = -std::numeric_limits<double>::max();
    vars_upperbound[i] = std::numeric_limits<double>::max();
  }

  // the upper and lower limits for the steering (delta) are -25 to 25 degrees
  for (size_t i = DELTA_FST; i <= DELTA_LST; ++i) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // the upper and lower limits for the acceleration are -1 and 1
  for (size_t i = A_FST; i <= A_LST; ++i) {
    vars_lowerbound[i] = -1;
    vars_upperbound[i] = 1;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (auto i = 0ul; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[X_FST] = vars[X_FST];
  constraints_lowerbound[Y_FST] = vars[Y_FST];
  constraints_lowerbound[PSI_FST] = vars[PSI_FST];
  constraints_lowerbound[V_FST] = vars[V_FST];
  constraints_lowerbound[CTE_FST] = vars[CTE_FST];
  constraints_lowerbound[EPSI_FST] = vars[EPSI_FST];

  constraints_upperbound[X_FST] = vars[X_FST];
  constraints_upperbound[Y_FST] = vars[Y_FST];
  constraints_upperbound[PSI_FST] = vars[PSI_FST];
  constraints_upperbound[V_FST] = vars[V_FST];
  constraints_upperbound[CTE_FST] = vars[CTE_FST];
  constraints_upperbound[EPSI_FST] = vars[EPSI_FST];


  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  std::vector<double> result = {solution.x[DELTA_FST], solution.x[A_FST]};

  // also add the predicted x/y values to be visualised
  for (auto i=0ul; i<N-1; ++i) {
    result.push_back(solution.x[X_FST + i + 1]);
    result.push_back(solution.x[Y_FST + i + 1]);
  }

  return result;
}
