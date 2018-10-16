#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <chrono>

using CppAD::AD;
using namespace std::chrono;

long long getCurrentTime() {
    milliseconds mills = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    return mills.count();
}


// TODO: Set the timestep length and duration
size_t N = 10;
double dt = .1;
long long prev_time = getCurrentTime();
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

const double target_cte = 0;
const double target_epsi = 0;
double target_v = 10;
double steering_actuator_weight = 1;
double accelerator_actuator_weight = 1;
double error_weight = 300.;
double psi_error_weight = 300.;
double steering_smoothing_weight = 2;
double accelerator_smoothing_weight = 1;

const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

unsigned int iter_count = 0;
double running_cost = 0.;

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
    auto current_time = getCurrentTime();
// dt = (current_time - prev_time) / 1000.0;
//    cout << "delta time between operations: " << dt << endl;
    prev_time = current_time;
    AD<double> cost = 0;

    // calculate cost
    for (auto i = 0; i < N; i++) {
      cost += error_weight * CppAD::pow(vars[cte_start + i] - target_cte, 2);
      cost += psi_error_weight * CppAD::pow(vars[epsi_start + i] - target_epsi, 2);
      cost += CppAD::pow(vars[v_start + i] - target_v, 2);
      // cost += CppAD::pow(CppAD::abs(vars[cte_start + t]), 2);
    }

    for (auto i = 0; i < N -1; i++) {
      cost += steering_actuator_weight * CppAD::pow(vars[delta_start + i], 2);
      cost += accelerator_actuator_weight * CppAD::pow(vars[a_start + i], 2);
    }

    for (auto i = 0; i < N - 2; i++) {
      cost += steering_smoothing_weight * CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
      cost += accelerator_smoothing_weight * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
    }
    
    // TODO: remove print 

    // TODO: remove printt
    // cout << "cost after cost function: " << cost << endl;

    // fg at index 0 is cost
    fg[0] = cost;


    // Setup constraints
    // We add 1 to each of the staring indices due to cost being located at 
    // index 0 of `fg`.
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // set rest of the contraints
    for (auto i = 0; i < N -1; i++) {
      // at timestep t + 1
      AD<double>  x1 = vars[x_start + i + 1];
      AD<double>  y1 = vars[y_start + i + 1];
      AD<double>  psi1 = vars[psi_start + i + 1];
      AD<double>  v1 = vars[v_start + i + 1];
      AD<double>  cte1 = vars[cte_start + i + 1];
      AD<double>  epsi1 = vars[epsi_start + i + 1];

      // at timestemp t
      AD<double> x0 = vars[x_start + i];
      AD<double> y0 = vars[y_start + i];
      AD<double> psi0 = vars[psi_start + i];
      AD<double> v0 = vars[v_start + i];
      AD<double> cte0 = vars[cte_start + i];
      AD<double> epsi0 = vars[epsi_start + i];
      
      AD<double> delta0 = vars[delta_start + i];
      AD<double> a0 = vars[a_start + i];
      
      //AD<double> f0 = coeffs[0] + coeffs[1] * x0;
      //AD<double> psides0 = CppAD::atan(coeffs[1]);
      AD<double> f0 = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
      AD<double> psides0 = CppAD::atan(3 * coeffs[3] * x0 * x0 + 2 * coeffs[2] * x0 + coeffs[1]);

      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt)) * -1;
      fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt) * -1;
      // TODO: remove print
      // cout << "x for index: " << i + 2 << " " << fg[2 + x_start + i] << endl;
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

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  size_t n_vars = N * 6 + (N - 1) * 2;
  // TODO: Set the number of constraints
  size_t n_constraints = 6 * N;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  
  // set all non-actuartor upper and lower limits
  // to the max negative and positive values
  for (auto i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
//    vars_lowerbound[i] = numeric_limits<double>::min();
    vars_upperbound[i] = 1.0e19;
//    vars_upperbound[i] = numeric_limits<double>::max();
  }
  
  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // TODO: tune this params
  for (auto i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -.436332 * Lf;
    vars_upperbound[i] = .436332 * Lf;
  }

  // Acceleration/decceleration upper and lower limits.
  // TODO: tune these values
  for (auto i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.;
    vars_upperbound[i] = 1.;
  }
  
  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Set upper and lower bound contraints from state
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;
 
  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;
  
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
  options += "Numeric max_cpu_time          30.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  double cost = solution.obj_value;
  running_cost += cost;
  iter_count++;
  const unsigned int thresh = 100;
  vector<double> costs_(thresh - 1);
  if (iter_count < thresh) {
    costs_[iter_count - 1] = cost;
  } else if (iter_count % (thresh + 1) == 0) {
    cout << "average cost: " << running_cost / iter_count << endl;
    auto standardDeviation = 0.;
    for(auto i = 0; i < costs_.size(); ++i)
      standardDeviation += pow(costs_[i] - running_cost / iter_count, 2);
    cout << "completed with std dev: " << sqrt(standardDeviation / costs_.size()) << endl;
  }
  
  // std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> result;
  
  result.push_back(solution.x[delta_start]);
  result.push_back(solution.x[a_start]);

  for (auto i = 0; i < N - 1; i++) {
    result.push_back(solution.x[x_start + i + 1]);
    result.push_back(solution.x[y_start + i + 1]);
  } 

  return result;
}
