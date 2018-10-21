#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <chrono>

using CppAD::AD;
using namespace std::chrono;

// Set the timestep length and duration
size_t N = 15;
double dt = .05;

const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N - 1;

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

class FG_eval {
public:
    // Fitted polynomial coefficients
    Eigen::VectorXd coeffs_;
    Weights weights_;
    VelocityTarget velocity_;

    FG_eval(Eigen::VectorXd &coeffs, Weights weights, VelocityTarget velocity) : coeffs_(coeffs), weights_{weights},
                                                                                 velocity_{velocity} {}

    typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

    void operator()(ADvector &fg, const ADvector &vars) {
        AD<double> cost = 0;

        // calculate cost
        for (auto i = 0; i < N; i++) {
            cost += CppAD::pow(vars[cte_start + i], 2);
            cost += CppAD::pow(vars[epsi_start + i], 2);
            cost += CppAD::pow(vars[v_start + i] - velocity_.getVariableVelocity(), 2);
//       cost += CppAD::pow(CppAD::abs(vars[cte_start + i]), 2);
        }

        for (auto i = 0; i < N - 1; i++) {
            cost += weights_.steering_actuator_weight * CppAD::pow(vars[delta_start + i], 2);
            cost += weights_.accelerator_actuator_weight * CppAD::pow(vars[a_start + i], 2);
        }

        for (auto i = 0; i < N - 2; i++) {
            cost += weights_.steering_smoothing_weight *
                    CppAD::pow(vars[delta_start + i + 1] - vars[delta_start + i], 2);
            cost += weights_.accelerator_smoothing_weight * CppAD::pow(vars[a_start + i + 1] - vars[a_start + i], 2);
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
        for (auto i = 1; i < N; i++) {
            // at timestep t + 1
            AD<double> x1 = vars[x_start + i];
            AD<double> y1 = vars[y_start + i];
            AD<double> psi1 = vars[psi_start + i];
            AD<double> v1 = vars[v_start + i];
            AD<double> cte1 = vars[cte_start + i];
            AD<double> epsi1 = vars[epsi_start + i];

            // at timestep t
            AD<double> x0 = vars[x_start + i - 1];
            AD<double> y0 = vars[y_start + i - 1];
            AD<double> psi0 = vars[psi_start + i - 1];
            AD<double> v0 = vars[v_start + i - 1];
            AD<double> cte0 = vars[cte_start + i - 1];
            AD<double> epsi0 = vars[epsi_start + i - 1];

            AD<double> delta0 = vars[delta_start + i - 1];
            AD<double> a0 = vars[a_start + i - 1];

            AD<double> f0 = coeffs_[0] + coeffs_[1] * x0 + coeffs_[2] * x0 * x0 + coeffs_[3] * x0 * x0 * x0;
            AD<double> psides0 = CppAD::atan(3 * coeffs_[3] * x0 * x0 + 2 * coeffs_[2] * x0 + coeffs_[1]);

            fg[1 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
            fg[1 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
            fg[1 + psi_start + i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);
            fg[1 + v_start + i] = v1 - (v0 + a0 * dt);
            fg[1 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
            fg[1 + epsi_start + i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
        }
    }
};

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double throttle, double steering_angle) {
    bool ok = true;
    typedef CPPAD_TESTVECTOR(double) Dvector;

    auto x = state[0];
    auto y = state[1];
    auto psi = state[2];
    auto v = state[3];
    auto cte = state[4];
    auto epsi = state[5];

    x += v * latency_in_seconds_ * cos(psi);
    y += v * latency_in_seconds_ * sin(psi);
    psi += prev_steering_ * v * latency_in_seconds_ / Lf;
    v += prev_throttle_ * latency_in_seconds_;
    cte = y - cte;
    epsi = psi - epsi;

    // cout << "current cte: " << cte << endl;

    size_t n_vars = N * 6 + (N - 1) * 2;
    size_t n_constraints = 6 * N;

    // Initial value of the independent variables.
    // SHOULD BE 0 besides initial state.
    Dvector vars(n_vars);
    for (int i = 0; i < n_vars; i++) {
        vars[i] = 0;
    }

    vars[x_start] = x;
    vars[y_start] = y;
    vars[psi_start] = psi;
    vars[v_start] = v;
    vars[cte_start] = cte;
    vars[epsi_start] = epsi;

    Dvector vars_lowerbound(n_vars);
    Dvector vars_upperbound(n_vars);
    // set all non-actuartor upper and lower limits
    // to the max negative and positive values
    for (auto i = 0; i < delta_start; i++) {
        vars_lowerbound[i] = -1.0e19;
        vars_upperbound[i] = 1.0e19;
    }

    // The upper and lower limits of delta are set to -25 and 25
    // degrees (values in radians).
    for (auto i = delta_start; i < a_start; i++) {
        vars_lowerbound[i] = -.436332;
        vars_upperbound[i] = .436332;
    }

    // Acceleration/decceleration upper and lower limits.
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

    updateLatency();
    // cout << "latency in seconds: " << latency_in_seconds_ << endl;

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

    // calculate target velocity based on cte
    velocity_.updateVelocityBasedOnError(cte);

    // cout << "calc'd target velocity: " << velocity_.getVariableVelocity() << " with cte " << cte << endl;

    // object that computes objective and constraints
    FG_eval fg_eval{coeffs, weights_, velocity_};

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
    double cost = solution.obj_value;

    // std::cout << "Cost " << cost << std::endl;

    auto steering = solution.x[delta_start];
    auto acceleration = solution.x[a_start];

    prev_steering_ = steering;
    prev_throttle_ = acceleration;

    vector<double> result;

    result.push_back(steering);
    result.push_back(acceleration);

    for (auto i = 0; i < N - 1; i++) {
        result.push_back(solution.x[x_start + i + 1]);
        result.push_back(solution.x[y_start + i + 1]);
    }

    return result;
}

void MPC::updateLatency() {
    auto current_time = getCurrentTime();
    latency_in_seconds_ = (current_time - prev_time_) / 1000. - dt;
    prev_time_ = current_time;
}

long long MPC::getCurrentTime() {
    milliseconds mills = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
    return mills.count();
}

void VelocityTarget::updateVelocityBasedOnError(double error) {
    auto eff_cte = abs(error);

    if (eff_cte > max_error_) calculated_velocity_ *= .9;
    else {
        calculated_velocity_ = (1 - eff_cte / max_error_) * target_velocity_;
    }
}

double VelocityTarget::getVariableVelocity() {
    return calculated_velocity_;
}
