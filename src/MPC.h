#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

/**
 * Parameter weights to tune the MPC algorithm.
 */
class Weights {
public:
    double steering_actuator_weight;
    double accelerator_actuator_weight;
    double steering_smoothing_weight;
    double accelerator_smoothing_weight;
};

/**
 * Represents the target or reference velocity to match.
 * Has variable velocity based on error.
 */
class VelocityTarget {
public:
    explicit VelocityTarget(double target_velocity) : target_velocity_(target_velocity), calculated_velocity_(target_velocity) {}

    double target_velocity_;
    void updateVelocityBasedOnError(double error);
    double getVariableVelocity();
private:
    /**
     * calculated velocity based on error
     */
    double calculated_velocity_;
    const double max_error_ = 7;
};

/**
 * Model predictive controller to predict steering and throttle.
 */
class MPC {
public:
    MPC(double target_velocity,
        double steering_actuator_weight,
        double accelerator_actuator_weight,
        double steering_smoothing_weight,
        double accelerator_smoothing_weight): weights_{
            steering_actuator_weight,
            accelerator_actuator_weight,
            steering_smoothing_weight,
            accelerator_smoothing_weight}, velocity_{target_velocity} {};

    virtual ~MPC() = default;

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuations.
    vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, double throttle, double steering_angle);

private:
    Weights weights_;
    VelocityTarget velocity_;
    double latency_in_seconds_{.1};
    double prev_steering_{0.};
    double prev_throttle_{0.};

    static long long getCurrentTime();
    long long prev_time_{getCurrentTime()};

    void updateLatency();
};



#endif /* MPC_H */
