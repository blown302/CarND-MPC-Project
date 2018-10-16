#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

extern double steering_actuator_weight;
extern double error_weight;
extern double psi_error_weight;
extern double steering_smoothing_weight;
extern double accelerator_actuator_weight;
extern double accelerator_smoothing_weight;
extern double target_v;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuations.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
