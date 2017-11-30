#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

/****************************
MPC hyper parameters
****************************/

struct hyper_params {

  // N  -> number of actuations
  // dt -> time elapsed between actuations
  // T  -> prediction horizon -> N * dt
  const size_t N = 18;
  const double dt = 0.05;

  // Model was obtained by measuring the radius formed by running the vehicle in the
  // simulator around in a circle with a constant steering angle and velocity on a
  // flat terrain.
  //
  // Lf was tuned until the the radius formed by the simulating the model
  // presented in the classroom matched the previous radius.
  //
  // This is the length from front to CoG that has a similar radius.
  const double Lf = 2.67;

  // reference velocity
  const double ref_v = 48;

  // weights for cost functions importance
  const double weight_cte = 3e+1;
  const double weight_epsi = 3e+1;
  const double weight_v = 1;
  const double weight_delta = 3e+4;
  const double weight_delta_change = 3e+2;
  const double weight_a = 3e-2;
  const double weight_a_change = 3e-2;
};

class MPC {
public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
