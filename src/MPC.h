#ifndef MPC_H
#define MPC_H

#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <cppad/cppad.hpp>
#include <vector>

using namespace std;
using CppAD::AD;

/****************************
MPC hyper parameters
****************************/

struct helper {

  // N  -> number of actuations
  // dt -> time elapsed between actuations
  // T  -> prediction horizon -> N * dt
  const int N = 12;
  const double dt = 0.1;

  // Model was obtained by measuring the radius formed by running the vehicle in the
  // simulator around in a circle with a constant steering angle and velocity on a
  // flat terrain.
  //
  // Lf was tuned until the the radius formed by the simulating the model
  // presented in the classroom matched the previous radius.
  //
  // length from front to CoG that has a similar radius.
  const double Lf = 2.67;

  // reference velocity
  const double ref_v = 110;

  // weights for cost functions importance
  const double weight_cte = 12e3;
  const double weight_epsi = 12e2;
  const double weight_v = 1;
  const double weight_delta = 12e2;
  const double weight_delta_change = 12;
  const double weight_a = 12;
  const double weight_a_change = 12;

  // evaluate polynomial
  template<typename T>
  T polyeval(const Eigen::VectorXd &coeffs, const T &x) {
    T result = 0.0;
    for (int loop = 0; loop < coeffs.size(); loop++) {
      result += coeffs[loop] * pow(x, loop);
    }
    return result;
  }

  // evaluate polynomial and return value is for angles
  template<typename T>
  T polyeval(const Eigen::VectorXd &coeffs, const T &x, bool is_angle) {
    T result = 0.0;
    for (int loop = 1; loop < coeffs.size(); loop++) {
      result += loop * coeffs[loop] * pow(x, loop - 1);
    }
    return CppAD::atan(result);
  }

  // Fit a polynomial.
  // https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
  template<typename T>
  T polyfit(const T &xvals, const T &yvals, const int order) {
    assert(xvals.size() == yvals.size());
    assert(order >= 1 && order <= xvals.size() - 1);
    Eigen::MatrixXd A(xvals.size(), order + 1);

    for (int i = 0; i < xvals.size(); i++) {
      A(i, 0) = 1.0;
    }

    for (int j = 0; j < xvals.size(); j++) {
      for (int i = 0; i < order; i++) {
        A(j, i + 1) = A(j, i) * xvals(j);
      }
    }

    auto Q = A.householderQr();
    T result = Q.solve(yvals);
    return result;
  }

  // degree to radian
  double deg2rad(double x) { return x * M_PI / 180; }

  // radian to degree
  double rad2deg(double x) { return x * 180 / M_PI; }

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
