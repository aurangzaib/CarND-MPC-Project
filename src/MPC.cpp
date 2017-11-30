#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>

using CppAD::AD;

/****************************
Define initial MPC parameters
****************************/

// define N and dt
// N  -> number of actuations
// dt -> time elapsed between actuations
// T  -> prediction horizon -> N * dt
const size_t N = 25;
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
const double ref_v = 45.0;

// weights for cost function
// weight determines importance
// of a cost function
const double W_CTE = 8.0;
const double W_EPSI = 0.30;
const double W_V = 0.261;
const double W_DELTA = 6000.0;
const double W_A = 0.00001;

/****************************
Define start array positions
****************************/

// The solver takes all the state variables and actuator
// variables in a singular vector.
// define array positions of state variables, errors, actuators
// define start positions
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;

  FG_eval(const Eigen::VectorXd &coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;

  void operator()(ADvector &fg, const ADvector &vars) {
    /*
     * fg is a vector where cost function and vehicle model/constraints are defined
     * vars is a vector containing all variables used by cost function i.e:
     *    x, y, v, psi --> state
     *    cte, epsi    --> errors
     *    delta,a      --> control inputs
     * fg[0] contains cost
     */

    /****************************
     Set Cost for Errors, Actuation
     and Sequential Smoothing
     ****************************/

    // set initial cost
    fg[0] = 0;
    // cost based on reference state
    for (int t = 0; t < N; t++) {
      // cte error
      fg[0] += W_CTE * CppAD::pow(vars[cte_start + t], 2);
      // orientation (heading) error
      fg[0] += W_EPSI * CppAD::pow(vars[epsi_start + t], 2);
      // velocity error
      fg[0] += W_V * CppAD::pow(vars[v_start + t] - ref_v, 2);
    }
    // cost based on control inputs
    // no abrupt control input change
    for (int t = 0; t < N - 1; t++) {
      fg[0] += W_DELTA * CppAD::pow(vars[delta_start + t], 2);
      fg[0] += W_A * CppAD::pow(vars[a_start + t], 2);
    }
    // minimize value gap between sequential actuations
    for (int t = 0; t < N - 2; t++) {
      fg[0] += W_DELTA * CppAD::pow(vars[delta_start + t + 1] - vars[delta_start + t], 2);
      fg[0] += W_A * CppAD::pow(vars[a_start + t + 1] - vars[a_start + t], 2);
    }

    /****************************
     Set state and errors in constraints
     Actuation is not part of constraints
     ****************************/

    // set values of state, and errors variables
    fg[1 + x_start]     = vars[x_start];
    fg[1 + y_start]     = vars[y_start];
    fg[1 + psi_start]   = vars[psi_start];
    fg[1 + v_start]     = vars[v_start];
    fg[1 + cte_start]   = vars[cte_start];
    fg[1 + epsi_start]  = vars[epsi_start];

    // starting from 1
    // 0 is initial state
    // initial states are not part of optimizer solver
    for (int t = 1; t < N; t++) {
      AD<double> x0       = vars[x_start + t - 1], 
                 x1       = vars[x_start + t];
      AD<double> y0       = vars[y_start + t - 1], 
                 y1       = vars[y_start + t];
      AD<double> psi0     = vars[psi_start + t - 1], 
                 psi1     = vars[psi_start + t];
      AD<double> v0       = vars[v_start + t - 1], 
                 v1       = vars[v_start + t];
      AD<double> f0 = 0.0;
      for (int loop=0; loop < coeffs.size(); loop++) {
        f0 += coeffs[loop] * pow(x0, loop);
      }
      AD<double> cte0     = f0 - y0, 
                 cte1     = vars[cte_start + t];
      AD<double> psi_des  = 0.0;
      for (int loop = 1; loop < coeffs.size(); loop++) {
        psi_des += loop * coeffs[loop] * pow(x0, loop - 1);
      }
      psi_des = CppAD::atan(psi_des);
      AD<double> epsi0    = psi0 - psi_des, 
                 epsi1    = vars[epsi_start + t + 1];
      AD<double> delta    = vars[delta_start + t - 1];
      AD<double> a        = vars[a_start + t - 1];
      
      // using previous values for actuations
      // handling latency
      // if (t > 1) {
      //   a = vars[a_start + t - 2];
      //   delta = vars[delta_start + t - 2] * -1;
      // }
      
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + v0 * (delta / Lf) * dt);
      fg[1 + v_start + t] = v1 - (v0 + a * dt);
      fg[1 + cte_start + t] = cte1 - (cte0 + v0 * CppAD::sin(epsi0) * dt);
      fg[1 + epsi_start + t] = epsi1 - (epsi0 + v0 * (delta / Lf) * dt);
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

  // Set the number of model variables (includes both states and errors).
  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // (10 * 6) + (9 * 2)
  // N actuations -> N - 1
  size_t n_vars = N * 6 + (N - 1) * 2;
  // number of constraints
  size_t n_constraints = N * 6;

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (size_t loop = 0; loop < n_vars; loop += 1) {
    vars[loop] = 0.0;
  }

  // set initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  // lower and upper boundaries for variables
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // boundaries for x, y, psi, v, cte, epsi
  for (int loop = 0; loop < delta_start; loop += 1) {
    vars_lowerbound[loop] = -1.0e19;
    vars_upperbound[loop] = +1.0e19;
  }
  // boundaries for steering angle
  for (size_t loop = delta_start; loop < a_start; loop += 1) {
    vars_lowerbound[loop] = -25 * M_PI / 180;
    vars_upperbound[loop] = +25 * M_PI / 180;
  }
  // boundaries for acceleration
  for (size_t loop = a_start; loop < n_vars; loop += 1) {
    vars_lowerbound[loop] = -1.0;
    vars_upperbound[loop] = +1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);

  // all are zero except the initial state indices
  for (size_t loop = 0; loop < n_constraints; loop += 1) {
    constraints_lowerbound[loop] = 0.0;
    constraints_upperbound[loop] = 0.0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_upperbound[x_start] = x;

  constraints_lowerbound[y_start] = y;
  constraints_upperbound[y_start] = y;

  constraints_lowerbound[psi_start] = psi;
  constraints_upperbound[psi_start] = psi;

  constraints_lowerbound[v_start] = v;
  constraints_upperbound[v_start] = v;

  constraints_lowerbound[cte_start] = cte;
  constraints_upperbound[cte_start] = cte;

  constraints_lowerbound[epsi_start] = epsi;
  constraints_upperbound[epsi_start] = epsi;

  // instance which computes objectives & constraints
  FG_eval fg_eval(coeffs);

  // options for IPOPT solver
  std::string options;
  options += "Integer print_level  0\n";
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(options, vars, vars_lowerbound,
                                        vars_upperbound, constraints_lowerbound,
                                        constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // https://www.coin-or.org/CppAD/Doc/ipopt_solve.htm

  // cost
  auto cost = solution.obj_value;

  // steering angle (delta) and throttle (a)
  vector<double> actuations_and_mpc_xy = {solution.x[delta_start], solution.x[a_start]};

  // TODO:
  // instead of returning any value
  // these values can be stored in object of MPC class
  // then directly use them in main class instead of returning from here

  // x and y trajectory
  for (int loop = 0; loop < N - 1; loop += 1) {
    actuations_and_mpc_xy.push_back(solution.x[x_start + loop + 1]);
    actuations_and_mpc_xy.push_back(solution.x[y_start + loop + 1]);
  }

  std::cout << "Cost " << cost << std::endl;
  return actuations_and_mpc_xy;
}
