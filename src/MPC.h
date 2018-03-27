#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

const size_t N = 10;
const double dt = 0.1;

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

const size_t x_start = 0;
const size_t y_start = x_start + N;
const size_t psi_start = y_start + N;
const size_t v_start = psi_start + N;
const size_t cte_start = v_start + N;
const size_t epsi_start = cte_start + N;
const size_t delta_start = epsi_start + N;
const size_t a_start = delta_start + N -1;

// Reference CTE, EPSI, and speed
const double ref_cte = 0.0;
const double ref_epsi = 0.0;
const double ref_v = 70.0;

// Weights for the cost function
const double w_cte = 2500.0;
const double w_epsi = 3200.0;
const double w_v = 1.0;
const double w_delta = 7.0;
const double w_a = 4.0;
const double w_dv = 700.0;
const double w_ddelta = 200.0;
const double w_da = 10.0;

const double deg25rad = 0.436332;
const double bound = 1.0e19;
const double maxthrottle = 1.0;

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
