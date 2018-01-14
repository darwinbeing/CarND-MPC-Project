#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

/** Unit conversion constants: */
static const double MMETERS_PER_MILE =  1609344.0;
static const double METERS_PER_MILE = MMETERS_PER_MILE / 1000.0;
static const int SECONDS_PER_MINUTE = 60;
static const int MINUTES_PER_HOUR = 60;
static const int64_t SECONDS_PER_HOUR = SECONDS_PER_MINUTE * MINUTES_PER_HOUR;

/** convert between meters per second and miles per hour */
static inline double mph2mps(double mph) {
  return mph * METERS_PER_MILE / SECONDS_PER_HOUR;
}

/** convert from meters per second to miles per hour  */
static inline double mps2mph(double mps) {
  return mps * SECONDS_PER_HOUR / METERS_PER_MILE;
}

// To convert between miles per hour (mph) and meters per second (m/s)
/* static inline double mph2mps(double mph) {return mph * 0.44704;} */
/* static inline double mps2mph(double mps) {return mps * 2.23694;} */

class MPC {
 public:
  vector<double> mpc_x_vals;
  vector<double> mpc_y_vals;

  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs, Eigen::VectorXd weights);
};

#endif  // MPC_H
