#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

// Set the timestep length and duration
const size_t N  = 10;
const double dt = 0.1;

const size_t x_start = 0;
const size_t y_start = x_start+N;
const size_t psi_start = y_start+N;
const size_t v_start = psi_start+N;
const size_t cte_start = v_start+N;
const size_t epsi_start = cte_start+N;

const size_t delta_start = epsi_start+N;
const size_t a_start = delta_start+N-1;

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
// double Lf = 2.67;

class FG_eval {
private:
  double Lf;
  double target_cte;
  double target_epsi;
  double target_v;

public:
  // Fitted polynomial coefficients
  Eigen::VectorXd poly_coeffs;
  FG_eval(Eigen::VectorXd poly_coeffs) {
    this->poly_coeffs = poly_coeffs;
    Lf = 2.67;
    target_cte = 0;
    target_epsi = 0;
    target_v = 50;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // Implement MPC
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    // NOTE: You'll probably go back and forth between this function and
    // the Solver function below.
    fg[0] = 0;

    // The part of the cost based on the reference state.
    for (uint i=0; i<N; i++) {
      fg[0] += CppAD::pow(vars[cte_start+i]-target_cte, 2);
      fg[0] += CppAD::pow(vars[epsi_start+i]-target_epsi, 2);
      fg[0] += CppAD::pow(vars[v_start+i]-target_v, 2);
    }

    // Minimize the use of actuators.
    for (uint i=0; i <(N-1); i++) {
      fg[0] += CppAD::pow(vars[delta_start+i], 2);
      fg[0] += CppAD::pow(vars[a_start+i], 2);
    }

    // Minimize the value gap between sequential actuations.
    for (uint i=0; i<(N-2); i++) {
      fg[0] += 400 * CppAD::pow(vars[delta_start+i+1] - vars[delta_start+i], 2);
      fg[0] += CppAD::pow(vars[a_start+i+1] - vars[a_start+i], 2);
    }

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[1+x_start] = vars[x_start];
    fg[1+y_start] = vars[y_start];
    fg[1+psi_start] = vars[psi_start];
    fg[1+v_start] = vars[v_start];
    fg[1+cte_start] = vars[cte_start];
    fg[1+epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (uint i=0; i<(N-1); i++) {
      // The state at time t+1 .
      AD<double> x1 = vars[x_start+i+1];
      AD<double> y1 = vars[y_start+i+1];
      AD<double> psi1 = vars[psi_start+i+1];
      AD<double> v1 = vars[v_start+i+1];
      AD<double> cte1 = vars[cte_start+i+1];
      AD<double> epsi1 = vars[epsi_start+i+1];

      // The state at time t.
      AD<double> x0 = vars[x_start+i];
      AD<double> y0 = vars[y_start+i];
      AD<double> psi0 = vars[psi_start+i];
      AD<double> v0 = vars[v_start+i];
      AD<double> cte0 = vars[cte_start+i];
      AD<double> epsi0 = vars[epsi_start+i];

      // Only consider the actuation at time t.
      AD<double> delta0 = vars[delta_start+i];
      AD<double> a0 = vars[a_start+i];

      AD<double> f0 = poly_coeffs[0] +
                      poly_coeffs[1]*x0 +
                      poly_coeffs[2]*x0*x0 +
                      poly_coeffs[3]*x0*x0*x0;
      AD<double> psides0 = CppAD::atan(poly_coeffs[1] +
                                       2*poly_coeffs[2]*x0 +
                                       3*poly_coeffs[3]*x0*x0);

      // x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      fg[2+x_start+i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);

      // y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      fg[2+y_start+i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);

      // psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      fg[2+psi_start+i] = psi1 - (psi0 + v0 * delta0 / Lf * dt);

      // v_[t+1] = v[t] + a[t] * dt
      fg[2+v_start+i] = v1 - (v0 + a0 * dt);

      // cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      fg[2+cte_start+i] = cte1 - ((f0 - y0) + v0 * CppAD::sin(epsi0) * dt);

      // epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
      fg[2+epsi_start+i] = epsi1 - ((psi0 - psides0) + v0 * delta0 / Lf * dt);
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd poly_coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  // 4 * 10 + 2 * 9
  const size_t n_vars =  N * 8 - 2;
  // Set the number of constraints
  const size_t n_constraints = N * 6;

  const double x = state[0];
  const double y = state[1];
  const double psi = state[2];
  const double v = state[3];
  const double cte = state[4];
  const double epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (uint i=0; i<n_vars; i++) {
    vars[i] = 0;
  }

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);

  // Set lower and upper limits for variables.
  for (uint i=0; i<delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  for (uint i=delta_start; i<a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  for (uint i=a_start; i<n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (uint i = 0; i<n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

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
  FG_eval fg_eval(poly_coeffs);

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

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> mpc_output;
  // actuators output
  mpc_output.push_back(solution.x[delta_start]);
  mpc_output.push_back(solution.x[a_start]);

  // coordinates pair output
  for (uint i=1; i<N; i++) {
    // mpc_x
    mpc_output.push_back(solution.x[i]);
    // mpc_y
    mpc_output.push_back(solution.x[y_start+i]);
  }

  return mpc_output;
}
