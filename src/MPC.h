#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

extern size_t N;
extern double Lf;
extern double dt;
extern double ref_v;
extern int cte_wt;
extern int epsi_wt;
extern int v_wt;
extern int delta_wt;
extern int a_wt;
extern int delta_diff_wt;
extern int a_diff_wt;

class MPC
{
public:

  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  vector<double> Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
