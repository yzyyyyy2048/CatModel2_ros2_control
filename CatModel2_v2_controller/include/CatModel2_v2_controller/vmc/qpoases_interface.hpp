#ifndef QPOASES_INTERFACE_HPP
#define QPOASES_INTERFACE_HPP

#include "qp_solver.hpp"

class QPoasesInterface : public QPSolver {
 public:
  // Constructor.
  QPoasesInterface();

  // Destructor.
  ~QPoasesInterface();

  void TriggerPrinting();
  void PreventPrinting();

  const double& SolveTime() const;
  double& SolveTime();
  // Solves the quadratic program
  //
  //   minimize 0.5 * x^T Q x + c^T x
  //   subject to lb <= A x <= ub
  //
  // Returns the optimal solution x.
  virtual void Solve(const Eigen::MatrixXd& Q,
                     const Eigen::VectorXd& f,
                     const Eigen::MatrixXd& A,
                     const Eigen::VectorXd& lb,
                     const Eigen::VectorXd& ub) override;
  virtual void Solve() override;
  
 private:
  // QPoases QP solver structures

  // timer
  double time_;

  // debug
  bool show_flag_;
};

#endif  // QPOASES_INTERFACE_HPP