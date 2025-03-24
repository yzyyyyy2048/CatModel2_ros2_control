#include "CatModel2_v2_controller/vmc/qpoases_interface.hpp"
#include <qpOASES.hpp>
#include "CatModel2_v2_controller/vmc/robot_timer.hpp"

QPoasesInterface::QPoasesInterface() : QPSolver() {
  time_ = 0.0;
  show_flag_ = false;
}

QPoasesInterface::~QPoasesInterface() {}

void QPoasesInterface::TriggerPrinting() { show_flag_ = true; }
void QPoasesInterface::PreventPrinting() { show_flag_ = false; }

const double& QPoasesInterface::SolveTime() const {
  return time_;
}

double& QPoasesInterface::SolveTime() {
  return time_;
}

void QPoasesInterface::Solve(const Eigen::MatrixXd& Q,
                                const Eigen::VectorXd& f,
                                const Eigen::MatrixXd& A,
                                const Eigen::VectorXd& lb,
                                const Eigen::VectorXd& ub) {
  QPSolver::Solve(Q, f, A, lb, ub);
}

void QPoasesInterface::Solve() {

  Timer timer;
  timer.start();
  qpOASES::QProblem problem (GetNv(), GetNc());
  qpOASES::Options op;
  op.setToMPC();
  op.printLevel = qpOASES::PL_LOW;
  problem.setOptions(op);

  qpOASES::int_t nWSR = 500;

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> Q_row = GetQ();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> A_row = GetA();

  int rval = problem.init(Q_row.data(), Getf().data(), A_row.data(), NULL, NULL, Getlb().data(), Getub().data(), nWSR);
  int rval2 = problem.getPrimalSolution(SetSolved().data());

  if(rval2 != qpOASES::SUCCESSFUL_RETURN)
    printf("failed to solve!\n");

  SolveTime() = 0.001 * timer.get();
}