#include "CatModel2_v2_controller/vmc/qp_solver.hpp"
#include <iostream>

QPSolver::QPSolver() : nv_(0), nc_(0) {
  SetProblem();
}

QPSolver::~QPSolver() {}

void QPSolver::Solve(const Eigen::MatrixXd& Q,
                     const Eigen::VectorXd& f,
                     const Eigen::MatrixXd& A,
                     const Eigen::VectorXd& lb,
                     const Eigen::VectorXd& ub) {
  SetProblem(Q, f, A, lb, ub);
  Solve();
}

void QPSolver::SetProblem(int nv, int nc) {
  nv_ = nv;
  nc_ = nc;
  SetProblem();
}

void QPSolver::SetProblem() {
  int nv = nv_;
  int nc = nc_;

  Q_ = Eigen::MatrixXd::Zero(nv, nv);
  f_ = Eigen::VectorXd::Zero(nv);
  A_ = Eigen::MatrixXd::Zero(nc, nv);
  lb_ = Eigen::VectorXd::Zero(nc);
  ub_ = Eigen::VectorXd::Zero(nc);

  solved_ = Eigen::VectorXd::Zero(nv);
}

void QPSolver::SetProblem(const Eigen::MatrixXd& Q,
                          const Eigen::VectorXd& f,
                          const Eigen::MatrixXd& A,
                          const Eigen::VectorXd& lb,
                          const Eigen::VectorXd& ub) {
  nv_ = Q.rows();
  nc_ = A.rows();
  SetQ(Q);
  Setf(f);
  SetA(A);
  Setlb(lb);
  Setub(ub);
  SetSolved().setZero(GetNv());
}


void QPSolver::SetQ(const Eigen::MatrixXd& Q) { 
  if (Q.rows() == nv_ && Q.cols() == nv_) {
    Q_ = Q; 
  }
  else {
    std::cout << "QPSolver set Q fail" << std::endl;
  }
}

void QPSolver::Setf(const Eigen::VectorXd& f) { 
  if (f.rows() == nv_) {
    f_ = f; 
  }
  else {
    std::cout << "QPSolver set f fail" << std::endl;
  }
}

void QPSolver::SetA(const Eigen::MatrixXd& A) { 
  if (A.rows() == nc_ && A.cols() == nv_) {
    A_ = A;
  }
  else {
    std::cout << "QPSolver set A fail" << std::endl;
  }
}

void QPSolver::Setlb(const Eigen::VectorXd& lb) { 
  if (lb.rows() == nc_) {
    lb_ = lb;
  }
  else {
    std::cout << "QPSolver set lb fail" << std::endl;
  }
}

void QPSolver::Setub(const Eigen::VectorXd& ub) { 
  if (ub.rows() == nc_) {
    ub_ = ub;
  }
  else {
    std::cout << "QPSolver set ub fail" << std::endl;
  }
}
Eigen::VectorXd& QPSolver::SetSolved() { return solved_; }

const int& QPSolver::GetNv() const { return nv_; }
const int& QPSolver::GetNc() const { return nc_; }
Eigen::MatrixXd& QPSolver::GetQ() { return Q_; }
Eigen::VectorXd& QPSolver::Getf() { return f_; }
Eigen::MatrixXd& QPSolver::GetA() { return A_; }
Eigen::VectorXd& QPSolver::Getlb() { return lb_; }
Eigen::VectorXd& QPSolver::Getub() { return ub_; }
const Eigen::VectorXd& QPSolver::GetSolved() const { return solved_; }