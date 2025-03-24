#ifndef QP_SOLVER_HPP
#define QP_SOLVER_HPP

#include <Eigen/Core>
#include <Eigen/Dense>

class QPSolver {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Constructor.
  QPSolver();

  // Destructor.
  ~QPSolver();

  // Solves the quadratic program
  //
  //   minimize 0.5 * x^T Q x + f^T x
  //   subject to lb <= A x <= ub
  //
  // Returns the optimal solution x.
  virtual void Solve(const Eigen::MatrixXd& Q,
                     const Eigen::VectorXd& f,
                     const Eigen::MatrixXd& A,
                     const Eigen::VectorXd& lb,
                     const Eigen::VectorXd& ub);

  virtual void Solve() = 0;

  void SetProblem();
  void SetProblem(int nv, int nc);
  void SetProblem(const Eigen::MatrixXd& Q,
                  const Eigen::VectorXd& f,
                  const Eigen::MatrixXd& A,
                  const Eigen::VectorXd& lb,
                  const Eigen::VectorXd& ub);

  void SetQ(const Eigen::MatrixXd& Q);
  void Setf(const Eigen::VectorXd& f);
  void SetA(const Eigen::MatrixXd& A);
  void Setlb(const Eigen::VectorXd& lb);
  void Setub(const Eigen::VectorXd& ub);
  Eigen::VectorXd& SetSolved();

  const int& GetNv() const;
  const int& GetNc() const;
  Eigen::MatrixXd& GetQ();
  Eigen::VectorXd& Getf();
  Eigen::MatrixXd& GetA();
  Eigen::VectorXd& Getlb();
  Eigen::VectorXd& Getub();
  const Eigen::VectorXd& GetSolved() const;
 private:
  int nv_;
  int nc_;
  Eigen::MatrixXd Q_;
  Eigen::VectorXd f_;
  Eigen::MatrixXd A_;
  Eigen::VectorXd lb_;
  Eigen::VectorXd ub_;
  Eigen::VectorXd solved_;

};

#endif  // QP_SOLVER_HPP