#include "unos/optimize_strategy/levenberg_marquardt_strategy.hh"

#include <Eigen/Dense>

namespace unos {

LevenbergMarquardtStratege::LevenbergMarquardtStratege()
    : mu_(0),
      tau_(0.001), /* The algorithm is not very sensitive to the choice of tau,
  but as a rule of thumb, one should use a small value, eg tau =1e-6 if x0 is
  believed to be a good approximation to x¤. Otherwise, use tau =1e-3 or even
  tau =1.*/
      max_iter_(50) {}

void LevenbergMarquardtStratege::init(const SparseMatrix::UniquePtr& jacobian) {
  initMu(jacobian);
}

TrustRegionStrategy::Summary LevenbergMarquardtStratege::computeStep(
    const SparseMatrix::UniquePtr& jacobian, const double* residuals,
    double* step) {
  TrustRegionStrategy::Summary summary;

  Eigen::MatrixXd dense_jaco_matrix = jacobian->toDenseMatrix();
  int             parameter_size    = dense_jaco_matrix.cols();
  int             jacobian_size     = dense_jaco_matrix.rows();
  Eigen::Map<const Eigen::VectorXd> residuals_vec(residuals, jacobian_size);

  Eigen::Map<Eigen::VectorXd> h_lm(step, parameter_size);
  h_lm = (dense_jaco_matrix.transpose() * dense_jaco_matrix)
             .householderQr()
             .solve(-dense_jaco_matrix.transpose() * residuals_vec);
  summary.success = true;
  return summary;
}

void LevenbergMarquardtStratege::initMu(
    const SparseMatrix::UniquePtr& jacobian) {
  Eigen::MatrixXd dense_jaco_matrix = jacobian->toDenseMatrix();
  Eigen::MatrixXd A = dense_jaco_matrix.transpose() * dense_jaco_matrix;
  double          max_diagnoal_val = 0;
  for (size_t si = 0; si < dense_jaco_matrix.size(); ++si) {
    max_diagnoal_val = std::max(A(si, si), max_diagnoal_val);
  }
  mu_ = tau_ * max_diagnoal_val;
}

}  // namespace unos
