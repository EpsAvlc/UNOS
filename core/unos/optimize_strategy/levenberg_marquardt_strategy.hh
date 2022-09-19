#include "unos/optimize_strategy/trust_region_strategy.hh"

namespace unos {
/*
Levenberg-Marquardt method. See Chapter 3.2 in "METHODS FOR NON-LINEAR LEAST
SQUARES PROBLEMS".
*/
class LevenbergMarquardtStratege final : public TrustRegionStrategy {
 public:
  LevenbergMarquardtStratege();

  void init(const SparseMatrix::UniquePtr& jacobian) override final;

  TrustRegionStrategy::Summary computeStep(
      const SparseMatrix::UniquePtr& jacobian, const double* residuals,
      double* step) override final;

  bool isStepValid(const Eigen::VectorXd& prev_jaco, const Eigen::VectorXd& curr_jaco, const Eigen::VectorXd& );
 private:
  void   initMu(const SparseMatrix::UniquePtr& jacobian);
  double mu_;
  double tau_;
  int    max_iter_;
};
}  // namespace unos
