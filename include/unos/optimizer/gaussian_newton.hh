#ifndef NALIO_OPTIMIZER_GAUSSIAN_NEWTON_HH__
#define NALIO_OPTIMIZER_GAUSSIAN_NEWTON_HH__

#include <vector>

#include "unos/optimizer/optimizer.hh"

namespace unos {
class GaussianNewton : public Optimizer {
 public:
  void init(const Manifold& init_val,
            const std::vector<const CostFunction::Ptr>& functions) override {
    cost_functions_ = functions;
    val_ = init_val;
  }

  Eigen::VectorXd calculateDx() override {
    bool converged = false;
    uint16_t iter = 0;
    while (!converged && iter < max_iter_) {
      ++iter;
      Eigen::MatrixXd H(val_.dof(), val_.dof());
      H.setZero();
      Eigen::VectorXd g(val_.dof());
      g.setZero();
      for (int ci = 0; ci < cost_functions_.size(); ++ci) {
        Eigen::VectorXd residual;
        Eigen::MatrixXd jacobian;
        cost_functions_[ci]->evaluate(val_, &residual, &jacobian);
        H += jacobian.transpose() * jacobian;
        g += -jacobian * residual;
      }
      Eigen::VectorXd dx = H.inverse() * g;
      val_.oplus(dx);
      if (dx.norm() < 1e-5) {
        converged = true;
      }
    }
    Eigen::VectorXd ret = val_.boxminus(&init_val_);
    return ret;
  }

 private:
  std::vector<const CostFunction::Ptr> cost_functions_;
  Manifold val_;
  Manifold init_val_;
  uint16_t max_iter_ = 10;
};
}  // namespace unos

#endif  // NALIO_OPTIMIZER_GAUSSIAN_NEWTON_HH__
