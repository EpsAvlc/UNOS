#ifndef UNOS_OPTIMIZER_PROBLEM_HH__
#define UNOS_OPTIMIZER_PROBLEM_HH__

#include "unos/cost_function/cost_function.hh"
#include "unos/manifold/manifold.hh"

namespace unos {

class Problem {
 public:
  void setManifold(Manifold::Ptr& manifold) { manifold_ = manifold; }

  void addCostFunctions(const CostFunction::Ptr& cost_function) {
    cost_functions_.push_back(cost_function);
  }

  void optimize() {
    for (size_t ci = 0; ci < cost_functions_.size(); ++ci) {
      Eigen::VectorXd residual;
      Eigen::MatrixXd jacobian;
      cost_functions_[ci]->evaluate(manifold_, &residual, &jacobian);
    }
  }

 private:
  Eigen::MatrixXd makeHessianAndJacobians(Eigen::MatrixXd* H, Eigen::MatrixXd* J) {}

  Eigen::VectorXd makeJacobians() {}

  Manifold::Ptr manifold_;
  std::vector<const CostFunction::Ptr> cost_functions_;
};
};  // namespace unos

#endif  // UNOS_OPTIMIZER_OPTIMIZER_HH__
