#include "trust_region_minimizer.hh"
#include <glog/logging.h>

namespace unos {

void TrustRegionMinimizer::init(const Options& options, double* parameters) {
  program_ptr_  = options.program_ptr;
  evalutor_ptr_ = options.evaluator_ptr;
  x_ = Eigen::Map<Eigen::VectorXd>(parameters, program_ptr_->numParameters());
  // TODO(caoming): create trajectory according to trust region type.
  strategy_ptr_ =
      TrustRegionStrategy::create(TrustRegionStrategy::Type::LevenbergMarquart);
}

void TrustRegionMinimizer::minimize(const Options& options,
                                    double*        parameters) {
  init(options, parameters);

  TerminateCondition terminate_condition;
  Eigen::VectorXd    delta_x(program_ptr_->numParameters());
  //   while (!isTerminated(&terminate_condition)) {
  Eigen::VectorXd residuals(program_ptr_->numResiduals());
  evalutor_ptr_->evaluate(x_.data(), residuals.data(),
                          program_ptr_->mutableJacobian().get());

  strategy_ptr_->computeStep(program_ptr_->mutableJacobian(), residuals.data(),
                             delta_x.data());
  LOG(INFO) << "delta x: " << delta_x.transpose() << std::endl;
  //   }
}

void TrustRegionMinimizer::iterationZero() {
  Eigen::VectorXd residuals(program_ptr_->numResiduals());
  evalutor_ptr_->evaluate(x_.data(), residuals.data(),
                          program_ptr_->mutableJacobian().get());
  strategy_ptr_->init(program_ptr_->mutableJacobian());
}
}  // namespace unos
