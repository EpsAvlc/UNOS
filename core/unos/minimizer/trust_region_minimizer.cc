#include "trust_region_minimizer.hh"

namespace unos {

void TrustRegionMinimizer::init(const Options& options, double* parameters) {
  program_ptr_  = options.program_ptr;
  evalutor_ptr_ = options.evaluator_ptr;
  x_ = Eigen::Map<Eigen::VectorXd>(parameters, program_ptr_->numParameters());
}

void TrustRegionMinimizer::minimize(const Options& options,
                                    double*        parameters) {
  init(options, parameters);

  TerminateCondition terminate_condition;
  while (!isTerminated(&terminate_condition)) {

  }
}

void TrustRegionMinimizer::iterationZero() {
  Eigen::VectorXd residuals(program_ptr_->numResiduals());
  evalutor_ptr_->evaluate(x_.data(), residuals.data(),
                          program_ptr_->mutableJacobian().get());
  strategy_ptr_->init(program_ptr_->mutableJacobian());
}
}  // namespace unos
