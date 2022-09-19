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
  max_iter_num_ = options.max_iteration_num;
}

void TrustRegionMinimizer::minimize(const Options& options,
                                    double*        parameters) {
  init(options, parameters);

  TerminateCondition terminate_condition;
  Eigen::VectorXd    h_lm(program_ptr_->numParameters());
  while (!isTerminated(&terminate_condition)) {
    ++iter_num_;
    Eigen::VectorXd residuals(program_ptr_->numResiduals());
    evalutor_ptr_->evaluate(x_.data(), residuals.data(),
                            program_ptr_->mutableJacobian().get());

    strategy_ptr_->computeStep(program_ptr_->mutableJacobian(),
                               residuals.data(), h_lm.data());
    // LOG(INFO) << "delta x: " << delta_x.transpose() << std::endl;
    if (h_lm.norm() <= sigma_2_ * (x_.norm() + sigma_2_)) {
      // TODO: Modify termiante condition
      break;
    } else {
      Eigen::VectorXd x_tmp = x_ + h_lm;
    }

  }
}

bool TrustRegionMinimizer::isTerminated(
    TerminateCondition* terminate_condition) {
  if (iter_num_ >= max_iter_num_) {
    return true;
    *terminate_condition = TerminateCondition::ReachMaxIteration;
  }

  return false;
}
void TrustRegionMinimizer::iterationZero() {
  Eigen::VectorXd residuals(program_ptr_->numResiduals());
  evalutor_ptr_->evaluate(x_.data(), residuals.data(),
                          program_ptr_->mutableJacobian().get());
  strategy_ptr_->init(program_ptr_->mutableJacobian());
}

bool TrustRegionMinimizer::isStepValid() {
  
}
}  // namespace unos
