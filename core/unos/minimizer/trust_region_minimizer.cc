#include "trust_region_minimizer.hh"
#include <glog/logging.h>
#include <iostream>

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
  Eigen::VectorXd    delta_x(program_ptr_->numParameters());
  while (!isTerminated(&terminate_condition)) {
    ++iter_num_;
    Eigen::VectorXd residuals(program_ptr_->numResiduals());
    evalutor_ptr_->evaluate(x_.data(), residuals.data(),
                            program_ptr_->mutableJacobian().get());

    strategy_ptr_->computeStep(program_ptr_->mutableJacobian(),
                               residuals.data(), delta_x.data());
    // LOG(INFO) << "delta x: " << delta_x.transpose() << std::endl;
    if (delta_x.norm() <= sigma_2_ * (x_.norm() + sigma_2_)) {
      // TODO(caoming): Modify termiante condition
      break;
    } else {
      if (isStepValid(delta_x)) {
        x_ = x_ + delta_x;
        strategy_ptr_->acceptStep(rho_);
      } else {
        strategy_ptr_->refuseStep(rho_);
      }
      // Eigen::VectorXd x_tmp = x_ + h_lm;
    }
  }
  return;
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

bool TrustRegionMinimizer::isStepValid(const Eigen::VectorXd& delta_x) {
  Eigen::VectorXd f_x(program_ptr_->numResiduals());
  evalutor_ptr_->evaluate(x_.data(), f_x.data(),
                          program_ptr_->mutableJacobian().get());

  Eigen::MatrixXd J           = program_ptr_->Jacobian().toDenseMatrix();
  Eigen::VectorXd candidate_x = x_ + delta_x;
  Eigen::VectorXd f_x_new(program_ptr_->numResiduals());
  evalutor_ptr_->evaluate(candidate_x.data(), f_x_new.data(), nullptr);

  // L(0) - L(h_lm) = -h_lm^T J^T f - 0.5 * h_lm^T J ^T J h_lm
  Eigen::VectorXd denominator =
      -2 * delta_x.transpose() * J.transpose() * f_x -
      delta_x.transpose() * J.transpose() * J * delta_x;

  rho_ = (f_x.squaredNorm() - f_x_new.squaredNorm()) / denominator.norm();

  return rho_ > 1e-3;
}
}  // namespace unos
