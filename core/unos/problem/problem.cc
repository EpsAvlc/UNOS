#include "unos/problem/problem.hh"
#include <glog/logging.h>

unos::Problem::Problem(const Config config) {
  program_ptr_.reset(new Program);
  evaluator_ptr_.reset(new Evaluator(program_ptr_));
  minimizer_ptr_ = Minimizer::create(Minimizer::Type::TrustRegion);
}

void unos::Problem::addParameterBlock(double* parameters, int size,
                                      ManifoldBase* manifold) {
  program_ptr_->addParameterBlock(parameters, size, manifold);
}

void unos::Problem::addResidualBlock(const CostFunction*  cost_function,
                                     const LossFunction*  loss_function,
                                     double* const* const parameters,
                                     int num_parameter_blocks) {
  program_ptr_->addResidualBlock(cost_function, loss_function, parameters,
                                 num_parameter_blocks);
}

void unos::Problem::optimize() {
  preprocess();
  Eigen::VectorXd state_vec(program_ptr_->numParametersDIM());
  program_ptr_->parameterBlocksToStateVector(state_vec.data());

  Minimizer::Options minimizer_options;
  minimizer_options.evaluator_ptr     = evaluator_ptr_;
  minimizer_options.max_iteration_num = 20;
  minimizer_options.program_ptr       = program_ptr_;

  minimizer_ptr_->minimize(minimizer_options, state_vec.data());
}

void unos::Problem::setParameterBlockConstant(double* parameters) {
  program_ptr_->setParameterBlockConstant(parameters);
}

void unos::Problem::setManifold(double* paramters, ManifoldBase* manifold) {
  program_ptr_->setManifold(paramters, manifold);
}

void unos::Problem::preprocess() {
  program_ptr_->mutableJacobian() = evaluator_ptr_->createJacobian();
  // program_ptr_->rearra
}
