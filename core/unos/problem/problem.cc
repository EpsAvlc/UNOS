#include "unos/problem/problem.hh"
#include <glog/logging.h>

unos::Problem::Problem(const Config config) {
  program_ptr_.reset(new Program);
  evaluator_ptr_.reset(new Evaluator(program_ptr_));
}

void unos::Problem::addParameterBlock(double* parameters, int size
                                      /*,  manifold*/) {
  program_ptr_->addParameterBlock(parameters, size /*, manifold*/);
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
  bool coveraged = false;
  // while (!coveraged) {
  double* state_data = new double[program_ptr_->numParameters()];
  program_ptr_->parameterBlocksToStateVector(state_data);
  double* residuals = new double[program_ptr_->numResiduals()];
  evaluator_ptr_->evaluate(state_data, residuals,
                           program_ptr_->mutableJacobian().get());
  LOG(INFO) << "-----------Jacobian:----------";
  LOG(INFO) << std::endl << program_ptr_->Jacobian().toDenseMatrix();
  LOG(INFO) << "-----------Residual:----------";
  // LOG(INFO) << residual;
  // }
}

void unos::Problem::preprocess() {
  program_ptr_->mutableJacobian() = evaluator_ptr_->createJacobian();
}
