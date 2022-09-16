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
  Eigen::MatrixXd jacobian;
  Eigen::VectorXd residual;
  bool            coveraged = false;
  while (!coveraged) {
    evaluator_ptr_->evaluate()
    LOG(INFO) << "-----------Jacobian:----------";
    LOG(INFO) << jacobian;
    LOG(INFO) << "-----------Residual:----------";
    LOG(INFO) << residual;
  }
}
