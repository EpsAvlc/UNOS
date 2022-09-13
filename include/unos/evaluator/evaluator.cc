#include "unos/evaluator/evaluator.hh"

namespace unos {
Evaluator::Evaluator(const std::shared_ptr<Problem>& problem_ptr) {
  problem_ptr_ = problem_ptr;
}

bool Evaluator::evaluate(const double* initial_val, double* residuals,
                         double** jacobians) {
  
}
}  // namespace unos
