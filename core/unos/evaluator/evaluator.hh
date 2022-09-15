#ifndef UNOS_EVALUATOR_HH__
#define UNOS_EVALUATOR_HH__

#include "unos/problem/problem.hh"

namespace unos {

class Evaluator {
 public:
  Evaluator(const std::shared_ptr<Problem>& problem_ptr);
  bool evaluate(const double* state, double* residuals,
                double** jacobians);

 private:
  std::shared_ptr<Problem> problem_ptr_;
};
}  // namespace unos

#endif  // NALIO_WS_EVALUATOR_HH__
