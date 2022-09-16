#ifndef UNOS_EVALUATOR_EVALUATOR_HH
#define UNOS_EVALUATOR_EVALUATOR_HH

#include "unos/jacobian_writer/jacobian_writer.hh"

namespace unos {

class Evaluator {
 public:
  Evaluator(const Program::Ptr& program_ptr);
  bool evaluate(const double* state, double* residuals, double** jacobians);

 private:
  double**            prepareJacobianSpace();
  void                releaseJacobianSpace(double** jacobian);
  Program::Ptr        program_ptr_;
  JacobianWriter::Ptr jacobian_writer_;
};
}  // namespace unos

#endif // UNOS_EVALUATOR_EVALUATOR_HH
