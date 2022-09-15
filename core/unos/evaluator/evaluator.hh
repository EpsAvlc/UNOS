#ifndef UNOS_EVALUATOR_HH__
#define UNOS_EVALUATOR_HH__

#include "unos/jacobian_writer/jacobian_writer.hh"
#include "unos/problem/program.hh"

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

#endif  // NALIO_WS_EVALUATOR_HH__
