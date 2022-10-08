#ifndef UNOS_EVALUATOR_EVALUATOR_HH
#define UNOS_EVALUATOR_EVALUATOR_HH

#include "unos/jacobian_writer/jacobian_writer.hh"

namespace unos {

class Evaluator {
 public:
  using Ptr = std::shared_ptr<Evaluator>;

  Evaluator(const Program::Ptr& program_ptr);

  SparseMatrix::UniquePtr createJacobian();

  bool evaluate(const double* state, double* residuals,
                SparseMatrix* jacobians);

  void plus(double* x, double* delta, double* x_plus_delta);

 private:
  double**            prepareJacobianSpace();
  void                releaseJacobianSpace(double** jacobian);
  Program::Ptr        program_ptr_;
  JacobianWriter::Ptr jacobian_writer_;
};
}  // namespace unos

#endif  // UNOS_EVALUATOR_EVALUATOR_HH
