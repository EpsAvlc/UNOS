#ifndef UNOS_JACOBIAN_WRITER_DENSE_JACOBIAN_WRITER_HH
#define UNOS_JACOBIAN_WRITER_DENSE_JACOBIAN_WRITER_HH

#include "unos/jacobian_writer/jacobian_writer.hh"

namespace unos {
class DenseJacobianWriter : public JacobianWriter {
 public:
  using Ptr = std::shared_ptr<DenseJacobianWriter>;
  DenseJacobianWriter(const Program::Ptr& program) : JacobianWriter(program) {}

  std::unique_ptr<SparseMatrix> createJacobian();
  void write(const ResidualBlock::Ptr& residual_block_ptr, double** jacobians,
             SparseMatrix* jacobian_matrix) override final;
};
}  // namespace unos

#endif // UNOS_JACOBIAN_WRITER_DENSE_JACOBIAN_WRITER_HH
