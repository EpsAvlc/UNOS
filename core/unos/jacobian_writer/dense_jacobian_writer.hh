#ifndef UNOS_DENSE_JACOBIAN_WRITER_HH
#define UNOS_DENSE_JACOBIAN_WRITER_HH

#include "unos/jacobian_writer/jacobian_writer.hh"

namespace unos {
class DenseJacobianWriter : public JacobianWriter {
 public:
  using Ptr = std::shared_ptr<DenseJacobianWriter>;
  DenseJacobianWriter(const Program::Ptr& program) : JacobianWriter(program) {}
  void write(const ResidualBlock::Ptr& residual_block_ptr,
             const int residual_offset, double** jacobians) override {
  
  }
};
}  // namespace unos

#endif  // UNOS_DENSE_JACOBIAN_WRITER_HH
