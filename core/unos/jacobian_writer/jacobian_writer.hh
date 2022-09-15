#ifndef UNOS_JACOBIAN_WRITER_HH
#define UNOS_JACOBIAN_WRITER_HH

#include <memory.h>
#include "unos/problem/program.hh"

namespace unos {
class JacobianWriter {
 public:
  using Ptr = std::shared_ptr<JacobianWriter>;
  enum Type { DENSE };
  JacobianWriter(const Program::Ptr& program_ptr) : program_ptr_(program_ptr) {}
  virtual void write(const ResidualBlock::Ptr& ptr, const int residual_offset,
                     double** jacobians) {}

  static JacobianWriter::Ptr create(const Program::Ptr& program_ptr,
                                    const Type&         type);

 protected:
  Program::Ptr program_ptr_;
};
};  // namespace unos

#endif  // JACOBIAN_WRITER_HH
