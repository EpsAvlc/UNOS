#include "unos/jacobian_writer/dense_jacobian_writer.hh"

namespace unos {
JacobianWriter::Ptr unos::JacobianWriter::create(
    const Program::Ptr& program_ptr, const Type& type) {
  switch (type) {
    case DENSE: {
      DenseJacobianWriter::Ptr ret(new DenseJacobianWriter(program_ptr));
      return std::static_pointer_cast<JacobianWriter>(ret);
    }
    default: {
      JacobianWriter::Ptr ret;
      return ret;
    }
  }
}
}  // namespace unos
