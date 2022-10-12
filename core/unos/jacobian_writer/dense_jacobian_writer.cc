#include "unos/jacobian_writer/dense_jacobian_writer.hh"
#include "unos/sparse_matrix/dense_sparse_matrix.hh"
#include "unos/utils/cast_utils.hh"

namespace unos {

std::unique_ptr<SparseMatrix> DenseJacobianWriter::createJacobian() {
  return std::make_unique<DenseSparseMatrix>(Eigen::MatrixXd(
      program_ptr_->numResiduals(), program_ptr_->numParametersDOF()));
}

void DenseJacobianWriter::write(const ResidualBlock::Ptr& residual_block_ptr,
                                double**                  jacobians,
                                SparseMatrix*             jacobian_matrix) {
  Eigen::MatrixXd* jaco_matrix_eigen =
      down_cast<DenseSparseMatrix*>(jacobian_matrix)->mutable_matrix();

  int residual_size = residual_block_ptr->size();

  for (size_t pi = 0; pi < residual_block_ptr->parameterBlocks().size(); ++pi) {
    const ParameterBlock::Ptr& parameter_block_ptr =
        residual_block_ptr->parameterBlocks()[pi];
    if (parameter_block_ptr->isConst()) {
      continue;
    }
    int                         parameter_size = parameter_block_ptr->dof();
    Eigen::Map<Eigen::MatrixXd> jaco_ref(jacobians[pi], residual_size,
                                         parameter_size);

    jaco_matrix_eigen->block(residual_block_ptr->jacobianOffset(),
                             parameter_block_ptr->jacobianOffset(),
                             residual_block_ptr->size(),
                             parameter_block_ptr->dof()) = jaco_ref;
  }
}

}  // namespace unos
