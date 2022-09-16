#ifndef UNOS_SPARSE_MATRIX_DENSE_SPARSE_MATRIX_HH
#define UNOS_SPARSE_MATRIX_DENSE_SPARSE_MATRIX_HH

#include <Eigen/Core>
#include "unos/sparse_matrix/sparse_matrix.hh"

namespace unos {
class DenseSparseMatrix : public SparseMatrix {
 public:
  DenseSparseMatrix(Eigen::MatrixXd m) : m_(std::move(m)) {}
  Eigen::MatrixXd* mutable_matrix() { return &m_; }

 private:
  Eigen::MatrixXd m_;
};
}  // namespace unos

#endif  // UNOS_SPARSE_MATRIX_DENSE_SPARSE_MATRIX_HH
