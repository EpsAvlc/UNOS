#ifndef UNOS_SPARSE_MATRIX_SPARSE_MATRIX_HH
#define UNOS_SPARSE_MATRIX_SPARSE_MATRIX_HH

#include <Eigen/Core>
#include <memory>

namespace unos {
class SparseMatrix {
 public:
  using UniquePtr                         = std::unique_ptr<SparseMatrix>;
  virtual Eigen::MatrixXd toDenseMatrix() = 0;

 private:
};
}  // namespace unos

#endif  // UNOS_SPARSE_MATRIX_SPARSE_MATRIX_HH
