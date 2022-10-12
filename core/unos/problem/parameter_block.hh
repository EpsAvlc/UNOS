#ifndef UNOS_PROBLEM_PARAMETER_BLOCK_HH
#define UNOS_PROBLEM_PARAMETER_BLOCK_HH

#include <memory>
#include <unos/manifold/manifold_base.hh>

namespace unos {
class ParameterBlock {
 public:
  using Ptr = std::shared_ptr<ParameterBlock>;

  ParameterBlock(double* data, int size, ManifoldBase* manifold = nullptr);

  static Ptr create(double* data, int size, ManifoldBase* manifold = nullptr);

  int dof() const {
    if (manifold_) {
      return manifold_->dof();
    } else {
      return size_;
    }
  }

  int dim() const { return size_; }

  double* mutableData();

  void getState(double* state);

  void setState(double const* state);

  double const* state() const;

  int jacobianOffset() const;

  void setJacobianOffset(const int jacobian_offset);

  void setConst(const bool is_const);

  ManifoldBase* getManifold() { return manifold_; }

  void setManifold(ManifoldBase* manifold);

  bool isConst() const { return is_const_; }

 private:
  double*       data_;
  ManifoldBase* manifold_;
  int           size_;
  int           jacobian_offset_;
  bool          is_const_;
};

};  // namespace unos

#endif  // UNOS_PROBLEM_PARAMETER_BLOCK_HH
