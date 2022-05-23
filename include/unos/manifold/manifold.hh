#ifndef UNOS_MANIFOLD_MANIFOLD_HH__
#define UNOS_MANIFOLD_MANIFOLD_HH__

#include <vector>

#include "unos/manifold/sub_manifold.hh"

namespace unos {

class Manifold : public SubManifold {
 public:
  template <typename T, typename... ARGS>
  Manifold(const T t, ARGS... arg) : dim_(0), dof_(0) {
    construct(t, arg...);
    setZero();
  }

  void oplus(const Eigen::VectorXd& input) override {
    uint16_t ind = 0;
    for (size_t si = 0; si < sub_manifolds_.size(); ++si) {
      sub_manifolds_[si]->oplus(
          input.block(ind, 0, sub_manifolds_[si]->dof(), 1));
      ind += sub_manifolds_[si]->dof();
    }
  }

  void boxplus(const Eigen::VectorXd& state) override {
    uint16_t ind = 0;
    for (size_t si = 0; si < sub_manifolds_.size(); ++si) {
      sub_manifolds_[si]->oplus(
          state.block(ind, 0, ind + sub_manifolds_[si]->dim(), 1));
      ind += sub_manifolds_[si]->dim();
    }
  }

  uint16_t dim() const override { return dim_; }
  uint16_t dof() const override { return dof_; }

  void setZero() override {
    for (size_t si = 0; si < sub_manifolds_.size(); ++si) {
      sub_manifolds_[si]->setZero();
    }
  }

  Eigen::VectorXd coeffs() const override {
    Eigen::VectorXd ret(dim_);
    uint16_t ind = 0;
    for (size_t si = 0; si < sub_manifolds_.size(); ++si) {
      Eigen::VectorXd sub_coeff = sub_manifolds_[si]->coeffs();
      for (size_t di = 0; di < sub_manifolds_[si]->dim(); ++di) {
        ret(ind + di) = sub_coeff(di);
      }
      ind += sub_manifolds_[si]->dim();
    }
    return ret;
  }

 private:
  template <typename T, typename... ARGS>
  void construct(const T t, ARGS... arg) {
    construct(t);
    construct(arg...);
  }
  template <typename T>
  void construct(const T t) {
    typename T::Ptr sub_manifold(new T);
    sub_manifolds_.push_back(sub_manifold);
    dof_ += sub_manifold->dof();
    dim_ += sub_manifold->dim();
  }

  std::vector<SubManifold::Ptr> sub_manifolds_;
  uint16_t dim_;
  uint16_t dof_;
};

};  // namespace unos

#endif  // UNOS_MANIFOLD_MANIFOLD_HH__
