#ifndef UNOS_MANIFOLD_MANIFOLD_HH__
#define UNOS_MANIFOLD_MANIFOLD_HH__

#include <vector>

#include "unos/manifold/sub_manifold.hh"

namespace unos {

class Manifold : public SubManifold {
 public:
  using Ptr = std::shared_ptr<Manifold>;

  Manifold() {}

  Manifold(const Manifold& rhs) {
    rhs.copyTo(this);
  }

  Manifold(const Manifold&& rhs) {
    rhs.copyTo(this);
  }

  template <typename T, typename... ARGS>
  Manifold(const T t, ARGS... arg) : dim_(0), dof_(0), type_id_("") {
    construct(t, arg...);
  }

  void operator=(const Manifold& rhs) { rhs.copyTo(this); }

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

  Eigen::VectorXd boxminus(const SubManifold* rhs) const override {
    const Manifold* rhs_derived = dynamic_cast<const Manifold*>(rhs);
    if (rhs_derived->type_id_ != this->type_id_) {
      throw(std::logic_error("Manifold must have the same type_id."));
    }

    Eigen::VectorXd ret(dof());
    uint16_t ind = 0;
    for (size_t si = 0; si < this->sub_manifolds_.size(); ++si) {
      ret.block(ind, 0, sub_manifolds_[si]->dof(), 1) =
          sub_manifolds_[si]->boxminus(rhs_derived->sub_manifolds_[si].get());
      ind += sub_manifolds_[si]->dof();
    }
    return ret;
  }

  uint16_t dim() const override { return dim_; }
  uint16_t dof() const override { return dof_; }
  std::string type_id() const override { return type_id_; };

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

  void copyTo(SubManifold* target) const override {
    Manifold* target_derived = dynamic_cast<Manifold*>(target);
    target_derived->dim_ = dim_;
    target_derived->dof_ = dof_;
    target_derived->type_id_ = type_id_;
    target_derived->sub_manifolds_.clear();
    target_derived->sub_manifolds_.resize(sub_manifolds_.size());
    for (size_t si = 0; si < sub_manifolds_.size(); ++si) {
      target_derived->sub_manifolds_[si] =
          createSubManifold(sub_manifolds_[si]->type_id());
      sub_manifolds_[si]->copyTo(target_derived->sub_manifolds_[si].get());
    }
  }

 private:
  template <typename T, typename... ARGS>
  void construct(const T& t, ARGS... arg) {
    construct(t);
    construct(arg...);
  }

  template <typename T>
  void construct(const T& t) {
    typename T::Ptr sub_manifold(new T(t));
    sub_manifolds_.push_back(sub_manifold);
    dof_ += sub_manifold->dof();
    dim_ += sub_manifold->dim();
    type_id_ += sub_manifold->type_id();
  }


  std::vector<SubManifold::Ptr> sub_manifolds_;
  uint16_t dim_;
  uint16_t dof_;
  std::string type_id_;
};

template <>
void Manifold::construct(const Manifold& m) {
  m.copyTo(this);
}

};  // namespace unos

#endif  // UNOS_MANIFOLD_MANIFOLD_HH__
