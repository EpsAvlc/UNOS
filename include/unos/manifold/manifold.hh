#ifndef UNOS_MANIFOLD_MANIFOLD_HH__
#define UNOS_MANIFOLD_MANIFOLD_HH__

#include <vector>

#include "unos/manifold/sub_manifold.hh"

namespace unos {

template <typename SM, typename... SMs>
struct ManifoldHelper {
  enum : int {
    DIM = SM::DIM + ManifoldHelper<SMs...>::DIM,
    DOF = SM::DOF + ManifoldHelper<SMs...>::DOF
  };
};

template <typename SM>
struct ManifoldHelper<SM> {
  enum { DIM = SM::DIM, DOF = SM::DOF };
};

template <typename... SMs>
class Manifold : public SubManifold {
 public:
  using Ptr = std::shared_ptr<Manifold>;

  enum {
    DIM = ManifoldHelper<SMs...>::DIM,
    DOF = ManifoldHelper<SMs...>::DOF,
    SubManifoldSize = sizeof...(SMs)
  };

  Manifold(const Manifold<SMs...>& rhs);

  Manifold(const Manifold<SMs...>&& rhs);

  Manifold();

  Manifold(const std::initializer_list<double>& init_list) {
    set(init_list);
  }

  void operator=(const Manifold& rhs);

  void oplus(const Eigen::VectorXd& input) override;

  void boxplus(const Eigen::VectorXd& state) override;

  template <size_t N>
  void set(const std::initializer_list<double>& init) {
    static_assert(N < SubManifoldSize);
    sub_manifolds_[N]->set(init);
  }

  void set(const std::initializer_list<double>&) override {
    throw("not implement yet.");
  }

  Eigen::VectorXd boxminus(const SubManifold* rhs) const override;

  std::string type_id() const override;

  void setZero() override;

  Eigen::VectorXd coeffs() const override;

  void copyTo(SubManifold* target) const override;

 private:
  template <typename T>
  void construct() {
    typename T::Ptr sub_manifold(new T());
    sub_manifolds_.push_back(sub_manifold);
    type_id_ += sub_manifold->type_id();
  };

  std::vector<SubManifold::Ptr> sub_manifolds_;
  std::string type_id_;

  template <size_t I, size_t BlockI, typename T, typename... Ts>
  void oplus(const Eigen::VectorXd& input);

  template <size_t I, size_t BlockI>
  void oplus(const Eigen::VectorXd& input) {}

  template <size_t I, size_t BlockI, typename T, typename... Ts>
  void boxplus(const Eigen::VectorXd& input);

  template <size_t I, size_t BlockI>
  void boxplus(const Eigen::VectorXd& input) {}

  template <size_t I, size_t BlockI, typename T, typename... Ts>
  void boxminus(const Manifold* rhs, Eigen::VectorXd* ret) const {
    ret->block(BlockI, 0, T::DOF, 1) =
        sub_manifolds_[I]->boxminus(rhs->sub_manifolds_[I].get());
    boxminus<I + 1, BlockI + T::DOF, Ts...>(rhs, ret);
  }

  template <size_t I, size_t BlockI>
  void boxminus(const Manifold* rhs, Eigen::VectorXd* ret) const {}

  template <size_t I, size_t BlockI, typename T, typename... Ts>
  void coeffs(Eigen::VectorXd* ret) const {
    Eigen::VectorXd sub_coeff = sub_manifolds_[I]->coeffs();
    for (size_t di = 0; T::DIM; ++di) {
      (*ret)(I + di) = sub_coeff(di);
    }
  }

  template <size_t I, size_t BlockI>
  void coeffs(Eigen::VectorXd* ret) const {}
};

template <typename... SMs>
inline Manifold<SMs...>::Manifold(const Manifold<SMs...>& rhs) {
  rhs.copyTo(this);
}

template <typename... SMs>
inline Manifold<SMs...>::Manifold() : type_id_("") {
  (construct<SMs>(), ...);
}

template <typename... SMs>
inline void Manifold<SMs...>::operator=(const Manifold& rhs) {
  rhs.copyTo(this);
}

template <typename... SMs>
inline void Manifold<SMs...>::oplus(const Eigen::VectorXd& input) {
  oplus<0, 0, SMs...>(input);
}

template <typename... SMs>
template <size_t I, size_t BlockI, typename T, typename... Ts>
void Manifold<SMs...>::oplus(const Eigen::VectorXd& input) {
  sub_manifolds_[I]->oplus(input.block(BlockI, 0, T::DOF, 1));
  oplus<I + 1, BlockI + T::DOF, Ts...>(input);
};

template <typename... SMs>
inline void Manifold<SMs...>::boxplus(const Eigen::VectorXd& state) {
  boxplus<0, 0, SMs...>(state);
}

template <typename... SMs>
template <size_t I, size_t BlockI, typename T, typename... Ts>
inline void Manifold<SMs...>::boxplus(const Eigen::VectorXd& input) {
  sub_manifolds_[I]->boxplus(input.block(BlockI, 0, T::DIM, 1));
  boxplus<I + 1, BlockI + T::DIM, Ts...>(input);
}

template <typename... SMs>
inline Eigen::VectorXd Manifold<SMs...>::boxminus(
    const SubManifold* rhs) const {
  const Manifold* rhs_derived = dynamic_cast<const Manifold*>(rhs);
  if (rhs_derived->type_id_ != this->type_id_) {
    throw(std::logic_error("Manifold must have the same type_id."));
  }

  Eigen::VectorXd ret(static_cast<int>(DOF));
  boxminus<0, 0, SMs...>(rhs_derived, &ret);
  return ret;
}

template <typename... SMs>
inline Manifold<SMs...>::Manifold(const Manifold&& rhs) {
  rhs.copyTo(this);
}

template <typename... SMs>
inline std::string Manifold<SMs...>::type_id() const {
  return type_id_;
};

template <typename... SMs>
inline void Manifold<SMs...>::setZero() {
  for (size_t si = 0; si < sub_manifolds_.size(); ++si) {
    sub_manifolds_[si]->setZero();
  }
}

template <typename... SMs>
inline Eigen::VectorXd Manifold<SMs...>::coeffs() const {
  Eigen::VectorXd ret(static_cast<int>(DIM));
  ret.setZero();
  coeffs<0, 0, SMs...>(&ret);
  return ret;
}

template <typename... SMs>
inline void Manifold<SMs...>::copyTo(SubManifold* target) const {
  Manifold* target_derived = dynamic_cast<Manifold*>(target);
  target_derived->type_id_ = type_id_;
  target_derived->sub_manifolds_.clear();
  target_derived->sub_manifolds_.resize(sub_manifolds_.size());
  for (size_t si = 0; si < sub_manifolds_.size(); ++si) {
    target_derived->sub_manifolds_[si] =
        createSubManifold(sub_manifolds_[si]->type_id());
    sub_manifolds_[si]->copyTo(target_derived->sub_manifolds_[si].get());
  }
}

};  // namespace unos

#endif  // UNOS_MANIFOLD_MANIFOLD_HH__
