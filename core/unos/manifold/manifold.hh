#ifndef UNOS_MANIFOLD_MANIFOLD_HH
#define UNOS_MANIFOLD_MANIFOLD_HH

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
    DIM             = ManifoldHelper<SMs...>::DIM,
    DOF             = ManifoldHelper<SMs...>::DOF,
    SubManifoldSize = sizeof...(SMs)
  };

  Manifold() { (construct<SMs>(), ...); }

  void boxplus(double const* const x, double const* const y,
               double* x_plus_y) const override {
    boxplus<0, 0, SMs...>(x, y, x_plus_y);
  }

  void oplus(double const* const x, double const* const delta_x,
             double* x_plus_delta_x) const override {
    oplus<0, 0, 0, SMs...>(x, delta_x, x_plus_delta_x);
  }

  std::string typeID() const override { return type_id_; };

 private:
  template <typename T>
  void construct() {
    typename T::Ptr sub_manifold(new T());
    sub_manifolds_.push_back(sub_manifold);
    type_id_ = type_id_ + "/" + sub_manifold->typeID();
  };

  // terminal condition oplus
  template <size_t I, size_t DeltaI, size_t ResI>
  void oplus(double const* const x, double const* const delta,
             double* x_plus_delta) const {}

  template <size_t I, size_t DeltaI, size_t ResI, typename T, typename... Ts>
  void oplus(double const* const x, double const* const delta,
             double* x_plus_delta) const {
    sub_manifolds_[I]->oplus(&x[ResI], &delta[DeltaI], &x_plus_delta[ResI]);
    oplus<I + 1, DeltaI + T::DOF, ResI + T::DIM, Ts...>(x, delta,
                                                          x_plus_delta);
  }

  template <size_t I, size_t BlockI, typename T, typename... Ts>
  void boxplus(double const* const x, double const* const y,
               double* x_plus_y) const {
    sub_manifolds_[I]->boxplus(&x[BlockI], &y[BlockI], &x_plus_y[BlockI]);
    boxplus<I + 1, BlockI + T::DIM, Ts...>(x, y, x_plus_y);
  }

  // terminal condition boxplus
  template <size_t I, size_t BlockI>
  void boxplus(double const* const x, double const* const y,
               double* x_plus_y) const {}

  std::vector<SubManifold::Ptr> sub_manifolds_;
  std::string                   type_id_;
};

};  // namespace unos

#endif  // UNOS_MANIFOLD_MANIFOLD_HH
