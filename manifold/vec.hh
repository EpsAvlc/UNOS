#ifndef UNOS_MANIFOLD_VEC_HH
#define UNOS_MANIFOLD_VEC_HH

#include <cstdarg>
#include "unos/manifold/sub_manifold.hh"

namespace unos {
template <int N>
class Vec : public SubManifold {
 public:
  using Ptr = std::shared_ptr<Vec<N>>;
  enum : int { DIM = N, DOF = N };
  void boxplus(const Eigen::VectorXd& s) override { s_ += s; }
  Eigen::VectorXd boxminus(const SubManifold* rhs) const override {
    const Vec<N>* rhs_derived = dynamic_cast<const Vec<N>*>(rhs);
    Eigen::VectorXd ret = this->s_ - rhs_derived->s_;
    return ret;
  }

  void oplus(const Eigen::VectorXd& s) override {
    if (s.size() != N) {
      throw(std::logic_error("Dismatch paramter in SO3. Input has size: " +
                             std::to_string(s.size())));
    }
    s_ += s;
  }

  std::string type_id() const override { return "Vec" + std::to_string(N); };

  void setZero() override { s_.setZero(); }
  Eigen::VectorXd coeffs() const override { return s_; }

  void copyTo(SubManifold* target) const override {
    Vec<N>* target_derived = dynamic_cast<Vec<N>*>(target);
    target_derived->s_ = s_;
  }

  void set(const std::initializer_list<double>& init) override {
    int ind = 0;
    for (auto iter = init.begin(); iter != init.end(); ++iter) {
      s_(ind) = *iter;
      ++ind;
    }
  }

 private:
  Eigen::Matrix<double, N, 1> s_;
};

using Vec1 = Vec<1>;
using Vec2 = Vec<2>;
using Vec3 = Vec<3>;
REGISTER_UNOS(SubManifold, Vec1, "Vec1");
REGISTER_UNOS(SubManifold, Vec2, "Vec2");
REGISTER_UNOS(SubManifold, Vec3, "Vec3");
}  // namespace unos

#endif // UNOS_MANIFOLD_VEC_HH
