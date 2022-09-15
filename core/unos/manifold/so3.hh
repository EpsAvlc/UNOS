#ifndef SO3_HH
#define SO3_HH

#include <cstdarg>
#include "unos/manifold/sub_manifold.hh"

namespace unos {
class SO3 : public SubManifold {
 public:
  enum : int { DIM = 4, DOF = 3 };
  using Ptr = std::shared_ptr<SO3>;

  void boxplus(const Eigen::VectorXd& s) override {
    if (s.size() != 4) {
      throw(std::logic_error("Dismatch paramter in SO3. Input has size: " +
                             std::to_string(s.size())));
    }
    Eigen::Vector4d coeff = s;
    q_ = q_ * Eigen::Quaterniond(coeff);
  }

  Eigen::VectorXd boxminus(const SubManifold* rhs) const override {
    const SO3* rhs_derived = dynamic_cast<const SO3*>(rhs);
    return log(rhs_derived->q_.conjugate() * this->q_);
  }

  void oplus(const Eigen::VectorXd& s) override {
    if (s.size() != 3) {
      throw(std::logic_error("Dismatch paramter in SO3. Input has size: " +
                             std::to_string(s.size())));
    }
    q_ = q_ * exp(s);
  }
  uint16_t dim() const { return 4; };
  uint16_t dof() const { return 3; };
  std::string type_id() const override { return "SO3"; };

  void setZero() override { q_.setIdentity(); }
  Eigen::VectorXd coeffs() const override { return q_.coeffs(); }

  void copyTo(SubManifold* target) const override {
    SO3* target_derived = dynamic_cast<SO3*>(target);
    target_derived->q_ = q_;
  }

  void set(const std::initializer_list<double>& init) override {
    if (init.size() != 4) {
      throw(std::invalid_argument("input dim is different with SO3 dim!"));
    }
    int ind = 0;
    for (auto iter = init.begin(); iter != init.end(); ++iter) {
      q_.matrix()(ind) = *iter;
      ++ind;
    }
  }

 private:
  static Eigen::Quaterniond exp(const Eigen::Vector3d& s) {
    Eigen::Quaterniond ret;
    double norm = s.norm();
    ret.vec() = sin(norm / 2) / norm * s;
    ret.w() = cos(norm / 2);

    return ret;
  }

  static Eigen::Vector3d log(const Eigen::Quaterniond& q) {
    double phi = atan2(q.vec().norm(), q.w()) * 2;
    Eigen::Vector3d n = q.vec() / sin(phi / 2);
    return phi * n;
  }

  Eigen::Quaterniond q_;
};
REGISTER_UNOS(SubManifold, SO3, "SO3");
}  // namespace unos

#endif  // SO3_HH
