#ifndef UNOS_MAINFOLD_MAINFOLD_HH__
#define UNOS_MAINFOLD_MAINFOLD_HH__

#include <array>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "unos/factory/factory.hh"

namespace unos {
class SubManifold {
 public:
  using Ptr = std::shared_ptr<SubManifold>;
  virtual void boxplus(const Eigen::VectorXd& s) = 0;
  virtual Eigen::VectorXd boxminus(const SubManifold* s) const = 0;
  virtual void oplus(const Eigen::VectorXd& i) = 0;
  virtual uint16_t dim() const = 0;
  virtual uint16_t dof() const = 0;
  virtual void setZero() = 0;
  virtual Eigen::VectorXd coeffs() const = 0;
  virtual std::string type_id() const = 0;
  virtual void copyTo(SubManifold*) const = 0;
};

class SO3 : public SubManifold {
 public:
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

template <uint16_t DIM>
class VecX : public SubManifold {
 public:
  using Ptr = std::shared_ptr<VecX<DIM>>;
  void boxplus(const Eigen::VectorXd& s) override { s_ += s; }
  Eigen::VectorXd boxminus(const SubManifold* rhs) const override {
    const VecX<DIM>* rhs_derived = dynamic_cast<const VecX<DIM>*>(rhs);
    Eigen::VectorXd ret = this->s_ - rhs_derived->s_;
    return ret;
  }

  void oplus(const Eigen::VectorXd& s) override {
    if (s.size() != DIM) {
      throw(std::logic_error("Dismatch paramter in SO3. Input has size: " +
                             std::to_string(s.size())));
    }
    s_ += s;
  }

  uint16_t dim() const override { return DIM; };
  uint16_t dof() const override { return DIM; };
  std::string type_id() const override { return "Vec" + std::to_string(DIM); };

  void setZero() override { s_.setZero(); }
  Eigen::VectorXd coeffs() const override { return s_; }

  void copyTo(SubManifold* target) const override {
    VecX<DIM>* target_derived = dynamic_cast<VecX<DIM>*>(target);
    target_derived->s_ = s_;
  }

 private:
  Eigen::Matrix<double, DIM, 1> s_;
};


SubManifold::Ptr createSubManifold(const std::string& type_id) {
  return unos::Factory<SubManifold>::produce_shared(type_id);
}

using Vec1 = VecX<1>;
using Vec2 = VecX<2>;
using Vec3 = VecX<3>;
REGISTER_UNOS(SubManifold, SO3, "SO3");
REGISTER_UNOS(SubManifold, Vec1, "Vec1");
REGISTER_UNOS(SubManifold, Vec2, "Vec2");
REGISTER_UNOS(SubManifold, Vec3, "Vec3");
}  // namespace unos

#endif  // UNOS_MAINFOLD_MAINFOLD_HH__
