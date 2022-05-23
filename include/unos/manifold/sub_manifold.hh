#ifndef UNOS_MAINFOLD_MAINFOLD_HH__
#define UNOS_MAINFOLD_MAINFOLD_HH__

#include <array>
#include <memory>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace unos {
class SubManifold {
 public:
  using Ptr = std::shared_ptr<SubManifold>;
  virtual void boxplus(const Eigen::VectorXd& s) = 0;
  virtual void oplus(const Eigen::VectorXd& i) = 0;
  virtual uint16_t dim() const = 0;
  virtual uint16_t dof() const = 0;
  virtual void setZero() = 0;
  virtual Eigen::VectorXd coeffs() const = 0;

 protected:
 private:
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
  void oplus(const Eigen::VectorXd& s) override {
    if (s.size() != 3) {
      throw(std::logic_error("Dismatch paramter in SO3. Input has size: " +
                             std::to_string(s.size())));
    }
    q_ = q_ * exp(s);
  }
  uint16_t dim() const { return 4; };
  uint16_t dof() const { return 3; };
  void setZero() override { q_.setIdentity(); }
  Eigen::VectorXd coeffs() const override { return q_.coeffs(); }

 private:
  Eigen::Quaterniond exp(const Eigen::Vector3d& s) {
    Eigen::Quaterniond ret;
    double norm = s.norm();
    ret.vec() = sin(norm / 2) / norm * s;
    ret.w() = cos(norm / 2);

    return ret;
  }
  Eigen::Quaterniond q_;
};

template <uint16_t DIM>
class VecX : public SubManifold {
 public:
  using Ptr = std::shared_ptr<VecX<DIM>>;
  void boxplus(const Eigen::VectorXd& s) override { s_ += s; }
  void oplus(const Eigen::VectorXd& s) override {
    if (s.size() != DIM) {
      throw(std::logic_error("Dismatch paramter in SO3. Input has size: " +
                             std::to_string(s.size())));
    }
    s_ += s;
  }
  uint16_t dim() const override { return DIM; };
  uint16_t dof() const override { return DIM; };
  void setZero() override {
    s_.setZero();
  }
  Eigen::VectorXd coeffs() const override { return s_; }

 private:
  Eigen::Matrix<double, DIM, 1> s_;
};

using Vec3 = VecX<3>;

}  // namespace unos

#endif  // UNOS_MAINFOLD_MAINFOLD_HH__
