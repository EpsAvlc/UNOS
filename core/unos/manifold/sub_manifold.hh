#ifndef UNOS_MANIFOLD_SUB_MANIFOLD_HH
#define UNOS_MANIFOLD_SUB_MANIFOLD_HH

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
  virtual void setZero() = 0;
  virtual Eigen::VectorXd coeffs() const = 0;
  virtual std::string type_id() const = 0;
  virtual void copyTo(SubManifold*) const = 0;
  virtual void set(const std::initializer_list<double>&) = 0;
  // virtual void 
};

SubManifold::Ptr createSubManifold(const std::string& type_id) {
  return unos::Factory<SubManifold>::produce_shared(type_id);
}
}  // namespace unos

#endif // UNOS_MANIFOLD_SUB_MANIFOLD_HH
