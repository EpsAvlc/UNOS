#ifndef UNOS_MANIFOLD_SUB_MANIFOLD_HH
#define UNOS_MANIFOLD_SUB_MANIFOLD_HH

#include <array>
#include <memory>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "unos/factory/factory.hh"

namespace unos {
class ManifoldBase {
 public:
  enum : int { DIM = 0, DOF = 0 };
  using Ptr = std::shared_ptr<ManifoldBase>;

  virtual void boxplus(double const* const x, double const* const y,
                       double* x_plus_y) const = 0;

  virtual void oplus(double const* const x, double const* const delta_x,
                     double* x_plus_delta_x) const = 0;

  virtual void oplusJacobian(double const* const x, double* jacobian) const = 0;

  virtual int dof() const = 0;

  virtual int dim() const = 0;

  virtual std::string typeID() const = 0;
  // virtual void
};

ManifoldBase::Ptr createSubManifold(const std::string& type_id);
}  // namespace unos

#endif  // UNOS_MANIFOLD_SUB_MANIFOLD_HH
