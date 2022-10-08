#include "unos/manifold/manifold_base.hh"

namespace unos {

ManifoldBase::Ptr createSubManifold(const std::string& type_id) {
  return unos::Factory<ManifoldBase>::produce_shared(type_id);
}

}  // namespace unos
