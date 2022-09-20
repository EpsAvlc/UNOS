#include "unos/manifold/sub_manifold.hh"

namespace unos {

SubManifold::Ptr createSubManifold(const std::string& type_id) {
  return unos::Factory<SubManifold>::produce_shared(type_id);
}

}
