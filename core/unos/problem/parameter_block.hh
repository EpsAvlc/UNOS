#ifndef UNOS_PROBLEM_PARAMETER_BLOCK_HH
#define UNOS_PROBLEM_PARAMETER_BLOCK_HH

#include <memory>
// #include <unos/manifold/sub_manifold.hh>

namespace unos {
class ParameterBlock {
 public:
  using Ptr = std::shared_ptr<ParameterBlock>;

  ParameterBlock(double* data, int size /*, SubManifold* sub = nullptr*/);

  static Ptr create(double* data, int size /*, SubManifold* sub = nullptr*/);

  int size();

  double* mutableData();

  void getState(double* state);

  void setState(double const* state);

  double const* state() const;

  int jacobianOffset() const;

  void setJacobianOffset(const int jacobian_offset);

 private:
  double* data_;
  int     size_;
  int     jacobian_offset_;
};

};  // namespace unos

#endif  // UNOS_PROBLEM_PARAMETER_BLOCK_HH
