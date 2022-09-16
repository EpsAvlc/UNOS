#ifndef UNOS_PROBLEM_PROBLEM_HH
#define UNOS_PROBLEM_PROBLEM_HH

#include <memory>
#include <unordered_map>

#include "unos/evaluator/evaluator.hh"
namespace unos {
class Problem {
 public:
  struct Config {
    Config() {}
    int max_iteration_num = 50;
  };

  Problem(const Config config = Config());

  void addParameterBlock(double* parameters, int size
                         /*, SubManifold* manifold = nullptr*/);

  void addResidualBlock(const CostFunction*  cost_function,
                        const LossFunction*  loss_function,
                        double* const* const parameters,
                        int                  num_parameter_blocks);

  template <typename... Ts>
  void addResidualBlock(const CostFunction* cost_function,
                        const LossFunction* loss_function, double* x0,
                        Ts*... xs) {
    const std::array<double*, sizeof...(Ts) + 1> parameter_blocks{{x0, xs...}};
    return addResidualBlock(cost_function, loss_function,
                            parameter_blocks.data(),
                            static_cast<int>(parameter_blocks.size()));
  }

  void optimize();

 private:
  Eigen::VectorXd makeState();

  std::shared_ptr<Program> program_ptr_;

  std::shared_ptr<Evaluator> evaluator_ptr_;
};
};  // namespace unos

#endif  // UNOS_PROBLEM_PROBLEM_HH
