#include <iostream>

#include <gtest/gtest.h>

#include "unos/manifold/manifold.hh"

TEST(UNOS, manifold) {
  unos::Manifold state{unos::SO3(), unos::Vec3()};
  std::cout << "dim: " << state.dim() << std::endl;
  std::cout << state.coeffs().transpose() << std::endl;

  Eigen::VectorXd perturbance(state.dof());
  for (size_t i = 0; i < state.dof(); ++i) {
    perturbance(i) = 0.01;
  }

  state.oplus(perturbance);
  
  std::cout << state.coeffs() << std::endl;
}
