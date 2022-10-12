#ifndef UNOS_TYPES_TYPES_H
#define UNOS_TYPES_TYPES_H

namespace unos {
enum LinearSolverType {
  // Solve the normal equations using a dense QR solver; based on
  // Eigen.
  DENSE_QR,

  // Solve the normal equations using a sparse cholesky solver; requires
  // SuiteSparse or CXSparse.
  SPARSE_NORMAL_CHOLESKY,
};

enum MinimizerType { TRUST_REGION };

enum TerminationType {
  // Minimizer terminated because one of the convergence criterion set
  // by the user was satisfied.
  //
  // 1.  (new_cost - old_cost) < function_tolerance * old_cost;
  // 2.  max_i |gradient_i| < gradient_tolerance
  // 3.  |step|_2 <= parameter_tolerance * ( |x|_2 +  parameter_tolerance)
  //
  // The user's parameter blocks will be updated with the solution.
  CONVERGENCE,

  // The solver ran for maximum number of iterations or maximum amount
  // of time specified by the user, but none of the convergence
  // criterion specified by the user were met. The user's parameter
  // blocks will be updated with the solution found so far.
  NO_CONVERGENCE,

  // The minimizer terminated because of an error.  The user's
  // parameter blocks will not be updated.
  FAILURE,
};

};  // namespace unos

#endif  // UNOS_TYPES_TYPES_H
