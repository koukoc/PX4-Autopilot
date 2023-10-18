// -----------------------------------------------------------------------------
// This file was autogenerated by symforce from template:
//     function/FUNCTION.h.jinja
// Do NOT modify by hand.
// -----------------------------------------------------------------------------

#pragma once

#include <matrix/math.hpp>

namespace sym {

/**
 * This function was autogenerated from a symbolic function. Do not modify by hand.
 *
 * Symbolic function: compute_yaw_312_innov_var_and_h_alternate
 *
 * Args:
 *     state: Matrix24_1
 *     P: Matrix23_23
 *     R: Scalar
 *     epsilon: Scalar
 *
 * Outputs:
 *     innov_var: Scalar
 *     H: Matrix23_1
 */
template <typename Scalar>
void ComputeYaw312InnovVarAndHAlternate(const matrix::Matrix<Scalar, 24, 1>& state,
                                        const matrix::Matrix<Scalar, 23, 23>& P, const Scalar R,
                                        const Scalar epsilon, Scalar* const innov_var = nullptr,
                                        matrix::Matrix<Scalar, 23, 1>* const H = nullptr) {
  // Total ops: 57

  // Input arrays

  // Intermediate terms (15)
  const Scalar _tmp0 = 2 * state(0, 0);
  const Scalar _tmp1 = 2 * state(1, 0);
  const Scalar _tmp2 = std::pow(state(3, 0), Scalar(2));
  const Scalar _tmp3 = std::pow(state(1, 0), Scalar(2));
  const Scalar _tmp4 = -2 * _tmp2 - 2 * _tmp3 + 1;
  const Scalar _tmp5 = -_tmp0 * state(3, 0);
  const Scalar _tmp6 = _tmp1 * state(2, 0);
  const Scalar _tmp7 = _tmp5 + _tmp6;
  const Scalar _tmp8 = _tmp7 + epsilon * ((((_tmp7) > 0) - ((_tmp7) < 0)) + Scalar(0.5));
  const Scalar _tmp9 = std::pow(_tmp8, Scalar(2));
  const Scalar _tmp10 = _tmp4 / _tmp9;
  const Scalar _tmp11 = Scalar(1.0) / (_tmp8);
  const Scalar _tmp12 = _tmp9 / (std::pow(_tmp4, Scalar(2)) + _tmp9);
  const Scalar _tmp13 = _tmp12 * (_tmp10 * (_tmp0 * state(2, 0) + _tmp1 * state(3, 0)) -
                                  _tmp11 * (-_tmp1 * state(0, 0) + 2 * state(2, 0) * state(3, 0)));
  const Scalar _tmp14 = _tmp12 * (_tmp10 * (_tmp2 - _tmp3 - std::pow(state(0, 0), Scalar(2)) +
                                            std::pow(state(2, 0), Scalar(2))) -
                                  _tmp11 * (_tmp5 - _tmp6));

  // Output terms (2)
  if (innov_var != nullptr) {
    Scalar& _innov_var = (*innov_var);

    _innov_var = R - _tmp13 * (-P(0, 0) * _tmp13 - P(2, 0) * _tmp14) -
                 _tmp14 * (-P(0, 2) * _tmp13 - P(2, 2) * _tmp14);
  }

  if (H != nullptr) {
    matrix::Matrix<Scalar, 23, 1>& _h = (*H);

    _h.setZero();

    _h(0, 0) = -_tmp13;
    _h(2, 0) = -_tmp14;
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
