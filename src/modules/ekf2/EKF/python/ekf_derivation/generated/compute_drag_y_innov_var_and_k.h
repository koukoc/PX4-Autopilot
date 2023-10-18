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
 * Symbolic function: compute_drag_y_innov_var_and_k
 *
 * Args:
 *     state: Matrix24_1
 *     P: Matrix23_23
 *     rho: Scalar
 *     cd: Scalar
 *     cm: Scalar
 *     R: Scalar
 *     epsilon: Scalar
 *
 * Outputs:
 *     innov_var: Scalar
 *     K: Matrix23_1
 */
template <typename Scalar>
void ComputeDragYInnovVarAndK(const matrix::Matrix<Scalar, 24, 1>& state,
                              const matrix::Matrix<Scalar, 23, 23>& P, const Scalar rho,
                              const Scalar cd, const Scalar cm, const Scalar R,
                              const Scalar epsilon, Scalar* const innov_var = nullptr,
                              matrix::Matrix<Scalar, 23, 1>* const K = nullptr) {
  // Total ops: 348

  // Input arrays

  // Intermediate terms (77)
  const Scalar _tmp0 = 2 * state(2, 0) * state(3, 0);
  const Scalar _tmp1 = 2 * state(1, 0);
  const Scalar _tmp2 = _tmp1 * state(0, 0);
  const Scalar _tmp3 = _tmp0 + _tmp2;
  const Scalar _tmp4 = std::pow(state(3, 0), Scalar(2));
  const Scalar _tmp5 = -2 * _tmp4;
  const Scalar _tmp6 = std::pow(state(2, 0), Scalar(2));
  const Scalar _tmp7 = -2 * _tmp6;
  const Scalar _tmp8 = _tmp5 + _tmp7 + 1;
  const Scalar _tmp9 = -state(22, 0) + state(4, 0);
  const Scalar _tmp10 = 2 * state(0, 0);
  const Scalar _tmp11 = _tmp10 * state(3, 0);
  const Scalar _tmp12 = _tmp1 * state(2, 0);
  const Scalar _tmp13 = _tmp11 + _tmp12;
  const Scalar _tmp14 = -state(23, 0) + state(5, 0);
  const Scalar _tmp15 = _tmp10 * state(2, 0);
  const Scalar _tmp16 = -_tmp15;
  const Scalar _tmp17 = _tmp1 * state(3, 0);
  const Scalar _tmp18 = _tmp16 + _tmp17;
  const Scalar _tmp19 = _tmp13 * _tmp14 + _tmp18 * state(6, 0);
  const Scalar _tmp20 = _tmp19 + _tmp8 * _tmp9;
  const Scalar _tmp21 = std::pow(state(1, 0), Scalar(2));
  const Scalar _tmp22 = 1 - 2 * _tmp21;
  const Scalar _tmp23 = _tmp22 + _tmp7;
  const Scalar _tmp24 = -_tmp2;
  const Scalar _tmp25 = _tmp0 + _tmp24;
  const Scalar _tmp26 = _tmp15 + _tmp17;
  const Scalar _tmp27 = _tmp14 * _tmp25 + _tmp26 * _tmp9;
  const Scalar _tmp28 = _tmp23 * state(6, 0) + _tmp27;
  const Scalar _tmp29 = _tmp22 + _tmp5;
  const Scalar _tmp30 = -_tmp11;
  const Scalar _tmp31 = _tmp12 + _tmp30;
  const Scalar _tmp32 = _tmp3 * state(6, 0) + _tmp31 * _tmp9;
  const Scalar _tmp33 = _tmp14 * _tmp29 + _tmp32;
  const Scalar _tmp34 = std::sqrt(Scalar(std::pow(_tmp20, Scalar(2)) + std::pow(_tmp28, Scalar(2)) +
                                         std::pow(_tmp33, Scalar(2)) + epsilon));
  const Scalar _tmp35 = cd * rho;
  const Scalar _tmp36 = Scalar(0.5) * _tmp34 * _tmp35;
  const Scalar _tmp37 = 2 * _tmp20;
  const Scalar _tmp38 = 2 * _tmp28;
  const Scalar _tmp39 = 2 * _tmp33;
  const Scalar _tmp40 = Scalar(0.25) * _tmp33 * _tmp35 / _tmp34;
  const Scalar _tmp41 =
      -_tmp3 * _tmp36 - _tmp3 * cm - _tmp40 * (_tmp18 * _tmp37 + _tmp23 * _tmp38 + _tmp3 * _tmp39);
  const Scalar _tmp42 = std::pow(state(0, 0), Scalar(2));
  const Scalar _tmp43 = -_tmp42;
  const Scalar _tmp44 = -_tmp6;
  const Scalar _tmp45 = -_tmp12;
  const Scalar _tmp46 = -_tmp0;
  const Scalar _tmp47 = -_tmp21;
  const Scalar _tmp48 = _tmp4 + _tmp47;
  const Scalar _tmp49 = _tmp42 + _tmp44;
  const Scalar _tmp50 = _tmp27 + state(6, 0) * (_tmp48 + _tmp49);
  const Scalar _tmp51 =
      -_tmp36 * _tmp50 -
      _tmp40 * (_tmp38 * (_tmp14 * (_tmp21 + _tmp4 + _tmp43 + _tmp44) + _tmp9 * (_tmp11 + _tmp45) +
                          state(6, 0) * (_tmp24 + _tmp46)) +
                _tmp39 * _tmp50) -
      _tmp50 * cm;
  const Scalar _tmp52 = _tmp43 + _tmp6;
  const Scalar _tmp53 = -_tmp17;
  const Scalar _tmp54 =
      _tmp14 * (_tmp30 + _tmp45) + _tmp9 * (_tmp48 + _tmp52) + state(6, 0) * (_tmp15 + _tmp53);
  const Scalar _tmp55 = -_tmp4;
  const Scalar _tmp56 =
      -_tmp36 * _tmp54 -
      _tmp40 * (_tmp37 * (_tmp14 * (_tmp42 + _tmp47 + _tmp55 + _tmp6) + _tmp32) + _tmp39 * _tmp54) -
      _tmp54 * cm;
  const Scalar _tmp57 = _tmp29 * cm;
  const Scalar _tmp58 = _tmp13 * _tmp37;
  const Scalar _tmp59 = _tmp25 * _tmp38;
  const Scalar _tmp60 = _tmp29 * _tmp39;
  const Scalar _tmp61 = _tmp29 * _tmp36;
  const Scalar _tmp62 = -_tmp40 * (-_tmp58 - _tmp59 - _tmp60) + _tmp57 + _tmp61;
  const Scalar _tmp63 = _tmp21 + _tmp55;
  const Scalar _tmp64 = _tmp40 * (_tmp37 * (_tmp14 * (_tmp2 + _tmp46) + _tmp9 * (_tmp16 + _tmp53) +
                                            state(6, 0) * (_tmp52 + _tmp63)) +
                                  _tmp38 * (_tmp19 + _tmp9 * (_tmp49 + _tmp63)));
  const Scalar _tmp65 = -_tmp40 * (_tmp58 + _tmp59 + _tmp60) - _tmp57 - _tmp61;
  const Scalar _tmp66 = _tmp31 * cm;
  const Scalar _tmp67 = _tmp31 * _tmp36;
  const Scalar _tmp68 = _tmp37 * _tmp8;
  const Scalar _tmp69 = _tmp26 * _tmp38;
  const Scalar _tmp70 = _tmp31 * _tmp39;
  const Scalar _tmp71 = -_tmp40 * (_tmp68 + _tmp69 + _tmp70) - _tmp66 - _tmp67;
  const Scalar _tmp72 = -_tmp40 * (-_tmp68 - _tmp69 - _tmp70) + _tmp66 + _tmp67;
  const Scalar _tmp73 = P(22, 22) * _tmp62;
  const Scalar _tmp74 = P(21, 21) * _tmp72;
  const Scalar _tmp75 =
      R +
      _tmp41 * (P(0, 5) * _tmp51 - P(1, 5) * _tmp64 + P(2, 5) * _tmp56 + P(21, 5) * _tmp72 +
                P(22, 5) * _tmp62 + P(3, 5) * _tmp71 + P(4, 5) * _tmp65 + P(5, 5) * _tmp41) +
      _tmp51 * (P(0, 0) * _tmp51 - P(1, 0) * _tmp64 + P(2, 0) * _tmp56 + P(21, 0) * _tmp72 +
                P(22, 0) * _tmp62 + P(3, 0) * _tmp71 + P(4, 0) * _tmp65 + P(5, 0) * _tmp41) +
      _tmp56 * (P(0, 2) * _tmp51 - P(1, 2) * _tmp64 + P(2, 2) * _tmp56 + P(21, 2) * _tmp72 +
                P(22, 2) * _tmp62 + P(3, 2) * _tmp71 + P(4, 2) * _tmp65 + P(5, 2) * _tmp41) +
      _tmp62 * (P(0, 22) * _tmp51 - P(1, 22) * _tmp64 + P(2, 22) * _tmp56 + P(21, 22) * _tmp72 +
                P(3, 22) * _tmp71 + P(4, 22) * _tmp65 + P(5, 22) * _tmp41 + _tmp73) -
      _tmp64 * (P(0, 1) * _tmp51 - P(1, 1) * _tmp64 + P(2, 1) * _tmp56 + P(21, 1) * _tmp72 +
                P(22, 1) * _tmp62 + P(3, 1) * _tmp71 + P(4, 1) * _tmp65 + P(5, 1) * _tmp41) +
      _tmp65 * (P(0, 4) * _tmp51 - P(1, 4) * _tmp64 + P(2, 4) * _tmp56 + P(21, 4) * _tmp72 +
                P(22, 4) * _tmp62 + P(3, 4) * _tmp71 + P(4, 4) * _tmp65 + P(5, 4) * _tmp41) +
      _tmp71 * (P(0, 3) * _tmp51 - P(1, 3) * _tmp64 + P(2, 3) * _tmp56 + P(21, 3) * _tmp72 +
                P(22, 3) * _tmp62 + P(3, 3) * _tmp71 + P(4, 3) * _tmp65 + P(5, 3) * _tmp41) +
      _tmp72 * (P(0, 21) * _tmp51 - P(1, 21) * _tmp64 + P(2, 21) * _tmp56 + P(22, 21) * _tmp62 +
                P(3, 21) * _tmp71 + P(4, 21) * _tmp65 + P(5, 21) * _tmp41 + _tmp74);
  const Scalar _tmp76 = Scalar(1.0) / (math::max<Scalar>(_tmp75, epsilon));

  // Output terms (2)
  if (innov_var != nullptr) {
    Scalar& _innov_var = (*innov_var);

    _innov_var = _tmp75;
  }

  if (K != nullptr) {
    matrix::Matrix<Scalar, 23, 1>& _k = (*K);

    _k.setZero();

    _k(21, 0) =
        _tmp76 * (P(21, 0) * _tmp51 - P(21, 1) * _tmp64 + P(21, 2) * _tmp56 + P(21, 22) * _tmp62 +
                  P(21, 3) * _tmp71 + P(21, 4) * _tmp65 + P(21, 5) * _tmp41 + _tmp74);
    _k(22, 0) =
        _tmp76 * (P(22, 0) * _tmp51 - P(22, 1) * _tmp64 + P(22, 2) * _tmp56 + P(22, 21) * _tmp72 +
                  P(22, 3) * _tmp71 + P(22, 4) * _tmp65 + P(22, 5) * _tmp41 + _tmp73);
  }
}  // NOLINT(readability/fn_size)

// NOLINTNEXTLINE(readability/fn_size)
}  // namespace sym
