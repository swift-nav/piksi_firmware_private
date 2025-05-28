/**
 * Copyright (C) 2020 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

#ifndef PVT_COMMON_BUTTERWORTH_H_
#define PVT_COMMON_BUTTERWORTH_H_

#include <pvt_common/containers/circular_buffer.h>
#include <pvt_common/containers/static_vector.h>

namespace pvt_common {

namespace implementation {

// M_PI is a POSIX extension and actually not part of C11. We're defining PI
// here for maximum portability.
constexpr double pi = 3.14159265358979323846264338327950288;

// See https://swift-nav.atlassian.net/l/c/yEhH5tBv for details about the
// derivation of the algorithm below.
template <typename ArithmeticType, typename ItemType, unsigned int n,
          unsigned int k>
class SecondOrderSection {
 public:
  SecondOrderSection(ArithmeticType f_c, ArithmeticType f_s) noexcept
      : nominator_{}, denominator_{}, inputs_{}, old_outputs_{} {
    const ArithmeticType alpha =
        2.0 * std::cos(2.0 * pi * (2. * k + n + 1.) / 4. / n);
    const ArithmeticType gamma = 1.0 / std::tan(pi * f_c / f_s);
    const ArithmeticType a0 = gamma * gamma - alpha * gamma + 1.0;
    denominator_[0] = 2.0 * (1.0 - gamma * gamma) / a0;
    denominator_[1] = (gamma * gamma + alpha * gamma + 1.0) / a0;
    nominator_[0] = 1.0 / a0;
    nominator_[1] = 2.0 / a0;
    nominator_[2] = 1.0 / a0;
    f_sampling_ = f_s;

    inputs_.clear();
    old_outputs_.clear();
  };

  ItemType step(const ItemType &input) noexcept {
    inputs_.insert(input);
    if (inputs_.size() != inputs_.get_max_size() ||
        old_outputs_.size() != old_outputs_.get_max_size()) {
      old_outputs_.insert(input);
      return input;
    }
    auto X = inputs_.crbegin();       // current and previous inputs
    auto Y = old_outputs_.crbegin();  // previous outputs
    ItemType output =
        (nominator_[0] * X[0] + nominator_[1] * X[1] + nominator_[2] * X[2] -
         denominator_[0] * Y[0] - denominator_[1] * Y[1]);
    old_outputs_.insert(output);
    return output;
  };

  ArithmeticType get_phase(ArithmeticType omega) const {
    ArithmeticType real_part_nominator =
        nominator_[0] + nominator_[1] * std::cos(omega / f_sampling_) +
        nominator_[2] * std::cos(2 * omega / f_sampling_);
    ArithmeticType imag_part_nominator =
        -nominator_[1] * std::sin(omega / f_sampling_) -
        nominator_[2] * std::sin(2 * omega / f_sampling_);
    ArithmeticType phase_nominator =
        std::atan2(imag_part_nominator, real_part_nominator);

    ArithmeticType real_part_denominator =
        1 + denominator_[0] * std::cos(omega / f_sampling_) +
        denominator_[1] * std::cos(2 * omega / f_sampling_);
    ArithmeticType imag_part_denominator =
        -denominator_[0] * std::sin(omega / f_sampling_) -
        denominator_[1] * std::sin(2 * omega / f_sampling_);
    ArithmeticType phase_denominator =
        std::atan2(imag_part_denominator, real_part_denominator);
    return phase_nominator - phase_denominator;
  };

 private:
  std::array<ArithmeticType, 3> nominator_;
  std::array<ArithmeticType, 2> denominator_;
  pvt_common::containers::CircularBuffer<ItemType, 3> inputs_;
  pvt_common::containers::CircularBuffer<ItemType, 2> old_outputs_;
  ArithmeticType f_sampling_;
};

template <typename ArithmeticType, typename ItemType>
class FirstOrderSection {
 public:
  FirstOrderSection(ArithmeticType f_c, ArithmeticType f_s) noexcept
      : denominator_{}, inputs_{}, old_output_{} {
    const ArithmeticType gamma = 1.0 / std::tan(pi * f_c / f_s);
    const ArithmeticType a0 = 1.0 + gamma;
    denominator_ = (1.0 - gamma) / a0;
    nominator_[0] = 1.0 / a0;
    nominator_[1] = 1.0 / a0;
    f_sampling_ = f_s;
    inputs_.clear();
  };

  ItemType step(const ItemType &input) noexcept {
    inputs_.insert(input);
    if (inputs_.size() != inputs_.get_max_size()) {
      old_output_ = input;
      return input;
    }
    auto X = inputs_.crbegin();  // current and previous inputs
    ItemType output = nominator_[0] * X[0] + nominator_[1] * X[1] -
                      denominator_ * old_output_;
    old_output_ = output;
    return output;
  };

  ArithmeticType get_phase(ArithmeticType omega) const {
    ArithmeticType real_part_nominator =
        nominator_[0] + nominator_[1] * std::cos(omega / f_sampling_);
    ArithmeticType imag_part_nominator =
        -nominator_[1] * std::sin(omega / f_sampling_);
    ArithmeticType phase_nominator =
        std::atan2(imag_part_nominator, real_part_nominator);

    ArithmeticType real_part_denominator =
        1 + denominator_ * std::cos(omega / f_sampling_);
    ArithmeticType imag_part_denominator =
        -denominator_ * std::sin(omega / f_sampling_);
    ArithmeticType phase_denominator =
        std::atan2(imag_part_denominator, real_part_denominator);
    return phase_nominator - phase_denominator;
  };

 private:
  std::array<ArithmeticType, 2> nominator_;
  ArithmeticType denominator_;
  pvt_common::containers::CircularBuffer<ItemType, 2> inputs_;
  ItemType old_output_;
  ArithmeticType f_sampling_;
};

template <typename ArithmeticType, typename ItemType, unsigned int n,
          unsigned int k, typename = void>
class SecondOrderChain
    : private SecondOrderSection<ArithmeticType, ItemType, n, k> {
 public:
  SecondOrderChain(ArithmeticType f_c, ArithmeticType f_s) noexcept
      : SecondOrderSection<ArithmeticType, ItemType, n, k>(f_c, f_s),
        next_chain_item_{f_c, f_s} {};

  ItemType step(const ItemType &input) noexcept {
    return SecondOrderSection<ArithmeticType, ItemType, n, k>::step(
        next_chain_item_.step(input));
  };

  ArithmeticType get_phase(ArithmeticType omega) const {
    return SecondOrderSection<ArithmeticType, ItemType, n, k>::get_phase(
               omega) +
           next_chain_item_.get_phase(omega);
  };

 private:
  SecondOrderChain<ArithmeticType, ItemType, n, k + 1> next_chain_item_;
};

template <typename ArithmeticType, typename ItemType, unsigned int n,
          unsigned int k>
class SecondOrderChain<ArithmeticType, ItemType, n, k,
                       std::enable_if_t<k == n / 2 - 1>>
    : private SecondOrderSection<ArithmeticType, ItemType, n, k> {
 public:
  SecondOrderChain(ArithmeticType f_c, ArithmeticType f_s) noexcept
      : SecondOrderSection<ArithmeticType, ItemType, n, k>(f_c, f_s){};

  ItemType step(const ItemType &input) noexcept {
    return SecondOrderSection<ArithmeticType, ItemType, n, k>::step(input);
  };

  ArithmeticType get_phase(ArithmeticType omega) const {
    return SecondOrderSection<ArithmeticType, ItemType, n, k>::get_phase(omega);
  };
};

}  // namespace implementation

constexpr bool is_odd_and_not_one(unsigned int n) {
  return ((n % 2) == 1) && (n != 1);
}

constexpr unsigned int kMaxFilterOrder = 2;

/* Class template for an n-th order Butterworth filter, where the order is
 * even. The ArithmeticType is the basic type used for internal computation
 * and needs to be of float-type. The ItemType can be any class (e.g.
 * Eigen::Vector3d) using a base float-type. */
template <typename ArithmeticType, typename ItemType, unsigned int order,
          typename = void>
class ButterworthFilter
    : implementation::SecondOrderChain<ArithmeticType, ItemType, order, 0> {
 public:
  static_assert(std::is_floating_point<ArithmeticType>::value,
                "ArithmeticType must be a floating-point type");
  static_assert(
      std::is_same<
          ArithmeticType,
          std::decay_t<decltype(1.0 * std::declval<ArithmeticType>())>>::value,
      "ArithmeticType must support multiplication with a scalar");
  static_assert(
      std::is_same<ArithmeticType, std::decay_t<decltype(
                                       std::declval<ArithmeticType>() +
                                       std::declval<ArithmeticType>())>>::value,
      "ArithmeticType must support addition with another ArithmeticType");
  static_assert(order <= kMaxFilterOrder,
                "Only orders up to 2 have been validated");

  ButterworthFilter(ArithmeticType f_c, ArithmeticType f_s)
      : implementation::SecondOrderChain<ArithmeticType, ItemType, order, 0>(
            f_c, f_s),
        cutoff_frequency_{f_c} {};
  using implementation::SecondOrderChain<ArithmeticType, ItemType, order,
                                         0>::step;

  ArithmeticType get_average_passband_grp_delay() const {
    ArithmeticType omega = implementation::pi * cutoff_frequency_;
    return -implementation::SecondOrderChain<ArithmeticType, ItemType, order,
                                             0>::get_phase(omega) /
           omega;
  };

 private:
  ArithmeticType cutoff_frequency_;
};

/* Class template for a first-th order Butterworth filter. The ArithmeticType
 * is the basic type used for internal computation and needs to be of
 * float-type. The ItemType can be any class (e.g. Eigen::Vector3d) using a base
 * float-type. */
template <typename ArithmeticType, typename ItemType, unsigned int order>
class ButterworthFilter<ArithmeticType, ItemType, order,
                        std::enable_if_t<order == 1>>
    : implementation::FirstOrderSection<ArithmeticType, ItemType> {
 public:
  static_assert(std::is_floating_point<ArithmeticType>::value,
                "ArithmeticType must be a floating-point type");
  static_assert(
      std::is_same<
          ArithmeticType,
          std::decay_t<decltype(1.0 * std::declval<ArithmeticType>())>>::value,
      "ArithmeticType must support multiplication with a scalar");
  static_assert(
      std::is_same<ArithmeticType, std::decay_t<decltype(
                                       std::declval<ArithmeticType>() +
                                       std::declval<ArithmeticType>())>>::value,
      "ArithmeticType must support addition with another ArithmeticType");

  ButterworthFilter(ArithmeticType f_c, ArithmeticType f_s)
      : implementation::FirstOrderSection<ArithmeticType, ItemType>(f_c, f_s),
        cutoff_frequency_{f_c} {};
  using implementation::FirstOrderSection<ArithmeticType, ItemType>::step;

  ArithmeticType get_average_passband_grp_delay() const {
    ArithmeticType omega = implementation::pi * cutoff_frequency_;
    return -implementation::FirstOrderSection<ArithmeticType,
                                              ItemType>::get_phase(omega) /
           omega;
  };

 private:
  ArithmeticType cutoff_frequency_;
};

/* Class template for an n-th order Butterworth filter, where the order is
 * odd but not first-order (defined separately). The ArithmeticType is the
 * basic type used for internal computation and needs to be of float-type. The
 * ItemType can be any class (e.g. Eigen::Vector3d) using a base float-type. */
template <typename ArithmeticType, typename ItemType, unsigned int order>
class ButterworthFilter<ArithmeticType, ItemType, order,
                        std::enable_if_t<is_odd_and_not_one(order)>>
    : implementation::FirstOrderSection<ArithmeticType, ItemType>,
      implementation::SecondOrderChain<ArithmeticType, ItemType, order - 1, 0> {
 public:
  static_assert(std::is_floating_point<ArithmeticType>::value,
                "ArithmeticType must be a floating-point type");
  static_assert(
      std::is_same<
          ArithmeticType,
          std::decay_t<decltype(1.0 * std::declval<ArithmeticType>())>>::value,
      "ArithmeticType must support multiplication with a scalar");
  static_assert(
      std::is_same<ArithmeticType, std::decay_t<decltype(
                                       std::declval<ArithmeticType>() +
                                       std::declval<ArithmeticType>())>>::value,
      "ArithmeticType must support addition with another ArithmeticType");
  static_assert(order <= kMaxFilterOrder,
                "Only orders up to 2 have been validated");

  ButterworthFilter(ArithmeticType f_c, ArithmeticType f_s)
      : implementation::FirstOrderSection<ArithmeticType, ItemType>(f_c, f_s),
        implementation::SecondOrderChain<ArithmeticType, ItemType, order - 1,
                                         0>(f_c, f_s),
        cutoff_frequency_{f_c} {};
  ItemType step(const ItemType &input) noexcept {
    return implementation::
        SecondOrderChain<ArithmeticType, ItemType, order - 1, 0>::step(
            implementation::FirstOrderSection<ArithmeticType, ItemType>::step(
                input));
  }

  ArithmeticType get_average_passband_grp_delay() const {
    ArithmeticType omega = implementation::pi * cutoff_frequency_;
    return -(implementation::SecondOrderChain<ArithmeticType, ItemType, order,
                                              0>::get_phase(omega) +
             implementation::FirstOrderSection<ArithmeticType,
                                               ItemType>::get_phase(omega)) /
           omega;
  };

 private:
  ArithmeticType cutoff_frequency_;
};

}  // namespace pvt_common
#endif  // PVT_COMMON_BUTTERWORTH_H_
