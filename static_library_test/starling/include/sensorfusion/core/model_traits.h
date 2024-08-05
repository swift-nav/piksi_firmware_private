//
// Copyright (C) 2019 Swift Navigation Inc.
// Contact: Swift Navigation <dev@swiftnav.com>
//
// This source is subject to the license found in the file 'LICENSE' which must
// be be distributed together with this source. All other rights reserved.
//
// THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
// EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
//
//
// This file defines several traits that apply to model types. A valid model
// must have the following member types:
//   * Annotator - A instantiation of the sensorfusion::Annotator template
//   * XType - An annotated vector from the aforementioned Annotator
//   * YType - An annotated vector from the aforementioned Annotator
//   * JType - An annotated matrix from the aforementioned Annotator
// A valid model must have the following member functions as well:
//   * YType f(const XType&);
//   * JType dydx(const XType&);
//

#ifndef SENSORFUSION_CORE_MODEL_TRAITS_H
#define SENSORFUSION_CORE_MODEL_TRAITS_H

#include <sensorfusion/core/annotated_vector.h>
#include <type_traits>

namespace sensorfusion {
namespace traits {

template <typename ModelT, typename = void>
struct has_annotator_type : public std::false_type {};
template <typename ModelT>
struct has_annotator_type<
    ModelT, std::enable_if_t<is_annotator<typename ModelT::Annotator>::value>>
    : public std::true_type {};

template <typename ModelT, typename = void>
struct has_x_type : public std::false_type {};
template <typename ModelT>
struct has_x_type<
    ModelT, std::enable_if_t<has_annotator_type<ModelT>::value &&
                             ModelT::Annotator::template is_annotated_vector<
                                 typename ModelT::XType>::value>>
    : public std::true_type {};

template <typename ModelT, typename = void>
struct has_y_type : public std::false_type {};
template <typename ModelT>
struct has_y_type<
    ModelT, std::enable_if_t<has_annotator_type<ModelT>::value &&
                             ModelT::Annotator::template is_annotated_vector<
                                 typename ModelT::YType>::value>>
    : public std::true_type {};

template <typename ModelT, typename = void>
struct has_j_type : public std::false_type {};
template <typename ModelT>
struct has_j_type<
    ModelT, std::enable_if_t<has_annotator_type<ModelT>::value &&
                             ModelT::Annotator::template is_annotated_matrix<
                                 typename ModelT::JType>::value>>
    : public std::true_type {};

template <typename ModelT, typename = void>
struct has_f_function : public std::false_type {};
template <typename ModelT>
struct has_f_function<
    ModelT,
    std::enable_if_t<has_x_type<ModelT>::value && has_y_type<ModelT>::value &&
                     std::is_same<decltype(std::declval<ModelT>().f(
                                      std::declval<typename ModelT::XType>())),
                                  typename ModelT::YType>::value>>
    : public std::true_type {};

template <typename ModelT, typename = void>
struct has_dydx_function : public std::false_type {};
template <typename ModelT>
struct has_dydx_function<
    ModelT,
    std::enable_if_t<has_x_type<ModelT>::value && has_j_type<ModelT>::value &&
                     std::is_same<decltype(std::declval<ModelT>().dydx(
                                      std::declval<typename ModelT::XType>())),
                                  typename ModelT::JType>::value>>
    : public std::true_type {};

template <typename ModelT>
struct is_valid_model
    : public std::conditional<
          has_annotator_type<ModelT>::value && has_x_type<ModelT>::value &&
              has_y_type<ModelT>::value && has_j_type<ModelT>::value &&
              has_f_function<ModelT>::value && has_dydx_function<ModelT>::value,
          std::true_type, std::false_type>::type {};

// Helper type useful for diagnosing why a model isn't valid.
// Note, is_valid_model<ModelT> is preferred to check validity since it can
// participate in SFINAE overloading
template <typename ModelT>
struct diagnose_model {
  static_assert(has_annotator_type<ModelT>::value,
                "Model has invalid Annotator type");
  static_assert(has_x_type<ModelT>::value, "Model has invalid XType");
  static_assert(has_y_type<ModelT>::value, "Model has invalid YType");
  static_assert(has_j_type<ModelT>::value, "Model has invalid JType");
  static_assert(has_f_function<ModelT>::value,
                "Model has invalid f member function");
  static_assert(has_dydx_function<ModelT>::value,
                "Model has invalid dydx member function");
};

}  // namespace traits
}  // namespace sensorfusion

#endif  // SENSORFUSION_CORE_MODEL_TRAITS_H
