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
// This file defines a class which is able to generate Eigen matrices which
// are annotated by field names. That is, you can use this class to define a
// vector consisting of 3 position value and 3 velocity values and use the
// annotatations to safely access each subset of fields by name instead of by
// index. This is meant to make matrix/vector manipulation easier to read and
// safer to use since the field indices are generated automatically at
// compile-time.
//

#ifndef SENSORFUSION_CORE_ANNOTATED_VECTOR_H
#define SENSORFUSION_CORE_ANNOTATED_VECTOR_H

#include <tuple>
#include "pvt_common/eigen_custom.h"
#include "sensorfusion/core/traits.h"

namespace sensorfusion {

template <size_t SIZE>
struct ElementAnnotation {
  static constexpr size_t size = SIZE;
};

namespace traits {

// Checks to see if a given type contains the information needed to be
// considered an ElementAnnotation type
template <typename T, typename = void>
struct is_element_annotation : std::false_type {};

template <typename T>
struct is_element_annotation<T, std::enable_if_t<(T::size > 0)>>
    : std::true_type {};

// Checks if all types in a given set meet the requirements to be considered an
// ElementAnnotation type
template <typename T, typename... Others>
struct are_all_element_annotations {
  static constexpr bool value = is_element_annotation<T>::value &&
                                are_all_element_annotations<Others...>::value;
};

template <typename T>
struct are_all_element_annotations<T> : public is_element_annotation<T> {};

// Specialization of are_all_element_annotations which unrolls a tuple of types
// to check
template <typename... Ts>
struct are_all_element_annotations<std::tuple<Ts...>>
    : public are_all_element_annotations<Ts...> {};

// Helper trait for getting the size of an element annotation type
template <typename T>
struct element_annotation_getter {
  static_assert(
      is_element_annotation<T>::value,
      "Can only get element annotation sizes from element annotations!");
  static constexpr size_t value = T::size;
};

// Sums the ElementAnnotation size for an entire tuple of ElementAnnotations
template <typename TupleT>
struct sum_tuple_annotation_size {
  static_assert(are_all_element_annotations<TupleT>::value,
                "Can only sum ElementAnnotation types");
  static constexpr size_t value =
      traits::sum_tuple_range_values<element_annotation_getter, TupleT, 0,
                                     std::tuple_size<TupleT>::value>::value;
};

// Gets the offset for the annotation at index I of the tuple type TupleT. The
// offset being the sum of the ElementAnnotation sizes preceding the annotation
// at index I
template <typename TupleT, size_t I>
struct get_annotation_offset {
  static constexpr size_t value =
      traits::sum_tuple_range_values<element_annotation_getter, TupleT, 0,
                                     I>::value;
};

// Specialization for the first element in a tuple, whose offset is 0
template <typename TupleT>
struct get_annotation_offset<TupleT, 0> {
  static constexpr size_t value = 0;
};

}  // namespace traits

template <typename... Ts>
struct AnnotationCollection {
  static_assert(
      traits::are_types_unique<Ts...>::value,
      "You can't use the same annotation twice in the same collection");

  using Tuple = std::tuple<Ts...>;

  template <typename T>
  static constexpr size_t get_index() {
    return traits::get_tuple_index<Tuple, T>::value;
  }

  template <typename T>
  static constexpr size_t get_offset() {
    constexpr size_t index = get_index<T>();
    return traits::get_annotation_offset<Tuple, index>::value;
  }

  template <typename T>
  static constexpr size_t get_length() {
    constexpr size_t index = get_index<T>();
    return std::tuple_element<index, Tuple>::type::size;
  }

  template <typename T>
  static constexpr bool is_in_collection() {
    return traits::is_in_tuple<Tuple, T>::value;
  }

  static constexpr size_t get_total_length() {
    return traits::sum_tuple_annotation_size<Tuple>::value;
  }
};

// This represents an empty set of annotations.
// This is used when defining an annotated vector, which is just a matrix with a
// single column what isn't annotated with anything.
struct EmptyAnnotationCollection {
  using Tuple = std::tuple<>;

  static constexpr size_t get_total_length() { return 1; }
};

template <typename... AnnotationTypes>
class Annotator {
  static_assert(traits::are_all_element_annotations<AnnotationTypes...>::value,
                "Annotator can only annotate with ElementAnnotation types");
  static_assert(traits::are_types_unique<AnnotationTypes...>::value,
                "Annotation types must be unique");

  using TypesTuple = std::tuple<AnnotationTypes...>;

 public:
  template <typename RowTypeAnnotations, typename ColTypeAnnotations,
            size_t RowCount = RowTypeAnnotations::get_total_length(),
            size_t ColCount = ColTypeAnnotations::
                get_total_length()>  // If we're not given any column
                                     // annotations set the col count to 1 to
                                     // act like a vector
  class annotated_matrix_impl
      : public Eigen::Matrix<double, RowCount, ColCount> {
   public:
    using EigenBaseType = Eigen::Matrix<double, RowCount, ColCount>;
    using RowAnnotations = RowTypeAnnotations;
    using ColAnnotations = ColTypeAnnotations;

    static_assert(traits::is_tuple_a_subset<
                      TypesTuple, typename RowAnnotations::Tuple>::value,
                  "All elements of an annotated matrix must be given to the "
                  "parent Annotator");
    static_assert(traits::is_tuple_a_subset<
                      TypesTuple, typename ColAnnotations::Tuple>::value,
                  "All elements of an annotated matrix must be given to the "
                  "parent Annotator");

    annotated_matrix_impl() : EigenBaseType() {}

    template <typename T>
    annotated_matrix_impl(T &&other) : EigenBaseType{std::forward<T>(other)} {};

    template <typename T>
    annotated_matrix_impl &operator=(T &&other) {
      EigenBaseType::operator=(std::forward<T>(other));
      return *this;
    }

    template <typename RowAnnotation, typename ColAnnotation>
    auto set() {
      static_assert(RowAnnotations::template is_in_collection<RowAnnotation>(),
                    "Can only get types that are used to annotate a matrix");
      static_assert(ColAnnotations::template is_in_collection<ColAnnotation>(),
                    "Can only get types that are used to annotate a matrix");

      constexpr std::size_t row_offset =
          RowAnnotations::template get_offset<RowAnnotation>();
      constexpr std::size_t row_length =
          RowAnnotations::template get_length<RowAnnotation>();

      constexpr std::size_t col_offset =
          ColAnnotations::template get_offset<ColAnnotation>();
      constexpr std::size_t col_length =
          ColAnnotations::template get_length<ColAnnotation>();

      return this->template block<row_length, col_length>(row_offset,
                                                          col_offset);
    }

    template <typename Annotation>
    auto set() {
      static_assert(RowAnnotations::template is_in_collection<Annotation>(),
                    "Can only get types that are used to annotate a vector");
      static_assert(std::tuple_size<typename ColAnnotations::Tuple>::value == 0,
                    "Calling get with a single type on a matrix");

      constexpr size_t offset =
          RowAnnotations::template get_offset<Annotation>();
      constexpr size_t length =
          RowAnnotations::template get_length<Annotation>();

      return this->template block<length, 1>(offset, 0);
    }

    template <typename Annotation>
    auto set_rows() {
      static_assert(RowAnnotations::template is_in_collection<Annotation>(),
                    "Can only get types that are used to annotate a vector");

      constexpr size_t row_offset =
          RowAnnotations::template get_offset<Annotation>();
      constexpr size_t row_length =
          RowAnnotations::template get_length<Annotation>();

      return this->template block<row_length, ColCount>(row_offset, 0);
    }

    template <typename RowAnnotation, typename ColAnnotation>
    auto get() const {
      static_assert(RowAnnotations::template is_in_collection<RowAnnotation>(),
                    "Can only get types that are used to annotate a matrix");
      static_assert(ColAnnotations::template is_in_collection<ColAnnotation>(),
                    "Can only get types that are used to annotate a matrix");

      constexpr std::size_t row_offset =
          RowAnnotations::template get_offset<RowAnnotation>();
      constexpr std::size_t row_length =
          RowAnnotations::template get_length<RowAnnotation>();

      constexpr std::size_t col_offset =
          ColAnnotations::template get_offset<ColAnnotation>();
      constexpr std::size_t col_length =
          ColAnnotations::template get_length<ColAnnotation>();

      return this->template block<row_length, col_length>(row_offset,
                                                          col_offset);
    }

    template <typename Annotation>
    auto get() const {
      static_assert(RowAnnotations::template is_in_collection<Annotation>(),
                    "Can only get types that are used to annotate a vector");
      static_assert(std::tuple_size<typename ColAnnotations::Tuple>::value == 0,
                    "Calling get with a single type on a matrix isn't allowed");

      constexpr size_t offset =
          RowAnnotations::template get_offset<Annotation>();
      constexpr size_t length =
          RowAnnotations::template get_length<Annotation>();

      return this->template block<length, 1>(offset, 0);
    }
  };

  template <typename... VectorAnnotations>
  class annotated_vector_t
      : public annotated_matrix_impl<AnnotationCollection<VectorAnnotations...>,
                                     EmptyAnnotationCollection> {
    static_assert(
        traits::are_all_types_in_tuple<TypesTuple, VectorAnnotations...>::value,
        "All elements of an annotated vector must be given to the parent "
        "Annotator");
    static_assert(traits::are_types_unique<VectorAnnotations...>::value,
                  "Annotated vectors can't contain repeated anotations");

    using BaseType =
        annotated_matrix_impl<AnnotationCollection<VectorAnnotations...>,
                              EmptyAnnotationCollection>;

   public:
    using BaseType::operator=;
    using BaseType::BaseType;
  };

  template <typename... Ts>
  struct is_annotated_vector : std::false_type {};
  template <typename... Ts>
  struct is_annotated_vector<annotated_vector_t<Ts...>> : std::true_type {};

  template <class Row, class Col>
  class annotated_matrix_t
      : public annotated_matrix_impl<typename Row::RowAnnotations,
                                     typename Col::RowAnnotations> {
    static_assert(is_annotated_vector<Row>::value,
                  "The Row type must be an annotated matrix");
    static_assert(traits::is_tuple_a_subset<
                      TypesTuple, typename Row::RowAnnotations::Tuple>::value,
                  "All elements of an annotated vector must be given to the "
                  "parent Annotator");

    static_assert(is_annotated_vector<Col>::value,
                  "The Column type must be an annotated matrix");
    static_assert(traits::is_tuple_a_subset<
                      TypesTuple, typename Col::RowAnnotations::Tuple>::value,
                  "All elements of an annotated vector must be given to the "
                  "parent Annotator");

    using BaseType = annotated_matrix_impl<typename Row::RowAnnotations,
                                           typename Col::RowAnnotations>;

   public:
    using BaseType::operator=;
    using BaseType::BaseType;
  };

  template <typename T>
  struct is_annotated_matrix : std::false_type {};
  template <typename Row, typename Col>
  struct is_annotated_matrix<annotated_matrix_t<Row, Col>> : std::true_type {};
};

namespace traits {

template <typename... Ts>
struct is_annotator : public std::false_type {};
template <typename... Ts>
struct is_annotator<Annotator<Ts...>> : public std::true_type {};

}  // namespace traits

}  // namespace sensorfusion

#endif  // SENSORFUSION_CORE_ANNOTATED_VECTOR_H
