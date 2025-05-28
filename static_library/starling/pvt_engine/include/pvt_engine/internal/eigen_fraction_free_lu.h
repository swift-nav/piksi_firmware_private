/*
 * Copyright (C) 2018 Swift Navigation Inc.
 * Contact: Swift Navigation <dev@swiftnav.com>
 *
 * This source is subject to the license found in the file 'LICENSE' which must
 * be distributed together with this source. All other rights reserved.
 *
 * THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
 * EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
 */

/*
 * Borrowed largely from FullPivLU.h
 */

#ifndef FRACTION_FREE_LU_H
#define FRACTION_FREE_LU_H

#include <limits>
#include <numeric>

#define EIGEN_STATIC_ASSERT_INTEGER(TYPE)         \
  EIGEN_STATIC_ASSERT(NumTraits<TYPE>::IsInteger, \
                      THIS_FUNCTION_IS_FOR_INTEGER_NUMERIC_TYPES)

constexpr Eigen::Index NO_PIVOT_FOUND = -1;

template <typename MatrixType>
bool find_any_pivot_in_block(const MatrixType &X, const Eigen::Index &start_row,
                             const Eigen::Index &start_col,
                             Eigen::Index *pivot_row, Eigen::Index *pivot_col) {
  for (Eigen::Index col = start_col; col < X.cols(); ++col) {
    for (Eigen::Index row = start_row; row < X.rows(); ++row) {
      if (X(row, col) != 0) {
        *pivot_row = row;
        *pivot_col = col;
        return true;
      }
    }
  }
  *pivot_row = NO_PIVOT_FOUND;
  *pivot_col = NO_PIVOT_FOUND;
  return false;
}

namespace Eigen {
namespace internal {
template <typename _MatrixType>
struct traits<FractionFreeLU<_MatrixType>> : traits<_MatrixType> {
  typedef MatrixXpr XprKind;
  typedef SolverStorage StorageKind;
  enum { Flags = 0 };
};

}  // end namespace internal

template <typename Scalar>
Scalar gcd(Scalar n1, Scalar n2) {
  return (n2 == 0) ? n1 : gcd(n2, n1 % n2);
}

template <typename MatrixType>
typename internal::traits<MatrixType>::Scalar gcd(const MatrixType &X) {
  typename internal::traits<MatrixType>::Scalar gcd_ = 1;
  bool found_nonzero = false;
  for (Eigen::Index i = 0; i < X.rows(); ++i) {
    for (Eigen::Index j = 0; j < X.cols(); ++j) {
      if (X(i, j) != 0) {
        gcd_ = found_nonzero ? gcd(gcd_, std::abs(X(i, j))) : X(i, j);
        found_nonzero = true;
      }
    }
  }
  return gcd_;
}

/*
 * Computes the LU decomposition of an integer matrix using the method
 * described in:
 *
 * Dureisseix, D. (2012).
 * Generalized fraction-free LU factorization for singular systems with kernel
 * extraction.
 * Linear Algebra and Its Applications, 436(1), 27–40.
 * doi:10.1016/j.laa.2011.06.013
 *
 * The decomposition produces a factorized version of a matrix A such
 * that P A = L D^-1 U
 *
 * Where P is a matrix of row permutations, L is lower diagonal
 * D is diagonal and U is upper diagonal.  Unlike the floating point
 * LU decomposition both L and U have non unit diagonal values that
 * are directly related to the matrix D.  Namely,
 *
 *     |p_0       |
 * L = |L_10 p_1  |
 *     |L_m0 ... 1|
 *
 *     |p_0 U_01 U_0n|
 * U = |     p_1     |
 *     |     ...  p_m|
 *
 * D =  diag(p_0, ..., p_i * p_i-1, p_n-1)
 *
 * See the reference above for more details.
 *
 * NB: There are some errors in that paper. Which include,
 *
 *  - Section 4, the tests cases the expected D is given with
 *    an off diagonal element which clearly can't be the case.
 *
 *  - Algorithm 4, line 8 you need to search for pivots in the
 *    rank kpivot in [k, n] and lpivot in [k, m] NOT the
 *    suggested kpivot in [k+1, n] and lpivot in [k+1, n]
 *
 */
template <typename _MatrixType>
class FractionFreeLU : public SolverBase<FractionFreeLU<_MatrixType>> {
 public:
  typedef _MatrixType MatrixType;
  typedef SolverBase<FractionFreeLU> Base;

  EIGEN_GENERIC_PUBLIC_INTERFACE(FractionFreeLU)
  enum {
    MaxRowsAtCompileTime = MatrixType::MaxRowsAtCompileTime,
    MaxColsAtCompileTime = MatrixType::MaxColsAtCompileTime
  };
  typedef typename internal::plain_row_type<MatrixType, StorageIndex>::type
      IntRowVectorType;
  typedef typename internal::plain_col_type<MatrixType, StorageIndex>::type
      IntColVectorType;
  typedef PermutationMatrix<ColsAtCompileTime, MaxColsAtCompileTime>
      PermutationQType;
  typedef PermutationMatrix<RowsAtCompileTime, MaxRowsAtCompileTime>
      PermutationPType;
  typedef typename MatrixType::PlainObject PlainObject;
  typedef Eigen::Matrix<Scalar, RowsAtCompileTime, 1,
                        internal::traits<MatrixType>::Options,
                        MaxRowsAtCompileTime, 1>
      VectorType;

  /**
   * \brief Default Constructor.
   *
   * The default constructor is useful in cases in which the user intends to
   * perform decompositions via LU::compute(const MatrixType&).
   */
  FractionFreeLU();

  /** \brief Default Constructor with memory preallocation
   *
   * Like the default constructor but with preallocation of the internal data
   * according to the specified problem \a size.
   * \sa FractionFreeLU()
   */
  FractionFreeLU(Index rows, Index cols);

  /** Constructor.
   *
   * \param matrix the matrix of which to compute the LU decomposition.
   *               It is required to be nonzero.
   */
  template <typename InputType>
  explicit FractionFreeLU(const EigenBase<InputType> &matrix);

  /** \brief Constructs a LU factorization from a given matrix
   *
   * This overloaded constructor is provided for \link InplaceDecomposition
   * inplace decomposition \endlink when \c MatrixType is a Eigen::Ref.
   *
   * \sa FractionFreeLU(const EigenBase&)
   */
  template <typename InputType>
  explicit FractionFreeLU(EigenBase<InputType> &matrix);

  /** Computes the LU decomposition of the given matrix.
   *
   * \param matrix the matrix of which to compute the LU decomposition.
   *               It is required to be nonzero.
   *
   * \returns a reference to *this
   */
  template <typename InputType>
  FractionFreeLU &compute(const EigenBase<InputType> &matrix) {
    m_lu = matrix.derived();
    computeInPlace();
    return *this;
  }

  Scalar scaleByFactor(const Scalar input, const double &factor) const {
    return static_cast<Scalar>(std::round(static_cast<double>(input) * factor));
  }

  Scalar scaleRowElement(const Scalar input, const Index &row) const {
    return static_cast<Scalar>(
        std::round(static_cast<double>(input) * m_row_scale_factors[row]));
  }

  Scalar scaleElement(const Scalar input, const Index &row,
                      const Index &col) const {
    return static_cast<Scalar>(std::round(
        static_cast<double>(input) *
        ((row > col) ? m_col_scale_factors[col] : m_row_scale_factors[row])));
  }

  double get_prev_to_curr_row_scale_factor(const Index &curr_row) const {
    double prev_to_curr_row_scale_factor =
        (curr_row > 0)
            ? static_cast<double>(m_row_scale_factors[curr_row - 1]) /
                  static_cast<double>(m_lu_multiplication_factors[curr_row - 1])
            : 1.0;
    prev_to_curr_row_scale_factor *=
        static_cast<double>(m_lu_multiplication_factors[curr_row]);
    prev_to_curr_row_scale_factor /=
        static_cast<double>(m_lu_division_factors[curr_row]);
    return prev_to_curr_row_scale_factor;
  }

  double get_prev_to_curr_col_scale_factor(const Index &curr_col) const {
    double prev_to_curr_col_scale_factor =
        (curr_col > 0) ? static_cast<double>(m_col_scale_factors[curr_col - 1])
                       : 1.0;
    if (curr_col > 0) {
      prev_to_curr_col_scale_factor *=
          static_cast<double>(m_lu_multiplication_factors[curr_col - 1]);
    }
    prev_to_curr_col_scale_factor /=
        static_cast<double>(m_lu_division_factors[curr_col]);
    return prev_to_curr_col_scale_factor;
  }

  /** \returns the LU decomposition matrix: the upper-triangular part is U, the
   * unit-lower-triangular part is L (at least for square matrices; in the
   * non-square
   * case, special care is needed, see the documentation of class
   * FractionFreeLU).
   *
   * \sa matrixL(), matrixU()
   */
  inline MatrixType matrixLU() const {
    eigen_assert(m_isInitialized && "LU is not initialized.");
    MatrixType LU(m_lu);
    for (Index row = 0; row < LU.rows(); ++row) {
      for (Index col = 0; col < LU.cols(); ++col) {
        LU(row, col) = scaleElement(LU(row, col), row, col);
      }
    }
    return LU;
  }

  MatrixType getL() const {
    MatrixType L(
        m_lu.leftCols(m_lu.rows()).template triangularView<Eigen::Lower>());
    L.diagonal() = m_lu.diagonal();
    for (Index row = 0; row < L.rows(); ++row) {
      for (Index col = 0; col <= row; ++col) {
        L(row, col) = scaleElement(L(row, col), row, col);
      }
    }
    L(L.rows() - 1, L.rows() - 1) = 1;
    return L;
  }

  MatrixType getU() const {
    MatrixType U = m_lu.template triangularView<Eigen::Upper>();
    U.bottomRows(m_lu.rows() - m_rank).array().fill(0);
    for (Index row = 0; row < m_rank; ++row) {
      for (Index col = row; col < U.cols(); ++col) {
        U(row, col) = scaleElement(U(row, col), row, col);
      }
    }
    return MatrixType(U);
  }

  VectorType getDiagonal() const;
  /** \returns the permutation matrix P
   *
   * \sa permutationQ()
   */
  EIGEN_DEVICE_FUNC inline const PermutationPType &getP() const {
    eigen_assert(m_isInitialized && "LU is not initialized.");
    return m_p;
  }

  EIGEN_DEVICE_FUNC inline const PermutationPType &getQ() const {
    eigen_assert(m_isInitialized && "LU is not initialized.");
    return m_q;
  }

  /** \returns the kernel of the matrix, also called its null-space. The columns
   * of the returned matrix
   * will form a basis of the kernel.
   *
   * \note If the kernel has dimension zero, then the returned matrix is a
   * column-vector filled with zeros.
   *
   * \note This method has to determine which pivots should be considered
   * nonzero.
   *       For that, it uses the threshold value that you can control by
   * calling
   *       setThreshold(const RealScalar&).
   *
   * \sa image()
   */
  MatrixType kernel() const;
  /** \returns the determinant of the matrix of which
   * *this is the LU decomposition. It has only linear complexity
   * (that is, O(n) where n is the dimension of the square matrix)
   * as the LU decomposition has already been computed.
   *
   * \note This is only for square matrices.
   *
   * \note For fixed-size matrices of size up to 4, MatrixBase::determinant()
   * offers
   *       optimized paths.
   *
   * \warning a determinant can be very big or small, so for matrices
   * of large enough dimension, there is a risk of overflow/underflow.
   *
   * \sa MatrixBase::determinant()
   */
  typename internal::traits<MatrixType>::Scalar determinant() const;

  inline bool riskOfOverflow() const { return m_riskOfOverflow; }

  /** \returns the rank of the matrix of which *this is the LU decomposition.
   *
   * \note This method has to determine which pivots should be considered
   * nonzero.
   *       For that, it uses the threshold value that you can control by
   * calling
   *       setThreshold(const RealScalar&).
   */
  inline Index rank() const { return m_rank; }

  /** \returns the dimension of the kernel of the matrix of which *this is the
   * LU decomposition.
   *
   * \note This method has to determine which pivots should be considered
   * nonzero.
   *       For that, it uses the threshold value that you can control by
   * calling
   *       setThreshold(const RealScalar&).
   */
  inline Index dimensionOfKernel() const {
    eigen_assert(m_isInitialized && "LU is not initialized.");
    return cols() - rank();
  }

  /** \returns true if the matrix of which *this is the LU decomposition
   * represents an injective
   *          linear map, i.e. has trivial kernel; false otherwise.
   *
   * \note This method has to determine which pivots should be considered
   * nonzero.
   *       For that, it uses the threshold value that you can control by
   * calling
   *       setThreshold(const RealScalar&).
   */
  inline bool isInjective() const {
    eigen_assert(m_isInitialized && "LU is not initialized.");
    return rank() == cols();
  }

  /** \returns true if the matrix of which *this is the LU decomposition
   * represents a surjective
   *          linear map; false otherwise.
   *
   * \note This method has to determine which pivots should be considered
   * nonzero.
   *       For that, it uses the threshold value that you can control by
   * calling
   *       setThreshold(const RealScalar&).
   */
  inline bool isSurjective() const {
    eigen_assert(m_isInitialized && "LU is not initialized.");
    return rank() == rows();
  }

  /** \returns true if the matrix of which *this is the LU decomposition is
   * invertible.
   *
   * \note This method has to determine which pivots should be considered
   * nonzero.
   *       For that, it uses the threshold value that you can control by
   * calling
   *       setThreshold(const RealScalar&).
   */
  inline bool isInvertible() const {
    eigen_assert(m_isInitialized && "LU is not initialized.");
    return isInjective() && (m_lu.rows() == m_lu.cols());
  }

  MatrixType reconstructedMatrix() const;

  MatrixType reducedRowEchelonForm() const;

  EIGEN_DEVICE_FUNC inline Index rows() const { return m_lu.rows(); }
  EIGEN_DEVICE_FUNC inline Index cols() const { return m_lu.cols(); }

 protected:
  static void check_template_parameters() {
    EIGEN_STATIC_ASSERT_INTEGER(Scalar);
  }

  void computeInPlace();

  MatrixType m_original_matrix;
  MatrixType m_lu;
  VectorType m_lu_multiplication_factors;
  VectorType m_lu_division_factors;
  PermutationPType m_p;
  PermutationQType m_q;
  IntRowVectorType m_rowPermutations;
  IntColVectorType m_colPermutations;
  Eigen::Index m_rank;
  bool m_isInitialized;
  signed char m_permutationDeterminant;
  bool m_riskOfOverflow;
  Eigen::Matrix<double, Eigen::Dynamic, 1, 0, MatrixType::MaxRowsAtCompileTime,
                1>
      m_row_scale_factors;
  Eigen::Matrix<double, Eigen::Dynamic, 1, 0, MatrixType::MaxColsAtCompileTime,
                1>
      m_col_scale_factors;
};

template <typename MatrixType>
FractionFreeLU<MatrixType>::FractionFreeLU() : m_isInitialized(false) {}

template <typename MatrixType>
FractionFreeLU<MatrixType>::FractionFreeLU(Index rows, Index cols)
    : m_original_matrix(rows, cols),
      m_lu(rows, cols),
      m_lu_multiplication_factors(rows),
      m_lu_division_factors(rows),
      m_p(rows),
      m_q(cols),
      m_rowPermutations(rows),
      m_colPermutations(cols),
      m_rank(0),
      m_isInitialized(false),
      m_permutationDeterminant(1),
      m_riskOfOverflow(false),
      m_row_scale_factors(rows),
      m_col_scale_factors(cols) {}

template <typename MatrixType>
template <typename InputType>
FractionFreeLU<MatrixType>::FractionFreeLU(const EigenBase<InputType> &matrix)
    : m_original_matrix(matrix.derived()),
      m_lu(matrix.rows(), matrix.cols()),
      m_lu_multiplication_factors(matrix.rows()),
      m_lu_division_factors(matrix.rows()),
      m_p(matrix.rows()),
      m_q(matrix.rows()),
      m_rowPermutations(matrix.rows()),
      m_colPermutations(matrix.cols()),
      m_rank(0),
      m_isInitialized(false),
      m_permutationDeterminant(1),
      m_riskOfOverflow(false),
      m_row_scale_factors(matrix.rows()),
      m_col_scale_factors(matrix.cols()) {
  compute(matrix.derived());
}

template <typename MatrixType>
template <typename InputType>
FractionFreeLU<MatrixType>::FractionFreeLU(EigenBase<InputType> &matrix)
    : m_original_matrix(matrix.derived()),
      m_lu(matrix.derived()),
      m_lu_multiplication_factors(matrix.rows()),
      m_lu_division_factors(matrix.rows()),
      m_p(matrix.rows()),
      m_q(matrix.cols()),
      m_rowPermutations(matrix.rows()),
      m_colPermutations(matrix.cols()),
      m_rank(0),
      m_isInitialized(false),
      m_permutationDeterminant(1),
      m_riskOfOverflow(false),
      m_row_scale_factors(matrix.rows()),
      m_col_scale_factors(matrix.cols()) {
  computeInPlace();
}

template <typename MatrixType>
typename FractionFreeLU<MatrixType>::VectorType
FractionFreeLU<MatrixType>::getDiagonal() const {
  VectorType diag = m_lu.diagonal();
  for (Index index = 0; index < diag.size(); ++index) {
    diag[index] = scaleElement(diag[index], index, index);
  }
  diag[diag.size() - 1] = diag[diag.size() - 2];
  // the first entry remains the same, the last
  // is the n-1 diagonal value and the rest
  // are the product of the i and i-1.
  for (Eigen::Index i = diag.size() - 2; i > 0; --i) {
    auto tmp = diag[i];
    (void)tmp;
    diag[i] *= diag[i - 1];
    assert(diag[i] / diag[i - 1] == tmp);  // overflow!
  }
  return diag;
}

template <typename MatrixType>
void FractionFreeLU<MatrixType>::computeInPlace() {
  check_template_parameters();

  // the permutations are stored as int indices, so just to be sure:
  eigen_assert(m_lu.rows() <= NumTraits<int>::highest() &&
               m_lu.cols() <= NumTraits<int>::highest());

  m_rank = 0;
  m_rowPermutations.resize(m_lu.rows());
  m_colPermutations.resize(m_lu.cols());

  const Eigen::Index m = m_lu.rows();
  const Eigen::Index n = m_lu.cols();
  Eigen::Index size = m_lu.diagonalSize();

  if (size == 0) {
    m_p.setIdentity(m);
    m_q.setIdentity(n);
    return;
  }
  const Eigen::Index max_rank = std::min(m, n);

  double max_value = static_cast<double>(std::numeric_limits<Scalar>::max());
  Scalar overflow_risk_threshold = static_cast<Scalar>(std::sqrt(max_value));

  Scalar pivot_value;
  Scalar old_pivot_value = 1;
  Eigen::Index number_of_swaps = 0;
  Eigen::Index pivot_row, pivot_col;

  m_row_scale_factors.topRows(m).fill(1.0);
  m_col_scale_factors.topRows(n).fill(1.0);
  m_lu_division_factors.topRows(m > 2 ? 2 : m).fill(1);
  for (Eigen::Index row = 0; row < max_rank; ++row) {
    /*
     * Pivoting strategy:
     */
    m_rowPermutations.coeffRef(row) = row;
    m_colPermutations.coeffRef(row) = row;

    bool found_pivot =
        find_any_pivot_in_block(m_lu, row, row, &pivot_row, &pivot_col);
    if (!found_pivot) {
      // Matrix is rank deficient, we add some regularization so the algorithm
      // can continue, but flag these rows as being null.
      m_lu_multiplication_factors[row] = 1;
      m_lu(row, row) = scaleByFactor(
          old_pivot_value, 1.0 / get_prev_to_curr_row_scale_factor(row));
    } else {
      ++m_rank;
      // Apply pivots
      if (pivot_row != row) {
        m_rowPermutations.coeffRef(row) = pivot_row;
        m_lu.row(pivot_row).swap(m_lu.row(row));
        ++number_of_swaps;
      }
      if (pivot_col != row) {
        m_colPermutations.coeffRef(row) = pivot_col;
        m_lu.col(pivot_col).swap(m_lu.col(row));
        ++number_of_swaps;
      }

      m_lu_multiplication_factors[row] =
          gcd(m_lu.row(row).segment(row, m_lu.cols() - row));
      m_lu.row(row).segment(row, m_lu.cols() - row) /=
          m_lu_multiplication_factors[row];
    }

    if (row > 0 && row + 1 < m) {
      m_lu_division_factors[row + 1] =
          scaleElement(old_pivot_value, row - 1, row - 1);
    }

    /*
     * Decomposition step.
     */
    pivot_value = m_lu(row, row);
    for (Eigen::Index i = row + 1; i < m; ++i) {
      Scalar A_ik = m_lu(i, row);
      for (Eigen::Index j = row + 1; j < n; ++j) {
        m_lu(i, j) = pivot_value * m_lu(i, j) - A_ik * m_lu(row, j);
        if (std::abs(m_lu(i, j)) >= overflow_risk_threshold) {
          m_riskOfOverflow = true;
        }
      }
    }
    old_pivot_value = pivot_value;

    m_row_scale_factors[row] = ((row > 0) ? m_row_scale_factors[row - 1] : 1.0);
    m_row_scale_factors[row] *= get_prev_to_curr_row_scale_factor(row);
    if (row < m - 1) {
      m_col_scale_factors[row] =
          ((row > 0) ? m_col_scale_factors[row - 1] : 1.0);
      m_col_scale_factors[row] *= get_prev_to_curr_col_scale_factor(row);
    }
  }

  for (Eigen::Index i = 0; i < m; ++i) {
    for (Eigen::Index j = 0; j < n; ++j) {
      if (std::abs(scaleElement(m_lu(i, j), i, j)) >= overflow_risk_threshold) {
        m_riskOfOverflow = true;
      }
    }
  }

  m_p.setIdentity(m);
  for (Index k = max_rank - 1; k >= 0; --k) {
    m_p.applyTranspositionOnTheRight(k, m_rowPermutations.coeff(k));
  }

  m_q.setIdentity(n);
  for (Index k = 0; k < max_rank; ++k) {
    m_q.applyTranspositionOnTheRight(k, m_colPermutations.coeff(k));
  }

  m_permutationDeterminant = (number_of_swaps % 2 != 0) ? -1 : 1;
}

template <typename MatrixType>
typename internal::traits<MatrixType>::Scalar
FractionFreeLU<MatrixType>::determinant() const {
  eigen_assert(m_isInitialized && "LU is not initialized.");
  eigen_assert(m_lu.rows() == m_lu.cols() &&
               "You can't take the determinant of a non-square matrix!");
  if (m_rank < m_lu.rows()) {
    return 0;
  }

  Index last_index = m_lu.rows() - 1;
  Scalar last_diag_element =
      scaleElement(m_lu(last_index, last_index), last_index, last_index);
  return Scalar(m_permutationDeterminant) * last_diag_element;
}

template <typename MatrixType>
MatrixType FractionFreeLU<MatrixType>::reducedRowEchelonForm() const {
  MatrixType R(m_lu);

  Eigen::Index i, j, k;
  assert(m_rank > 0);

  Eigen::Index m = R.rows();
  Eigen::Index n = R.cols();
  R.bottomRows(m - m_rank).fill(0);

  /* Convert row echelon form to reduced row echelon form */
  if (m_rank > 1) {
    for (k = 0; k < n - m_rank; ++k) {
      for (i = m_rank - 2; i >= 0; --i) {
        auto tmp = scaleRowElement(R(m_rank - 1, m_rank - 1), m_rank - 1) *
                   R(i, m_rank + k);
        for (j = i + 1; j < m_rank; ++j) {
          auto tmp2 = R(i, j) * R(j, m_rank + k);
          if (j > m_rank - 2) {
            tmp2 = scaleRowElement(tmp2, j);
          }
          tmp -= tmp2;
        }
        R(i, m_rank + k) = tmp / R(i, i);
      }
    }
    for (j = 0; j < n; ++j) {
      R(m_rank - 1, j) = scaleRowElement(R(m_rank - 1, j), m_rank - 1);
    }
    Scalar scaled_determinant = R(m_rank - 1, m_rank - 1);
    /* clear pivot columns */
    for (i = 0; i < m_rank; ++i) {
      for (j = 0; j < m_rank; ++j) {
        if (i == j) {
          R(j, i) = scaled_determinant;
        } else {
          R(j, i) = 0;
        }
      }
    }
  }

  // Prevent the accumulation of large integers which is particularly
  // prevalent when computing the LU decomposition of the output of
  // a kernel computation.
  Scalar gcd_ = gcd(R.topRows(m_rank));
  if (gcd_ != 1) {
    R /= gcd_;
  }

  return R;
}

/** \returns the matrix represented by the decomposition,
 * i.e., it returns the product: \f$ P^{-1} L U Q^{-1} \f$.
 * This function is provided for debug purposes. */
template <typename MatrixType>
MatrixType FractionFreeLU<MatrixType>::reconstructedMatrix() const {
  eigen_assert(m_isInitialized && "LU is not initialized.");
  MatrixType X = MatrixType::Zero(m_lu.rows(), m_lu.cols());

  const auto diag = getDiagonal();
  Scalar x = 0;
  Scalar multiplier = 1;
  Scalar L_ik, U_kj;

  MatrixType LU = matrixLU();
  for (Eigen::Index i = 0; i < m_lu.rows(); ++i) {
    for (Eigen::Index j = 0; j < m_lu.cols(); ++j) {
      Scalar denominator = 1;
      Eigen::Index min_ij = std::min(i, j);
      for (Eigen::Index k = 0; k <= min_ij; ++k) {
        // For `L.row(i) * U.col(j)` taking into account the fact that the
        // lower right diagonal element of L is a 1, not the value given in
        // the matrix m_lu.
        if (k == i && k == m_lu.rows() - 1) {
          L_ik = 1;
        } else {
          L_ik = LU(i, k);
        }

        // Despite the fact that U may hold non zero values on rows
        // below the rank of the matrix those were placeholders
        // introduced during the decomposition phase and shouldn't
        // be considered during reconstruction.
        if (k >= m_rank) {
          U_kj = 0;
        } else {
          U_kj = LU(k, j);
        }

        x = L_ik * U_kj;

        // Here we accumulate the numerator and denominator, but only apply
        // the denominator at the end since we may otherwise end up with
        // non exact integer division.
        if (diag[k] % denominator == 0) {
          multiplier = (diag[k] / denominator);
          X(i, j) = X(i, j) * multiplier + x;
          denominator = diag[k];
        } else if (denominator % diag[k] == 0) {
          multiplier = (denominator / diag[k]);
          X(i, j) = X(i, j) + x * multiplier;
        } else {
          X(i, j) = diag[k] * X(i, j) + denominator * x;
          denominator *= diag[k];
        }
      }
      // Make sure we aren't losing information.
      assert(X(i, j) % denominator == 0);
      X(i, j) /= denominator;
    }
  }

  MatrixType orig = m_p.inverse() * X * m_q.inverse();
  assert(m_original_matrix == orig);
  return orig;
}

/********* Implementation of kernel() ***************************/

template <typename MatrixType>
MatrixType FractionFreeLU<MatrixType>::kernel() const {
  {
    if (m_rank == 0) {
      return m_q * MatrixType::Identity(m_lu.cols(), m_lu.cols());
    }
    Eigen::Index i, j;
    auto R = reducedRowEchelonForm();
    Eigen::Index n = R.cols();
    const Eigen::Index nullity = n - m_rank;
    typename MatrixType::Scalar den = 0;

    MatrixType K(R);
    K.resize(n, nullity);
    K.fill(0);
    den = R(0, 0);

    for (i = 0; i < nullity; ++i) {
      for (j = 0; j < m_rank; ++j) {
        K(j, i) = R(j, m_rank + i);
      }
      K(m_rank + i, i) = -den;
    }

    for (Index idx = 0; idx < nullity; ++idx) {
      Scalar gcd_ = gcd(K.col(idx));
      K.col(idx) /= gcd_;
    }

    auto kernel = m_q * K;
    assert((m_original_matrix * kernel).cwiseAbs2().sum() == 0);
    for (Index col = 0; col < nullity; ++col) {
      assert(kernel.col(col).cwiseAbs2().sum() != 0);
    }
    return kernel;
  };
}
/******* MatrixBase methods *************************************/

/** \lu_module
 *
 * \return the full-pivoting LU decomposition of \c *this.
 *
 * \sa class FractionFreeLU
 */
template <typename Derived>
inline const FractionFreeLU<typename DenseBase<Derived>::PlainObject>
MatrixBase<Derived>::fractionFreeLU() const {
  return FractionFreeLU<typename MatrixBase<Derived>::PlainObject>(eval());
}

}  // end namespace Eigen

#endif  // FRACTION_FREE_LU_H
