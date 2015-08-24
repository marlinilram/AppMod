// Copyright Kai (Kevin) Xu
// All rights reserved.
//
// This file is created by Kai (Kevin) Xu;
// you may redistribute it under the terms of the Q Public
//  License version 1.0. See the file LICENSE.QPL distributed with CGAL.
//
// Licensees holding a valid commercial license may use this file in
// accordance with the commercial license agreement provided with the software.
//
// This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
// WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
//
//
// Author(s)     : Kai (Kevin) Xu


#ifndef CGAL_CHOLMOD_MATRIX
#define CGAL_CHOLMOD_MATRIX

#include "../../extern/cholmod/include/cholmod.h"
#include <cassert>
#include <algorithm>
#include <vector>
#define FALSE   0
#define TRUE    1
#define NULL    0

/// The class Cholmod_matrix
/// is a C++ wrapper around CHOLMOD' matrix type cholmod_sparse.
///
/// This kind of matrix can be either symmetric or not. Symmetric
/// matrices store only the lower triangle.
///
/// Concept: Model of the SparseLinearAlgebraTraits_d::Matrix concept.
///
template<class T>       // Tested with T = float or double
struct Cholmod_matrix
{
public:
	// Public types
	typedef double T;//T NT;

private:
	// Utility class to convert matrix's T type to the corresponding HOLMOD constant
	template<class T> struct Cholmod_dtype {};
	template<> struct Cholmod_dtype<double> {
		enum { DTYPE = CHOLMOD_DOUBLE };
	};
	template<> struct Cholmod_dtype<float>  {
		enum { DTYPE = CHOLMOD_DOUBLE };	// only double precision is supported
	};

	class Column
	{
	public:

		// Vector of values + vector of indices (linked)
		std::vector<T>   m_values;
		std::vector<int> m_indices;
		~Column(){
			clear_column();
		}

	public:

		// Return the number of elements in the column
		int dimension() const    { return m_values.size(); }

		void clear_column(void)
		{
			m_values.clear();
			m_indices.clear();
		}

		// column{index} <- column{index} + val
		void add_coef(int index, T val)
		{
			// Search for element in vectors
			std::vector<int>::iterator          index_it;
			typename std::vector<T>::iterator   value_it;
			for (index_it = m_indices.begin(), value_it = m_values.begin();
				index_it != m_indices.end();
				index_it++, value_it++) {
				if(*index_it == index) {
					*value_it += val;       // +=
					return;
				}
			}
			// Element doesn't exist yet if we reach this point
			m_indices.push_back(index);
			m_values.push_back(val);
		}

		// column{index} <- val
		void set_coef(int index, T val)
		{
			// Search for element in vectors
			std::vector<int>::iterator          index_it;
			typename std::vector<T>::iterator   value_it;
			for (index_it = m_indices.begin(), value_it = m_values.begin();
				index_it != m_indices.end();
				index_it++, value_it++) {
				if(*index_it == index) {
					*value_it = val;        // =
					return;
				}
			}
			// Element doesn't exist yet if we reach this point
			m_indices.push_back(index);
			m_values.push_back(val);
		}

		// return column{index} (0 by default)
		T get_coef(int index) const
		{
			// Search for element in vectors
			std::vector<int>::const_iterator        index_it;
			typename std::vector<T>::const_iterator value_it;
			for (index_it = m_indices.begin(), value_it = m_values.begin();
				index_it != m_indices.end();
				index_it++, value_it++) {
				if(*index_it == index)
					return *value_it;       // return value
			}

			// Element doesn't exist yet if we reach this point
			return 0;
		}
	}; // class Column

// Public operations
public:

    /// Create a square matrix initialized with zeros.
    Cholmod_matrix(int  dim,                  ///< Matrix dimension.
				   bool is_symmetric,
                   cholmod_common *common) ///< Symmetric/hermitian?
    {
        assert(dim > 0);
		assert(common->dtype == Cholmod_dtype<T>::DTYPE);
        m_row_dimension = dim;
        m_column_dimension = dim;
        m_sparse = NULL;
		m_stype = (is_symmetric) ? -1 : 0;
		m_columns = new Column[m_column_dimension];
		m_common = common;
    }

    /// Create a rectangular matrix initialized with zeros.
    Cholmod_matrix(int  rows,                 ///< Matrix dimensions.
                   int  columns,
				   bool is_symmetric,
				   cholmod_common *common) ///< Symmetric/hermitian?
    {
        assert(rows > 0);
        assert(columns > 0);
        if (is_symmetric) {
            assert(rows == columns);
        }
		assert(common->dtype == Cholmod_dtype<T>::DTYPE);
        m_row_dimension = rows;
        m_column_dimension = columns;
        m_sparse = NULL;
		m_stype = (is_symmetric) ? -1 : 0;
		m_columns = new Column[m_column_dimension];
		m_common = common;
    }

    /// Delete this object and the wrapped TAUCS matrix.
    ~Cholmod_matrix()
    {
		// Delete the columns array
		delete [] m_columns;
		m_columns = NULL;
        // Delete the the wrapped TAUCS matrix
        if (m_sparse != NULL) {
			cholmod_free_sparse(&m_sparse, m_common);
            m_sparse = NULL;
        }
    }
	void clear_sparse(void)
	{
		for (int col=0; col<m_column_dimension; col++) {
			m_columns[col].clear_column();
		}
		if (m_sparse != NULL) {
			cholmod_free_sparse(&m_sparse, m_common);
			m_sparse = NULL;
		}
	}

    /// Return the matrix number of rows
    int row_dimension() const    { return m_row_dimension; }
    /// Return the matrix number of columns
    int column_dimension() const { return m_column_dimension; }

	/// Read access to a matrix coefficient.
	///
	/// Preconditions:
	/// - 0 <= i < row_dimension().
	/// - 0 <= j < column_dimension().
    T get_coef(int i, int j) const
    {
        // For symmetric matrices, we store only the lower triangle
        // => swap i and j if (i, j) belongs to the upper triangle
		if (m_stype!=0 && j>i) {
            std::swap(i, j);
		}
        assert(i < m_row_dimension);
        assert(j < m_column_dimension);
		return m_columns[j].get_coef(i);
    }

    /// Write access to a matrix coefficient: a_ij <- val.
    ///
    /// Optimization:
    /// For symmetric matrices, Taucs_matrix stores only the lower triangle
    /// set_coef() does nothing if (i, j) belongs to the upper triangle.
    ///
    /// Preconditions:
    /// - 0 <= i < row_dimension().
    /// - 0 <= j < column_dimension().
    void set_coef(int i, int j, T val)
    {
		if (m_stype!=0 && j>i) {
            return;
		}
        assert(i < m_row_dimension);
        assert(j < m_column_dimension);
		m_columns[j].set_coef(i, val);
    }

    /// Write access to a matrix coefficient: a_ij <- a_ij + val.
    ///
    /// Optimization:
    /// For symmetric matrices, Taucs_matrix stores only the lower triangle
    /// add_coef() does nothing if (i, j) belongs to the upper triangle.
    ///
    /// Preconditions:
    /// - 0 <= i < row_dimension().
    /// - 0 <= j < column_dimension().
    void add_coef(int i, int j, T val)
    {
		if (m_stype!=0 && j>i) {
            return;
		}
        assert(i < m_row_dimension);
        assert(j < m_column_dimension);
		m_columns[j].add_coef(i, val);
    }

	const cholmod_sparse* get_cholmod_sparse() const
	{
		if (m_sparse == NULL) {
			return calc_cholmod_sparse();
		}
		return m_sparse;
	}
    /// Construct and return the CHOLMOD sparse matrix wrapped by this object.
    /// Note: the CHOLMOD sparse matrix returned by this method is valid
    ///       only until the next call to set_coef(), add_coef() or get_taucs_matrix().
    const cholmod_sparse* calc_cholmod_sparse() const
    {
        if (m_sparse != NULL) {
            cholmod_free_sparse(&m_sparse, m_common);
            m_sparse = NULL;
        }
		// Compute the number of non null elements in the matrix
		int nb_max_nz = 0;
		for (int col=0; col<m_column_dimension; col++) {
			nb_max_nz += m_columns[col].dimension();
		}
		m_sparse = cholmod_allocate_sparse(m_row_dimension, m_column_dimension, nb_max_nz,
										   FALSE, TRUE, m_stype, CHOLMOD_REAL, m_common);
		
		int	*colptr = (int*)(m_sparse->p);
		int	*rowind = (int*)(m_sparse->i);
		T	*values = (T*)(m_sparse->x);
		colptr[0] = 0;
		for (int col=0; col<m_column_dimension; col++)
		{
			// Number of non null elements of the column
			int nb_elements = m_columns[col].dimension();
			if (nb_elements == 0) {
				// Start of next column will be:
				colptr[col+1] = colptr[col];
			} else {
				// Fast copy of column indices and values
				memcpy(&rowind[colptr[col]], &m_columns[col].m_indices[0], nb_elements*sizeof(int));
				memcpy(&values[colptr[col]], &m_columns[col].m_values[0],  nb_elements*sizeof(T));
				// Start of next column will be:
				colptr[col+1] = colptr[col] + nb_elements;
			}
		}

        return m_sparse;
    }

	bool is_symmetric()
	{
		return (m_stype != 0);
	}

private:
	/// Taucs_matrix cannot be copied (yet)
	Cholmod_matrix(const Cholmod_matrix& rhs);
	Cholmod_matrix& operator=(const Cholmod_matrix& rhs);

// Fields
private:

    // Matrix dimensions
    int			m_row_dimension, m_column_dimension;

	// Symmetric/hermitian=1; else=0
	int m_stype;

	// Columns array
	Column			*m_columns;

	cholmod_common	*m_common;

    /// The actual TAUCS matrix wrapped by this object.
    // This is in fact a COPY of the columns array
    mutable cholmod_sparse* m_sparse;

}; // Taucs_matrix


/// The class Taucs_symmetric_matrix is a C++ wrapper
/// around a TAUCS *symmetric* matrix (type taucs_ccs_matrix).
///
/// Symmetric matrices store only the lower triangle.
///
/// Concept: Model of the SparseLinearAlgebraTraits_d::Matrix concept.

template<class T>       // Tested with T = taucs_single or taucs_double
                        // May also work with T = taucs_dcomplex and taucs_scomplex
struct Cholmod_symmetric_matrix
    : public Cholmod_matrix<T>
{
// Public types
public:

    typedef T NT;

// Public operations
public:

    /// Create a square SYMMETRIC matrix initialized with zeros.
    /// The max number of non 0 elements in the matrix is automatically computed.
    Cholmod_symmetric_matrix(int  dim, cholmod_common *common)                  ///< Matrix dimension.
        : Cholmod_matrix<T>(dim, true, common)
    {
    }

    /// Create a square SYMMETRIC matrix initialized with zeros.
    Cholmod_symmetric_matrix(int  rows,                 ///< Matrix dimensions.
                             int  columns,
                             int  nb_max_elements = 0,
							 cholmod_common *common)  ///< Max number of non 0 elements in the
                                                      ///< matrix (automatically computed if 0).
        : Cholmod_matrix<T>(rows, columns, true, common)
    {
    }
};


#endif // CGAL_CHOLMOD_MATRIX
