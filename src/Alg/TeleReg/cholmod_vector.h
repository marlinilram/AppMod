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


#ifndef CGAL_CHOLMOD_VECTOR
#define CGAL_CHOLMOD_VECTOR

#include "../../extern/cholmod/include/cholmod.h"
#include <cassert>
#include <algorithm>
#include <vector>


//CGAL_BEGIN_NAMESPACE

/// The class Cholmod_vector
/// is a C++ wrapper around CHOLMOD' vector type, which is a nrow*1 dense matrix.
///
/// Concept: Model of the SparseLinearAlgebraTraits_d::Vector concept.

template<class T>       // Tested with T = double or float
class Cholmod_vector
{
public:
	// Public types
    typedef T NT;
private:
	// Utility class to convert matrix's T type to the corresponding HOLMOD constant
	template<class T> struct Cholmod_dtype {};
	template<> struct Cholmod_dtype<double> {
		enum { DTYPE = CHOLMOD_DOUBLE };
	};
	template<> struct Cholmod_dtype<float>  {
		enum { DTYPE = CHOLMOD_SINGLE };
	};

// Public operations
public:

    /// Create a vector initialized with zeros.
    Cholmod_vector(int dimension, cholmod_common *common)
    {
		assert(dimension > 0);
		assert(common->dtype == Cholmod_dtype<T>::DTYPE);
		m_common = common;
		m_dense = cholmod_zeros(dimension, 1, CHOLMOD_REAL, m_common);
    }

    /// Copy constructor.
    Cholmod_vector(const Cholmod_vector& toCopy, cholmod_common *common)
    {
		assert(common->dtype == Cholmod_dtype<T>::DTYPE);
		m_common = common;
        m_dense = cholmod_copy_dense(toCopy.m_dense, m_common);
    }

    /// operator =()
    Cholmod_vector& operator =(const Cholmod_vector& toCopy)
    {
		cholmod_free_dense(&m_dense, m_common);
		m_dense = cholmod_copy_dense(toCopy.m_dense, m_common);
        return *this;
    }

    ~Cholmod_vector()
    {
		cholmod_free_dense(&m_dense, m_common);
		m_dense = NULL;
    }

    /// Return the vector's number of coefficients.
    int dimension() const {
        return m_dense->nrow;
    }

	void clear_zero(void)
	{
		int d = dimension();
		cholmod_free_dense(&m_dense, m_common);
		m_dense = cholmod_zeros(d, 1, CHOLMOD_REAL, m_common);
	}

    /// Read/write access to a vector coefficient.
    ///
    /// Preconditions:
    /// 0 <= i < dimension().
    T operator[](int i) const {
        assert(i < m_dense->nrow);
        return ((T*)(m_dense->x))[i];
    }
    T& operator[](int i) {
        assert(i < m_dense->nrow);
        return ((T*)(m_dense->x))[i];
    }

    /// Get TAUCS vector wrapped by this object.
    const cholmod_dense* get_cholmod_dense() const {
        return m_dense;
    }
    cholmod_dense* get_cholmod_dense() {
        return m_dense;
    }

// Fields
private:
	mutable cholmod_dense *m_dense;
	cholmod_common *m_common;
};

//CGAL_END_NAMESPACE

#endif // CGAL_CHOLMOD_VECTOR
