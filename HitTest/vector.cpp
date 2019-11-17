/*
 * File:   vector.cpp
 * Author: Benjamin Russig
 *
 * Created on 05.09.2013 19:55:07
 */


//////
//
// Includes
//

// Implemented header
#include "vector.h"



//////
//
// Default namespaces
//

// Implemented namespaces
using namespace htest;



//////
//
// Static initializations
//

//
// SVector3
//
template <class flt_type>
const SVector3<flt_type> SVector3<flt_type>::null_vec = {{{0.0f, 0.0f, 0.0f}}};
template <class flt_type>
const SVector3<flt_type> SVector3<flt_type>::x_unit_vec = {{{1.0f, 0.0f, 0.0f}}};
template <class flt_type>
const SVector3<flt_type> SVector3<flt_type>::y_unit_vec = {{{0.0f, 1.0f, 0.0f}}};
template <class flt_type>
const SVector3<flt_type> SVector3<flt_type>::z_unit_vec = {{{0.0f, 0.0f, 1.0f}}};


//
// SVector4
//
template <class flt_type>
const SVector4<flt_type> SVector4<flt_type>::null_vec   = {{{0.0f, 0.0f, 0.0f, 0.0f}}};
template <class flt_type>
const SVector4<flt_type> SVector4<flt_type>::null_vec_h = {{{0.0f, 0.0f, 0.0f, 1.0f}}};
template <class flt_type>
const SVector4<flt_type> SVector4<flt_type>::x_unit_vec = {{{1.0f, 0.0f, 0.0f, 0.0f}}};
template <class flt_type>
const SVector4<flt_type> SVector4<flt_type>::y_unit_vec = {{{0.0f, 1.0f, 0.0f, 0.0f}}};
template <class flt_type>
const SVector4<flt_type> SVector4<flt_type>::z_unit_vec = {{{0.0f, 0.0f, 1.0f, 0.0f}}};
template <class flt_type>
const SVector4<flt_type> SVector4<flt_type>::w_unit_vec = {{{0.0f, 0.0f, 0.0f, 1.0f}}};
template <class flt_type>
const SVector4<flt_type> SVector4<flt_type>::x_unit_vec_h = {{{1.0f, 0.0f, 0.0f, 1.0f}}};
template <class flt_type>
const SVector4<flt_type> SVector4<flt_type>::y_unit_vec_h = {{{0.0f, 1.0f, 0.0f, 1.0f}}};
template <class flt_type>
const SVector4<flt_type> SVector4<flt_type>::z_unit_vec_h = {{{0.0f, 0.0f, 1.0f, 1.0f}}};



//////
//
// Explicit template instantiations
//

// Only floating point versions are intended
template struct HTEST_API SVector3<float>;
template struct HTEST_API SVector3<double>;
template struct HTEST_API SVector4<float>;
template struct HTEST_API SVector4<double>;
