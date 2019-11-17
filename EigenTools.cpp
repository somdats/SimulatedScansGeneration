
//////
//
// Includes
//

// Eigen library
#include <Eigen/Geometry>

// Implemented header
#include "EigenTools.h"



//////
//
// Class implementation
//

// EigenHelper
//

template <class flt_type>
typename EigenTypes<flt_type>::Mat4 EigenHelper<flt_type>::rotate44(
	const typename EigenTypes<flt_type>::Vec3 &axis, flt_type angle
)
{
	// Prepare transformation data as per Eigen API specs
	Eigen::Transform<flt_type, 3, Eigen::Affine> rot;
	rot = Eigen::AngleAxis<flt_type>(angle, axis);

	// Return as 4x4 homogenous matrix
	return std::move(EigenTypes<flt_type>::Mat4(rot.data()));
}

template <class flt_type>
typename EigenTypes<flt_type>::Mat4 EigenHelper<flt_type>::translate44(
	const typename EigenTypes<flt_type>::Vec3 &vector
)
{
	// Prepare transformation data as per Eigen API specs
	Eigen::Transform<flt_type, 3, Eigen::Affine> transl;
	transl = Eigen::Translation<flt_type, 3>(vector);

	// Return as 4x4 homogenous matrix
	return std::move(EigenTypes<flt_type>::Mat4(transl.data()));
}

template <class flt_type>
typename EigenTypes<flt_type>::Mat4 EigenHelper<flt_type>::translate44(
	flt_type x, flt_type y, flt_type z
)
{
	// Prepare transformation data as per Eigen API specs
	Eigen::Transform<flt_type, 3, Eigen::Affine> transl;
	transl = Eigen::Translation<flt_type, 3>(EigenTypes<flt_type>::Vec3(x, y, z));

	// Return as 4x4 homogenous matrix
	return std::move(EigenTypes<flt_type>::Mat4(transl.data()));
}



//////
//
// Explicit template instantiations
//

// Only floating point and some integer variants are intended
template APISPEC EigenHelper<float>;
template APISPEC EigenHelper<double>;
