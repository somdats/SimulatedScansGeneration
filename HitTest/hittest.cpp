/*
 * File:   hitteset.cpp
 * Author: Benjamin Russig
 *
 * Created on 29.10.2014 20:13:55
 */


//////
//
// Includes
//

// C++ STL
#include <cmath>
#include <vector>
#include <algorithm>
#include <utility>

// Local includes
#include "vector.h"
#include "util.h"

// Implemented header
#include "hittest.h"



//////
//
// Default namespaces
//

// Implemented namespaces
using namespace htest;
using namespace htest::util;



//////
//
// Macros
//

// Vector subscripts
#define X 0
#define Y 1
#define Z 2
#define W 3

// Morton / Z-order non-min/max corner indices
#define CORNER_x1y0z0 0
#define CORNER_x0y1z0 1
#define CORNER_x1y1z0 2
#define CORNER_x0y0z1 3
#define CORNER_x1y0z1 4
#define CORNER_x0y1z1 5

// Box plane indices
#define PLANE_FRONT  0
#define PLANE_BACK   1
#define PLANE_LEFT   2
#define PLANE_RIGHT  3
#define PLANE_BOTTOM 4
#define PLANE_TOP    5

// Gets minimum and maximum of three values
#define FIND_MINMAX(val0, val1, val2) \
	min = (val0 < val1) ? ((val0 < val2) ? val0 : val2) : \
	                      ((val1 < val2) ? val1 : val2); \
	max = (val0 > val1) ? ((val0 > val2) ? val0 : val2) : \
	                      ((val1 > val2) ? val1 : val2)

// Multiplies a 3-element floating point array by a scalar
#define SCALAR_MULT(vec, scalar) ((*(SVector3<flt_type>*)vec) * (scalar))
	
// Subtracts two 3-element floating point arrays
#define SUB_FF(vec0, vec1) \
	((*(SVector3<flt_type>*)vec0) - (*(SVector3<flt_type>*)vec1))
	
// Adds a vector and a 3-element floating point array
#define ADD_VF(vec0, vec1) ((vec0) + (*(SVector3<flt_type>*)vec1))

// Computes the dot product of a vector and a 3-element floating point array
#define DOT_VF(vec0, vec1) ((vec0) * (*(SVector3<flt_type>*)vec1))
// Computes the dot product of two 3-element floating point arrays
#define DOT_FF(vec0, vec1) ((*(SVector3<flt_type>*)vec0) * (*(SVector3<flt_type>*)vec1))

// Computes the cross product of a 3-element floating point array and a vector
#define CROSS_FV(vec0, vec1) ((SVector3<flt_type>*)vec0)->cross(vec1)
// Computes the cross product of two 3-element floating point arrays
#define CROSS_FF(vec0, vec1) \
	((SVector3<flt_type>*)vec0)->cross(*(SVector3<flt_type>*)vec1)

// Checks if a vector is equal to a floating point array
#define EQUAL_VF(vec0, vec1) ((vec0) == (*(SVector3<flt_type>*)vec1))
// Checks if a vector is not equal to a floating point array
#define NEQUAL_VF(vec0, vec1) ((vec0) != (*(SVector3<flt_type>*)vec1))
// Checks if a vector is not equal to either of the three floating point arrays
#define IS_NEITHER_VF(vec, vec0, vec1, vec2) \
	(NEQUAL_VF(vec, vec0) && NEQUAL_VF(vec, vec1) && NEQUAL_VF(vec, vec2))



//////
//
// Private functions
//

// Sorts the vertices of a convex polygon around an axis using selection sort, also
// eliminates freak duplicates
template <class flt_type>
unsigned int sortPolygon (SVector3<flt_type> *points, unsigned int num,
                          const SVector3<flt_type> &axis)
{
	flt_type atans [8];
	SVector3<flt_type> cntr = SVector3<flt_type>::null_vec;

	// Determine center from which to spawn axis
	for (unsigned int i=0; i<num; i++)
		cntr += points[i];
	cntr /= flt_type(num);

	// Cache absolute axis components
	flt_type axismag_x = std::abs(axis.x),
	         axismag_y = std::abs(axis.y),
	         axismag_z = std::abs(axis.z);

	// Select appropriate projection plane according to axis
	if (axismag_x > axismag_y && axismag_x > axismag_z)
	{
		// Precalculate atans
		for (unsigned int i=0; i<num; i++)
			atans[i] = std::atan2(points[i].y-cntr.y, points[i].z-cntr.z) + flt_type(7);

		// Sort according to axis direction
		if (axis.x > 0)
		{
			for (unsigned int i=0; i<num; i++)
			{
				for (unsigned int j=i+1; j<num; j++)
				{
					if (atans[j] < atans[i])
					{
						std::swap(points[i], points[j]);
						std::swap(atans[i],  atans[j]);
						continue;
					}
					else if (atans[j] == atans[i])
					{
						// Special handling for duplicates
						std::swap(points[j], points[--num]);
						std::swap(atans[j],  atans[num]);
						if (atans[j] < atans[i])
						{
							std::swap(points[i], points[j]);
							std::swap(atans[i],  atans[j]);
						}
					}
				}
			}
			return num;
		}
		for (unsigned int i=0; i<num; i++)
		{
			for (unsigned int j=i+1; j<num; j++)
			{
				if (atans[j] > atans[i])
				{
					std::swap(points[i], points[j]);
					std::swap(atans[i],  atans[j]);
					continue;
				}
				else if (atans[j] == atans[i])
				{
					// Special handling for duplicates
					std::swap(points[j], points[--num]);
					std::swap(atans[j],  atans[num]);
					if (atans[j] < atans[i])
					{
						std::swap(points[i], points[j]);
						std::swap(atans[i],  atans[j]);
					}
				}
			}
		}
		return num;
	}
	else if (axismag_y > axismag_x && axismag_y > axismag_z)
	{
		// Precalculate atans
		for (unsigned int i=0; i<num; i++)
			atans[i] = std::atan2(points[i].x-cntr.x, points[i].z-cntr.z) + flt_type(7);

		// Sort according to axis direction
		if (axis.y > 0)
		{
			for (unsigned int i=0; i<num; i++)
			{
				for (unsigned int j=i+1; j<num; j++)
				{
					if (atans[j] < atans[i])
					{
						std::swap(points[i], points[j]);
						std::swap(atans[i],  atans[j]);
						continue;
					}
					else if (atans[j] == atans[i])
					{
						// Special handling for duplicates
						std::swap(points[j], points[--num]);
						std::swap(atans[j],  atans[num]);
						if (atans[j] < atans[i])
						{
							std::swap(points[i], points[j]);
							std::swap(atans[i],  atans[j]);
						}
					}
				}
			}
			return num;
		}
		for (unsigned int i=0; i<num; i++)
		{
			for (unsigned int j=i+1; j<num; j++)
			{
				if (atans[j] > atans[i])
				{
					std::swap(points[i], points[j]);
					std::swap(atans[i],  atans[j]);
					continue;
				}
				else if (atans[j] == atans[i])
				{
					// Special handling for duplicates
					std::swap(points[j], points[--num]);
					std::swap(atans[j],  atans[num]);
					if (atans[j] < atans[i])
					{
						std::swap(points[i], points[j]);
						std::swap(atans[i],  atans[j]);
					}
				}
			}
		}
		return num;
	}
	else
	{
		// Precalculate atans
		for (unsigned int i=0; i<num; i++)
			atans[i] = std::atan2(points[i].x-cntr.x, points[i].y-cntr.y) + flt_type(7);

		// Sort according to axis direction
		if (axis.z > 0)
		{
			for (unsigned int i=0; i<num; i++)
			{
				for (unsigned int j=i+1; j<num; j++)
				{
					if (atans[j] < atans[i])
					{
						std::swap(points[i], points[j]);
						std::swap(atans[i],  atans[j]);
						continue;
					}
					else if (atans[j] == atans[i])
					{
						// Special handling for duplicates
						std::swap(points[j], points[--num]);
						std::swap(atans[j],  atans[num]);
						if (atans[j] < atans[i])
						{
							std::swap(points[i], points[j]);
							std::swap(atans[i],  atans[j]);
						}
					}
				}
			}
			return num;
		}
		for (unsigned int i=0; i<num; i++)
		{
			for (unsigned int j=i+1; j<num; j++)
			{
				if (atans[j] > atans[i])
				{
					std::swap(points[i], points[j]);
					std::swap(atans[i],  atans[j]);
					continue;
				}
				else if (atans[j] == atans[i])
				{
					// Special handling for duplicates
					std::swap(points[j], points[--num]);
					std::swap(atans[j],  atans[num]);
					if (atans[j] < atans[i])
					{
						std::swap(points[i], points[j]);
						std::swap(atans[i],  atans[j]);
					}
				}
			}
		}
	}
	return num;
}



//////
//
// Class implementation
//

//
// _
//
template <class flt_type>
bool _<flt_type>::pointInTriangle (const flt_type *point, const flt_type *p0,
                                   const flt_type *p1, const flt_type *p2)
{
	flt_type u, v, helper, dot00, dot01, dot02, dot11, dot12;
	SVector3<flt_type> edge0, edge1, vp;

	// Get plane base vectors from edges originating in p0 and the point's position
	// vector in the plane
	edge0 = SUB_FF(p1, p0);
	edge1 = SUB_FF(p2, p0);
	vp    = SUB_FF(point,  p0);

	// Compute dot products
	dot00 = edge0 * edge0;
	dot01 = edge0 * edge1;
	dot02 = edge0 * vp;
	dot11 = edge1 * edge1;
	dot12 = edge1 * vp;

	// Compute barycentric coordinates
	helper = flt_type(1) / (dot00 * dot11 - dot01 * dot01);
	u = (dot11 * dot02 - dot01 * dot12) * helper;
	v = (dot00 * dot12 - dot01 * dot02) * helper;

	// Check if point is in triangle
	flt_type UpV = u+v;
	return ( u  > flt_type(0) || fptools<flt_type>::equal( u,  flt_type(0))) &&
	       ( v  > flt_type(0) || fptools<flt_type>::equal( v,  flt_type(0))) &&
	       (UpV < flt_type(1) || fptools<flt_type>::equal(UpV, flt_type(1)));
}

template <class flt_type>
bool _<flt_type>::pointInTriangle (
	flt_type *u, flt_type *v, const flt_type *point,
	const flt_type *p0, const flt_type *p1, const flt_type *p2
)
{
	flt_type helper, dot00, dot01, dot02, dot11, dot12;
	SVector3<flt_type> edge0, edge1, vp;

	// Get plane base vectors from edges originating in p0 and the point's position
	// vector in the plane
	edge0 = SUB_FF(p1, p0);
	edge1 = SUB_FF(p2, p0);
	vp    = SUB_FF(point,  p0);

	// Compute dot products
	dot00 = edge0 * edge0;
	dot01 = edge0 * edge1;
	dot02 = edge0 * vp;
	dot11 = edge1 * edge1;
	dot12 = edge1 * vp;

	// Compute barycentric coordinates
	helper = flt_type(1) / (dot00 * dot11 - dot01 * dot01);
	*u = (dot11 * dot02 - dot01 * dot12) * helper;
	*v = (dot00 * dot12 - dot01 * dot02) * helper;

	// Check if point is in triangle
	return *u >= flt_type(0) && *v >= flt_type(0) && (*u)+(*v) <= flt_type(1);
}

template <class flt_type>
bool _<flt_type>::rayPlaneIntersect (flt_type *out, const flt_type *origin,
                                     const flt_type *dir, const flt_type *plane, flt_type *t_out)
{
	flt_type dot;

	// Compute dot product of normal and direction in order to rule out parallel case
	dot = DOT_FF(dir, plane);
	if (dot == flt_type(0))
		return false;

	// Calculate and store intersection point
	flt_type t = (-(plane[W] + DOT_FF(plane, origin)) / dot);
	*(SVector3<flt_type>*)out = ADD_VF(SCALAR_MULT(dir, t), origin);
	if (t_out)
		*t_out = t;

	return true;
}

template <class flt_type>
bool _<flt_type>::linePlaneIntersect (flt_type *out, const flt_type *l0,
                                      const flt_type *l1, const flt_type *plane)
{
	flt_type dot, t;
	SVector3<flt_type> dir;

	// Compute dot product of normal and direction in order to rule out parallel case
	dir = SUB_FF(l1, l0);
	dot = DOT_VF(dir, plane);
	if (dot == flt_type(0))
		return false;

	// Calculate t and reject early if value is beyond one of the line end points
	t = -(plane[W]+DOT_FF(plane, l0))/dot;
	if (t < flt_type(0) || t > flt_type(1))
		return false;

	// Calculate and store intersection point
	*(SVector3<flt_type>*)out = ADD_VF(t*dir, l0);

	return true;
}

template <class flt_type>
bool _<flt_type>::linePlaneIntersect_exc (flt_type *out, const flt_type *l0,
                                          const flt_type *l1, const flt_type *plane)
{
	flt_type dot, t;
	SVector3<flt_type> dir;

	// Compute dot product of normal and direction in order to rule out parallel case
	dir = SUB_FF(l1, l0);
	dot = DOT_VF(dir, plane);
	if (dot == flt_type(0))
		return false;

	// Calculate t and reject early if value is on or beyond one of the line end points
	t = -(plane[W]+DOT_FF(plane, l0))/dot;
	if (t <= flt_type(0) || t >= flt_type(1))
		return false;

	// Calculate and store intersection point
	*(SVector3<flt_type>*)out = ADD_VF(t*dir, l0);

	return true;
}

template <class flt_type>
bool _<flt_type>::rayTriIntersect (
	flt_type *u, flt_type *v, const flt_type *origin, const flt_type *dir,
	const flt_type *p0, const flt_type *p1, const flt_type *p2
)
{
	flt_type det, _u, _v;
	SVector3<flt_type> edge0, edge1, tvec, pvec, qvec;

	// Get plane base vectors from edges originating in p0
	edge0 = SUB_FF(p1, p0);
	edge1 = SUB_FF(p2, p0);

	// Calculate determinant - also used for calculating u
	pvec = CROSS_FV(dir, edge1);
	det = edge0 * pvec;

	// If determinant is zero, ray lies in plane of triangle
	if (det == flt_type(0))
		return false;

	// Calculate distance from p0 to ray origin
	tvec = SUB_FF(origin, p0);

	// Calculate u and reject early if value is outside barycentric triangle bounds
	_u = (tvec * pvec) / det;
	if (_u < flt_type(0) || _u > flt_type(1))
		return false;

	// Prepare to test v and implicit t
	qvec = tvec.cross(edge0);

	// Test implicit t and reject early if origin lies on the triangle plane
	if ((edge1 * qvec) / det == flt_type(0))
		return false;

	// Calculate v and reject if [u,v] is outside barycentric triangle bounds
	_v = DOT_VF(qvec, dir) / det;
	if (_v < flt_type(0) || _u+_v > flt_type(1))
		return false;

	// Store results
	*u = _u;
	*v = _v;

	return true;
}

template <class flt_type>
bool _<flt_type>::rayTriIntersect (
	flt_type *out, const flt_type *origin, const flt_type *dir, const flt_type *p0,
	const flt_type *p1, const flt_type *p2, flt_type *t_out
)
{
	flt_type det, u, v;
	SVector3<flt_type> edge0, edge1, tvec, pvec, qvec;

	// Get plane base vectors from edges originating in p0
	edge0 = SUB_FF(p1, p0);
	edge1 = SUB_FF(p2, p0);

	// Calculate determinant - also used for calculating u
	pvec = CROSS_FV(dir, edge1);
	det = edge0 * pvec;

	// If determinant is zero, ray lies in plane of triangle
	if (det == flt_type(0))
		return false;

	// Calculate distance from p0 to ray origin
	tvec = SUB_FF(origin, p0);

	// Calculate u and reject early if value is outside barycentric triangle bounds
	u = (tvec * pvec) / det;
	if (u < flt_type(0) || u > flt_type(1))
		return false;

	// Prepare to test v and implicit t
	qvec = tvec.cross(edge0);

	// Test t and reject if origin lies on the triangle plane
	flt_type t = (edge1 * qvec) / det;
	if (t == flt_type(0))
		return false;

	// Calculate v and reject if [u,v] is outside barycentric triangle bounds
	v = DOT_VF(qvec, dir) / det;
	if (v < flt_type(0) || u+v > flt_type(1))
		return false;

	// Store results
	out[X] = u*p1[X] + v*p2[X] + (flt_type(1)-u-v)*p0[X];
	out[Y] = u*p1[Y] + v*p2[Y] + (flt_type(1)-u-v)*p0[Y];
	out[Z] = u*p1[Z] + v*p2[Z] + (flt_type(1)-u-v)*p0[Z];
	if (t_out)
		*t_out = t;

	return true;
}

template <class flt_type>
bool _<flt_type>::lineTriIntersect (
	flt_type *u, flt_type *v, const flt_type *l0, const flt_type *l1,
	const flt_type *p0, const flt_type *p1, const flt_type *p2
)
{
	flt_type _u, _v, t, det;
	SVector3<flt_type> edge0, edge1, tvec, pvec, qvec, dir;

	// Get plane base vectors from edges originating in p0
	edge0 = SUB_FF(p1, p0);
	edge1 = SUB_FF(p2, p0);

	// Calculate determinant - also used for calculating u
	dir = SUB_FF(l1, l0);
	pvec = dir.cross(edge1);
	det = edge0 * pvec;

	// If determinant is zero, ray lies in plane of triangle
	if (det == flt_type(0))
		return false;

	// Calculate distance from p0 to first line end point
	tvec = SUB_FF(l0, p0);

	// Calculate u and reject early if value is outside barycentric triangle bounds
	_u = (tvec * pvec) / det;
	if (_u < flt_type(0) || _u > flt_type(1))
		return false;

	// Prepare to test t and v
	qvec = tvec.cross(edge0);

	// Calculate t and reject early if value is beyond one of the line end points
	t = (edge1 * qvec) / det;
	if (t < flt_type(0) || t > flt_type(1))
		return false;

	// Calculate v and reject if [u,v] is outside barycentric triangle bounds
	_v = (dir * qvec) / det;
	if (_v < flt_type(0) || _u+_v > flt_type(1))
		return false;

	// Store results
	*u = _u;
	*v = _v;

	return true;
}

template <class flt_type>
bool _<flt_type>::lineTriIntersect (
	flt_type *out, const flt_type *l0, const flt_type *l1,
	const flt_type *p0, const flt_type *p1, const flt_type *p2
)
{
	flt_type u, v, t, det;
	SVector3<flt_type> edge0, edge1, tvec, pvec, qvec, dir;

	// Get plane base vectors from edges originating in p0
	edge0 = SUB_FF(p1, p0);
	edge1 = SUB_FF(p2, p0);

	// Calculate determinant - also used for calculating u
	dir = SUB_FF(l1, l0);
	pvec = dir.cross(edge1);
	det = edge0 * pvec;

	// If determinant is zero, ray lies in plane of triangle
	if (det == flt_type(0))
		return false;

	// Calculate distance from p0 to first line end point
	tvec = SUB_FF(l0, p0);

	// Calculate u and reject early if value is outside barycentric triangle bounds
	u = (tvec * pvec) / det;
	if (u < flt_type(0) || u > flt_type(1))
		return false;

	// Prepare to test t and v
	qvec = tvec.cross(edge0);

	// Calculate t and reject early if value is beyond one of the line end points
	t = (edge1 * qvec) / det;
	if (t < flt_type(0) || t > flt_type(1))
		return false;

	// Calculate v and reject if [u,v] is outside barycentric triangle bounds
	v = (dir * qvec) / det;
	if (v < flt_type(0) || u+v > flt_type(1))
		return false;

	// Store results
	out[X] = u*p1[X] + v*p2[X] + (flt_type(1)-u-v)*p0[X];
	out[Y] = u*p1[Y] + v*p2[Y] + (flt_type(1)-u-v)*p0[Y];
	out[Z] = u*p1[Z] + v*p2[Z] + (flt_type(1)-u-v)*p0[Z];

	return true;
}

template <class flt_type>
bool _<flt_type>::lineTriIntersect_exc (
	flt_type *u, flt_type *v, const flt_type *l0, const flt_type *l1,
	const flt_type *p0, const flt_type *p1, const flt_type *p2
)
{
	flt_type _u, _v, t, det;
	SVector3<flt_type> edge0, edge1, tvec, pvec, qvec, dir;

	// Get plane base vectors from edges originating in p0
	edge0 = SUB_FF(p1, p0);
	edge1 = SUB_FF(p2, p0);

	// Calculate determinant - also used for calculating u
	dir = SUB_FF(l1, l0);
	pvec = dir.cross(edge1);
	det = edge0 * pvec;

	// If determinant is zero, ray lies in plane of triangle
	if (det == flt_type(0))
		return false;

	// Calculate distance from p0 to first line end point
	tvec = SUB_FF(l0, p0);

	// Calculate u and reject early if value is outside barycentric triangle bounds
	_u = (tvec * pvec) / det;
	if (_u < flt_type(0) || _u > flt_type(1))
		return false;

	// Prepare to test t and v
	qvec = tvec.cross(edge0);

	// Calculate t and reject early if value is on or beyond one of the line end points
	t = (edge1 * qvec) / det;
	if (t <= flt_type(0) || t >= flt_type(1))
		return false;

	// Calculate v and reject if [u,v] is outside barycentric triangle bounds
	_v = (dir * qvec) / det;
	if (_v < flt_type(0) || _u+_v > flt_type(1))
		return false;

	// Store results
	*u = _u;
	*v = _v;

	return true;
}

template <class flt_type>
bool _<flt_type>::lineTriIntersect_exc (
	flt_type *out, const flt_type *l0, const flt_type *l1,
	const flt_type *p0, const flt_type *p1, const flt_type *p2
)
{
	flt_type u, v, t, det;
	SVector3<flt_type> edge0, edge1, tvec, pvec, qvec, dir;

	// Get plane base vectors from edges originating in p0
	edge0 = SUB_FF(p1, p0);
	edge1 = SUB_FF(p2, p0);

	// Calculate determinant - also used for calculating u
	dir = SUB_FF(l1, l0);
	pvec = dir.cross(edge1);
	det = edge0 * pvec;

	// If determinant is zero, ray lies in plane of triangle
	if (det == flt_type(0))
		return false;

	// Calculate distance from p0 to first line end point
	tvec = SUB_FF(l0, p0);

	// Calculate u and reject early if value is outside barycentric triangle bounds
	u = (tvec * pvec) / det;
	if (u < flt_type(0) || u > flt_type(1))
		return false;

	// Prepare to test t and v
	qvec = tvec.cross(edge0);

	// Calculate t and reject early if value is on or beyond one of the line end points
	t = (edge1 * qvec) / det;
	if (t <= flt_type(0) || t >= flt_type(1))
		return false;

	// Calculate v and reject if [u,v] is outside barycentric triangle bounds
	v = (dir * qvec) / det;
	if (v < flt_type(0) || u+v > flt_type(1))
		return false;

	// Store results
	out[X] = u*p1[X] + v*p2[X] + (flt_type(1)-u-v)*p0[X];
	out[Y] = u*p1[Y] + v*p2[Y] + (flt_type(1)-u-v)*p0[Y];
	out[Z] = u*p1[Z] + v*p2[Z] + (flt_type(1)-u-v)*p0[Z];

	return true;
}

template <class flt_type>
bool _<flt_type>::boxBoxTest (
	const SAABox<flt_type> &box0, const SAABox<flt_type> &box1
)
{
	// Test for separating yz-plane
	if (box0.pmin.x >= box1.pmax.x || box0.pmax.x <= box1.pmin.x)
		return false;

	// Test for separating xz-plane
	if (box0.pmin.y >= box1.pmax.y || box0.pmax.y <= box1.pmin.y)
		return false;

	// Test for separating xy-plane
	if (box0.pmin.z >= box1.pmax.z || box0.pmax.z <= box1.pmin.z)
		return false;


	// No separating plane exists!
	return true;
}

template <class flt_type>
int _<flt_type>::boxPlaneTest (const SAABox<flt_type> &box, const flt_type *plane)
{
	struct _
	{
		// Why isn't this in the STL?
		inline static int sign (flt_type f)
		{
			return (f > flt_type(0)) ? 1 : (f < flt_type(0) ? -1 : 0);
		}
	};
	int p[8];

	// Non-min/max points of the box, in Morton / Z-order
	SVector3<flt_type> corners[6] = {
		{box.pmax.x, box.pmin.y, box.pmin.z},
		{box.pmin.x, box.pmax.y, box.pmin.z},
		{box.pmax.x, box.pmax.y, box.pmin.z},
		{box.pmin.x, box.pmin.y, box.pmax.z},
		{box.pmax.x, box.pmin.y, box.pmax.z},
		{box.pmin.x, box.pmax.y, box.pmax.z}
	};

	p[0] = _::sign(DOT_VF(box.pmin, plane) + plane[W]);
	p[1] = _::sign(DOT_VF(corners[0], plane) + plane[W]);
	p[2] = _::sign(DOT_VF(corners[1], plane) + plane[W]);
	p[3] = _::sign(DOT_VF(corners[2], plane) + plane[W]);
	p[4] = _::sign(DOT_VF(corners[3], plane) + plane[W]);
	p[5] = _::sign(DOT_VF(corners[4], plane) + plane[W]);
	p[6] = _::sign(DOT_VF(corners[5], plane) + plane[W]);
	p[7] = _::sign(DOT_VF(box.pmax, plane) + plane[W]);

	// Determine and return spatial relationship
	return (p[0] < flt_type(0) && p[1] < flt_type(0) && p[2] < flt_type(0) &&
	        p[3] < flt_type(0) && p[4] < flt_type(0) && p[5] < flt_type(0) &&
	        p[6] < flt_type(0) && p[7] < flt_type(0)) ?
	           -1 :
	           ((p[0] > flt_type(0) && p[1] > flt_type(0) && p[2] > flt_type(0) &&
	             p[3] > flt_type(0) && p[4] > flt_type(0) && p[5] > flt_type(0) &&
	             p[6] > flt_type(0) && p[7] > flt_type(0)) ? 1 : 0);
}

template <class flt_type>
bool _<flt_type>::boxTriTest (
	const SAABox<flt_type> &box,
	const flt_type *p0, const flt_type *p1, const flt_type *p2, const flt_type *normal
)
{
	flt_type min, max;


	// Test for separating yz-plane
	FIND_MINMAX(p0[X], p1[X], p2[X]);
	if (min > box.pmax.x || max < box.pmin.x ||
	    (   min == box.pmax.x && normal[Y] == flt_type(0) && normal[Z] == flt_type(0)
	     && normal[X] > flt_type(0)) ||
	    (   max == box.pmin.x && normal[Y] == flt_type(0) && normal[Z] == flt_type(0)
	     && normal[X] < flt_type(0)))
		return false;

	// Test for separating xz-plane
	FIND_MINMAX(p0[Y], p1[Y], p2[Y]);
	if (min > box.pmax.y || max < box.pmin.y ||
	    (   min == box.pmax.y && normal[X] == flt_type(0) && normal[Z] == flt_type(0)
	     && normal[Y] > flt_type(0)) ||
	    (   max == box.pmin.y && normal[X] == flt_type(0) && normal[Z] == flt_type(0)
	     && normal[Y] < flt_type(0)))
		return false;

	// Test for separating xy-plane
	FIND_MINMAX(p0[Z], p1[Z], p2[Z]);
	if (min > box.pmax.z || max < box.pmin.z ||
	    (   min == box.pmax.z && normal[X] == flt_type(0) && normal[Y] == flt_type(0)
	     && normal[Z] > flt_type(0)) ||
	    (   max == box.pmin.z && normal[X] == flt_type(0) && normal[Y] == flt_type(0)
	     && normal[Z] < flt_type(0)))
		return false;


	// Test for separating plane along triangle normal. Zero-volume overlap is treated as
	// a hit only if the normal faces the box.
	// (edge being used here as a temporary variable; real use is in following tests)
	flt_type v[8]; SVector3<flt_type> edge;
	v[0] = DOT_VF(box.pmin, normal);
	v[1] = DOT_VF(edge.set_xyz(box.pmax.x, box.pmin.y, box.pmin.z), normal);
	v[2] = DOT_VF(edge.set_xyz(box.pmin.x, box.pmax.y, box.pmin.z), normal);
	v[3] = DOT_VF(edge.set_xyz(box.pmax.x, box.pmax.y, box.pmin.z), normal);
	v[4] = DOT_VF(edge.set_xyz(box.pmin.x, box.pmin.y, box.pmax.z), normal);
	v[5] = DOT_VF(edge.set_xyz(box.pmax.x, box.pmin.y, box.pmax.z), normal);
	v[6] = DOT_VF(edge.set_xyz(box.pmin.x, box.pmax.y, box.pmax.z), normal);
	v[7] = DOT_VF(box.pmax, normal);
	max = min = v[0];
	for (unsigned int i=1; i<8; i++)
	{
		min = min < v[i] ? min : v[i];
		max = max > v[i] ? max : v[i];
	}
	v[0] = DOT_FF(p0, normal);
	if (min > v[0] || max <= v[0])
		return false;


	//
	// The following tests against the 9 cross product axes can be radically simplified
	// when examining the involved equations more closely:
	//
	// (1)  edge[j]  = p[(j+1) mod 3] - p[j]     | j=[0..2]
	//
	// (2)  axis[ij] = boxnormal[i] x edge[j]    | i,j=[0..2]
	//
	//                   boxnormal[i].y*edge[j].z - boxnormal[i].z*edge[j].y
	// (2a) axis[ij] = ( boxnormal[i].z*edge[j].x - boxnormal[i].x*edge[j].z )|i,j=[0..2]
	//                   boxnormal[i].x*edge[j].y - boxnormal[i].y*edge[j].x
	//
	// since the box is axis-aligned, the 3 boxnormals are the coordinate system's unit
	// vectors. This means the nth component of boxnormal[i] is 1 when i=n, 0 otherwise.
	// Looking at the case i,j = 0, it becomes apparent that
	//
	//                   0*edge[0].z - 0*edge[0].y          0
	// (3)  axis[00] = ( 0*edge[0].x - 1*edge[0].z ) = ( -edge[0].z )
	//                   1*edge[0].y - 0*edge[0].x        edge[0].y
	//
	// When the triangle vertices are projected onto the axis, the resulting dot products
	//
	//      v[0] = p[0].x*0 - p[0].y*edge[0].z + p[0].z*edge[0].y
	// (4)  v[1] = p[1].x*0 - p[1].y*edge[0].z + p[1].z*edge[0].y
	//      v[2] = p[2].x*0 - p[2].y*edge[0].z + p[2].z*edge[0].y
	//
	// when applying equation (1), become
	//
	//      v[0] = -p[0].y*(p[1].z-p[0].z) + p[0].z*(p[1].y-p[0].y)
	// (4a) v[1] = -p[1].y*(p[1].z-p[0].z) + p[1].z*(p[1].y-p[0].y)
	//      v[2] = -p[2].y*(p[1].z-p[0].z) + p[2].z*(p[1].y-p[0].y)
	//
	// which, using the distributive property, simplifies to the final equations
	//
	//      v[0] = p[0].z *  p[1].y   -  p[0].y *  p[1].z
	// (5)  v[1] = p[0].z *  p[1].y   -  p[0].y *  p[1].z
	//      v[2] = p[2].z * edge[0].y -  p[2].y * edge[0].z
	//
	// Equations (5) demonstrate the intuitive conclusion that the two triangle vertices
	// making up the involved triangle edge, p[0] and p[1], are projected onto the same
	// point on the axis. This means that only v[0] and v[2] need to be considered
	// when finding the min and max points to compare against the box, saving one
	// potentially expensive conditional expression.
	//
	// To further eliminate the need to project all eight box points onto the axis, the
	// halfsize of the box is being used as a "radius", which, when projected onto the
	// axis and offset from the box center, will always represent the maximal and minimal
	// projected box points. The storage location v[2] will be used to hold the projected
	// "radius".
	//       Additionally, to avoid having to project the box center around which to
	// apply the "radius" on each of the nine axes as well, the triangle vertices are
	// being transformed to the local coordinate space of the box.
	//

	SVector3<flt_type> edge_abs, pt0, pt1, pt2;

	// Transform triangle vertices into box space for the remaining tests
	pt0.y = p0[Y] - box.pmid[Y]; pt0.z = p0[Z] - box.pmid[Z];
	pt1.y = p1[Y] - box.pmid[Y]; pt1.z = p1[Z] - box.pmid[Z];
	pt2.y = p2[Y] - box.pmid[Y]; pt2.z = p2[Z] - box.pmid[Z];

	// Test for separating plane along cross(x-axis, 1st edge)
	edge_abs.y = std::abs(edge.y = pt1.y-pt0.y);
	edge_abs.z = std::abs(edge.z = pt1.z-pt0.z);
	v[0] = pt0.z * pt1.y   -  pt0.y * pt1.z;
	v[1] = pt2.z * edge.y  -  pt2.y * edge.z;
	v[2] = box.halfsize[Z]*edge_abs.y + box.halfsize[Y]*edge_abs.z;
	{ register unsigned int c = v[0] < v[1] ? 0 : 1;
	  min = v[c]; max = v[(++c)&0x1]; }
	if (min > v[2] || max < -v[2] || (min != max && (min == v[2] || max == -v[2])))
		return false;

	// Test for separating plane along cross(y-axis, 1st edge)
	pt0.x = p0[X]-box.pmid[X]; pt1.x = p1[X]-box.pmid[X]; pt2.x = p2[X]-box.pmid[X];
	edge_abs.x = std::abs(edge.x = pt1.x-pt0.x);
	v[0] = pt0.x * pt1.z   -  pt0.z * pt1.x;
	v[1] = pt2.x * edge.z  -  pt2.z * edge.x;
	v[2] = box.halfsize[X]*edge_abs.z + box.halfsize[Z]*edge_abs.x;
	{ register unsigned int c = v[0] < v[1] ? 0 : 1;
	  min = v[c]; max = v[(++c)&0x1]; }
	if (min > v[2] || max < -v[2] || (min != max && (min == v[2] || max == -v[2])))
		return false;

	// Test for separating plane along cross(z-axis, 1st edge)
	v[0] = pt0.y * pt1.x   -  pt0.x * pt1.y;
	v[1] = pt2.y * edge.x  -  pt2.x * edge.y;
	v[2] = box.halfsize[Y]*edge_abs.x + box.halfsize[X]*edge_abs.y;
	{ register unsigned int c = v[0] < v[1] ? 0 : 1;
	  min = v[c]; max = v[(++c)&0x1]; }
	if (min > v[2] || max < -v[2] || (min != max && (min == v[2] || max == -v[2])))
		return false;

	// Test for separating plane along cross(x-axis, 2nd edge)
	edge_abs.y = std::abs(edge.y = pt2.y-pt1.y);
	edge_abs.z = std::abs(edge.z = pt2.z-pt1.z);
	v[0] = pt0.z * edge.y - pt0.y * edge.z;
	v[1] = pt1.z * pt2.y  - pt1.y * pt2.z;
	v[2] = box.halfsize[Z]*edge_abs.y + box.halfsize[Y]*edge_abs.z;
	{ register unsigned int c = v[0] < v[1] ? 0 : 1;
	  min = v[c]; max = v[(++c)&0x1]; }
	if (min > v[2] || max < -v[2] || (min != max && (min == v[2] || max == -v[2])))
		return false;

	// Test for separating plane along cross(y-axis, 2nd edge)
	edge_abs.x = std::abs(edge.x = pt2.x-pt1.x);
	v[0] = pt0.x * edge.z  -  pt0.z * edge.x;
	v[1] = pt1.x * pt2.z   -  pt1.z * pt2.x;
	v[2] = box.halfsize[X]*edge_abs.z + box.halfsize[Z]*edge_abs.x;
	{ register unsigned int c = v[0] < v[1] ? 0 : 1;
	  min = v[c]; max = v[(++c)&0x1]; }
	if (min > v[2] || max < -v[2] || (min != max && (min == v[2] || max == -v[2])))
		return false;

	// Test for separating plane along cross(z-axis, 2nd edge)
	v[0] = pt0.y * edge.x  -  pt0.x * edge.y;
	v[1] = pt1.y * pt2.x   -  pt1.x * pt2.y;
	v[2] = box.halfsize[Y]*edge_abs.x + box.halfsize[X]*edge_abs.y;
	{ register unsigned int c = v[0] < v[1] ? 0 : 1;
	  min = v[c]; max = v[(++c)&0x1]; }
	if (min > v[2] || max < -v[2] || (min != max && (min == v[2] || max == -v[2])))
		return false;

	// Test for separating plane along cross(x-axis, 3rd edge)
	edge_abs.y = std::abs(edge.y = pt0.y-pt2.y);
	edge_abs.z = std::abs(edge.z = pt0.z-pt2.z);
	v[0] = pt2.z * pt0.y   -  pt2.y * pt0.z;
	v[1] = pt1.z * edge.y  -  pt1.y * edge.z;
	v[2] = box.halfsize[Z]*edge_abs.y + box.halfsize[Y]*edge_abs.z;
	{ register unsigned int c = v[0] < v[1] ? 0 : 1;
	  min = v[c]; max = v[(++c)&0x1]; }
	if (min > v[2] || max < -v[2] || (min != max && (min == v[2] || max == -v[2])))
		return false;

	// Test for separating plane along cross(y-axis, 3rd edge)
	edge_abs.x = std::abs(edge.x = pt0.x-pt2.x);
	v[0] = pt2.x * pt0.z   -  pt2.z * pt0.x;
	v[1] = pt1.x * edge.z  -  pt1.z * edge.x;
	v[2] = box.halfsize[X]*edge_abs.z + box.halfsize[Z]*edge_abs.x;
	{ register unsigned int c = v[0] < v[1] ? 0 : 1;
	  min = v[c]; max = v[(++c)&0x1]; }
	if (min > v[2] || max < -v[2] || (min != max && (min == v[2] || max == -v[2])))
		return false;

	// Test for separating plane along cross(z-axis, 3rd edge)
	v[0] = pt2.y * pt0.x   -  pt2.x * pt0.y;
	v[1] = pt1.y * edge.x  -  pt1.x * edge.y;
	v[2] = box.halfsize[Y]*edge_abs.x + box.halfsize[X]*edge_abs.y;
	{ register unsigned int c = v[0] < v[1] ? 0 : 1;
	  min = v[c]; max = v[(++c)&0x1]; }
	if (min > v[2] || max < -v[2] || (min != max && (min == v[2] || max == -v[2])))
		return false;


	// No separating plane exists!
	return true;
}

template <class flt_type>
bool _<flt_type>::boxRayIntersection (
	flt_type *out, flt_type *t_out, const SAABox<flt_type> &box, const flt_type *origin,
	const flt_type *dir
)
{
	static unsigned _debugi = -1;
	_debugi++;

	// Non-min/max points of the box, in Morton / Z-order
	/*SVector3<flt_type> corners[6] = {
		{ box.pmax.x, box.pmin.y, box.pmin.z },
		{ box.pmin.x, box.pmax.y, box.pmin.z },
		{ box.pmax.x, box.pmax.y, box.pmin.z },
		{ box.pmin.x, box.pmin.y, box.pmax.z },
		{ box.pmax.x, box.pmin.y, box.pmax.z },
		{ box.pmin.x, box.pmax.y, box.pmax.z }
	};*/

	// Box planes
	SVector4<flt_type> planes[6] = {
		{ 0.0, 0.0,-1.0, box.pmin.z }, // Front
		{ 0.0, 0.0, 1.0,-box.pmax.z }, // Back
		{-1.0, 0.0, 0.0, box.pmin.x }, // Left
		{ 1.0, 0.0, 0.0,-box.pmax.x }, // Right	
		{ 0.0,-1.0, 0.0, box.pmin.y }, // Bottom
		{ 0.0, 1.0, 0.0,-box.pmax.y }  // Top
	};


	////
	// Step 1: Check ray against all box faces

	SVector3<flt_type> isect[2], candidate; std::vector<flt_type> t(2); flt_type t_candidate;
	unsigned n = 0;

	// #0: common intersection handling
	auto handleIntersect = [&out, &t_out, &isect, &candidate, &t, &t_candidate, &n](void)
		-> void
	{
		if (n > 1)
			return;
		if (n && fptools<flt_type>::nequal(t[0], t_candidate))
		{
			isect[1] = std::move(candidate); t[n++] = t_candidate;
			register const bool cond = t[0] < t[1];
			*(SVector3<flt_type>*)out = cond ? isect[0] : isect[1];
			*t_out = cond ? t[0] : t[1];
			return;
			/*else if (n == 2 && fptools<flt_type>::nequal(t[1], t_candidate))
				throw std::runtime_error("[HitTest] !!!INTERNAL ERROR!!!");
			else
				throw std::runtime_error("[HitTest] !!!INTERNAL ERROR!!!");*/
		}
		isect[0] = std::move(candidate); t[n++] = t_candidate;
	};

	// #1: front face
	if (rayPlaneIntersect(candidate, origin, dir, planes[PLANE_FRONT], &t_candidate) &&
	    t_candidate > flt_type(0) &&
	    candidate.x >= box.pmin.x && candidate.x < box.pmax.x &&
	    candidate.y >= box.pmin.y && candidate.y < box.pmax.y)
	{
		isect[0] = std::move(candidate); t[n++] = t_candidate;
	}
	// #2: back face
	if (rayPlaneIntersect(candidate, origin, dir, planes[PLANE_BACK], &t_candidate) &&
	    t_candidate > flt_type(0) &&
	    candidate.x >  box.pmin.x && candidate.x <= box.pmax.x &&
	    candidate.y >= box.pmin.y && candidate.y <  box.pmax.y)
	{
		handleIntersect();
	}
	// #3: left face
	if (rayPlaneIntersect(candidate, origin, dir, planes[PLANE_LEFT], &t_candidate) &&
	    t_candidate > flt_type(0) &&
	    candidate.y >= box.pmin.y && candidate.y <  box.pmax.y &&
	    candidate.z >  box.pmin.z && candidate.z <= box.pmax.z)
	{
		handleIntersect();
	}
	// #4: right face
	if (rayPlaneIntersect(candidate, origin, dir, planes[PLANE_RIGHT], &t_candidate) &&
	    t_candidate > flt_type(0) &&
	    candidate.y >= box.pmin.y && candidate.y < box.pmax.y &&
	    candidate.z >= box.pmin.z && candidate.z < box.pmax.z)
	{
		handleIntersect();
	}
	// #5: bottom face
	if (rayPlaneIntersect(candidate, origin, dir, planes[PLANE_BOTTOM], &t_candidate) &&
	    t_candidate > flt_type(0) &&
	    candidate.x > box.pmin.x && candidate.x < box.pmax.x &&
	    candidate.z > box.pmin.z && candidate.z < box.pmax.z)
	{
		handleIntersect();
	}
	// #6: top face
	if (rayPlaneIntersect(candidate, origin, dir, planes[PLANE_TOP], &t_candidate) &&
	    t_candidate > flt_type(0) &&
	    candidate.x >= box.pmin.x && candidate.x <= box.pmax.x &&
	    candidate.z >= box.pmin.z && candidate.z <= box.pmax.z)
	{
		handleIntersect();
	}

	if (!n)
		return false;
	else if (n == 1)
	{
		*(SVector3<flt_type>*)out = isect[0];
		*t_out = t[0];
		return true;
	}

	register const bool cond = t[0] < t[1];
	*(SVector3<flt_type>*)out = cond ? isect[0] : isect[1];
	*t_out = cond ? t[0] : t[1];
	return true;
}

template <class flt_type>
unsigned int _<flt_type>::boxPlaneIntersect (
	flt_type *out, const SAABox<flt_type> &box, const flt_type *plane
)
{
	unsigned int n;

	// Non-min/max points of the box, in Morton / Z-order
	SVector3<flt_type> corners[6] = {
		{box.pmax.x, box.pmin.y, box.pmin.z},
		{box.pmin.x, box.pmax.y, box.pmin.z},
		{box.pmax.x, box.pmax.y, box.pmin.z},
		{box.pmin.x, box.pmin.y, box.pmax.z},
		{box.pmax.x, box.pmin.y, box.pmax.z},
		{box.pmin.x, box.pmax.y, box.pmax.z}
	};


	////
	// Step 1: Check plane against all 12 edges of the cube. The corner points are NOT
	//         treated as being part of any edge.

	n  = linePlaneIntersect_exc(
		out,       box.pmin,               corners[CORNER_x1y0z0], plane
	) ? 1 : 0;
	n += linePlaneIntersect_exc(
		out + n*3, box.pmin,               corners[CORNER_x0y0z1], plane
	) ? 1 : 0;
	n += linePlaneIntersect_exc(
		out + n*3, corners[CORNER_x0y0z1], corners[CORNER_x0y1z1], plane
	) ? 1 : 0;
	n += linePlaneIntersect_exc(
		out + n*3, corners[CORNER_x0y1z1], box.pmax,               plane
	) ? 1 : 0;
	n += linePlaneIntersect_exc(
		out + n*3, corners[CORNER_x1y1z0], box.pmax,               plane
	) ? 1 : 0;
	n += linePlaneIntersect_exc(
		out + n*3, corners[CORNER_x1y0z0], corners[CORNER_x1y1z0], plane
	) ? 1 : 0;

	if (n == 6)
		// At this point it's already possible to have discovered all intersections
		return sortPolygon((SVector3<flt_type>*)out, n, *(SVector3<flt_type>*)plane);

	n += linePlaneIntersect_exc(
		out + n*3, box.pmin              , corners[CORNER_x0y1z0], plane
	) ? 1 : 0;
	n += linePlaneIntersect_exc(
		out + n*3, corners[CORNER_x0y1z0], corners[CORNER_x0y1z1], plane
	) ? 1 : 0;
	n += linePlaneIntersect_exc(
		out + n*3, corners[CORNER_x0y1z0], corners[CORNER_x1y1z0], plane
	) ? 1 : 0;
	n += linePlaneIntersect_exc(
		out + n*3, corners[CORNER_x1y0z1], box.pmax,               plane
	) ? 1 : 0;
	n += linePlaneIntersect_exc(
		out + n*3, corners[CORNER_x1y0z1], corners[CORNER_x0y0z1], plane
	) ? 1 : 0;
	n += linePlaneIntersect_exc(
		out + n*3, corners[CORNER_x1y0z1], corners[CORNER_x1y0z0], plane
	) ? 1 : 0;

	if (n > 2)
		// It's possible to have already discovered all intersections at this point too
		return sortPolygon((SVector3<flt_type>*)out, n, *(SVector3<flt_type>*)plane);


	////
	// Step 2: Account for the special case that the plane intersects exactly through one
	//         or more of the box corners.

	if (DOT_VF(box.pmin, plane) + plane[3] == flt_type(0))
		*((SVector3<flt_type>*)(out + (n++)*3)) = box.pmin;

	if (DOT_VF(corners[0], plane) + plane[3] == flt_type(0))
		*((SVector3<flt_type>*)(out + (n++)*3)) = corners[0];

	if (DOT_VF(corners[1], plane) + plane[3] == flt_type(0))
		*((SVector3<flt_type>*)(out + (n++)*3)) = corners[1];

	if (DOT_VF(corners[2], plane) + plane[3] == flt_type(0))
		*((SVector3<flt_type>*)(out + (n++)*3)) = corners[2];

	if (DOT_VF(corners[3], plane) + plane[3] == flt_type(0))
		*((SVector3<flt_type>*)(out + (n++)*3)) = corners[3];

	if (DOT_VF(corners[4], plane) + plane[3] == flt_type(0))
		*((SVector3<flt_type>*)(out + (n++)*3)) = corners[4];

	if (DOT_VF(corners[5], plane) + plane[3] == flt_type(0))
		*((SVector3<flt_type>*)(out + (n++)*3)) = corners[5];

	if (DOT_VF(box.pmax, plane) + plane[3] == flt_type(0))
		*((SVector3<flt_type>*)(out + (n++)*3)) = box.pmax;


	// All possible intersection cases checked
	return sortPolygon((SVector3<flt_type>*)out, n, *(SVector3<flt_type>*)plane);
}

template <class flt_type>
unsigned int _<flt_type>::boxTriIntersect (
	flt_type *out, const SAABox<flt_type> &box,
	const flt_type *p0, const flt_type *p1, const flt_type *p2, const flt_type *normal
)
{
	unsigned int n;

	// Non-min/max points of the box, in Morton / Z-order
	SVector3<flt_type> corners[6] = {
		{box.pmax.x, box.pmin.y, box.pmin.z},
		{box.pmin.x, box.pmax.y, box.pmin.z},
		{box.pmax.x, box.pmax.y, box.pmin.z},
		{box.pmin.x, box.pmin.y, box.pmax.z},
		{box.pmax.x, box.pmin.y, box.pmax.z},
		{box.pmin.x, box.pmax.y, box.pmax.z}
	};

	// Box planes
	SVector4<flt_type> planes[6] = {
		{ 0.0, 0.0,-1.0, box.pmin.z}, // Front
		{ 0.0, 0.0, 1.0,-box.pmax.z}, // Back
		{-1.0, 0.0, 0.0, box.pmin.x}, // Left
		{ 1.0, 0.0, 0.0,-box.pmax.x}, // Right	
		{ 0.0,-1.0, 0.0, box.pmin.y}, // Bottom
		{ 0.0, 1.0, 0.0,-box.pmax.y}  // Top
	};


	////
	// Step 1: Check for triangle vertices that are encompassed by the box

	n = 0;
	if (pointInOnBox(box, p0))
	{
		out[n++] = p0[X]; out[1] = p0[Y]; out[2] = p0[Z];
	}
	if (pointInOnBox(box, p1))
	{
		out[3*n] = p1[X]; out[3*n + 1] = p1[Y]; out[3*(n++) + 2] = p1[Z];
	}
	if (pointInOnBox(box, p2))
	{
		out[3*n] = p2[X]; out[3*n + 1] = p2[Y]; out[3*(n++) + 2] = p2[Z];
	}

	if (n == 3)
		// The triangle is encompassed by the box completely
		return 3; // <-- no sorting needed, intersection polygon is exactly the triangle


	////
	// Step 2: Check if triangle plane is perpendicular and handle that as special cases

	// Temporary for further checking a potential intersection within an if... condition
	SVector3<flt_type> intersec;

	// (a) xy-parallel faces
	if (normal[X] == flt_type(0) && normal[Y] == flt_type(0))
	{
		// Check for intersected triangle edges
		// #1: left face
		if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_LEFT]) &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_LEFT]) &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_LEFT]) &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		// #2: right face
		if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_RIGHT]) &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_RIGHT]) &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_RIGHT]) &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		// #3: bottom face
		if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_BOTTOM]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_BOTTOM]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_BOTTOM]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		// #4: top face
		if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_TOP]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_TOP]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_TOP]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;

		// Check for intersected box edges
		// NOTE: Floating point inaccuracy can cause duplicates with earlier triangle
		//       edge tests
		n += lineTriIntersect_exc(
			out + n*3, box.pmin,               corners[CORNER_x0y0z1], p0, p1, p2
		) ? 1 : 0;
		n += lineTriIntersect_exc(
			out + n*3, corners[CORNER_x1y0z0], corners[CORNER_x1y0z1], p0, p1, p2
		) ? 1 : 0;
		n += lineTriIntersect_exc(
			out + n*3, corners[CORNER_x0y1z0], corners[CORNER_x0y1z1], p0, p1, p2
		) ? 1 : 0;
		n += lineTriIntersect_exc(
			out + n*3, corners[CORNER_x1y1z0], box.pmax,               p0, p1, p2
		) ? 1 : 0;

		// All possible intersection cases checked
		return sortPolygon((SVector3<flt_type>*)out, n, *(SVector3<flt_type>*)normal);
	}
	// (b) yz-parallel faces
	else if (normal[Y] == flt_type(0) && normal[Z] == flt_type(0))
	{
		// Check for intersected triangle edges
		// #1: front face
		if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_FRONT]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_FRONT]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_FRONT]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y)
			((SVector3<flt_type>*)out)[n++] = intersec;
		// #2: back face
		if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_BACK]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_BACK]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_BACK]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y)
			((SVector3<flt_type>*)out)[n++] = intersec;
		// #3: bottom face
		if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_BOTTOM]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_BOTTOM]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_BOTTOM]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		// #4: top face
		if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_TOP]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_TOP]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_TOP]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;

		// Check for intersected box edges
		// NOTE: Floating point inaccuracy can cause duplicates with earlier triangle
		//       edge tests
		n += lineTriIntersect_exc(
			out + n*3, box.pmin,               corners[CORNER_x1y0z0], p0, p1, p2
		) ? 1 : 0;
		n += lineTriIntersect_exc(
			out + n*3, corners[CORNER_x0y1z0], corners[CORNER_x1y1z0], p0, p1, p2
		) ? 1 : 0;
		n += lineTriIntersect_exc(
			out + n*3, corners[CORNER_x0y0z1], corners[CORNER_x1y0z1], p0, p1, p2
		) ? 1 : 0;
		n += lineTriIntersect_exc(
			out + n*3, corners[CORNER_x0y1z1], box.pmax,               p0, p1, p2
		) ? 1 : 0;

		// All possible intersection cases checked
		return sortPolygon((SVector3<flt_type>*)out, n, *(SVector3<flt_type>*)normal);
	}
	// (c) xz-parallel faces
	else if (normal[X] == flt_type(0) && normal[Z] == flt_type(0))
	{
		// Check for intersected triangle edges
		// #1: front face
		if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_FRONT]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_FRONT]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_FRONT]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y)
			((SVector3<flt_type>*)out)[n++] = intersec;
		// #2: back face
		if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_BACK]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_BACK]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_BACK]) &&
		    intersec.x >= box.pmin.x && intersec.x <= box.pmax.x &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y)
			((SVector3<flt_type>*)out)[n++] = intersec;
		// #3: left face
		if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_LEFT]) &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_LEFT]) &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_LEFT]) &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		// #4: right face
		if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_RIGHT]) &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_RIGHT]) &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;
		if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_RIGHT]) &&
		    intersec.y >= box.pmin.y && intersec.y <= box.pmax.y &&
		    intersec.z >= box.pmin.z && intersec.z <= box.pmax.z)
			((SVector3<flt_type>*)out)[n++] = intersec;

		// Check for intersected box edges
		// NOTE: Floating point inaccuracy can cause duplicates with earlier triangle
		//       edge tests
		n += lineTriIntersect_exc(
			out + n*3, box.pmin,               corners[CORNER_x0y1z0], p0, p1, p2
		) ? 1 : 0;
		n += lineTriIntersect_exc(
			out + n*3, corners[CORNER_x1y0z0], corners[CORNER_x1y1z0], p0, p1, p2
		) ? 1 : 0;
		n += lineTriIntersect_exc(
			out + n*3, corners[CORNER_x0y0z1], corners[CORNER_x0y1z1], p0, p1, p2
		) ? 1 : 0;
		n += lineTriIntersect_exc(
			out + n*3, corners[CORNER_x1y0z1], box.pmax,               p0, p1, p2
		) ? 1 : 0;

		// All possible intersection cases checked
		return sortPolygon((SVector3<flt_type>*)out, n, *(SVector3<flt_type>*)normal);
	}


	////
	// Step 3: Check for edge intersections by intersecting the box edges with the
	//         triangle plane

	n += lineTriIntersect_exc(
		out + n*3, corners[CORNER_x1y0z0], corners[CORNER_x1y1z0], p0, p1, p2
	) ? 1 : 0;
	n += lineTriIntersect_exc(
		out + n*3, corners[CORNER_x0y0z1], corners[CORNER_x0y1z1], p0, p1, p2
	) ? 1 : 0;
	n += lineTriIntersect_exc(
		out + n*3, box.pmin,               corners[CORNER_x1y0z0], p0, p1, p2
	) ? 1 : 0;
	n += lineTriIntersect_exc(
		out + n*3, box.pmin,               corners[CORNER_x0y0z1], p0, p1, p2
	) ? 1 : 0;
	n += lineTriIntersect_exc(
		out + n*3, corners[CORNER_x1y1z0], box.pmax,               p0, p1, p2
	) ? 1 : 0;
	n += lineTriIntersect_exc(
		out + n*3, corners[CORNER_x0y1z1], box.pmax,               p0, p1, p2
	) ? 1 : 0;

	if (n > 5)
		// At this point it's already possible to have discovered all intersections
		return sortPolygon((SVector3<flt_type>*)out, n, *(SVector3<flt_type>*)normal);

	n += lineTriIntersect_exc(
		out + n*3, box.pmin, corners[CORNER_x0y1z0],               p0, p1, p2
	) ? 1 : 0;
	n += lineTriIntersect_exc(
		out + n*3, corners[CORNER_x1y0z1], box.pmax,               p0, p1, p2
	) ? 1 : 0;
	n += lineTriIntersect_exc(
		out + n*3, corners[CORNER_x1y0z0], corners[CORNER_x1y0z1], p0, p1, p2
	) ? 1 : 0;
	n += lineTriIntersect_exc(
		out + n*3, corners[CORNER_x0y1z0], corners[CORNER_x1y1z0], p0, p1, p2
	) ? 1 : 0;
	n += lineTriIntersect_exc(
		out + n*3, corners[CORNER_x0y1z0], corners[CORNER_x0y1z1], p0, p1, p2
	) ? 1 : 0;
	n += lineTriIntersect_exc(
		out + n*3, corners[CORNER_x0y0z1], corners[CORNER_x1y0z1], p0, p1, p2
	) ? 1 : 0;

	if (n > 5)
		// It's possible to have already discovered all intersections at this point too
		return sortPolygon((SVector3<flt_type>*)out, n, *(SVector3<flt_type>*)normal);


	////
	// Step 4: Check for face intersections by testing the triangle edges against every
	//         face of the box while avoiding duplicates from step 2 by excluding
	//         intersections that lie on a box edge

	// NOTE: Floating point inaccuracy can still cause duplicates anyway

	// #1: front face
	if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_FRONT]) &&
	    intersec.x > box.pmin.x && intersec.x < box.pmax.x &&
	    intersec.y > box.pmin.y && intersec.y < box.pmax.y)
		((SVector3<flt_type>*)out)[n++] = intersec;
	if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_FRONT]) &&
	    intersec.x > box.pmin.x && intersec.x < box.pmax.x &&
	    intersec.y > box.pmin.y && intersec.y < box.pmax.y)
		((SVector3<flt_type>*)out)[n++] = intersec;
	if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_FRONT]) &&
	    intersec.x > box.pmin.x && intersec.x < box.pmax.x &&
	    intersec.y > box.pmin.y && intersec.y < box.pmax.y)
		((SVector3<flt_type>*)out)[n++] = intersec;
	// #2: back face
	if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_BACK]) &&
	    intersec.x > box.pmin.x && intersec.x < box.pmax.x &&
	    intersec.y > box.pmin.y && intersec.y < box.pmax.y)
		((SVector3<flt_type>*)out)[n++] = intersec;
	if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_BACK]) &&
	    intersec.x > box.pmin.x && intersec.x < box.pmax.x &&
	    intersec.y > box.pmin.y && intersec.y < box.pmax.y)
		((SVector3<flt_type>*)out)[n++] = intersec;
	if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_BACK]) &&
	    intersec.x > box.pmin.x && intersec.x < box.pmax.x &&
	    intersec.y > box.pmin.y && intersec.y < box.pmax.y)
		((SVector3<flt_type>*)out)[n++] = intersec;
	// #3: left face
	if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_LEFT]) &&
	    intersec.y > box.pmin.y && intersec.y < box.pmax.y &&
	    intersec.z > box.pmin.z && intersec.z < box.pmax.z)
		((SVector3<flt_type>*)out)[n++] = intersec;
	if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_LEFT]) &&
	    intersec.y > box.pmin.y && intersec.y < box.pmax.y &&
	    intersec.z > box.pmin.z && intersec.z < box.pmax.z)
		((SVector3<flt_type>*)out)[n++] = intersec;
	if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_LEFT]) &&
	    intersec.y > box.pmin.y && intersec.y < box.pmax.y &&
	    intersec.z > box.pmin.z && intersec.z < box.pmax.z)
		((SVector3<flt_type>*)out)[n++] = intersec;
	// #4: right face
	if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_RIGHT]) &&
	    intersec.y > box.pmin.y && intersec.y < box.pmax.y &&
	    intersec.z > box.pmin.z && intersec.z < box.pmax.z)
		((SVector3<flt_type>*)out)[n++] = intersec;
	if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_RIGHT]) &&
	    intersec.y > box.pmin.y && intersec.y < box.pmax.y &&
	    intersec.z > box.pmin.z && intersec.z < box.pmax.z)
		((SVector3<flt_type>*)out)[n++] = intersec;
	if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_RIGHT]) &&
	    intersec.y > box.pmin.y && intersec.y < box.pmax.y &&
	    intersec.z > box.pmin.z && intersec.z < box.pmax.z)
		((SVector3<flt_type>*)out)[n++] = intersec;
	// #5: bottom face
	if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_BOTTOM]) &&
	    intersec.x > box.pmin.x && intersec.x < box.pmax.x &&
	    intersec.z > box.pmin.z && intersec.z < box.pmax.z)
		((SVector3<flt_type>*)out)[n++] = intersec;
	if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_BOTTOM]) &&
	    intersec.x > box.pmin.x && intersec.x < box.pmax.x &&
	    intersec.z > box.pmin.z && intersec.z < box.pmax.z)
		((SVector3<flt_type>*)out)[n++] = intersec;
	if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_BOTTOM]) &&
	    intersec.x > box.pmin.x && intersec.x < box.pmax.x &&
	    intersec.z > box.pmin.z && intersec.z < box.pmax.z)
		((SVector3<flt_type>*)out)[n++] = intersec;
	// #6: top face
	if (linePlaneIntersect_exc(intersec, p0, p1, planes[PLANE_TOP]) &&
	    intersec.x > box.pmin.x && intersec.x < box.pmax.x &&
	    intersec.z > box.pmin.z && intersec.z < box.pmax.z)
		((SVector3<flt_type>*)out)[n++] = intersec;
	if (linePlaneIntersect_exc(intersec, p1, p2, planes[PLANE_TOP]) &&
	    intersec.x > box.pmin.x && intersec.x < box.pmax.x &&
	    intersec.z > box.pmin.z && intersec.z < box.pmax.z)
		((SVector3<flt_type>*)out)[n++] = intersec;
	if (linePlaneIntersect_exc(intersec, p2, p0, planes[PLANE_TOP]) &&
	    intersec.x > box.pmin.x && intersec.x < box.pmax.x &&
	    intersec.z > box.pmin.z && intersec.z < box.pmax.z)
		((SVector3<flt_type>*)out)[n++] = intersec;

	if (n > 6)
		// Again, it's possible to have discovered all intersections at this point.
		return sortPolygon((SVector3<flt_type>*)out, n, *(SVector3<flt_type>*)normal);


	////
	// Step 5: Check if any of the box corners are encompassed by the triangle

	SVector4<flt_type> tri_plane = {normal[X], normal[Y], normal[Z],
	                                // Mirrored w-component for faster plane checks
	                                DOT_FF(normal,p0)};

	if (fptools<flt_type>::equal(box.pmin*tri_plane.to_vec3(), tri_plane.w) &&
	    pointInTriangle(box.pmin, p0, p1, p2) &&
	    NEQUAL_VF(box.pmin, p0) && NEQUAL_VF(box.pmin, p1) && NEQUAL_VF(box.pmin, p2))
		((SVector3<flt_type>*)out)[n++] = box.pmin;

	if (fptools<flt_type>::equal(
	    	corners[CORNER_x1y0z0]*tri_plane.to_vec3(), tri_plane.w
	    ) &&
	    pointInTriangle(corners[CORNER_x1y0z0], p0, p1, p2) &&
	    NEQUAL_VF(corners[CORNER_x1y0z0], p0) && NEQUAL_VF(corners[CORNER_x1y0z0], p1) &&
	    NEQUAL_VF(corners[CORNER_x1y0z0], p2))
		((SVector3<flt_type>*)out)[n++] = corners[CORNER_x1y0z0];

	if (fptools<flt_type>::equal(
	    	corners[CORNER_x0y1z0]*tri_plane.to_vec3(), tri_plane.w
	    ) &&
	    pointInTriangle(corners[CORNER_x0y1z0], p0, p1, p2) &&
	    NEQUAL_VF(corners[CORNER_x0y1z0], p0) && NEQUAL_VF(corners[CORNER_x0y1z0], p1) &&
	    NEQUAL_VF(corners[CORNER_x0y1z0], p2))
		((SVector3<flt_type>*)out)[n++] = corners[CORNER_x0y1z0];

	if (fptools<flt_type>::equal(
	    	corners[CORNER_x1y1z0]*tri_plane.to_vec3(), tri_plane.w
	    ) &&
	    pointInTriangle(corners[CORNER_x1y1z0], p0, p1, p2) &&
	    NEQUAL_VF(corners[CORNER_x1y1z0], p0) && NEQUAL_VF(corners[CORNER_x1y1z0], p1) &&
	    NEQUAL_VF(corners[CORNER_x1y1z0], p2))
		((SVector3<flt_type>*)out)[n++] = corners[CORNER_x1y1z0];

	if (fptools<flt_type>::equal(
	    	corners[CORNER_x0y0z1]*tri_plane.to_vec3(), tri_plane.w
	    ) &&
	    pointInTriangle(corners[CORNER_x0y0z1], p0, p1, p2) &&
	    NEQUAL_VF(corners[CORNER_x0y0z1], p0) && NEQUAL_VF(corners[CORNER_x0y0z1], p1) &&
	    NEQUAL_VF(corners[CORNER_x0y0z1], p2))
		((SVector3<flt_type>*)out)[n++] = corners[CORNER_x0y0z1];

	if (fptools<flt_type>::equal(
	    	corners[CORNER_x1y0z1]*tri_plane.to_vec3(), tri_plane.w
	    ) &&
	    pointInTriangle(corners[CORNER_x1y0z1], p0, p1, p2) &&
	    NEQUAL_VF(corners[CORNER_x1y0z1], p0) && NEQUAL_VF(corners[CORNER_x1y0z1], p1) &&
	    NEQUAL_VF(corners[CORNER_x1y0z1], p2))
		((SVector3<flt_type>*)out)[n++] = corners[CORNER_x1y0z1];

	if (fptools<flt_type>::equal(
	    	corners[CORNER_x0y1z1]*tri_plane.to_vec3(), tri_plane.w
	    ) &&
	    pointInTriangle(corners[CORNER_x0y1z1], p0, p1, p2) &&
	    NEQUAL_VF(corners[CORNER_x0y1z1], p0) && NEQUAL_VF(corners[CORNER_x0y1z1], p1) &&
	    NEQUAL_VF(corners[CORNER_x0y1z1], p2))
		((SVector3<flt_type>*)out)[n++] = corners[CORNER_x0y1z1];

	if (fptools<flt_type>::equal(box.pmax*tri_plane.to_vec3(), tri_plane.w) &&
	    pointInTriangle(box.pmax, p0, p1, p2) &&
	    NEQUAL_VF(box.pmax, p0) && NEQUAL_VF(box.pmax, p1) && NEQUAL_VF(box.pmax, p2))
		((SVector3<flt_type>*)out)[n++] = box.pmax;


	// All possible intersection cases checked
	return sortPolygon((SVector3<flt_type>*)out, n, *(SVector3<flt_type>*)normal);
}



//////
//
// Explicit template instantiations
//

// Only floating point versions are intended
template struct HTEST_API _<float>;
template struct HTEST_API _<double>;
