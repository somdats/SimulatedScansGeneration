/**
 * @file   hitteset.h
 * @author Benjamin Russig
 *
 * Created on 29.10.2014 20:11:37
 */

#ifndef __HTEST_HITTEST_H__
#define __HTEST_HITTEST_H__


// Library setup
#include "_config.h"



//////
//
// Dependencies
//

// Local includes
#include "utils.h"



// Open library namespace
namespace htest {



//////
//
// Structs
//

/** @brief Describes an axis-aligned box. */
template <class flt_type>
struct SAABox
{
	/** @brief The corner of the box with the lowest coordinates. */
	SVector3<flt_type> pmin;
	/** @brief The corner of the box with the highest coordinates. */
	SVector3<flt_type> pmax;

	/**
	 * @brief
	 *		The point exactly in the middle of the box. This property must be kept
	 *		updated manually by calling @ref #updateSecondaryData whenever changes are
	 *		made to the box.
	 */
	SVector3<flt_type> pmid;

	/**
	 * @brief
	 *		A vector containing the box' half-extent in every dimension. This property
	 *		must be kept updated manually by calling @ref #updateSecondaryData whenever
	 *		changes are made to the box.
	 */
	SVector3<flt_type> halfsize;

	/**
	 * @brief
	 *		(Re-)initializes the box to @link #accomodate accomodate @endlink new points.
	 */
	inline void initialize (void)
	{
		pmin.set_all(util::fptools<flt_type>::INFINITY_POS);
		pmax.set_all(util::fptools<flt_type>::INFINITY_NEG);
	}

	/**
	 * @brief Minimally extends the box so that it encompasses a given point.
	 *
	 * @note
	 *		This method will not update the box' @link #pmid midpoint @endlink and @link
	 *		#halfsize half-size vector @endlink . If these values are needed, a
	 *		subsequent call to @ref #updateSecondaryData is required. This enables
	 *		bulk-adding of points and do the additional calculation just once afterwards.
	 *
	 * @param point
	 *		The point that shall be encompassed by the box.
	 */
	inline void accomodate (const SVector3<flt_type> &point)
	{
		pmin.x = pmin.x > point.x ? point.x : pmin.x;
		pmin.y = pmin.y > point.y ? point.y : pmin.y;
		pmin.z = pmin.z > point.z ? point.z : pmin.z;
		pmax.x = pmax.x < point.x ? point.x : pmax.x;
		pmax.y = pmax.y < point.y ? point.y : pmax.y;
		pmax.z = pmax.z < point.z ? point.z : pmax.z;
	}

	/**
	 * @brief Retreives the magnitude of the extent in the dimension the box is largest.
	 *
	 * @return the value of the box' largest extent.
	 */
	inline flt_type getMaxExtent (void)
	{
		SVector3<flt_type> all_extents = pmax - pmin;
		return
			(all_extents.x >= all_extents.y && all_extents.x >= all_extents.z) ?
				all_extents.x : (all_extents.y >= all_extents.z) ?
					all_extents.y :
					all_extents.z;
	}

	/**
	 * @brief
	 *		Recalculates additional properties of the box, namely its @link #pmid
	 *		midpoint @endlink and @link #halfsize half-size vector @endlink .
	 */
	inline void updateSecondaryData (void)
	{
		pmid = (pmin + pmax) / 2;
		// Unfortunately, this has to be a bit verbose in order to guarantee robustness /
		// floating point stability across a maximal amount of platforms and
		// architectures...
		halfsize = ((pmid - pmin) + (pmax - pmid))/2;
	}
};



//////
//
// Classes
//

/** @brief The hittest library's main templated function collection. */
template <class flt_type>
struct HTEST_API _
{
	/**
	 * @brief Tests if a point lies on a triangle.
	 *
	 * @note A point that lies on a triangle edge will test positively.
	 *
	 * @return @c true if the point lies on the triangle, @c false otherwise.
	 *
	 * @param point
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		point to test against the triangle.
	 * @param p0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first triangle vertex.
	 * @param p1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second triangle vertex.
	 * @param p2
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		third triangle vertex.
	 */
	static bool pointInTriangle (const flt_type *point, const flt_type *p0,
	                             const flt_type *p1, const flt_type *p2);

	/**
	 * @brief
	 *		Tests if a point lies on a triangle, and returns the barycentric coordinates
	 *		of the point with respect to the input vertices.
	 *
	 * @note A point that lies on a triangle edge will test positively.
	 *
	 * @return @c true if the point lies on the triangle, @c false otherwise.
	 *
	 * @param[out] u
	 *		Barycentric coordinate of @c point with respect to @c p1 .
	 * @param[out] v
	 *		Barycentric coordinate of @c point with respect to @c p2 .
	 * @param point
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		point to test against the triangle.
	 * @param p0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first triangle vertex.
	 * @param p1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second triangle vertex.
	 * @param p2
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		third triangle vertex.
	 */
	static bool pointInTriangle (flt_type *u, flt_type *v,
	                             const flt_type *point, const flt_type *p0,
	                             const flt_type *p1, const flt_type *p2);

	/**
	 * @brief
	 *		Tests whether a point lies inside an axis-aligned box, with the surface of
	 *		the box not counting as being inside.
	 *
	 * @return @c true if the point is inside the box, @c false otherwise.
	 *
	 * @param box
	 *		The box to test the point against.
	 * @param point
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		point to test against the box.
	 */
	inline static bool pointInBox (const SAABox<flt_type> &box, const flt_type *point)
	{
		return
			   (box.pmin.x < point[0] && box.pmin.y < point[1] && box.pmin.z < point[2])
			&& (box.pmax.x > point[0] && box.pmax.y > point[1] && box.pmax.z > point[2]);
	}

	/**
	 * @brief
	 *		Tests whether a point lies inside an axis-aligned box, the surface of the box
	 *		counting as being inside.
	 *
	 * @return @c true if the point is inside or on the box, @c false otherwise.
	 *
	 * @param box
	 *		The box to test the point against.
	 * @param point
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		point to test against the box.
	 */
	inline static bool pointInOnBox (const SAABox<flt_type> &box, const flt_type *point)
	{
		return
			   (box.pmin.x<=point[0] && box.pmin.y<=point[1] && box.pmin.z<=point[2])
			&& (box.pmax.x>=point[0] && box.pmax.y>=point[1] && box.pmax.z>=point[2]);
	}

	/**
	 * @brief
	 *		Tests whether a triangle is completely inside an axis-aligned box, with the
	 *		surface of the box not counting as inside.
	 *
	 * @return @c true if the triangle is inside the box, @c false otherwise.
	 *
	 * @param box
	 *		The box to test the triangle against.
	 * @param p0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first triangle vertex.
	 * @param p1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second triangle vertex.
	 * @param p2
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		third triangle vertex.
	 */
	inline static bool triInBox (const SAABox<flt_type> &box, const flt_type *p0,
	                             const flt_type *p1, const flt_type *p2)
	{
		return pointInBox(box, p0) && pointInBox(box, p1) && pointInBox(box, p2);
	}

	/**
	 * @brief
	 *		Tests whether a triangle is completely inside an axis-aligned box, the
	 *		surface of the box counting as inside.
	 *
	 * @return @c true if the triangle is inside or on the box, @c false otherwise.
	 *
	 * @param box
	 *		The box to test the triangle against.
	 * @param p0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first triangle vertex.
	 * @param p1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second triangle vertex.
	 * @param p2
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		third triangle vertex.
	 */
	inline static bool triInOnBox (const SAABox<flt_type> &box, const flt_type *p0,
	                               const flt_type *p1, const flt_type *p2)
	{
		return pointInOnBox(box, p0) && pointInOnBox(box, p1) && pointInOnBox(box, p2);
	}

	/**
	 * @brief
	 *		Performs a ray/plane intersection test, returning the actual intersection
	 *		point in addition to simply checking for intersection. This function rejects
	 *		early if there is no intersection, leaving the output variables untouched.
	 *
	 * @note
	 *		The origin point of the ray lying on the plane will be rejected as an
	 *		intersection!
	 *
	 * @return @c true if the ray and plane intersect, @c false otherwise.
	 *
	 * @param[out] out
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		intersection point, if any.
	 * @param origin
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		point of origin of the ray to test.
	 * @param dir
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		direction vector of the ray.
	 * @param plane
	 *		Four floating point numbers making up the x, y, z and w coefficients of the
	 *		plane equation.
	 * @param[out] t_out
	 *		Optional - Ray parameter of the intersection point, if any.
	 */
	static bool rayPlaneIntersect (flt_type *out, const flt_type *origin,
	                               const flt_type *dir, const flt_type *plane,
	                               flt_type *t_out = NULL);

	/**
	 * @brief
	 *		Performs a line/plane intersection test, returning the actual intersection
	 *		point in addition to simply checking for intersection. This function rejects
	 *		early if there is no intersection, leaving the output variables untouched.
	 *
	 * @return @c true if the line and plane intersect, @c false otherwise.
	 *
	 * @param[out] out
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		intersection point, if any.
	 * @param l0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first end point of the line.
	 * @param l1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second end point of the line.
	 * @param plane
	 *		Four floating point numbers making up the x, y, z and w coefficients of the
	 *		plane equation.
	 */
	static bool linePlaneIntersect (flt_type *out, const flt_type *l0,const flt_type *l1,
	                                const flt_type *plane);

	/**
	 * @brief
	 *		Performs a line/plane intersection test, returning the actual intersection
	 *		point in addition to simply checking for intersection. However, no
	 *		intersection will be reported if the intersection point is one of the line
	 *		end points (i.e. one of the line end points lies on the triangle). This
	 *		function rejects early if there is no intersection, leaving the output
	 *		variables untouched.
	 *
	 * @return @c true if the line and plane intersect, @c false otherwise.
	 *
	 * @param[out] out
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		intersection point, if any.
	 * @param l0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first end point of the line.
	 * @param l1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second end point of the line.
	 * @param plane
	 *		Four floating point numbers making up the x, y, z and w coefficients of the
	 *		plane equation.
	 */
	static bool linePlaneIntersect_exc (flt_type *out, const flt_type *l0,
	                                    const flt_type *l1, const flt_type *plane);

	/**
	 * @brief
	 *		Performs a ray/triangle intersection test, returning the barycentric
	 *		coordinates of the intersection point with respect to the input vertices.
	 *		This function rejects early if there is no intersection, leaving the output
	 *		variables untouched.
	 *
	 * @note
	 *		The origin point of the ray lying in the triangle will be rejected as an
	 *		intersection case!<br />
	 *		The algorithm used is an implementation of the plane equation-less method
	 *		proposed in the paper "Fast, Minimum Storage Ray/Triangle Intersection" by
	 *		Thomas Möller et al.
	 *
	 * @return @c true if the ray and triangle intersect, @c false otherwise.
	 *
	 * @param[out] u
	 *		Barycentric coordinate of the intersection point, if any, with respect to @c
	 *		p1 .
	 * @param[out] v
	 *		Barycentric coordinate of the intersection point, if any, with respect to @c
	 *		p2 .
	 * @param origin
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		point of origin of the ray to test.
	 * @param dir
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		direction vector of the ray.
	 * @param p0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first triangle vertex.
	 * @param p1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second triangle vertex.
	 * @param p2
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		third triangle vertex.
	 */
	static bool rayTriIntersect (flt_type *u, flt_type *v, const flt_type *origin,
	                             const flt_type *dir, const flt_type *p0,
	                             const flt_type *p1, const flt_type *p2);

	/**
	 * @brief
	 *		Performs a ray/triangle intersection test, returning the intersection point.
	 *		This function rejects early if there is no intersection, leaving the output
	 *		variables untouched.
	 *
	 * @note
	 *		The origin point of the ray lying in the triangle will be rejected as an
	 *		intersection case!<br />
	 *		The algorithm used is an implementation of the plane equation-less method
	 *		proposed in the paper "Fast, Minimum Storage Ray/Triangle Intersection" by
	 *		Thomas Möller et al.
	 *
	 * @return @c true if the ray and triangle intersect, @c false otherwise.
	 *
	 * @param[out] out
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		intersection point, if any.
	 * @param origin
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		point of origin of the ray to test.
	 * @param dir
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		direction vector of the ray.
	 * @param p0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first triangle vertex.
	 * @param p1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second triangle vertex.
	 * @param p2
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		third triangle vertex.
	 * @param[out] t_out
	 *		Optional - Ray parameter of the intersection point, if any.
	 */
	static bool rayTriIntersect (flt_type *out, const flt_type *origin,
	                             const flt_type *dir, const flt_type *p0,
	                             const flt_type *p1, const flt_type *p2,
	                             flt_type *t_out = NULL);

	/**
	 * @brief
	 *		Performs a line/triangle intersection test, returning the intersection point
	 *		as barycentric coordinates with respect to the input vertices. This function
	 *		rejects early if there is no intersection, leaving the output variables
	 *		untouched.
	 *
	 * @note
	 *		The algorithm used is a modified implementation of the plane-equation-less
	 *		method proposed in the paper "Fast, Minimum Storage Ray/Triangle
	 *		Intersection" by Thomas Möller et al.
	 *
	 * @return @c true if the line and triangle intersect, @c false otherwise.
	 *
	 * @param[out] u
	 *		Barycentric coordinate of the intersection point, if any, with respect to @c
	 *		p1 .
	 * @param[out] v
	 *		Barycentric coordinate of the intersection point, if any, with respect to @c
	 *		p2 .
	 * @param l0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first end point of the line.
	 * @param l1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second end point of the line.
	 * @param p0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first triangle vertex.
	 * @param p1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second triangle vertex.
	 * @param p2
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		third triangle vertex.
	 */
	static bool lineTriIntersect (flt_type *u, flt_type *v, const flt_type *l0,
	                              const flt_type *l1, const flt_type *p0,
	                              const flt_type *p1, const flt_type *p2);

	/**
	 * @brief
	 *		Performs a line/triangle intersection test, returning the intersection point.
	 *		This function rejects early if there is no intersection, leaving the output
	 *		variables untouched.
	 *
	 * @note
	 *		The algorithm used is a modified implementation of the plane-equation-less
	 *		method proposed in the paper "Fast, Minimum Storage Ray/Triangle
	 *		Intersection" by Thomas Möller et al.
	 *
	 * @return @c true if the line and triangle intersect, @c false otherwise.
	 *
	 * @param[out] out
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		intersection point, if any.
	 * @param l0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first end point of the line.
	 * @param l1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second end point of the line.
	 * @param p0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first triangle vertex.
	 * @param p1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second triangle vertex.
	 * @param p2
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		third triangle vertex.
	 */
	static bool lineTriIntersect (flt_type *out, const flt_type *l0, const flt_type *l1,
	                              const flt_type *p0, const flt_type *p1,
	                              const flt_type *p2);

	/**
	 * @brief
	 *		Performs a line/triangle intersection test, returning the intersection point
	 *		as barycentric coordinates with respect to the input vertices. However, no
	 *		intersection will be reported if the intersection point is one of the line
	 *		end points (i.e. one of the line end points lies on the triangle). This
	 *		function rejects early if there is no intersection, leaving the output
	 *		variables untouched.
	 *
	 * @note
	 *		The algorithm used is a modified implementation of the plane-equation-less
	 *		method proposed in the paper "Fast, Minimum Storage Ray/Triangle
	 *		Intersection" by Thomas Möller et al.
	 *
	 * @return @c true if the line and triangle intersect, @c false otherwise.
	 *
	 * @param[out] u
	 *		Barycentric coordinate of the intersection point, if any, with respect to @c
	 *		p1 .
	 * @param[out] v
	 *		Barycentric coordinate of the intersection point, if any, with respect to @c
	 *		p2 .
	 * @param l0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first end point of the line.
	 * @param l1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second end point of the line.
	 * @param p0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first triangle vertex.
	 * @param p1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second triangle vertex.
	 * @param p2
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		third triangle vertex.
	 */
	static bool lineTriIntersect_exc (flt_type *u, flt_type *v, const flt_type *l0,
	                                  const flt_type *l1, const flt_type *p0,
	                                  const flt_type *p1, const flt_type *p2);

	/**
	 * @brief
	 *		Performs a line/triangle intersection test, returning the intersection point.
	 *		However, no intersection will be reported if the intersection point is one of
	 *		the line end points (i.e. one of the line end points lies on the triangle).
	 *		This function rejects early if there is no intersection, leaving the output
	 *		variables untouched.
	 *
	 * @note
	 *		The algorithm used is a modified implementation of the plane-equation-less
	 *		method proposed in the paper "Fast, Minimum Storage Ray/Triangle
	 *		Intersection" by Thomas Möller et al.
	 *
	 * @return @c true if the line and triangle intersect, @c false otherwise.
	 *
	 * @param[out] out
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		intersection point, if any.
	 * @param l0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first end point of the line.
	 * @param l1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second end point of the line.
	 * @param p0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first triangle vertex.
	 * @param p1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second triangle vertex.
	 * @param p2
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		third triangle vertex.
	 */
	static bool lineTriIntersect_exc (flt_type *out, const flt_type *l0,
	                                  const flt_type *l1, const flt_type *p0,
	                                  const flt_type *p1, const flt_type *p2);

	/**
	 * @brief
	 *		Tests whether two axis-aligned boxes non-zero overlap, using the separating
	 *		axis theorem.
	 *
	 * @return @c true if tested positively for overlap, @c false otherwise.
	 *
	 * @param box0
	 *		The first box to test against the second.
	 * @param box1
	 *		The second box to test against the first.
	 */
	static bool boxBoxTest (const SAABox<flt_type> &box0, const SAABox<flt_type> &box1);

	/**
	 * @brief
	 *		Performs a plane/axis-aligned-box test.
	 *
	 * @note
	 *		For practicability, the return semantics are a bit different from the other
	 *		hit tests. Namely, applications will typically want to know which "side" of
	 *		the plane the box is on if it doesn't intersect (frustum culling etc.)
	 *
	 * @return
	 *		0 if the box intersects the plane, 1 if it doesn't intersect and the plane
	 *		normal points towards the box, -1 if it doesn't intersect and the plane
	 *		normal points away from the box.
	 *
	 * @param box
	 *		The box to test against the plane.
	 * @param plane
	 *		Four floating point numbers making up the x, y, z and w coefficients of the
	 *		plane equation.
	 */
	static int boxPlaneTest (const SAABox<flt_type> &box, const flt_type *plane);

	/**
	 * @brief
	 *		Tests whether a triangle overlaps with an axis-aligned box, using the
	 *		separating axis theorem.
	 *
	 * @note
	 *		Triangles that zero-volume overlap with the box will test positively if and
	 *		only if they have a non-zero planar overlap with one of the box faces AND the
	 *		triangle normal points towards the box. In constrast to @ref #boxBoxTest ,
	 *		this is the most useful default behaviour for box-triangle tests, as it
	 *		allows unique association of wellformed axis-aligned triangles to grid cells,
	 *		while not affecting correctness of collision / penetration test use cases.
	 *
	 * @return @c true if tested positively for overlap, @c false otherwise.
	 *
	 * @param box
	 *		The box to test the triangle against.
	 * @param p0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first triangle vertex.
	 * @param p1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second triangle vertex.
	 * @param p2
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		third triangle vertex.
	 * @param normal
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		triangle normal.
	 */
	static bool boxTriTest (const SAABox<flt_type> &box,
	                        const flt_type *p0, const flt_type *p1, const flt_type *p2,
	                        const flt_type *normal);

	/**
	* @brief
	*		Tests whether a ray intersects with an axis-algined box, and returns the
	*		intersection point and corresponding ray parameter closest to the ray
	*		origin.
	*
	* @note
	*		Rays that lie exactly on one of the box faces only count as a hit if they lie
	*		on one of the three minimal faces.
	*
	* @return @c true if tested positively for intersection, @c false otherwise.
	*
	* @param[out] out
	*		Three floating point numbers making up the x, y and z coordinates of the
	*		intersection point, if any.
	* @param[out] t_out
	*		Ray parameter of the intersection point, if any.
	* @param box
	*		The box to test the ray against.
	* @param orig
	*		Three floating point numbers making up the x, y and z coordinates of the
	*		ray origin position vector.
	* @param dir
	*		Three floating point numbers making up the x, y and z coordinates of the
	*		ray direction vector.
	*/
	static bool boxRayIntersection (
		flt_type *out, flt_type *t_out, const SAABox<flt_type> &box,
		const flt_type *origin, const flt_type *dir
	);

	/**
	 * @brief
	 *		Performs a plane/axis-aligned-box intersection.
	 *
	 * @note
	 *		The resulting intersection polygon will have clockwise winding around its
	 *		normal in positive direction, consistent with the expectation that you could
	 *		recalculate its normal by taking the cross product of any of its edge
	 *		vectors.
	 *
	 * @return
	 *		the number of vertices in the intersection polygon. Intuitively, a value of
	 *		less than 3 means that the plane doesn't intersect the box's volume.
	 *
	 * @param[out] out
	 *		Array of floating point number triples making up the x, y and z coordinates
	 *		of the resulting intersection polygon vertices - must have space for a
	 *		maximum of 6 vertices * 3 = 18 floating point numbers.
	 * @param box
	 *		The box to intersect with the plane.
	 * @param plane
	 *		Four floating point numbers making up the x, y, z and w coefficients of the
	 *		plane equation.
	 */
	static unsigned int boxPlaneIntersect (flt_type *out, const SAABox<flt_type> &box,
	                                       const flt_type *plane);

	/**
	 * @brief
	 *		Performs a triangle/axis-aligned-box intersection.
	 *
	 * @return
	 *		the number of vertices in the intersection polygon. Intuitively, a value of
	 *		less than 3 means that the triangle doesn't intersect the box's volume.
	 *
	 * @param[out] out
	 *		Array of floating point number triples making up the x, y and z coordinates
	 *		of the resulting intersection polygon vertices - must have space for a
	 *		maximum of 8 vertices * 3 = 24 floating point numbers.
	 * @param box
	 *		The box to intersect with the triangle.
	 * @param p0
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		first triangle vertex.
	 * @param p1
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		second triangle vertex.
	 * @param p2
	 *		Three floating point numbers making up the x, y and z coordinates of the
	 *		third triangle vertex.
	 * @param normal
	 *		Three floating point numbers making up the x, y and z components of the
	 *		triangle normal.
	 */
	static unsigned int boxTriIntersect (flt_type *out, const SAABox<flt_type> &box,
	                                     const flt_type *p0, const flt_type *p1,
	                                     const flt_type *p2, const flt_type *normal);
};


// Close library namespace
}


#endif // ifndef __HTEST_HITTEST_H__
