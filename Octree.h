
#ifndef __OCTREE_H__
#define __OCTREE_H__


//////
//
// Includes
//

// C++ STL
#include <vector>
#include <map>

// API config
#include "_config.h"

// Local includes
#include "Utils.h"
#include "EigenTools.h"



//////
//
// Structs
//

/** @brief Callback interface for octree clients. */
template <class flt_type>
struct APISPEC AABox
{

	////
	// Types

	/** @brief Real number type. */
	typedef flt_type real;

	/** @brief 3D vector type. */
	typename typedef EigenTypes<real>::Vec3 Vec3;


	////
	// Data members

	/** @brief Minimum corner of the axis aligned box. */
	Vec3 pmin;

	/** @brief Maximum corner of the axis aligned box. */
	Vec3 pmax;


	////
	// Object construction / destruction

	/** @brief Default constructor. */
	AABox() : pmin( Constants<real>::inf, Constants<real>::inf, Constants<real>::inf),
	          pmax(-Constants<real>::inf,-Constants<real>::inf,-Constants<real>::inf) {}

	/** @brief Constructs the box with the given minimum and maximum corners. */
	AABox(const Vec3 &pmin, const Vec3 &pmax) : pmin(pmin), pmax(pmax) {}


	////
	// Methods

	/** @brief Invalidates the box to an uninitialized state. */
	inline void invalidate (void)
	{
		pmin.z() = pmin.y() = pmin.x() =  Constants<real>::inf;
		pmax.z() = pmax.y() = pmax.x() = -Constants<real>::inf;
	}

	/** @brief Ensures the box includes the given point by extending it if necessary. */
	inline void inludePoint (const Vec3 &point)
	{
		pmin.x() = point.x() < pmin.x() ? point.x() : pmin.x();
		pmin.y() = point.y() < pmin.y() ? point.y() : pmin.y();
		pmin.z() = point.z() < pmin.z() ? point.z() : pmin.z();
		pmax.x() = point.x() > pmax.x() ? point.x() : pmax.x();
		pmax.y() = point.y() > pmax.y() ? point.y() : pmax.y();
		pmax.z() = point.z() > pmax.z() ? point.z() : pmax.z();
	}

	/** @brief Returns the extent vector of the box (i.e. pmax - pmin). */
	inline Vec3 extent (void) const
	{
		return std::move(pmax - pmin);
	}

	/** @brief Returns (pmin + pmax)/2. */
	inline Vec3 centroid (void) const
	{
		return std::move(0.5*(pmin + pmax));
	}

	/** @brief Checks if box extent greater than 0 in every dimension. */
	inline bool valid (void) const
	{
		return pmin.x() < pmax.x() && pmin.y() < pmax.y() && pmin.z() < pmax.z();
	}

	/** @brief Checks if box extent equal to or less than 0 in at least one dimension. */
	inline bool invalid (void) const
	{
		return pmin.x() >= pmax.x() || pmin.y() >= pmax.y() || pmin.z() >= pmax.z();
	}
};



//////
//
// Interfaces
//

/** @brief Callback interface for octree clients. */
template <class flt_type>
class APISPEC IOctreeClient
{

public:

	////
	// Types

	/** @brief Real number type. */
	typedef flt_type real;

	/** @brief 3D vector type. */
	typename typedef EigenTypes<real>::Vec3 Vec3;


	////
	// Object construction / destruction

	/** @brief Virtual destructor (causes vtable creation). */
	virtual ~IOctreeClient();


	////
	// Methods

	/** @brief Returns the id-th triangle held by the client. */
	virtual void queryTriangle(
		Vec3 *p0, Vec3 *p1, Vec3 *p2, Vec3 *normal, unsigned id
	) const = 0;
};



//////
//
// Classes
//

/**
 * @brief
 *		Octree implementation storing index arrays at the leaves, taylored for use with
 *		static triangle meshes.
 */
template <class flt_type>
class APISPEC Octree
{

public:

	////
	// Types

	/** @brief Real number type. */
	typedef flt_type real;

	/** @brief 3D vector type. */
	typename typedef EigenTypes<real>::Vec3 Vec3;

	/** @brief 3D Homogenous vector type. */
	typename typedef EigenTypes<real>::Vec4 Vec4;

	/** @brief Type representing list of IDs. */
	typedef std::vector<unsigned> IdArray;

	/** @brief Map ordering list of IDs along a ray. */
	typedef std::map<real, IdArray> RayIdMap;


protected:

	////
	// Data members

	/** @brief The Client that has the actual triangle information. */
	const IOctreeClient<real> &client;

	/** @brief Axis aligned cube of this tree node. */
	AABox<real> box;

	/** */
	unsigned maxDepth;

	/** */
	unsigned targetOccupancy;

	/** */
	Octree<real> *children [8];

	/** */
	AABox<real> child_box [8];

	/** */
	IdArray occupants;


	////
	// Private interface

	/**
	 * @brief
	 *		Calculates the @link #child_box child node boxes @endlink from the current
	 *		@link #box node bounding box @endlink .
	 */
	void calculateChildBoxes (void);

	/** @brief Inserts a triangle index into the octree. */
	void insertTriangle (
		unsigned id, const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &normal
	);


public:

	////
	// Object construction / destruction

	/** @brief Initializes the octree for the given client. */
	Octree(const IOctreeClient<real> &client, unsigned maxDepth=2,
	       unsigned targetOccupancy=8);

	/**
	 * @brief
	 *		Constructs the octree with the given triangles. Also needs the (tight
	 *		fitting) bounding box of the mesh in order to decide the root node size.
	 */
	Octree(const IOctreeClient<real> &client, const IdArray &triangles,
	       const AABox<real> &meshExtend, unsigned maxDepth=2,
	       unsigned targetOccupancy=8);

	/** @brief Virtual destructor (causes vtable creation). */
	~Octree();


	////
	// Methods

	/**
	* @brief
	*		Builds the octree for the provided triangles. Also needs the (tight fitting)
	*		bounding box of the mesh in order to decide the root node size.
	*/
	void build (const IdArray &triangles, const AABox<real> &meshExtend);

	/**
	 * @brief
	 *		Returns the data in all leaf nodes that intersect with the ray (if any),
	 *		ordered according to closeness to the ray origin. Returns true if a leaf
	 *		node was hit at all, and false otherwise.
	 */
	bool queryRay (RayIdMap *IDs, const Ray<real> &ray) const;

	/** @brief Completely empties the whole tree. */
	void clear (void);

	/** @brief Returns false if the tree has children or data. */
	bool isEmpty (void);

	/** @brief Returns true if the tree has children or data. */
	bool isNotEmpty(void);
};



#endif // ifndef __OCTREE_H__
