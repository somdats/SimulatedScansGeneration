
//////
//
// Includes
//

// Local includes
#include <vector>
#include <map>
#include <stdexcept>

// Local includes
#include "Ray.h"
#include "HitTest/vector.h"
#include "HitTest/hittest.h"

// Implemented header
#include "Octree.h"



//////
//
// Default namespaces
//

// Implemented namespaces
using namespace htest;



//////
//
// Helper functions
//

template <class flt_type>
SAABox<flt_type> toSAABox(const AABox<flt_type> &aabox)
{
	SAABox<flt_type> ret;
	ret.pmin.x = aabox.pmin.x(); ret.pmin.y = aabox.pmin.y(); ret.pmin.z = aabox.pmin.z();
	ret.pmax.x = aabox.pmax.x(); ret.pmax.y = aabox.pmax.y(); ret.pmax.z = aabox.pmax.z();
	ret.updateSecondaryData();
	return std::move(ret);
}

template <class flt_type>
AABox<flt_type> toAABox(const SAABox<flt_type> &aabox)
{
	AABox<flt_type> ret;
	ret.pmin.x() = aabox.pmin.x; ret.pmin.y() = aabox.pmin.y; ret.pmin.z() = aabox.pmin.z;
	ret.pmax.x() = aabox.pmax.x; ret.pmax.y() = aabox.pmax.y; ret.pmax.z() = aabox.pmax.z;
	return std::move(ret);
}



//////
//
// Interface pre-implementation
//

// IOctreeClient
//

template <class flt_type>
IOctreeClient<flt_type>::~IOctreeClient() {}



//////
//
// Class implementation
//

// Octree
//

template <class flt_type>
Octree<flt_type>::Octree(
	const IOctreeClient<real> &client, unsigned maxDepth, unsigned targetOccupancy
)
	: client(client), maxDepth(maxDepth), targetOccupancy(targetOccupancy),
	  children{nullptr}
{
}

template <class flt_type>
Octree<flt_type>::Octree(
	const IOctreeClient<real> &client, const IdArray &triangles,
	const AABox<real> &meshExtend, unsigned maxDepth, unsigned targetOccupancy
)
	: client(client), maxDepth(maxDepth), targetOccupancy(targetOccupancy),
	  children{nullptr}
{
	build(triangles, meshExtend);
}

template <class flt_type>
Octree<flt_type>::~Octree()
{
	clear();
}

template <class flt_type>
void Octree<flt_type>::calculateChildBoxes(void)
{
	Vec3 midpoint = box.centroid();

	// Lower-left-front
	child_box[0].pmin = box.pmin;
	//-
	child_box[0].pmax = midpoint;
	// Lower-right-front
	child_box[1].pmin.x() = midpoint.x();
	child_box[1].pmin.y() = box.pmin.y();
	child_box[1].pmin.z() = box.pmin.z();
	//-
	child_box[1].pmax.x() = box.pmax.x();
	child_box[1].pmax.y() = midpoint.y();
	child_box[1].pmax.z() = midpoint.z();
	// Upper-left-front
	child_box[2].pmin.x() = box.pmin.x();
	child_box[2].pmin.y() = midpoint.y();
	child_box[2].pmin.z() = box.pmin.z();
	//-
	child_box[2].pmax.x() = midpoint.x();
	child_box[2].pmax.y() = box.pmax.y();
	child_box[2].pmax.z() = midpoint.z();
	// Upper-right-front
	child_box[3].pmin.x() = midpoint.x();
	child_box[3].pmin.y() = midpoint.y();
	child_box[3].pmin.z() = box.pmin.z();
	//-
	child_box[3].pmax.x() = box.pmax.x();
	child_box[3].pmax.y() = box.pmax.y();
	child_box[3].pmax.z() = midpoint.z();

	// Lower-left-back
	child_box[4].pmin.x() = box.pmin.x();
	child_box[4].pmin.y() = box.pmin.y();
	child_box[4].pmin.z() = midpoint.z();
	//-
	child_box[4].pmax.x() = midpoint.x();
	child_box[4].pmax.y() = midpoint.y();
	child_box[4].pmax.z() = box.pmax.z();
	// Lower-right-back
	child_box[5].pmin.x() = midpoint.x();
	child_box[5].pmin.y() = box.pmin.y();
	child_box[5].pmin.z() = midpoint.z();
	//-
	child_box[5].pmax.x() = box.pmax.x();
	child_box[5].pmax.y() = midpoint.y();
	child_box[5].pmax.z() = box.pmax.z();
	// Upper-left-back
	child_box[6].pmin.x() = box.pmin.x();
	child_box[6].pmin.y() = midpoint.y();
	child_box[6].pmin.z() = midpoint.z();
	//-
	child_box[6].pmax.x() = midpoint.x();
	child_box[6].pmax.y() = box.pmax.y();
	child_box[6].pmax.z() = box.pmax.z();
	// Upper-right-back
	child_box[7].pmin = midpoint;
	//-
	child_box[7].pmax = box.pmax;
}

template <class flt_type>
void Octree<flt_type>::insertTriangle (
	unsigned id, const Vec3 &p0, const Vec3 &p1, const Vec3 &p2, const Vec3 &normal
)
{
	// Common init: convert box formats for HitTest API conformance
	SAABox<real> node = std::move(toSAABox(box));
	SAABox<real> child_node [8] = {
		std::move(toSAABox(child_box[0])), std::move(toSAABox(child_box[1])),
		std::move(toSAABox(child_box[2])), std::move(toSAABox(child_box[3])),
		std::move(toSAABox(child_box[4])), std::move(toSAABox(child_box[5])),
		std::move(toSAABox(child_box[6])), std::move(toSAABox(child_box[7]))
	};

	if (isEmpty())
	{
		// First ever triangle in the tree
		if (!_<real>::boxTriTest(node, p0.data(), p1.data(), p2.data(), normal.data()))
			// Sanity check
			throw std::runtime_error("[Octree] !!!INTERNAL ERROR!!!");
		occupants.push_back(id);
		return;
	}

	IdArray adds;
	if (occupants.size() > 0)
	{
		// Node is currently (still) a leaf
		if (occupants.size() < targetOccupancy || maxDepth < 2)
		{
			// Leaf not yet full
			occupants.push_back(id);
			return;
		}
		// Leaf now full, subdivide
		adds = std::move(occupants);
	}
	adds.push_back(id);

	// Insert triangles
	for (unsigned i=0; i<adds.size(); i++)
	{
		Vec3 _p0, _p1, _p2, _normal;
		client.queryTriangle(&_p0, &_p1, &_p2, &_normal, adds[i]);

		for (unsigned j=0; j<8; j++)
		{
			if (_<real>::boxTriTest(
			    	child_node[j], _p0.data(), _p1.data(), _p2.data(), _normal.data()
			    ))
			{
				if (!children[j])
				{
					children[j] = new Octree<real>(client, maxDepth-1, targetOccupancy);
					children[j]->box = std::move(toAABox(child_node[j]));
					children[j]->calculateChildBoxes();
				}
				children[j]->insertTriangle(adds[i], _p0, _p1, _p2, _normal);
			}
		}
	}
}

template <class flt_type>
void Octree<flt_type>::build(const IdArray &triangles, const AABox<real> &meshExtend)
{
	// Check that not already built
	if (box.valid())
		throw std::move(
			std::runtime_error("[Octree] Cannot call build() on already built tree!")
		);

	// Determine root node size
	Vec3 halfsize(std::move(meshExtend.extent()));
	if (halfsize.x() >= halfsize.y() && halfsize.x() >= halfsize.z())
		halfsize.z() = halfsize.y() = halfsize.x() = real(0.5)*halfsize.x();
	else if (halfsize.y() >= halfsize.x() && halfsize.y() >= halfsize.z())
		halfsize.z() = halfsize.y() = halfsize.x() = real(0.5)*halfsize.y();
	else
		halfsize.z() = halfsize.y() = halfsize.x() = real(0.5)*halfsize.z();

	// Determine root node axis-aligned box
	Vec3 center = std::move(meshExtend.centroid());
	box.pmin = center - halfsize;
	box.pmax = center + halfsize;

	// Sanity check
	if (!box.valid())
		throw std::move(
			std::runtime_error("[Octree] Mesh bounding box must not be degenerate!")
		);

	// Build up child boxes
	calculateChildBoxes();

	// Add triangles
	for (unsigned i=0; i<triangles.size(); i++)
	{
		Vec3 p0, p1, p2, normal;
		client.queryTriangle(&p0, &p1, &p2, &normal, triangles[i]);
		insertTriangle(triangles[i], p0, p1, p2, normal);
	}
}

template <class flt_type>
bool Octree<flt_type>::queryRay (RayIdMap *IDs, const Ray<real> &ray)
	const
{
	// Common init
	SAABox<real> node = std::move(toSAABox(box));
	Vec3 orig(
		ray.origin.x() / ray.origin.w(),
		ray.origin.y() / ray.origin.w(),
		ray.origin.z() / ray.origin.w()
	), isect;
	real t = Constants<real>::inf;

	// Check if ray even hits this sub tree
	if (_<real>::boxRayIntersection(isect.data(), &t, node, orig.data(), ray.direction.data()))
	{
		// Convenience shorthand
		RayIdMap &ids = *IDs;

		// Check if leaf node
		if (!occupants.empty())
		{
			RayIdMap::iterator at = ids.find(t);
			if (at != ids.end())
			{
				at->second.insert(
					at->second.end(), occupants.begin(), occupants.end()
				);
			}
			else
				ids.insert(std::move(std::make_pair(t, occupants)));
			return true;
		}

		// Query child nodes
		bool leaf_hit = false;
		for (unsigned i=0; i<8; i++)
			if (children[i])
				leaf_hit = children[i]->queryRay(IDs, ray) || leaf_hit;
		return leaf_hit;
	}

	return false;
}

template <class flt_type>
void Octree<flt_type>::clear (void)
{
	if (occupants.empty())
	{
		if (children[0]) delete children[0];
		if (children[1]) delete children[1];
		if (children[2]) delete children[2];
		if (children[3]) delete children[3];
		if (children[4]) delete children[4];
		if (children[5]) delete children[5];
		if (children[6]) delete children[6];
		if (children[7]) delete children[7];
	}
	else
		occupants.clear();
}

template <class flt_type>
bool Octree<flt_type>::isEmpty (void)
{
	return
		// In case of non-leaf node
		!(children[0] || children[1] || children[2] || children[3] || children[4] ||
		  children[5] || children[6] || children[7] ||
		// In case of leaf
		!occupants.empty());
}

template <class flt_type>
bool Octree<flt_type>::isNotEmpty(void)
{
	return
		// In case of non-leaf node
		(children[0] || children[1] || children[2] || children[3] || children[4] ||
		 children[5] || children[6] || children[7] ||
		// In case of leaf
		occupants.empty());
}



//////
//
// Explicit template instantiations
//

// Only floating point variants are intended
template APISPEC IOctreeClient<float>;
template APISPEC IOctreeClient<double>;
template APISPEC Octree<float>;
template APISPEC Octree<double>;
