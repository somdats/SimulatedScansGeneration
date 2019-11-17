
//////
//
// Includes
//

// C++ STL
#include <cmath>
#include <limits>
#include <vector>
#include <unordered_map>
#include <fstream>
#include <sstream>

// Local includes
#include "HitTest/vector.h"
#include "HitTest/hittest.h"
#include "Ray.h"

// Implemented header
#include "PolyMesh.h"



//////
//
// Helper functions
//

template <class flt_type>
typename PolyMesh<flt_type>::Vec3 toVec3 (const typename PolyMesh<flt_type>::Vec4 &vec_h)
{
	return
		std::move(PolyMesh<flt_type>::Vec3(
			vec_h.x()/vec_h.w(), vec_h.y()/vec_h.w(), vec_h.z()/vec_h.w()
		));
};

template <class flt_type>
void tesselateRectangleSurface(
	std::vector<typename PolyMesh<flt_type>::Vec4> *verts,
	std::vector<typename PolyMesh<flt_type>::Triangle> *tris,
	std::vector<unsigned> *triIDs,
	const typename PolyMesh<flt_type>::Vec2 &pmin,
	const typename PolyMesh<flt_type>::Vec2 &pmax,
	unsigned patchesPerAxis
)
{
	// Convenience shorthands
	typedef typename PolyMesh<flt_type>::real real;
	typedef typename PolyMesh<flt_type>::Vec4 Vec4;
	typedef typename PolyMesh<flt_type>::Triangle Triangle;
	std::vector<Vec4> &v = *verts;
	std::vector<Triangle> &t = *tris;

	// Grid dimensions
	unsigned vcount1d = patchesPerAxis + 1, vcount = vcount1d * vcount1d,
	         tcount = patchesPerAxis * patchesPerAxis * 2;

	// Make space
	v.resize(vcount); t.resize(tcount); triIDs->resize(tcount);

	// Vertex positions and normals
	for (unsigned y=0; y<vcount1d; y++)
	{
		real lerp_y = real(y) / real(vcount1d);
		for (unsigned x=0; x<vcount1d; x++)
		{
			// Prepare current sample
			real lerp_x = real(x) / real(vcount1d);
			Vec4 &curVert = v[y*vcount1d + x];
			curVert.x() = pmin.x() * (1 - lerp_x) + pmax.x() * lerp_x;
			curVert.y() = pmin.y() * (1 - lerp_y) + pmax.y() * lerp_y;
			curVert.z() = 0; curVert.w() = 1;
		}
	}

	// Triangles
	for (unsigned j=0; j<patchesPerAxis; j++)
	{
		for (unsigned i=0; i<patchesPerAxis; i++)
		{
			unsigned tId = (j*patchesPerAxis + i)*2;

			// Quad triangle 1                   Quad triangle 2
			t[tId].x() =  j   *vcount1d + i;     t[tId+1].x() =  j   *vcount1d + i+1;
			t[tId].y() =  j   *vcount1d + i+1;   t[tId+1].y() = (j+1)*vcount1d + i+1;
			t[tId].z() = (j+1)*vcount1d + i;     t[tId+1].z() = (j+1)*vcount1d + i;
			(*triIDs)[tId] = tId;                (*triIDs)[tId+1] = tId+1;
		}
	}
}



//////
//
// Class implementation
//

// PolyMesh
//

template <class flt_type>
PolyMesh<flt_type>::PolyMesh(const std::string &objfile, bool moveToCoM)
	: octree(*this, 8, 128)
{
	worldTransform.setIdentity();
	worldTransformInv.setIdentity();
	worldTransformInvT.setIdentity();

	// Init geometry if indicated
	if (!objfile.empty())
		loadMesh_obj(objfile, moveToCoM);
}

template <class flt_type>
PolyMesh<flt_type>::~PolyMesh() {}

template <class flt_type>
Intesection<flt_type> PolyMesh<flt_type>::intersectRay(
	const Ray<real> &ray
) const
{
	// Local helpers
	const Mat4 &transP = worldTransform,
	           &transN = worldTransformInvT;

	auto worldSpaceP = [&transP] (const Vec3 &point) -> Vec3
	{
		Vec4 ret = transP * Vec4(point.x(), point.y(), point.z(), 1);
		return std::move(Vec3(ret.x()/ret.w(), ret.y()/ret.w(), ret.z()/ret.w()));
	};
	auto worldSpaceN = [&transN] (const Vec3 &normal) -> Vec3
	{
		Vec4 ret = transN * Vec4(normal.x(), normal.y(), normal.z(), 0);
		return std::move(Vec3(ret.x(), ret.y(), ret.z()));
	};

	// Transform ray to local coordinate system
	Vec4 rayPoint(std::move(ray.point4At(1)));
	rayPoint = worldTransformInv * rayPoint;
	Ray<real> rLocal;
	rLocal.origin = worldTransformInv * ray.origin;
	rLocal.origin /= rLocal.origin.w();
	rLocal.direction.x() =
		(rayPoint.x()/rayPoint.w()) - rLocal.origin.x();
	rLocal.direction.y() =
		(rayPoint.y()/rayPoint.w()) - rLocal.origin.y();
	rLocal.direction.z() =
		(rayPoint.z()/rayPoint.w()) - rLocal.origin.z();
	rLocal.direction.normalize();

	// Intersection test
	Intesection<real> isect; isect.hit = false;
	typename Octree<real>::RayIdMap nodes;
	if (!octree.queryRay(&nodes, rLocal))
		return std::move(isect);


	for (typename Octree<real>::RayIdMap::iterator it=nodes.begin();it!=nodes.end();it++)
	{
		// Convenience shorthands
		typename Octree<real>::IdArray &trilist/*(tris.size())*/ = it->second;
		/*for (unsigned i=0; i<tris.size(); i++)
			trilist[i] = i;*/

		// Perfrom intersection tests on octree node triangles
		Vec3 orig(
			rLocal.origin.x() / rLocal.origin.w(),
			rLocal.origin.y() / rLocal.origin.w(),
			rLocal.origin.z() / rLocal.origin.w()
		);
		std::vector<real> t(trilist.size());
		for (unsigned i=0; i < trilist.size(); i++)
			t[i] = Constants<real>::inf;

		// Intersect with each triangle and keep track of hit closest to origin
		signed front = -1; isect.t = Constants<real>::inf;
		Vec3 isect_pos;
		for (unsigned i=0; i<trilist.size(); i++)
		{
			const Triangle &tri = tris[trilist[i]];
			Vec3 p0(std::move(toVec3<flt_type>(verts[tri.x()]))),
			     p1(std::move(toVec3<flt_type>(verts[tri.y()]))),
			     p2(std::move(toVec3<flt_type>(verts[tri.z()])));
			if (htest::_<real>::rayTriIntersect(
			    isect_pos.data(), (const real*)orig.data(), rLocal.direction.data(), p0.data(),
			    p1.data(), p2.data(), &t[i]
			))
			{
				if (t[i] < isect.t)
				{
					isect.hit = true;
					isect.t = t[i];
					front = i;
					isect.pos = std::move(worldSpaceP(isect_pos));
					isect.normal = std::move(worldSpaceN(triNormals[trilist[i]]));
				}
			}
		}

		if (isect.hit)
			// No need to consider further octree nodes
			break;
	}

	// Report results
	return std::move(isect);
}

template <class flt_type>
void PolyMesh<flt_type>::queryTriangle(Vec3 *p0, Vec3 *p1, Vec3 *p2, Vec3 *normal,
                                       unsigned id) const
{
	*p0 = std::move(toVec3<real>(verts[tris[id].x()]));
	*p1 = std::move(toVec3<real>(verts[tris[id].y()]));
	*p2 = std::move(toVec3<real>(verts[tris[id].z()]));
	*normal = triNormals[id];
}

template <class flt_type>
void PolyMesh<flt_type>::loadMesh_obj(const std::string &filename, bool moveToCoM)
{
	// Prepare additional octree data
	aabbox.invalidate();
	std::vector<unsigned> triIDs;

	// Prepare vertex -> triangles map
	std::unordered_map<unsigned, std::vector<unsigned> > vertTris;

	// Locate file
	std::fstream file(filename, std::fstream::in);
	if (!file)
		throw std::move(
			std::runtime_error("[PolyMesh] Specified .obj file could not be opened!")
		);

	// Parse file
	char dummy; const Triangle idOffset(0, 1, 1);
	while (!file.eof())
	{
		// Read in whole line
		std::string line; std::getline(file, line);

		// Skip everything except vertex and face declarations
		if (line[0] == 'v' || line[0] == 'V')
		{
			// Parse vertex
			std::stringstream line(line);

			// Read in vertex
			Vec4 vert;
			line >> dummy >> vert.x() >> vert.y() >> vert.z(); vert.w() = 1;

			// Add vertex
			aabbox.inludePoint(std::move(toVec3<real>(vert)));
			verts.push_back(std::move(vert));
		}
		else if (line[0] == 'f' || line[0] == 'F')
		{
			// Parse face
			std::istringstream line(line);

			// Read in common vertex id
			unsigned p0;
			line >> dummy >> p0;

			// Read in first face triangle
			Triangle newTri; newTri.x() = p0 - 1;
			line >> newTri.y() >> newTri.z();
			newTri -= idOffset; // Convert to 0-based indices
			triIDs.push_back((unsigned)tris.size());
			tris.push_back(newTri);
			triNormals.push_back(std::move(
				(   toVec3<real>(verts[newTri.y()])
				  - toVec3<real>(verts[newTri.x()])).cross
				(   toVec3<real>(verts[newTri.z()])
				  - toVec3<real>(verts[newTri.y()])).normalized()
			));
			unsigned triID = (unsigned)tris.size()-1;
			vertTris[newTri.x()].push_back(triID);
			vertTris[newTri.y()].push_back(triID);
			vertTris[newTri.z()].push_back(triID);

			// Loop through remaining face triangles (assumes planar, convex polygons)
			while (!line.eof())
			{
				newTri.y() = tris[tris.size()-1].z();
				line >> newTri.z();
				newTri -= idOffset; // Convert to 0-based indices
				triIDs.push_back((unsigned)tris.size());
				tris.push_back(newTri);
				triNormals.push_back(std::move(
					(   toVec3<real>(verts[newTri.y()])
					  - toVec3<real>(verts[newTri.x()])).cross
					(   toVec3<real>(verts[newTri.z()])
					  - toVec3<real>(verts[newTri.y()])).normalized()
				));
				triID = (unsigned)tris.size()-1;
				vertTris[newTri.x()].push_back(triID);
				vertTris[newTri.y()].push_back(triID);
				vertTris[newTri.z()].push_back(triID);
			}
		}
	}

	// Calculate vertex normals
	normals.resize(verts.size());
	for (unsigned i=0; i<verts.size(); i++)
	{
		const std::vector<unsigned> &triIDs = vertTris[i];
		Vec3 newNormal; newNormal.setZero();
		for (unsigned n=0; n<triIDs.size(); n++)
			newNormal += triNormals[triIDs[n]];
		newNormal /= real(triIDs.size()); newNormal.normalize();
		normals[i] = std::move(newNormal);
	}

	// Center model around vertex barycenter if requested
	if (moveToCoM)
	{
		Vec3 CoM; CoM.setZero();
		for (unsigned i=0; i<verts.size(); i++)
			CoM += std::move(toVec3<real>(verts[i]));
		CoM /= real(verts.size());
		aabbox.invalidate();
		for (unsigned i=0; i<verts.size(); i++)
		{
			verts[i].x() -= CoM.x();
			verts[i].y() -= CoM.y();
			verts[i].z() -= CoM.z();
			aabbox.inludePoint(std::move(toVec3<real>(verts[i])));
		}
	}

	// Build octree
	octree.build(triIDs, aabbox);
}

template <class flt_type>
void PolyMesh<flt_type>::genMesh_absValF(
	real c1, real c2, const Vec2 &pmin,const Vec2 &pmax, unsigned steps, bool moveToCoM
)
{
	// Prepare additional octree data
	aabbox.invalidate();
	std::vector<unsigned> triIDs;

	// Prepare vertex -> triangles map
	std::unordered_map<unsigned, std::vector<unsigned> > vertTris;

	// Prepare vertex grid
	tesselateRectangleSurface<real>(&verts, &tris, &triIDs, pmin, pmax, steps);

	// Evaluate absolute value function
	for (unsigned i=0; i<verts.size(); i++)
	{
		verts[i].z() = c1 * std::abs(verts[i].x()) + c2* std::abs(verts[i].y());
		aabbox.inludePoint(std::move(toVec3<real>(verts[i])));
	}

	// Calculate triangle normals
	triNormals.resize(tris.size());
	for (unsigned i=0; i<tris.size(); i++)
	{
		triNormals[i] = std::move(
			(   toVec3<real>(verts[tris[i].y()])
			  - toVec3<real>(verts[tris[i].x()])).cross
			(   toVec3<real>(verts[tris[i].z()])
			  - toVec3<real>(verts[tris[i].y()])).normalized()
		);
		vertTris[tris[i].x()].push_back(i);
		vertTris[tris[i].y()].push_back(i);
		vertTris[tris[i].z()].push_back(i);
	}

	// Calculate vertex normals
	normals.resize(verts.size());
	for (unsigned i=0; i<verts.size(); i++)
	{
		const std::vector<unsigned> &triIDs = vertTris[i];
		Vec3 newNormal; newNormal.setZero();
		for (unsigned n=0; n<triIDs.size(); n++)
			newNormal += triNormals[triIDs[n]];
		newNormal /= real(triIDs.size()); newNormal.normalize();
		normals[i] = std::move(newNormal);
	}

	// Center model around vertex barycenter if requested
	if (moveToCoM)
	{
		Vec3 CoM; CoM.setZero();
		for (unsigned i = 0; i<verts.size(); i++)
			CoM += std::move(toVec3<real>(verts[i]));
		CoM /= real(verts.size());
		aabbox.invalidate();
		for (unsigned i = 0; i<verts.size(); i++)
		{
			verts[i].x() -= CoM.x();
			verts[i].y() -= CoM.y();
			verts[i].z() -= CoM.z();
			aabbox.inludePoint(std::move(toVec3<real>(verts[i])));
		}
	}

	// Build octree
	octree.build(triIDs, aabbox);
}

template<class flt_type>
void PolyMesh<flt_type>::writeTransformedGroundTruthModel_obj(
	const std::string &filename, const Mat4 &groundTruthTrans
) const
{
	// Transform vertices
	std::vector<Vec4> transVerts(verts.size());
	for (unsigned i=0; i<transVerts.size(); i++)
	{
		transVerts[i] = std::move(groundTruthTrans * verts[i]);
		transVerts[i] /= transVerts[i].w();
	}
	// Transform normals
	Mat4 groundTruthTransIT = std::move(groundTruthTrans.inverse().transpose());
	std::vector<Vec4> transNrmls(triNormals.size());
	for (unsigned i = 0; i<transNrmls.size(); i++)
	{
		Vec4 normal(triNormals[i].x(), triNormals[i].y(), triNormals[i].z(), 0);
		transNrmls[i] = std::move(groundTruthTransIT * normal);
	}

	// Write .obj
	std::fstream objfile(filename, std::fstream::out);

	// Write vertices
	for (unsigned i=0; i<transVerts.size(); i++)
		objfile << "v " << transVerts[i].x() << ' ' << transVerts[i].y() << ' '
		                << transVerts[i].z() << std::endl;

	// Write normals
	for (unsigned i=0; i<transNrmls.size(); i++)
		objfile << "n " << transNrmls[i].x() << ' ' << transNrmls[i].y() << ' '
		                << transNrmls[i].z() << std::endl;
	// Write faces
	// XXX: Super awkward hack. Find out why normals get flipped in non-identity
	//      transform and fix!!!
	if (groundTruthTrans.isIdentity())
		for (unsigned i=1; i<=tris.size(); i++)
			objfile << "f " << tris[i-1].x()+1 << "//" << i << ' '
			                << tris[i-1].y()+1 << "//" << i << ' '
			                << tris[i-1].z()+1 << "//" << i << std::endl;
	else
		for (unsigned i=1; i<=tris.size(); i++)
			objfile << "f " << tris[i-1].z()+1 << "//" << i << ' '
			                << tris[i-1].y()+1 << "//" << i << ' '
			                << tris[i-1].x()+1 << "//" << i << std::endl;
}

template<class flt_type>
void PolyMesh<flt_type>::writeTransformedGroundTruthCloud_ply(
	const std::string &filename, const Mat4 &groundTruthTrans
) const
{
	// Transform vertices
	std::vector<Vec4> transVerts(verts.size());
	for (unsigned i=0; i<transVerts.size(); i++)
	{
		transVerts[i] = std::move(groundTruthTrans * verts[i]);
		transVerts[i] /= transVerts[i].w();
	}
	// Transform normals
	Mat4 groundTruthTransIT = std::move(groundTruthTrans.inverse().transpose());
	std::vector<Vec4> transNrmls(normals.size());
	for (unsigned i=0; i<transNrmls.size(); i++)
	{
		Vec4 normal(normals[i].x(), normals[i].y(), normals[i].z(), 0);
		transNrmls[i] = std::move(groundTruthTransIT * normal);
	}

	// Write .ply
	std::fstream plyfile(filename, std::fstream::out);

	// *.ply header
	plyfile << "ply" << std::endl
	        << "format ascii 1.0" << std::endl
	        << "element vertex " << transVerts.size() << std::endl
	        << "property float x" << std::endl
	        << "property float y" << std::endl
	        << "property float z" << std::endl
	        << "property float nx" << std::endl
	        << "property float ny" << std::endl
	        << "property float nz" << std::endl
	        << "end_header" << std::endl;

	// *.ply specific floating point settings
	plyfile.setf(std::ios::fixed); plyfile.precision(6);

	// Write point data
	for (unsigned i=0; i<transVerts.size(); i++)
	{
		plyfile << transVerts[i].x() <<" "<< transVerts[i].y() << " "
		        << transVerts[i].z() <<" "<< transNrmls[i].x() << " "
		        << transNrmls[i].y() <<" "<< transNrmls[i].z() << std::endl;
	}
}

template <class flt_type>
void PolyMesh<flt_type>::setWorldTransform(const Mat4 &mat)
{
	bool invertible;
	mat.computeInverseWithCheck(worldTransformInv, invertible);
	if (invertible)
	{
		worldTransform = mat;
		worldTransformInvT = worldTransformInv.transpose();
	}
	else
		throw std::move(
			std::runtime_error("[PolyMesh] Specified world coordinate system"
			                   " transformation matrix is not invertible!")
		);
}
template <class flt_type>
typename EigenTypes<flt_type>::Mat4 PolyMesh<flt_type>::
	getWorldTransform(void) const
{
	return worldTransform;
}
template <class flt_type>
void PolyMesh<flt_type>::getWorldTransform(Mat4 *out) const
{
	*out = worldTransform;
}



//////
//
// Explicit template instantiations
//

// Only floating point variants are intended
template APISPEC PolyMesh<float>;
template APISPEC PolyMesh<double>;
