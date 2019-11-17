
#ifndef __POLY_MESH_H__
#define __POLY_MESH_H__


//////
//
// Includes
//

// API config
#include "_config.h"

// Local includes
#include "EigenTools.h"
#include "Octree.h"
#include "IScannableObject.h"



//////
//
// Classes
//

/**
 * @brief Class representing a @link IScannableObject scannable @endlink polygonal mesh.
 */
template <class flt_type>
class APISPEC PolyMesh : public IScannableObject<flt_type>,
                         public IOctreeClient<flt_type>
{

public:

	////
	// Types

	/** @brief Real number type. */
	typename typedef IScannableObject<flt_type>::real real;

	/** @brief 2D vector type. */
	typename typedef IScannableObject<real>::Vec2 Vec2;

	/** @brief 3D vector type. */
	typename typedef IScannableObject<real>::Vec3 Vec3;

	/** @brief 3D Homogenous vector type. */
	typename typedef IScannableObject<real>::Vec4 Vec4;

	/** @brief 3D homogenous matrix type. */
	typename typedef IScannableObject<real>::Mat4 Mat4;

	/** @brief Indexed 3-point simplex (i.e. triangle) type. */
	typename typedef EigenTypes<unsigned>::Vec3 Triangle;


protected:

	////
	// Data members

	/** @brief Mesh vertices. */
	std::vector<Vec4> verts;

	/** @brief Vertex normals. */
	std::vector<Vec3> normals;

	/** @brief Triangle list. */
	std::vector<Triangle> tris;

	/** @brief Triangle normals. */
	std::vector<Vec3> triNormals;

	/** @brief Tight-fitting axis-aligned bounding box of the mesh. */
	AABox<real> aabbox;

	/** @brief Octree instance for accelerated intersection testing. */
	Octree<real> octree;

	/**
	 * @brief
	 *		Affine transformation modeling the object's position and orientation in
	 *		world space.
	 */
	Mat4 worldTransform;

	/** @brief The inverse of @ref #worldTransform . */
	Mat4 worldTransformInv;

	/** @brief The transpose of @ref #worldTransformInv . */
	Mat4 worldTransformInvT;


public:

	////
	// Object construction / destruction

	/**
	 * @brief
	 *		Default constructor. If so desired, builds the object from the provided .obj
	 *		mesh file.
	 */
	PolyMesh(const std::string &objfile="", bool moveToCoM=true);

	/** @brief The destructor. */
	virtual ~PolyMesh();


	////
	// Methods

	/**
	 * @brief
	 *		Implements @link Octree octree @endlink interface @ref
	 *		IOctreeClient::queryTriangle .
	 */
	void queryTriangle(Vec3 *p0, Vec3 *p1, Vec3 *p2, Vec3 *normal, unsigned id) const;

	/**
	 * @brief
	 *		Tests for intersection with the given ray and returns the ray parameter t at
	 *		the intersection point (if any) that is closest to the ray origin.
	 */
	virtual Intesection<real> intersectRay (const Ray<real> &ray) const;

	/** @brief Loads gemoetry from an .obj file. */
	virtual void loadMesh_obj(const std::string &filename, bool moveToCoM=true);

	/**
	 * @brief
	 *		Generates a triangle mesh from the absolute value function
	 *		z = c1*|x| + c2*|y| within the given bounds xmin,ymin and xmax,ymax with the
	 *		desired amount of steps.
	 */
	virtual void genMesh_absValF(real c1, real c2, const Vec2 &pmin, const Vec2 &pmax,
	                             unsigned steps, bool moveToCoM = true);

	/**
	 * @brief
	 *		Transforms the model with the given ground truth transformation and writes
	 *		the resulting mesh to an .obj file.
	 */
	virtual void writeTransformedGroundTruthModel_obj(const std::string &filename,
	                                                  const Mat4 &groundTruthTrans) const;
	/**
	 * @brief
	 *		Transforms the model vertices with the given ground truth transformation and
	 *		writes the resulting point cloud to a .ply file.
	 */
	virtual void writeTransformedGroundTruthCloud_ply(const std::string &filename,
	                                                  const Mat4 &groundTruthTrans) const;


	////
	// Setters / getters

	/** @brief Sets @ref #worldTransform . */
	virtual void setWorldTransform (const Mat4 &mat);
	/** @brief Returns current contents of @ref #worldTransform . */
	virtual Mat4 getWorldTransform (void) const;
	/** @brief Writes current contents of @ref #worldTransform to matrix @c out . */
	virtual void getWorldTransform (Mat4 *out) const;
};



#endif // ifndef __POLY_MESH_H__
