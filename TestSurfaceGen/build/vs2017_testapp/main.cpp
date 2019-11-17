
//////
//
// Includes
//

// C++ STL
#include <vector>
#include <fstream>

// Local includes
#include "Ray.h"
#include "Projector.h"
#include "HyperbolicParaboloid.h"



//////
//
// Local types
//

// Desired floating point precision
typedef float real;

// Eigen type shortcuts
typedef EigenTypes<real>::Vec3 Vec3;
typedef EigenTypes<real>::Vec4 Vec4;



//////
//
// Module-wide constants
//

namespace {
	constexpr unsigned short RES_X = 640;
	constexpr unsigned short RES_Y = 480;
}



//////
//
// Application entry point
//

int main (int argc, char* argv[])
{
	// Set up test projector
	Projector<real> projector(RES_X, RES_Y);
	projector.applyLookAt(Vec4(0, 0, 1.5, 1), Vec4(0, 0, 0, 1), Vec3(0, 1, 0));
	projector.applyFovNearFar(90, real(0.01), 1000);

	// Cast test ray
	Ray<real> testRay = projector.castRay(RES_X/2, RES_Y/2);

	// Scan saddle function
	HyperbolicParaboloid<real> saddle(0.5, -0.5, 1, 1, 0, 0);
	std::vector<Vec3> pixels; pixels.reserve(RES_X*RES_Y);
	for (unsigned pixel=0; pixel<RES_X*RES_Y; pixel++)
	{
		// Shortcuts for current pixel coordinates
		const short x = pixel % RES_X,
		            y = pixel / RES_X;

		// Cast sampling ray from current pixel
		Ray<real> ray = projector.castRay(x, y);

		// Intersect with saddle function
		real t;
		if (saddle.intersectRay(&t, ray))
		{
			Vec3 point = ray.point3At(t);
			if (std::isnan(point.x()) || std::isnan(point.y()) || std::isnan(point.z()))
				t = 0;
			if (std::isinf(point.x()) || std::isinf(point.y()) || std::isinf(point.z()))
				t = 0;
			else
				pixels.push_back(point);
		}
	}

	// Write sampling to file
	std::fstream file("out.txt", std::fstream::out);
	for (unsigned pixel=0; pixel<pixels.size(); pixel++)
	{
		/*const short x = pixel % RES_X,
		            y = pixel / RES_X;*/
		file << /*x << " " << y << " " << */pixels[pixel].x() << " " << pixels[pixel].y() << " " << pixels[pixel].z() << std::endl;
	}
	file.close();

	return 0;
}
