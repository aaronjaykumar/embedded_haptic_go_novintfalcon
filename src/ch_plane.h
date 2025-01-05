#ifndef CH_PLANE_h
#define CH_PLANE_h

// CHAI3D includes
#include "chai3d.h"

using namespace chai3d;
// class for 3D planes
class ch_plane
{
public:
	// constructor of plane
	ch_plane() {};

	// destructor of plane
	virtual ~ch_plane() {};

	// compute the plane which contains the given triangle: plane represented as ax+by+cz+d = 0
	// first compute the plane normal
	void ch_computePlane(const unsigned int TirangleIndex, cMultiMesh* obj);

	// return plane normal
	inline cVector3d ch_getPlaneNormal() const { return normal; }

	// return distance of plane from origin
	inline double ch_getPlaneD() const { return d; }

protected:
	// plane normal
	cVector3d normal;

	// distance from the origin
	double d;
};


#endif