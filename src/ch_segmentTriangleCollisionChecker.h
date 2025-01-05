#ifndef CH_SEGMENTTRIANGLECOLLISIONCHECKER_H
#define CH_SEGMENTTRIANGLECOLLISIONCHECKER_H

// CH lab
// class for GO-device segment-triangle collision checking for a given object

// system includes
#include <vector>

// CHAI3D includes
#include "chai3d.h"

// local includes
#include "ch_plane.h"

using namespace chai3d;
using namespace std;

#define SMALL_NUM  0.00000001 // anything that avoids division overflow	

class ch_segmentTriangleCollisionChecker
{

	friend class ch_GOAlgorithm;

public:

	// constructor
	ch_segmentTriangleCollisionChecker(cMultiMesh* obj);

	// destructor
	virtual ~ch_segmentTriangleCollisionChecker() {};

	// check for GO-device segment-triangle collisions
	void ch_checkCollisions(const cVector3d& lastDevicePosition, const cVector3d& currentDevicePosition, cVector3d intersectionPoint);

	// called from ch_checkCollisions()
	int ch_checkSegTriangleCollision(const unsigned int TriangleIndex, const cVector3d& lastDevicePosition, const cVector3d& currentDevicePosition, cVector3d& intersectionPoint);

	// check if a given point lies inside a given triangle
	bool ch_pointInTriangle(const cVector3d& intersectionPoint, const cVector3d& vertex0, const cVector3d& vertex1, const cVector3d& vertex2);

	// check if the intersection point and the third triangle vertex lie on the same side of the side of the triangle
	// formed by the first two vertices
	bool ch_sameSide(const cVector3d& intersectionPoint, const cVector3d& third_vertex, const cVector3d& first_vertex, const cVector3d& second_vertex);

	// highlight the collided triangles
	void ch_highlightTriangles();

	// 'un'-highlight the collided triangles after some time
	void ch_unHighlightTriangles();

	// delete last element of the vector
	inline void ch_popBack() { collidedTriangleIndex.pop_back(); }

	// clear the collidedTriangleIndex vector
	inline void ch_clearCollidedTriangleIndex() { collidedTriangleIndex.clear(); }

protected:
	// the cMesh object for which we will check collisions
	cMultiMesh *object;

	// number of triangles on the current object
	unsigned int numTrianglesObject;

	// indices of triangles collided
	vector <int> collidedTriangleIndex;

	// planes corresponding to the triangles
	vector <ch_plane> planesForTriangles;
};

#endif