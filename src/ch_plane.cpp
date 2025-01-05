#include "ch_plane.h"

//------------------------------------------------------------------------------


// compute the normal and distance from origin for the plane that contains the given triangle
void ch_plane::ch_computePlane(const unsigned int TriangleIndex, cMultiMesh* obj)
{
	cVector3d v0, v1, v2;	// vertices of the triangle
	cVector3d v01, v02;		// vectors from vertex 0 to vertices 1 and 2	


	//// debug code
	//cVector3d temp_vec;
	//temp_vec.copyfrom(triangle->getVertex0()->getPos());
		
	v0 = obj->getVertexPos(obj->getMesh(0)->m_triangles->getVertexIndex0(TriangleIndex));
	v1 = obj->getVertexPos(obj->getMesh(0)->m_triangles->getVertexIndex1(TriangleIndex));
	v2 = obj->getVertexPos(obj->getMesh(0)->m_triangles->getVertexIndex2(TriangleIndex));

		v0 = cAdd(obj->getMesh(0)->getGlobalPos(), cMul(obj->getMesh(0)->getGlobalRot(), v0));
		v1 = cAdd(obj->getMesh(0)->getGlobalPos(), cMul(obj->getMesh(0)->getGlobalRot(), v1));
		v2 = cAdd(obj->getMesh(0)->getGlobalPos(), cMul(obj->getMesh(0)->getGlobalRot(), v2));

		v1.subr(v0, v01);
		v2.subr(v0, v02);

		v01.crossr(v02, normal);

		// ensure that the normal points away from the object surface and not
		// into it
		//if(!(v1.dot(normal) < 0))
		//normal.negate();


		normal.normalize();	// gives a, b and c coeffs. of the plane

		// compute coeff. d 
		d = cDot(normal, v0);
	}
