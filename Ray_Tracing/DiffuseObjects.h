#ifndef DIFFUSEOBJECTS_H
#define DIFFUSEOBJECTS_H

#include "primitives.h"
#include "SpecularObjects.h"
#include "Functions.h"
#include <algorithm>
#include "defines.h"

/*
	DiffuseObjects are objects that only have diffuse reflection.
	The model of diffuse reflection considered was the Lambertian one. 
	It considers that the Intensity is proportional to the dot product of the incoming ray and the reflex ray.

	Moreover, all these objects inherit from their respective Specular pairs.
*/


/*
	DiffusePlane:
	Defines a plane that only has Diffuse reflection
	All planes are defined by a normal vector and a point.

*/
class DiffusePlane : virtual public SpecularPlane
{
public:
	//Defines a plane with all his variables set to zero
	DiffusePlane();
	//Defines a plane passing by aPoint, and with a given aNormal vector. Also takes anIntensity in BGR order.
	DiffusePlane(Eigen::Vector3f aPoint,Eigen::Vector3f aNormal, Eigen::Vector3f anIntensity);
	//Defines a plane passing by a point (xP,yP,zP), and with a given normal vector (xN,yN,zN). Also takes anIntensity (Bi,Gi,Ri) in BGR order.
	DiffusePlane(float xP,float yP,float zP, float xN,float yN, float zN, float Bi, float Gi, float Ri);

	//Reflection control
	//Return what will be the reflection caused by an incident IRay
	//In mirror like reflection there is only one possible reflect ray, but in other forms more can occur,
	//so it returns a vector of IRays. Moreover, if the incident IRay doesn't hit the object, a empty container is returned.
	//As this object is specular id doesn't need information about the scene that he is into.
	virtual vector<std::shared_ptr<IRay>> Reflection(std::shared_ptr<IRay> aRay, Scene* aScene = NULL); //IF aCene = NULL -> Object is considered a Specular one
	virtual ~DiffusePlane() {}
};

/*
	DiffusePlane:
	Defines a sphere that only has Diffuse reflection
	All spheres are defined by a center and a radius.
*/

class DiffuseSphere : virtual public SpecularSphere
{
public:
	//Defines a sphere with all his variables set to zero
	DiffuseSphere();
	//Defines a sphere with aCenter and aRadius. Also takes an anIntensity in BGR order.
	DiffuseSphere(Eigen::Vector3f aCenter,float aRadius, Eigen::Vector3f anIntensity);
	//Defines a sphere with a center(xC,yC,zC) and aRadius. Also takes anInentisty(Bi,Gi,Ri) in BGR order.
	DiffuseSphere(float xC,float yC,float zC, float aRadius, float Bi, float Gi, float Ri);

	//Reflection control
	//Return what will be the reflection caused by an incident IRay
	//In mirror like reflection there is only one possible reflect ray, but in other forms more can occour,
	//so it returns a vector of IRays. Moreover, if the incident IRay doesn't hit the object, a empty container is returned.
	//As this object is specular id doesn't need information about the scene that he is into.
	virtual vector<std::shared_ptr<IRay>> Reflection(std::shared_ptr<IRay> aRay, Scene* aScene = NULL);

	virtual ~DiffuseSphere() {}
};
#endif // !DIFFUSEOBJECTS_H
