#pragma once

#ifndef primitives_H
#define primitives_H

////#define DEBUG
//#ifdef DEBUG
//	#include <vld.h> 
//#endif

#include "opencv2/opencv.hpp"
#include <vector>
#include <stdlib.h>
#include <Eigen/Dense>
#include "Functions.h"
#include <memory>
#include "defines.h"

//#define REFLEX_COUNTER 10

using namespace std;


// Classes prototypes
class SceneObj;			// Interface of objects in scene
class Scene;			// Scene maneger
class IRay;				// Interface of Ray
class Ray;				// Implementing Ray, following inteface's restrictions
class Ilight;			// Interface of Ray
class light;			// Implamenting light, following interface's restrictions
class CollisionObject;	// Handle collision's information
class IScreen;			// Interface of screen
class Screen;			// Implementing screen, following interface's restrictions
class IPhysicalObject;	// Interface of physical objects, such as spheres and plans 


// SceneObj : Interface of every component of the scene
//			  Imposes a center position to localize the object on the scene, if the corresponding Get & Set methods
//			  and a print method, which gives the essential information of the object
class SceneObj
{
protected: 
	Eigen::Vector3f center;

public:
	// SetCenter methods
	virtual void SetCenter(float x,float y,float z) = 0;
	virtual void SetCenter(Eigen::Vector3f aCenter) = 0;

	// GetCenter method
	virtual Eigen::Vector3f GetCenter() = 0;
	
	// Print method
	virtual void Print() = 0;
	
	virtual ~SceneObj() {}
	//virtual ~SceneObj();
};

// IScreen : Interface of screen-classes
//			 Imposes the methods used in the tracer class (Getpixel, Getdiscretv) to run through and render the scene 
//			 Also defines methods to handle its position in the scene
class IScreen : virtual public SceneObj
{
public:
	
	//WARNING: Assumes the Screen was already instantiated
	//Changing the center, maintains directions and size 
	virtual void SetCenter(float x,float y,float z) = 0;
	virtual void SetCenter(Eigen::Vector3f aCenter) = 0;

	virtual Eigen::Vector3f GetCenter() = 0;
	virtual Eigen::Vector3f GetStartCorner() = 0;

	//Resets v1,v2 that stores the plane's "dimension"
	virtual void SetSize(float x1,float y1,float z1,float x2,float y2,float z2) = 0;
	virtual void SetSize(Eigen::Vector3f av1, Eigen::Vector3f av2) = 0;

	//Returns the screen's size {norm(v1),norm(v2)}
	virtual Eigen::Vector2f GetSize() = 0;
	virtual void Print() = 0;

	//Set the number of pixels that are used to "render" the screen
	virtual void SetPixel(Eigen::Vector2i aPixelDim) = 0;
	virtual Eigen::Vector2i GetPixel() = 0;

	//Returns the distance between each image pixel, which is the elementary pace of renderization
	virtual Eigen::Vector3f GetDiscretv1() = 0;
	virtual Eigen::Vector3f GetDiscretv2() = 0;	

	virtual ~IScreen() {}
protected: 
	virtual void UpCorner() = 0;
	//virtual ~IScreen();
};

// CollisionObject	: Class used to defines whether a collision has happened or not
// and saving the point of collision for futher use
class CollisionObject
{
private:
	// Informs exitance of collision
	bool DoItCollide ;
	// Informs position of collision 
	Eigen::Vector3f position;
public:
	// Constructors
	CollisionObject();
	CollisionObject(float x,float y, float z);
	CollisionObject(Eigen::Vector3f aColisionPosition);
	
	// Get methods
	bool IsThereCollision();
	Eigen::Vector3f GetCollisionPosition();
	
	// Print methods
	void Print();

	//Destructor
	virtual ~CollisionObject() {}
};

class Screen : virtual public IScreen
{
protected:
	// Position of screen center and its corner
	Eigen::Vector3f corner,center;
	// 
	Eigen::Vector3f v1,v2,discretv1,discretv2;
	Eigen::Vector2i pixelDim;
public:
	Screen();
	Screen(Eigen::Vector3f aCenter,Eigen::Vector3f av1, Eigen::Vector3f av2, Eigen::Vector2i apixelDim);
	Screen(float xC,float yC,float zC, float x_v1, float y_v1,float z_v1,float x_v2, float y_v2,float z_v2, int w_Pixel, int h_Pixel);

	//WARNING: Assumes the Screen was already instantiated
	//Changing the center, maintains directions and size 
	virtual void SetCenter(float x,float y,float z);
	virtual void SetCenter(Eigen::Vector3f aCenter);

	virtual Eigen::Vector3f GetCenter();
	//WARNING: Assumes that v1 and v2 are already defined
	virtual Eigen::Vector3f GetStartCorner();

	//Resets v1,v2 that stores the plane's "dimension"
	virtual void SetSize(float x_v1, float y_v1,float z_v1,float x_v2, float y_v2,float z_v2);
	virtual void SetSize(Eigen::Vector3f av1, Eigen::Vector3f av2);

	//Returns the screen's size {norm(v1),norm(v2)}
	virtual Eigen::Vector2f GetSize();
	virtual void Print();

	//Set the number of pixels that are used to "render" the screen
	virtual void SetPixel(Eigen::Vector2i aPixelDim);
	virtual Eigen::Vector2i GetPixel();

	//Returns the distance between each image pixel, which is the elementary pace of renderization
	virtual Eigen::Vector3f GetDiscretv1();
	virtual Eigen::Vector3f GetDiscretv2();

	virtual ~Screen() {}

protected :
		virtual void UpCorner();
		virtual void UpDisc();
	//virtual ~Screen() {}
};



// Ilight : interface that deals with position and intensity of the light
class Ilight : virtual public SceneObj
{
protected: 
	//variable
	Eigen::Vector3f intensity;
public:
	// Set methods
		// Set intensity
	virtual void SetIntensity(void) = 0;
	virtual void SetIntensity(Eigen::Vector3f intensity_value) = 0;	
	virtual void SetIntensity(float) = 0;
	virtual void SetIntensity(float value_b, float value_g, float value_r) = 0;
		// Set center
	virtual void SetCenter() = 0;
	virtual void SetCenter(float x,float y, float z) = 0;
	virtual void SetCenter(float x) = 0;
	virtual void SetCenter(Eigen::Vector3f) = 0;
	
	// Get methods
	virtual Eigen::Vector3f GetCenter() = 0;
	virtual Eigen::Vector3f GetIntensity() = 0;

	// Print method
	virtual void Print() = 0;

	// Detects wheather a collison between a ray object and a light has happened
	virtual CollisionObject Collision(std::shared_ptr<IRay> aRay ) = 0;
	
	// Destructor
	virtual ~Ilight() {}

}; 


// light : Ponctual ligh. Interacts with rays determing what colors would be intensified or not 
class light : virtual public Ilight
{

public:
	// Construtors
	light();
	light(Eigen::Vector3f center_value, float value_b, float value_g,float value_r);
	light(Eigen::Vector3f center_value, Eigen::Vector3f intensity_value);
	light(Eigen::Vector3f center_value);
	
	// Set methods
	virtual void SetIntensity(void);
	virtual void SetIntensity(Eigen::Vector3f intensity_value);	
	virtual void SetIntensity(float);
	virtual void SetIntensity(float value_b, float value_g, float value_r);

	virtual void SetCenter();
	virtual void SetCenter(float x,float y, float z);
	virtual void SetCenter(float x);
	virtual void SetCenter(Eigen::Vector3f);
	
	// Get methods
	virtual Eigen::Vector3f GetCenter();
	virtual Eigen::Vector3f GetIntensity();

	// Print method
	virtual void Print();

	//  Detects wheather a collison between a ray object and a light has happened
	virtual CollisionObject Collision(std::shared_ptr<IRay> aRay );
	virtual ~light() {}
};
 
// IObserver : Interface of observer
class IObserver : virtual public SceneObj
{
public:
	virtual void SetCenter(float x,float y,float z) = 0;
	virtual void SetCenter(Eigen::Vector3f aCenter) = 0;
	virtual Eigen::Vector3f GetCenter() = 0;
	virtual void Print() = 0;

	virtual ~IObserver() {}
};

// Observer	: Interacts with screen and determinates the rays direction
class Observer : public IObserver
{
public:
	Observer();
	Observer(float x,float y, float z);
	Observer(Eigen::Vector3f aCenter);

	virtual void SetCenter(float x,float y,float z);
	virtual void SetCenter(Eigen::Vector3f aCenter);
	virtual Eigen::Vector3f GetCenter();
	virtual void Print();

	virtual ~Observer() {}
};
	
// IRay	: Contraints to be used in tracer engine
class IRay
{
public:
	// Set methods
	virtual void SetIntensity(void) = 0;
	virtual void SetIntensity(Eigen::Vector3f  intensity_value) = 0;	
	virtual void SetIntensity(float) = 0;
	virtual void SetIntensity(float value_b, float value_g, float value_r) = 0;

	virtual void SetDirection() = 0;
	virtual void SetDirection(float) = 0;
	virtual void SetDirection(Eigen::Vector3f) = 0;
	virtual void SetDirection(float,float,float) = 0;

	virtual	void SetReflectionlimite() = 0;
	virtual	void SetReflectionlimite(int) = 0;

	virtual void SetPoint() = 0;
	virtual void SetPoint(Eigen::Vector3f) = 0;
	virtual void SetPoint(float,float,float) = 0;

	// Get methods
	virtual Eigen::Vector3f GetIntensity() = 0;
	virtual Eigen::Vector3f GetPoint() = 0;
	virtual Eigen::Vector3f GetDirection() = 0;
	virtual int GetReflectionCounter() = 0;

	// Print method
	virtual void Print() = 0;

	// Class functions
	virtual void Setlight(int light) = 0;
	virtual int IsGoingToLight() {return -1; }

	virtual ~IRay() {}
};

// Ray  : Ray defined by its direction, point emitted, color buffer and reflection limitation 
//		: Implementation of constrains imposed by its interface
//		: Also has a reflection counter to limit the number of reflections which can tend to infinity
class Ray : virtual public IRay 
{
protected:
	//Variables
	int reflex_counter;
	Eigen::Vector3f point;
	Eigen::Vector3f direction;
	Eigen::Vector3f intensity;
	int light_dir;

public:
	//Constructors
	Ray();
	Ray(Eigen::Vector3f);
	Ray(Eigen::Vector3f,Eigen::Vector3f,Eigen::Vector3f ,int, int aLight = -1);
	Ray(float,float,float,float,float,float,int, int aLight = -1);
	Ray(float,float,float,float,float,float,float,float,float,int, int aLight = -1);

	//Set methods
	virtual void SetIntensity(void);
	virtual void SetIntensity(Eigen::Vector3f  intensity_value);	
	virtual void SetIntensity(float);
	virtual void SetIntensity(float value_b, float value_g, float value_r);
	
	virtual void SetDirection(void);
	virtual void SetDirection(float);
	virtual void SetDirection(Eigen::Vector3f);
	virtual void SetDirection(float,float,float);

	virtual	void SetReflectionlimite(void);
	virtual	void SetReflectionlimite(int);

	virtual void SetPoint();
	virtual void SetPoint(Eigen::Vector3f);
	virtual void SetPoint(float,float,float);

	//Get methods
	virtual Eigen::Vector3f GetIntensity();
	virtual Eigen::Vector3f GetPoint();
	virtual Eigen::Vector3f GetDirection();
	virtual int GetReflectionCounter();

	//Print method
	virtual void Print();
	
	// Class functions
	virtual int IsGoingToLight() {return light_dir; }
	virtual void Setlight(int aLight) {light_dir = aLight; }

	virtual ~Ray() {}
};

// IPhysicalObject : Interface that defines essential fonctions used in by the tracer
class IPhysicalObject : virtual public SceneObj
{
protected:
	//Variables
	Eigen::Vector3f colorIntensity;
	static int ObjectCounter;

public:
	//Gets and Sets
	static int getCount() {return IPhysicalObject::ObjectCounter;}
	virtual void SetCenter(float x,float y,float z) = 0;
	virtual void SetCenter(Eigen::Vector3f aCenter) = 0;
	virtual Eigen::Vector3f GetCenter() = 0;

	//Print method
	virtual void Print() = 0;

	//Color control
	//Set and get the color intensity (a.k.a. how many the light source will be attenuated 
	virtual void SetColorIntensity(float B,float G,float R) = 0;
	virtual void SetColorIntensity(Eigen::Vector3f aColorIntensisy) = 0;
	virtual Eigen::Vector3f GetColorIntensity() = 0;

	//Geometry control
	//Check if point(x,y,z) in the space is in the ?Surface? of the object
	virtual bool InGeometry(float x,float y, float z) = 0;
	virtual bool InGeometry(Eigen::Vector3f aPoint) = 0;


	//Collision Control
	//Check if from a starting point(x,y,z) in another IPhysicalObject
	//	occurs a collision at this Object.
	//virtual CollisionObject Collision(IPhysicalObject& aPhyObject, float x,float y, float z) = 0;
	//virtual CollisionObject Collision(IPhysicalObject& aPhyObject, Eigen::Vector3f aPoint) = 0;
	//Check if a Ray collides with this IPhysicalObject
	virtual CollisionObject Collision(std::shared_ptr<IRay> aRay ) = 0;
	
	//Reflection control
	//Return what will be the reflection caused by an IRay
	virtual vector<std::shared_ptr<IRay>> Reflection(std::shared_ptr<IRay> aRay, Scene* aScene = NULL) = 0;

	virtual bool IsRefractive() {return false;}
	virtual ~IPhysicalObject() {}
};


// Scene class: Manages objects, set manually or via .txt file, in scene
//			  :	Informs numbers of inserted objects
//			  : Detect collision of a ray object with objects in Scene
class Scene 
{
private:
	//Varibles
	vector <IScreen*> screen_vector;
	vector <IObserver*> obs_vector;
	vector <Ilight*> light_vector;
	vector <IPhysicalObject*> physic_vector;

public:
	//Constructor
	Scene();
	Scene(IScreen*,IObserver*);
	
	//Sets and Gets
	void SetScreen(IScreen*);
	void SetObserver(IObserver*);
	void Setlight( Ilight*);
	void SetPhysical( IPhysicalObject*);
	
	vector <IScreen*> GetScreen();
	vector <IObserver*>  GetObserver();
	vector <Ilight*> Getlight();
	vector <IPhysicalObject*> GetPhysical();
	IScreen* GetScreen(int);
	IObserver*  GetObserver(int);
	Ilight* Getlight(int);
	IPhysicalObject* GetPhysical(int);

	// Class functions
		// Return a array with the number of each object type
	int* numbers();
		// Return the number of a especific object type ( screen, Light, object or observer )
	int  numbers(int);
		// Collision funtion 
	bool Collision(std::shared_ptr<IRay> ray);
		// Auxiliary function to ReadScene which reads and provides a int vector with the parameters 
	void ReadData ( float* p, stringstream* ss);
		// Read a .txt formated file
	void ReadScene( char* input );
		
	// Print method
	void Print();

	//Destructor
	virtual ~Scene() {}
};

  // Creation of enum class used to read data from *.txt files in /test.h/ReadScene 

 enum sub_parts
 {
	 comment1 = 0, screen, observer, Light, object,space = 0, empty = 0
 };

 enum object_type
 {
	 comment2 = 0, specularPlane, specularRectangle ,specularSphere, 
	 diffusePlane, diffuseRectangle, diffuseSphere,mrplane,mrsphere
 };

 // The enumators that have 0 as value will be ignored by the ReadScene function

#endif // !primitives_H

