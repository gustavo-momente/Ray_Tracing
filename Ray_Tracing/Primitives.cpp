

#include "SpecularObjects.h"
#include "Ray_tracer.h"
#include "DiffuseObjects.h"
#include "MultiReflectionObjects.h"
#include <fstream>
#include <sstream>
#include "primitives.h"

int IPhysicalObject::ObjectCounter = 0;

/*	Light	*/
light::light()
{ center << 0,0,0; intensity << 0,0,0; }

light::light(Eigen::Vector3f center_value, float value_b, float value_g,float value_r)
{ center = center_value; intensity << value_b,value_g,value_r; }

light::light(Eigen::Vector3f center_value, Eigen::Vector3f intensity_value)
{ center = center_value; intensity = intensity_value; }

light::light(Eigen::Vector3f center_value)
{ center = center_value; intensity <<0,0,0; }

void light::SetIntensity(void)
{ intensity <<0,0,0; }

void light::SetIntensity(Eigen::Vector3f intensity_value)
{ intensity = intensity_value;}

void light::SetIntensity(float x)
{ intensity <<x,x,x; }

void light::SetIntensity(float value_b, float value_g,float value_r)
{ intensity <<value_b,value_g,value_r; }

void light::SetCenter()
{ center << 0,0,0; }

void light::SetCenter(float x,float y, float z)
{ center << x,y,z; }

void light::SetCenter(float x)
{ center  << x,x,x; }

void light::SetCenter(Eigen::Vector3f center_value)
{center = center_value;}

Eigen::Vector3f light::GetCenter()
{ return center; } 

Eigen::Vector3f light::GetIntensity()
{ return intensity; }

void light::Print()
{ std::cout << "Center : " << GetCenter() << std::endl << "Intensity : " << GetIntensity() << std::endl; }

CollisionObject light::Collision(std::shared_ptr<IRay> aRay ) 
{
	Eigen::Vector3f v = aRay->GetDirection();
	Eigen::Vector3f v1 = center - aRay->GetPoint();
	Eigen::Vector3f res = v.cross(v1);
	
	if( CompareLight(res[0],0) && CompareLight(res[1],0) && CompareLight(res[2],0) )
	{
		float d = v1.dot(v);
		return ((d < 0) ? CollisionObject() : CollisionObject(center));
	//	if(d < 0 ) return CollisionObject();
	//	else 
	//	{
	//		std::cout << "LIGHT HIT!\n";
	//		return CollisionObject(center);
	//	}
	}
	
	return CollisionObject();
	//if ( (GetCenter() - aRay->GetPoint() ).dot(aRay->GetDirection()) == 0)
	//	return CollisionObject(GetCenter());
	//else
	//	return CollisionObject();
}


//Start:	Screen Declaration
Screen::Screen() 
{
	center << 0,0,0;
	corner << 0,0,0;
	v1 << 0,0,0;
	v2 << 0,0,0;
	pixelDim << 0,0;
	discretv1 << 0,0,0;
	discretv2 << 0,0,0;
}

Screen::Screen(Eigen::Vector3f aCenter,Eigen::Vector3f av1, Eigen::Vector3f av2, Eigen::Vector2i apixelDim) 
{
	center = aCenter;
	v1 = av1;
	v2 = av2;
	try
	{
		if(v1.dot(v2) != 0)
			throw NOT_PERMENDICULAR;
	}catch(const int& e)
	{
		if(e == NOT_PERMENDICULAR)
		{
			while(v1.dot(v2) != 0)
			{
				std::cout << "Vectors don't form a Rectangle,  <v1,v2> = " << v1.dot(v2) <<std::endl;
				std::cout << "Old vector v1: "<<v1[0] <<" ," <<v1[1] <<" ," <<v1[2] <<std::endl;
				std::cout << "Old vector v2: "<<v2[0] <<" ," <<v2[1] <<" ," <<v2[2] <<"\nPlease, insert a new v2(comma separated): ";
#ifndef DEBUG
				std::scanf("%f,%f,%f", &v2[0],&v2[1],&v2[2]);

#endif // !DEBUG
			}
		}
	}
	pixelDim = apixelDim;
	UpCorner();
	UpDisc();
}

Screen::Screen(float xC,float yC,float zC, float x_v1, float y_v1,float z_v1,float x_v2, float y_v2,float z_v2, int w_Pixel, int h_Pixel)
{
	center << xC,yC,zC;
	v1 << x_v1,y_v1,z_v1;
	v2 << x_v2,y_v2,z_v2;
	try
	{
		if(v1.dot(v2) != 0)
			throw NOT_PERMENDICULAR;
	}catch(const int& e)
	{
		if(e == NOT_PERMENDICULAR)
		{
			while(v1.dot(v2) != 0)
			{
				std::cout << "Vectors don't form a Rectangle,  <v1,v2> = " << v1.dot(v2) <<std::endl;
				std::cout << "Old vector v1: "<<v1[0] <<" ," <<v1[1] <<" ," <<v1[2] <<std::endl;
				std::cout << "Old vector v2: "<<v2[0] <<" ," <<v2[1] <<" ," <<v2[2] <<"\nPlease, insert a new v2(comma separated): ";
#ifndef DEBUG
				std::scanf("%f,%f,%f", &v2[0],&v2[1],&v2[2]);

#endif // !DEBUG
			}
		}
	}
	pixelDim << w_Pixel,h_Pixel;
	UpCorner();
	UpDisc();
}

void Screen::SetCenter(float x,float y,float z)
{
	center << x,y,z;
	UpCorner();
}

void Screen::SetCenter(Eigen::Vector3f aCenter)
{
	center = aCenter;
	UpCorner();
}

Eigen::Vector3f Screen::GetCenter() { return center; }

Eigen::Vector3f Screen::GetStartCorner() { return corner; }



void Screen::SetSize(float x_v1, float y_v1,float z_v1,float x_v2, float y_v2,float z_v2)
{
	v1 << x_v1,y_v1,z_v1;
	v2 << x_v2,y_v2,z_v2;
	try
	{
		if(v1.dot(v2) != 0)
			throw NOT_PERMENDICULAR;
	}catch(const int& e)
	{
		if(e == NOT_PERMENDICULAR)
		{
			while(v1.dot(v2) != 0)
			{
				std::cout << "Vectors don't form a Rectangle,  <v1,v2> = " << v1.dot(v2) <<std::endl;
				std::cout << "Old vector v1: "<<v1[0] <<" ," <<v1[1] <<" ," <<v1[2] <<std::endl;
				std::cout << "Old vector v2: "<<v2[0] <<" ," <<v2[1] <<" ," <<v2[2] <<"\nPlease, insert a new v2(comma separated): ";
#ifndef DEBUG
				std::scanf("%f,%f,%f", &v2[0],&v2[1],&v2[2]);

#endif // !DEBUG
			}
		}
	}
	UpCorner();
	UpDisc();
}

void Screen::SetSize(Eigen::Vector3f av1, Eigen::Vector3f av2)
{
	v1 = av1;
	v2 = av2;
	try
	{
		if(v1.dot(v2) != 0)
			throw NOT_PERMENDICULAR;
	}catch(const int& e)
	{
		if(e == NOT_PERMENDICULAR)
		{
			while(v1.dot(v2) != 0)
			{
				std::cout << "Vectors don't form a Rectangle,  <v1,v2> = " << v1.dot(v2) <<std::endl;
				std::cout << "Old vector v1: "<<v1[0] <<" ," <<v1[1] <<" ," <<v1[2] <<std::endl;
				std::cout << "Old vector v2: "<<v2[0] <<" ," <<v2[1] <<" ," <<v2[2] <<"\nPlease, insert a new v2(comma separated): ";
#ifndef DEBUG
				std::scanf("%f,%f,%f", &v2[0],&v2[1],&v2[2]);

#endif // !DEBUG
			}
		}
	}
	UpCorner();
	UpDisc();
}

Eigen::Vector2f Screen::GetSize() { return Eigen::Vector2f(v1.norm(),v2.norm()); }
void Screen::UpCorner()
{
	corner = center -(v1+v2)/2;
}

void Screen::UpDisc()
{
	discretv1 = v1*(1/(float)pixelDim[0]);
	discretv2 = v2*(1/(float)pixelDim[1]);
}

void Screen::Print() 
{
	std::cout << "Center : " << GetCenter() << std::endl << "Size : " << GetSize() << std::endl << "Corner : " << GetStartCorner() << std::endl; 
}

void Screen::SetPixel(Eigen::Vector2i aPixelDim) { pixelDim = aPixelDim; }
Eigen::Vector2i Screen::GetPixel() { return pixelDim ;}
Eigen::Vector3f Screen::GetDiscretv1(){return discretv1; }
Eigen::Vector3f Screen::GetDiscretv2(){return discretv2; }
//End:		Screen Declaration

//Start:	Observer Declaration
Observer::Observer()
{
	center << 0,0,0;
}

Observer::Observer(float x, float y, float z)
{
	center <<x,y,z;
}

Observer::Observer(Eigen::Vector3f aCenter)
{
	center = aCenter;
}

void Observer::SetCenter(float x,float y, float z)
{
	center << x,y,z;
}

void Observer::SetCenter(Eigen::Vector3f aCenter)
{
	center = aCenter;
}

Eigen::Vector3f Observer::GetCenter()
{
	return center;
}


void Observer::Print()
{
	std::cout << "Center : " << GetCenter() << std::endl;
}
//End:		Observer Declaration

/*	Ray	*/

	Ray::Ray()
	{ direction << 0,0,0; point << 0,0,0; light_dir = -1; reflex_counter = REFLEX_COUNTER;  intensity << 1,1,1;}
	
	Ray::Ray(Eigen::Vector3f point_value)
	{ direction <<0,0,0; point = point_value; light_dir = -1; reflex_counter = REFLEX_COUNTER; intensity << 1,1,1;}

	Ray::Ray(Eigen::Vector3f point_value,Eigen::Vector3f direction_value,Eigen::Vector3f intensity_value, int counter_value, int aLight)
	{ direction = direction_value; direction.normalize(); point = point_value; reflex_counter = counter_value; intensity = intensity_value;  light_dir = aLight;}

	Ray::Ray(float x_point, float y_point, float z_point, float x_dir,float y_dir,float z_dir, float value_b, float value_g,float value_r,int counter_value, int aLight)
	{ direction << x_dir,y_dir,z_dir; direction.normalize(); point << x_point,y_point,z_point; reflex_counter = counter_value; intensity << value_b,value_g,value_r; light_dir = aLight;}

	Ray::Ray(float x_point, float y_point, float z_point, float x_dir,float y_dir,float z_dir,int counter_value, int aLight)
	{ direction << x_dir,y_dir,z_dir; direction.normalize(); point << x_point,y_point,z_point; reflex_counter = counter_value; intensity << 1,1,1; light_dir = aLight;}

	void Ray::SetIntensity(void)
	{ intensity << 1,1,1; }

	void Ray::SetIntensity(Eigen::Vector3f intensity_value)
	{ intensity = intensity_value;}

	void Ray::SetIntensity(float x)
	{ intensity << x,x,x; }

	void Ray::SetIntensity(float value_b, float value_g,float value_r)
	{ intensity << value_b,value_g,value_r; }

	Eigen::Vector3f Ray::GetIntensity() { return intensity; }

	void Ray::SetDirection(void)
	{ direction << 0,0,0; }

	void Ray::SetDirection(float x)
	{ direction << x,x,x; direction.normalize();	}
	
	void Ray::SetDirection(Eigen::Vector3f direction_value)
	{ direction = direction_value; direction.normalize();}
	
	void Ray::SetDirection(float x,float y, float z)
	{ direction << x,y,z; direction.normalize();	}

	void Ray::SetReflectionlimite(void)
	{ reflex_counter = 0; }

	void Ray::SetReflectionlimite(int counter_value)
	{ reflex_counter = counter_value; }

	void Ray::SetPoint()
	{ point << 0,0,0;}
	
	void Ray::SetPoint(Eigen::Vector3f point_value)
	{ point =point_value;}
	
	void Ray::SetPoint(float x,float y,float z)
	{ point << x,y,z;}
	
	Eigen::Vector3f Ray::GetPoint()
	{return point;}
	
	Eigen::Vector3f Ray::GetDirection()
	{return direction; }

	int Ray::GetReflectionCounter()
	{return reflex_counter; }

	void Ray::Print()
	{ 	std::cout << "Point : " << GetPoint() << std::endl << "Direction: " << GetDirection() << std::endl<< "Reflex_Limit : " << GetReflectionCounter() << std::endl; }

//Start:	CollisionObject Declaration

CollisionObject::CollisionObject() { DoItCollide = false; position <<0,0,0; }

CollisionObject::CollisionObject(float x,float y,float z)
{ 
	DoItCollide = true;
	position << x,y,z;
}

CollisionObject::CollisionObject(Eigen::Vector3f aColisionPosition)
{
	DoItCollide = true;
	position = aColisionPosition;
}

bool CollisionObject::IsThereCollision() { return DoItCollide; }
Eigen::Vector3f CollisionObject::GetCollisionPosition(){ return position; }
void CollisionObject::Print() { std::cout <<"Collision: "<< DoItCollide <<"\nCollision point: "<< position[0] <<" , "<< position[1] <<" , "<< position[2] <<"\n";}

//End:		CollisionObject Declaration


//Begin:	Scene Declaration

	Scene::Scene()
	{
		screen_vector.clear();	
		obs_vector.clear();
		light_vector.clear();
		physic_vector.clear();

	}
	
	Scene::Scene(IScreen* screen_obj,IObserver* obs_obj)
	{
		screen_vector.insert(screen_vector.begin(), screen_obj); 
		obs_vector.insert   (obs_vector.begin(),	obs_obj);
		light_vector.clear();
		physic_vector.clear();

	}

	// Sets and gets of each part of Scene

	void Scene::SetScreen(IScreen* screen_obj)
	{
		screen_vector.push_back(screen_obj);
	}
	
	void Scene::SetObserver(IObserver* obs_obj)
	{
		obs_vector.push_back(obs_obj);
	}
		
	void Scene::Setlight(Ilight* light_obj)
	{
		light_vector.push_back(light_obj);

	}

	void Scene::SetPhysical(IPhysicalObject* physic_obj)
	{
		physic_vector.push_back(physic_obj);
	}
	
	vector<IScreen*> Scene::GetScreen()
	{
		return screen_vector;
	}

	vector<IObserver*> Scene::GetObserver()
	{
		return obs_vector;
	}

	vector<Ilight*> Scene::Getlight()
	{
		return light_vector;
	}
	
	vector<IPhysicalObject*> Scene::GetPhysical()
	{
		return physic_vector;
	}
		IScreen* Scene::GetScreen(int i)
	{
		return screen_vector[i];
	}

	IObserver* Scene::GetObserver(int i)
	{
		return obs_vector[i];
	}

	Ilight* Scene::Getlight(int i)
	{
		return light_vector[i];
	}
	
	IPhysicalObject* Scene::GetPhysical(int i)
	{
		return physic_vector[i];
	}


	// Print: print on terminal all the scene characteristics
	void Scene::Print()
	{
		std::cout << "\n Number of objects in Scene : \n" << std::endl;
		std::cout << " Screens   : "			<< screen_vector.size()	<< std::endl;
		std::cout << " Observers : "			<< obs_vector.size()	<< std::endl;
		std::cout << " lights    : "			<< light_vector.size()	<< std::endl;
		std::cout << " Objects   : "			<< physic_vector.size() << std::endl;
		std::cout <<												       std::endl;
	}

	// Numbers: Informs the corresponding quantity of each object type in the scene
	// Input : Null or specific object
	int* Scene::numbers()
	{
		int* temp = (int *) malloc(4*sizeof(int));

		temp[screen]	= screen_vector.size();
		temp[observer]	= obs_vector.size();
		temp[Light]		= light_vector.size();
		temp[object]	= physic_vector.size();
		return temp;
	}

		int Scene::numbers(int i)
	{
		switch(i)
		{
			case screen		: return screen_vector.size();	break;
			case observer	: return obs_vector.size();		break;
			case Light		: return light_vector.size();	break;
			case object		: return physic_vector.size();	break;
			default			: return -1;					break;
		}
	}

	// Collision: Detects first collision and returns false if it is a light source and true if its a physical object
	// Input : Ray object
	bool Scene::Collision( std::shared_ptr<IRay> ray)
	{
		CollisionObject collision;
		int n_physical = numbers(object);
		int n_lights = numbers(Light);
		float physic_distance = -1, light_distance = -1; // impossible value initialization
		int physic_num = 0, light_num = 0;

		// Check the closest collision regarding each physical object in the scene
		for(int i = 0; i < n_physical; i++ )
		{
 			collision = physic_vector[i]->Collision(ray);
			if( collision.IsThereCollision() ) // check collision without creating output ray_vector
			{
				if( ( ((collision.GetCollisionPosition() - (ray)->GetPoint()).norm() <= physic_distance || physic_distance < 0 )
					&& (collision.GetCollisionPosition() - (ray)->GetPoint()).norm() > LIGHT_EPS )  )						// Is there a closer object? 
				{																											// Treatment for numerical errors in the superfice of physical objects
					physic_distance = (collision.GetCollisionPosition() - (ray)->GetPoint()).norm();						// saves smaller distance, if yes
					physic_num = i;																						    // saves Light object number
				}
			}
		}

		// Check the closest collision regarding each light source in the scene
		for(int i = 0; i < n_lights; i++ )
		{
			collision = light_vector[i]->Collision(ray);
			if(collision.IsThereCollision()) // check collision without creating output ray_vector
			{
				if(  ((collision.GetCollisionPosition() - (ray)->GetPoint()).norm() <= light_distance) || light_distance < 0) // Is there a closer object?
				{
					light_distance = (collision.GetCollisionPosition() - (ray)->GetPoint()).norm();							  // saves smaller distance, if yes
					light_num = i;																						      // saves Light object number
				}
			}
		}

		if(light_distance >= 0 && physic_distance >=0)
		{
			if(  light_distance < physic_distance && light_distance != -1 )
			{
				return false;
			}
			else
			{
				if(physic_vector[physic_num]->IsRefractive())
					return false;
				return true;
			}
		}
		else if(light_distance >= 0)
		{
			return false;
		}
		else //physic_distance >= 0
		{
			if(physic_vector[physic_num]->IsRefractive())
				return false;
			return true;
		}

}

	//ReadData: Is an auxiliary method to ReadScene which reads the parameters of each new part of the scene
	//Input : Line of the file with iterator starting from parameters and int vetor to receive read values
	void Scene::ReadData ( float* p, stringstream* ss)
	{
		string buff;
		int j = 0;
		while(buff != "endl" && j < 24) // Loop until end of the line or 24 words maximum in a line;
		{
			*ss>>buff;
			p[j] = (float)atof(buff.c_str());
			//cout<<p[j]<<endl;
			j++;
		}
	}

	// ReadScene : Given a pre-defined-formated-.txt file, this methods creates the parts of the scene declared on the file
	// and adds them to the corresponding Scene object.
	// Input: *.txt pre-formated-file
	void Scene::ReadScene(char* input)
	{

	ifstream myfile;
	myfile.open(input);	
	Scene*  scene = new Scene(); 

		if (myfile.is_open())
		{
			string line;
			stringstream line_ss;
			float p[25]; // Variable that will receive char parameters from line
			for(int a = 0; a<24; a++) p[a] = 0;
			string buff;

			// Defines the three maps used in to convert string into enum for the switch
			// Map One : First word on text
			std::map<std::string, sub_parts > map_one;
			map_one["screen"]		= screen;
			map_one["observer"]		= observer;
			map_one["light"]		= Light;
			map_one["object"]		= object;
			map_one["#"]			= comment1;
			map_one[" "]			= space;
			map_one[""]				= empty;
			map_one["\n"]			= empty;
		
			//Map two : differs different objects
			std::map<std::string, object_type >  map_two;
			map_two["SpecularPlane"]		= specularPlane;
			map_two["SpecularRectangle"]	= specularRectangle ;
			map_two["SpecularSphere"]		= specularSphere;
			map_two["DiffusePlane" ]		= diffusePlane;
			map_two["SpecularSphere"]		= specularSphere;
			map_two["DiffuseRectangle"]		= diffuseRectangle;
			map_two["DiffuseSphere"]		= diffuseSphere; 
			map_two["MRPlane"]				= mrplane;
			map_two["MRSphere"]				= mrsphere;
			
			// Map Three : defines the methods type
			enum methode_type {  type0,type1,type2,type3,type4,type5	};

			std::map<std::string, methode_type >  map_three;
			map_three["type0"]	=  type0;
			map_three["type1"]	=  type1;
			map_three["type2"]	=  type2;
			map_three["type3"]	=  type3;
			map_three["type4"]	=  type4;
			map_three["type5"]	=  type5;
			map_three["0"]		=  type0;
			map_three["1"]		=  type1;
			map_three["2"]		=  type2;
			map_three["3"]		=  type3;
			map_three["4"]		=  type4;
			map_three["5"]		=  type5;
		
			//cout<<"File opened"<< endl;

			//Gets each line from myfile and saves it on char line
			while ( getline (myfile,line) )
			{
				// clear streamstring buffer and loads with next line
				line_ss.str( std::string() );
				line_ss.clear();
				line_ss<<line;
				
			
				//Read the first word in line and associate with a map_one enum
				line_ss>>buff;
				sub_parts switch_on = map_one[buff];
				//Detects which part of the scene was declared and creates the corresponding object.
					switch (switch_on)
				{
					case screen:
						{
							line_ss>>buff;
							methode_type type = map_three[buff];  

							switch (type)
							{
							case type0:
								{
									Screen* aScreen = new Screen();
									SetScreen(aScreen);
									break;
								}

							case type1:
								{
									ReadData(p, &line_ss);
									Screen* aScreen = new Screen(Eigen::Vector3f(p[0],p[1],p[2]) ,Eigen::Vector3f(p[3],p[4],p[5]), Eigen::Vector3f(p[6],p[7],p[8]) , Eigen::Vector2i(p[9],p[10]));
									SetScreen(aScreen);
									break;
								}
							default:
								cout<< "Error: Incorrect methode type"<<endl;
								break;
							}
							break;
						}

				
					case observer:
						{
							line_ss>>buff;
							methode_type type = map_three[buff];

							switch (type)
							{
							case type0:
								{
									Observer* aObserver = new Observer();
									SetObserver(aObserver);
									break;
								}
							
							case type1:
								{
									ReadData(p, &line_ss);
									Observer* aObserver = new Observer(Eigen::Vector3f(p[0],p[1],p[2]));
									SetObserver(aObserver);
									break;
								}
							default:
								cout<< "Error: Incorrect methode type"<<endl;
								break;
							}
							break;
						}



					case Light:
						{
							line_ss>>buff;
							methode_type type = map_three[buff];

							switch (type)
							{
							case type0:
								{
									ReadData(p, &line_ss);
									light* aLight = new light();
									Setlight(aLight);
									break;
								}

							case type1:
								{

									ReadData(p, &line_ss);
									light* aLight = new light(Eigen::Vector3f(p[0],p[1],p[2]), p[3], p[4], p[5]);
									Setlight(aLight);
									break;
								}

							case type2:
								{
									ReadData(p, &line_ss);
									light* aLight = new light(Eigen::Vector3f(p[0],p[1],p[2]) ,Eigen::Vector3f(p[3],p[4],p[5]));
									Setlight(aLight);
									break;
								}

							case type3:
								{
									ReadData(p, &line_ss);
									light* aLight = new light(Eigen::Vector3f(p[0],p[1],p[2]));
									Setlight(aLight);
									break;
								}

							default:
								cout<< "Error: Incorrect methode type"<<endl;
								break;
							}
							break;
						}



					case object:
						{
							line_ss>>buff;
							object_type theOne = map_two[buff];

							switch(theOne)
							{
								case specularPlane:
									{
										line_ss>>buff;
										methode_type type = map_three[buff];
										switch (type)
										{
										case type0:
											{
												ReadData(p, &line_ss);
												SpecularPlane* obj = new SpecularPlane();
												SetPhysical( obj );
												break;
											}

										case type1:
											{
												ReadData(p, &line_ss);
												SpecularPlane* obj = new SpecularPlane( Eigen::Vector3f(p[0],p[1],p[2]) ,Eigen::Vector3f(p[3],p[4],p[5]), Eigen::Vector3f(p[6],p[7],p[8]) );
												SetPhysical( obj );
												break;
											}

										case type2:
											{
												ReadData(p, &line_ss);
												SpecularPlane* obj = new SpecularPlane( p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]);
												SetPhysical( obj );
												break;
											}
										default:
											cout<< "Error: Incorrect methode type"<<endl;
											break;
										}
										break;
									}

								//case specularRectangle :
								//	{
								//		SpecularRectangle* obj = new SpecularRectangle();
								//		SetPhysical( obj );

								//		break;
								//	}

								case specularSphere:
									{
										line_ss>>buff;
										methode_type type = map_three[buff];

										switch (type)
										{
										case type0:
											{
												ReadData(p, &line_ss);
												SpecularSphere* obj = new SpecularSphere();
												SetPhysical( obj );
												break;
											}
																				
										case type1:
											{
												ReadData(p, &line_ss);
												SpecularSphere* obj = new SpecularSphere( Eigen::Vector3f(p[0],p[1],p[2]), p[3], Eigen::Vector3f(p[4],p[5],p[6]) );
												SetPhysical( obj );
												break;

											}

										case type2:
											{
												ReadData(p, &line_ss);
												SpecularSphere* obj = new SpecularSphere( p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
												SetPhysical( obj );
												break;
											}

										default:
											cout<< "Error: Incorrect methode type"<<endl;
											break;																				
										}
										break;
									}

								case diffusePlane:
									{
										line_ss>>buff;
										methode_type type = map_three[buff];

										switch (type)
										{
										case type0:
											{
												DiffusePlane* obj = new DiffusePlane();
												SetPhysical( obj );
												break;

											}

										case type1:
											{
												ReadData(p, &line_ss);
												DiffusePlane* obj = new DiffusePlane( Eigen::Vector3f(p[0],p[1],p[2]) ,Eigen::Vector3f(p[3],p[4],p[5]), Eigen::Vector3f(p[6],p[7],p[8]) );
												SetPhysical( obj );
												break;
											}

										case type2:
											{
												DiffusePlane* obj = new DiffusePlane( p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8]);
												SetPhysical( obj );
												break;
											}

										default:
											cout<< "Error: Incorrect methode type"<<endl;
											break;
										}
										break;
									}

								//case diffuseRectangle:
								//	{
								//		DiffuseRectangle* obj = new DiffuseRectangle();
								//		SetPhysical( obj );
								//		break;
								//	}

								case diffuseSphere:
									{
										line_ss>>buff;
										methode_type type = map_three[buff];

										switch (type)
										{
										case type0:
											{
												DiffuseSphere* obj = new DiffuseSphere();
												SetPhysical( obj );
												break;
											}

										case type1:
											{
												ReadData(p, &line_ss);
												DiffuseSphere* obj = new DiffuseSphere(Eigen::Vector3f(p[0],p[1],p[2]), p[3], Eigen::Vector3f(p[4],p[5],p[6]) );
												SetPhysical( obj );
												break;
											}

										case type2:
											{
												ReadData(p, &line_ss);
												DiffuseSphere* obj = new DiffuseSphere( p[0], p[1], p[2], p[3], p[4], p[5], p[6]);
												SetPhysical( obj );
												break;
											}
										default:
											cout<< "Error: Incorrect methode type"<<endl;
											break;
										}
										break;
									}	

								case mrplane:
									{
										line_ss>>buff;
										methode_type type = map_three[buff];

										switch (type)
										{
										case type0:
											{
												MRPlane* obj = new MRPlane();
												SetPhysical( obj );
												break;
											}

										case type1:
											{
												ReadData(p, &line_ss);
												MRPlane* obj = new MRPlane( Eigen::Vector3f(p[0],p[1],p[2]) ,Eigen::Vector3f(p[3],p[4],p[5]), Eigen::Vector3f(p[6],p[7],p[8]), p[9], p[10], p[11], p[12] );
												SetPhysical( obj );
												break;

											}

										case type2:
											{
												ReadData(p, &line_ss);
												MRPlane* obj = new MRPlane( p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10], p[11], p[12] );
												SetPhysical( obj );
												break;
											}
										default:
											cout<< "Error: Incorrect methode type"<<endl;
											break;
										}
										break;
									}	

								case mrsphere:
									{
										line_ss>>buff;
										methode_type type = map_three[buff];

										switch (type)
										{
										case type0:
											{
												MRSphere* obj = new MRSphere();
												SetPhysical( obj );
												break;
											}

										case type1:
											{
												ReadData(p, &line_ss);
												MRSphere* obj = new MRSphere( Eigen::Vector3f(p[0],p[1],p[2]), p[3], Eigen::Vector3f(p[4],p[5],p[6]), p[7], p[8], p[9], p[10] );
												SetPhysical( obj );
												break;
											}

										case type2:
											{
												ReadData(p, &line_ss);
												MRSphere* obj = new MRSphere( p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7], p[8], p[9], p[10] );
												SetPhysical( obj );
												break;
											}


										default:
											cout<< "Error: Incorrect methode type"<<endl;
											break;
										}
										break;
									}	
							}
							break;
						}

					case 0:
						buff.clear();
						break;
								
					default:
						cout<<" Txt file is not in the correct format"<< endl;
						break;
				}
			}
		std::cout<< " File Readed " << std::endl;
		myfile.close();
		Print();
	}

	else std::cout << "Unable to open file\n";

}





//End:		Scene Declaration


