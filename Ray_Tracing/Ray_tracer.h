#include "opencv2/opencv.hpp"
#include <vector>
#include <stdlib.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <float.h>
#include "primitives.h"
#include <exception>
#include <algorithm>    
#include <memory>
#include "defines.h"
#include <omp.h>

class Ray_tracer
{
private:
	// Variables
	vector<Scene*> scene_vector;
	vector<Scene*>::iterator scene;
	vector<cv::Mat*> image_vector;
	string output_name;

public:
	//Constructors
	Ray_tracer();
	Ray_tracer(Scene*);
	Ray_tracer(vector<Scene*>);
	Ray_tracer(vector<Scene*>, char*);
	Ray_tracer(Scene*, char*);
	
	// Set variables methods
	void SetScene(Scene*);
	void SetOutput(char*);

	// Get method
	string GetOutput();

	// return which object is the reflects a given ray first
	bool FirstCollision( std::shared_ptr<IRay> ray, bool* light_flag, int* access_num);
	// Follow a Ray path and its reflectioned sub-rays, calculating the final color of the pixel localized int the start point of the first ray 
	Eigen::Vector3f Follow( vector<std::shared_ptr<IRay>> ray_vector);
	// Creats the primary ray which will be followed
	vector<std::shared_ptr<IRay>> BaseRay( int i, int j );
	// Threats pixel to create final image
	cv::Vec3b Threat_pixel( Eigen::Vector3f pixel );

	// Save final image with the outputname in the same folder as the .exe was executed
	void SaveImage(cv::Mat img);
	// Check scene existance and possibility to proceed with the tracer method
	void CheckScene();
	// Renders the image
	void Tracer();
	// Informs the color of a single ray ( debug )
	void Test_singleRay(int i, int j) ; 


	//destructor
	virtual ~Ray_tracer() {}
};

