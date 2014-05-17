#include"Ray_tracer.h"
#include"primitives.h"
#include<string>
#include<exception>

#define Ray_intensity Eigen::Vector3f(1,1,1)
#define BLACK Eigen::Vector3f (0,0,0)

template<typename Content>
bool deleteAll_test( Content * theElement ) 
{
	delete theElement; 
	return true; 
}


// Constructors

	// Empty constructor
	Ray_tracer::Ray_tracer()
	{
		scene_vector.clear();
		image_vector.clear();
		SetOutput("Output.png");
	}

	// Tracer constructor receiving a existing scene
	Ray_tracer::Ray_tracer(Scene* scene_obj)
	{
		scene_vector.push_back(scene_obj);
		image_vector.clear();
		SetOutput("Output.png");
	}

	// Tracer constructor receiving a finite number of scene in a vector structure
	Ray_tracer::Ray_tracer(vector<Scene*> scene_vector_value)
	{ 
		scene_vector = scene_vector_value;
		SetOutput("Output.png");
	}

	// Constructor receiving a vector of scenes and an output name
	Ray_tracer::Ray_tracer(vector<Scene*> scene_vector_value, char* output_value)
	{
		scene_vector = scene_vector_value;
		SetOutput(output_value);
	}

	// Constructor receiving a single scene and an output name
	Ray_tracer::Ray_tracer(Scene* scene_obj, char* output_value)
	{
		scene_vector.push_back(scene_obj);
		image_vector.clear();
		SetOutput(output_value);
	}


	// Set scene method
	void Ray_tracer::SetScene(Scene* scene_obj)
	{
		scene_vector.push_back(scene_obj);
	}


	// Get output name method
	string Ray_tracer::GetOutput()
	{
		return output_name;
	}

	// SaveImage: Saves image to the stipulate output name by the user
	void Ray_tracer::SaveImage(cv::Mat img)
	{
		imwrite( output_name, img );
	}
	
	// Set output name 
	void Ray_tracer::SetOutput( char* output_value ) 
	{
		string s(output_value);
		output_name = s;
	}

	// CheckScene: Check whether the scene entered by user is valid or not 
	//
	void Ray_tracer::CheckScene()
	{		
			try // Check Scene existence
			{				
				if(scene_vector.empty())
				{
					std::cout<< "\n No scene detected "<< endl;						
					throw 1 ;
				}
			}
			catch (int n) { cout << " \nError : " << n << endl; cout << "Please correct or reload scene" << endl; }
			
			int i = 0;
			for(vector <Scene*>::iterator it = scene_vector.begin() ; it != scene_vector.end() ; ++it)
			{
				try 
				{			
					try // Check Screen existence
					{						
						if((*it)->numbers(screen) == 0)
						{
							std::cout<< "\n No screen detected! "<< endl;
							throw 2;
						}
					}
					catch (const int& n) { throw; }			

					try	// Check Observer existence
					{ 
						if((*it)->numbers(observer) == 0) 
						{
							std::cout<< "\n No observer detected! "<< endl;
							throw 3;
						}
					}
					catch (const int& n) { throw; }

					try // Check screen-observer alignment error
					{			
						vector <IObserver*> temp_obs  = (*it)->GetObserver();
						vector <IScreen*> temp_screen = (*it)->GetScreen();
						Eigen::Vector3f	temp_vec = temp_screen[0]->GetCenter() - temp_obs[0]->GetCenter();
						Eigen::Vector3f	temp_vec2;
					
						if(temp_obs.size() == 1 && 
						   temp_screen.size() == 1  &&
						   temp_vec.dot(temp_vec2) == 0)
						{
							std::cout<<" \n Observer and Screen in the same plane!"<<endl;
							throw 4 ;
						}
					}
					catch (const int& n) { throw; }
				}
				
				catch (int n) { cout << " \n Error : " << n << endl; cout << " \n\n Exception occurred in scene["<< i << "]" << " . Please correct or reload scene" << endl; }
				i++;
			}
}

	// FirstColliosion: Detects the first object in the the way of ray propagation
	// also informs the whether the collision is with a light or object
	bool Ray_tracer::FirstCollision( std::shared_ptr<IRay> ray, bool* light_flag, int* access_num)
 {
		CollisionObject collision;
		float physic_distance = -1, light_distance = -1; // impossible value initialization
		int physic_num = 0, light_num = 0;

		// Check the closest collision regarding each physical object in the scene
		for(int i = 0; i < (*scene)->numbers(object); i++ )
		{
 			collision = (*scene)->GetPhysical(i)->Collision(ray);
			if( collision.IsThereCollision() ) // check collision without creating output ray_vector
			{
 				if( ( ((collision.GetCollisionPosition() - (ray)->GetPoint()).norm() <= physic_distance || physic_distance < 0 ) && (collision.GetCollisionPosition() - (ray)->GetPoint()).norm() > LIGHT_EPS )  ) // Is there a closer object?
				{																											// threats the case that the reflected ray collides in the same object that has emitted it, due to numerical approximations 
					physic_distance = (collision.GetCollisionPosition() - (ray)->GetPoint()).norm();						// saves smaller distance, if yes
					physic_num = i;	// saves physical object number
				}
			}
		}

		// Check the closest collision regarding each lighy source in the scene
		for(int i = 0; i < (*scene)->numbers(Light); i++ )
		{
			collision = (*scene)->Getlight(i)->Collision(ray);
			if(collision.IsThereCollision()) // check collision without creating output ray_vector
			{
				if((collision.GetCollisionPosition() - (ray)->GetPoint()).norm() <= light_distance || light_distance < 0) // Is there a closer object?
				{
					light_distance = (collision.GetCollisionPosition() - (ray)->GetPoint()).norm();						  // saves smaller distance, if yes
					light_num = i;																						  // saves Light object number
				}
			}
		}

		// Threats the possible answers cases

		if(physic_distance < 0 && light_distance < 0) // No collision happened
		{
			return false;
		}
		else // Collision Happened
		{
			if(light_distance >= 0 && physic_distance >=0)
			{
				if(  light_distance < physic_distance && light_distance != -1 )
				{
					try{(*light_flag) = true;}
					catch (const std::exception &e)
					{
						std::cout << e.what() <<std::endl;
					}
					*access_num = light_num;
				}
				else
				{
					*light_flag = false;
					*access_num = physic_num;
				}
			}
			else if(light_distance >= 0)
			{
				*light_flag = true;
				*access_num = light_num;
			}
			else //physic_distance >= 0
			{
				*light_flag = false;
				*access_num = physic_num;
			}
			return true;
		}		
 }
	
	// Follow Description :
	// Follows a given ray and all reflected rays created by objects interaction
	// returns the color of the pixel in the screen where the ray started calculated by the interaction of each sub-ray with the scene
	Eigen::Vector3f Ray_tracer::Follow( vector<std::shared_ptr<IRay>> ray_vector)
	{
		Eigen::Vector3f pixel_color(0,0,0);
		bool* light_flag = new bool;
		int* access_num = new int;
		for( vector<std::shared_ptr<IRay>>::iterator it = ray_vector.begin() ; it != ray_vector.end() ; ++it)
		{	
			if((*it)->IsGoingToLight() >= 0 ) //If is a Diffuse ray going to a Selected Light 
			{
				 pixel_color += (*it)->GetIntensity().cwiseProduct((*scene)->Getlight((*it)->IsGoingToLight())->GetIntensity()); //object pixel_color according to light intensity
			}
			else
			{
				if( FirstCollision(*it, light_flag, access_num) ) // Collision case
				{
					if (*light_flag) // light collision
					{		
 						pixel_color += (*it)->GetIntensity().cwiseProduct((*scene)->Getlight(*access_num)->GetIntensity()); //object pixel_color according to light intensity
					}
					else // Physical collision
					{
						vector<std::shared_ptr<IRay>> reflected_rays = (*scene)->GetPhysical(*access_num)->Reflection(*it,*scene);
						if( reflected_rays.empty() ) // Last Ray treatment
						{
							pixel_color += BLACK;
						}
						else // At least one reflection ray
						{
							pixel_color += Follow( reflected_rays ); // Follow each emitted ray path
						}
					}
				}
				else // No collision case
				{
					pixel_color += BLACK;
				}
			}
		}
		delete(light_flag);
		delete(access_num);
		return pixel_color;
	}

	// Threat_pixel: Handle saturation of pixel value that cannot exceed 255
	// also converts float to char (used in the image creation )
	cv::Vec3b Ray_tracer::Threat_pixel( Eigen::Vector3f pixel )
	{
		cv::Vec3b threated;
		for (int i = 0 ; i < 3; i++)
		{
			if( pixel[i] > 255 ) 
				threated[i] = (uchar)255;
			else
				threated[i] = (uchar) pixel[i];
		}
		return threated;
		
	}

	// BaseRay: Creates the first ray that will emitted from the observer
	// that will serve as entry to the function follow.
	vector<std::shared_ptr<IRay>> Ray_tracer::BaseRay ( int i, int j )
	{
		// Defines the first observer-screen ray parameters 
		Eigen::Vector3f d_v1 = (*scene)->GetScreen(0)->GetDiscretv1();
		Eigen::Vector3f d_v2 = (*scene)->GetScreen(0)->GetDiscretv2(); 
		Eigen::Vector3f pixel_position = (*scene)->GetScreen(0)->GetStartCorner() + (d_v1 + d_v2)/2 + (float)i*d_v1 + (float)j*d_v2 ;
		Eigen::Vector3f observe_screen_direction = pixel_position - (*scene)->GetObserver(0)->GetCenter();

		//Creates the effective Base Ray with the parameters calculated before
		shared_ptr<IRay> base_ray(new Ray( pixel_position , observe_screen_direction , Ray_intensity , REFLEX_COUNTER ));
		
		// Puts the base ray in a vector, necessary to the utilization of the Following method
		vector<std::shared_ptr<IRay>> base_ray_vector;	
		base_ray_vector.push_back(base_ray);

		return base_ray_vector;
	}

	
	// Tracer : Render the image through calculation of each pixel color, 
	//			displays the final image and saves it with the given output name
	void Ray_tracer::Tracer()
	{
		
		std::cout << "\n Tracing image..." << std::endl;
		#ifdef PARALLEL
			const int num_procs = (omp_get_num_procs() > 1) ? (omp_get_num_procs()-1):1;
			std::cout<<"Using "<< num_procs <<" threads.\n";
		#endif //PARALLEL

		// Execute the rendering in each existing scene
		for(scene = scene_vector.begin() ; scene != scene_vector.end() ; ++scene)
		{

			Eigen::Vector3f pixel;

			// Defining Img and adding it on vector.
			Eigen::Vector2i screen_size =  (*scene)->GetScreen(0)->GetPixel();
			cv::Mat  img( screen_size[1], screen_size[0], CV_8UC3);

			image_vector.push_back(&img);

			#ifdef PARALLEL
				#pragma omp parallel for num_threads(num_procs)
			#endif // PARALLEL
			for (int i = 0 ;  i < screen_size[0] ; i ++ ) // Screen vertical vectorial discretization pace
			{	
				for( int j = 0;  j < screen_size[1] ; j ++ ) // Screen horizontal vectorial discretization pace
				{
					// Calculate pixel color
					pixel = Follow( BaseRay(i,j) ); 
					// Basic pixel treatment 
					img.at<cv::Vec3b>(j,i) = Threat_pixel(pixel);
				}
			}
			

			
			////saves image
			SaveImage(img);
			
			#ifdef SHOW_OUTPUT
				// Creates display window
				cv::namedWindow("Test", CV_WINDOW_AUTOSIZE );
				// Display it on window
				imshow( "Test", img );
				cv::waitKey(0);
			#endif
		}
	}

	// Test_singleRay : Method created for debugging reason
	//					Same thing as the tracer but for a single pixel to avoid complete calculation time
	void Ray_tracer::Test_singleRay(int i, int j) 
	{
		
		std::cout << "\n Single Ray Test..." << std::endl;


		for(scene = scene_vector.begin() ; scene != scene_vector.end() ; ++scene)
		{

			Eigen::Vector3f pixel;

			std::cout << "\nCalculating color of pixel["<<i<<"]["<<j<<"] :"<< std::endl;

			pixel = Follow( BaseRay(i,j) ); 
				
			std::cout<< endl;
			std::cout<<pixel<<endl;
			std::cout<<Threat_pixel(pixel)<<endl;

		}
	}
	
