#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sstream>
#include <iostream>
#include <cmath>

#include "stage.hh"

using namespace Stg;

const int popsize = 2;
const double WIFI_RANGE = 2.0;

class Robot{
	public:
		std::string name;
		ModelPosition* position;
		std::vector <int> robotsInWifiRange;

};

class MSRG{
	public:
		//do msrg procedure on every tick..
		static int Callback(World* world, void* userarg){
			MSRG* msrg = reinterpret_cast<MSRG*>(userarg);
			msrg->ComputeWifiConnectivity();
			if (msrg->debug){
				msrg->DebugInformation();
			}
			msrg->Tick(world);
			return 0;
		}

		//initializer
		MSRG(unsigned int popsize, bool debug) : population_size(popsize), robots(new Robot[population_size]), debug(debug){
			
		}

		//destructor
		~MSRG(){
			delete[] robots;
		}

		//connect to world bots
		void connect(World* world){
			std::cout << population_size << " robots should be in world\n";
			//update all robots..
			for (unsigned int idx = 0; idx < population_size; idx++){
				std::stringstream name;
				name << "msrg" << idx+1;
				robots[idx].name = name.str();

				ModelPosition* posmod = reinterpret_cast<ModelPosition*>(world->GetModel(robots[idx].name));
				assert(posmod != 0);

				robots[idx].position = posmod;
				robots[idx].position->Subscribe();

			}

			world->AddUpdateCallback(MSRG::Callback, reinterpret_cast<void*>(this));
		}

		void ComputeWifiConnectivity(){
			for (unsigned int i = 0; i < population_size; i++){
				robots[i].robotsInWifiRange.clear();
				for (unsigned int j = 0; j < population_size; j++){
					if (j != i){
						Pose iPose = robots[i].position->GetPose();
						Pose jPose = robots[j].position->GetPose();
						if (iPose.Distance2D(jPose) <= WIFI_RANGE){
							robots[i].robotsInWifiRange.push_back(j);
						}
					}
				}
			}
		}

		void DebugInformation(){
			for (unsigned int i = 0; i < population_size; i++){
				std::cout << "Robot " << i << ":\nIn range: ";
				for (std::vector<int>::iterator iter = robots[i].robotsInWifiRange.begin(); iter != robots[i].robotsInWifiRange.end(); ++iter){
					std::cout << "msrg" << (*iter)+1 << ", ";
				}
				std::cout << "\n";
			}
		}

		//actual logic
		void Tick(World* world){
		}

	protected:
		unsigned int population_size;
		Robot* robots;
		bool debug;
};

int main(int argc, char* argv[]){
	Init( &argc, &argv);

	WorldGui world(500, 400, "MSRG Simulation");
	world.Load( "msrg.world" );
	
	MSRG msrg(popsize, true);
	msrg.connect(&world);

	world.Run();

	return 0;
}
