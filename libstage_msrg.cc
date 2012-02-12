#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sstream>
#include <iostream>
#include <cmath>

#include "stage.hh"

using namespace Stg;

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
		MSRG(bool debug) : debug(debug){
			
		}

		//connect to world bots
		void connect(World* world){
			for (unsigned int idx = 0; ; idx++){
				std::stringstream name;
				name << "msrg" << idx;
				
				std::cout << name.str() << "\n";

				if ((world->GetModel(name.str())) == NULL){
					break;
				}

				ModelPosition* posmod = reinterpret_cast<ModelPosition*>(world->GetModel(name.str()));

				Robot robot;

				robot.name = name.str();
				robot.position = posmod;
				robot.position->Subscribe();
				robots.push_back(robot);
			}

			world->AddUpdateCallback(MSRG::Callback, reinterpret_cast<void*>(this));
		}

		void ComputeWifiConnectivity(){
			for (unsigned int i = 0; i < robots.size(); i++){
				robots[i].robotsInWifiRange.clear();
				for (unsigned int j = 0; j < robots.size(); j++){
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
			for (unsigned int i = 0; i < robots.size(); i++){
				std::cout << "Robot " << i << ":\nIn range: ";
				for (std::vector<int>::iterator iter = robots[i].robotsInWifiRange.begin(); iter != robots[i].robotsInWifiRange.end(); ++iter){
					std::cout << "msrg" << *iter << ", ";
				}
				std::cout << "\n";
			}
		}

		//actual logic
		void Tick(World* world){
		}

	protected:
		std::vector<Robot> robots;
		bool debug;
};

int main(int argc, char* argv[]){
	Init( &argc, &argv);

	WorldGui world(500, 400, "MSRG Simulation");
	world.Load( "msrg.world" );

	bool debug = false;


	if (argc > 1){
		std::string s(argv[1]);
		if (s.compare("debug") == 0){
			debug =true;
		}
	}

	MSRG msrg(debug);
	msrg.connect(&world);

	world.Run();

	return 0;
}
