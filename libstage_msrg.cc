#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sstream>
#include <iostream>
#include <cmath>

#include "stage.hh"

using namespace Stg;

const double PI = 3.14159;
const double WIFI_RANGE = 2.0;
const double RADIAN_PER_DEGREE = 0.01745;
const int READY = 0;
const int START = 1;
const int TURNING = 2;
const int FINISHED_TURNING = 3;
const int MOVING = 4;
const int STORE_POSITION = 5;
const int STOPPED = 6;

double WorldAngle(double human_angle){
	return human_angle*RADIAN_PER_DEGREE;
}

double HumanAngle(double world_angle){
	return world_angle / RADIAN_PER_DEGREE;
}

double WorldAngle360(double angle){
	return fmod(angle + WorldAngle(360), WorldAngle(360));
}

class SRGNode{
	public:
		SRGNode* parent;
		std::vector <SRGNode*> children;
		Pose pose;
		std::vector <double> bearings;
		std::vector <double> intensities;
		std::vector <meters_t> ranges;

		SRGNode(SRGNode* parent, Pose pose) : parent(parent), pose(pose){
		}

		std::string ToString(){
			std::stringstream ss;
			ss << "(" << pose.x << "," << pose.y << ")\nReadings:\n";
			for (unsigned int i = 0; i < bearings.size(); i++){
				ss << bearings[i] << ":" << intensities[i] << ":" << ranges[i] << "\n";
			}

			return ss.str();
		}

};

class Robot{
	public:
		std::string name;
		ModelPosition* position;
		ModelRanger* ranger;
		std::vector <int> robotsInWifiRange;
		SRGNode* srg;
		SRGNode* current;
		int state;
		Velocity lastVelocity;
		/*
			Targets are global pose values.
		*/
		double targetX;
		double targetY;

		Robot() : srg(NULL), current(NULL), state(1){

		}

		void TurnToGlobalAngle(double globalAngle){
			Pose currentPose = position->GetGlobalPose();
			double angleToMove = fmod(globalAngle + WorldAngle(360)-WorldAngle360(currentPose.a), WorldAngle(360));
			position->GoTo(0, 0, angleToMove);
			state = TURNING;
		}

		void MoveForward(double distance){
			position->GoTo(distance, 0, 0);
			state = MOVING;
		}

		bool IsMovingForward(){
			Velocity v = position->GetVelocity();
			return !((fabs(lastVelocity.x) > fabs(v.x)  && fabs(v.x) <= 0.001));
		}

		void StopMoving(){
			position->Stop();
		}

		void UpdateLastVelocity(){
			lastVelocity = position->GetVelocity();
		}

		void UpdateRotationState(){
			if (!IsRotating()){
				StopMoving();
				state = READY;
			}
		}

		void UpdateMovingForwardState(){
			if (!IsMovingForward()){
				std::cout << "not moving anymore\n";
				StopMoving();
				state = STORE_POSITION;
			}else{
				std::cout << "still moving..\n";
			}
		}

		bool IsRotating(){
			Velocity v = position->GetVelocity();
			return !(fabs(lastVelocity.a) > fabs(v.a)  && fabs(v.a) <= 0.01);
		}

};

int PositionUpdate( ModelPosition* model, Robot* robot ){
	if (robot->state == START || robot->state == STORE_POSITION){
		Pose currentPose = robot->position->GetGlobalPose();
		std::vector <ModelRanger::Sensor> sensors = robot->ranger->GetSensorsMutable();
		std::vector <double> bearings;
		std::vector <double> intensities;
		std::vector <meters_t> ranges;
		
		for (unsigned int i = 0; i < sensors.size(); i++){
			bearings.push_back(sensors[i].bearings[0]);
			intensities.push_back(sensors[i].intensities[0]);
			ranges.push_back(sensors[i].ranges[0]);
		}
		std::cout << "=================\n";
		std::cout << currentPose.x << "\n";
		std::cout << currentPose.y << "\n";
		std::cout << currentPose.z << "\n";
		std::cout << currentPose.a << "\n";
		if (robot->current == NULL){
			SRGNode* currentNode = new SRGNode(NULL, robot->position->GetPose());
			currentNode->bearings = bearings;
			currentNode->intensities = intensities;
			currentNode->ranges = ranges;
			robot->srg = currentNode;
			robot->current = currentNode;
			
		}else{
			SRGNode* currentNode = new SRGNode(robot->current, robot->position->GetPose());
			currentNode->bearings = bearings;
			currentNode->intensities = intensities;
			currentNode->ranges = ranges;
			robot->current->children.push_back(currentNode);
			robot->current = currentNode;
		}
	}

	if (robot->state == READY){
//		robot->MoveForward(2.0);
	}else if (robot->state == START){
		robot->MoveForward(0.5);
	}else if (robot->state == TURNING){
		robot->UpdateRotationState();
	}else if (robot->state == MOVING){
		robot->UpdateMovingForwardState();
	}else if (robot->state == STORE_POSITION){
		robot->state = STOPPED;
	}

	robot->UpdateLastVelocity();

	return 0;
}

class MSRG{
	public:
		//do msrg procedure on every tick..
		static int Callback(World* world, void* userarg){
			MSRG* msrg = reinterpret_cast<MSRG*>(userarg);
			msrg->ComputeWifiConnectivity();
			msrg->Tick(world);
			if (msrg->debug){
				msrg->DebugInformation();
			}
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

				ModelRanger* ranger = reinterpret_cast<ModelRanger*>(posmod->GetUnusedModelOfType( "ranger" ));

				Robot* robot = new Robot();
				robot->name = name.str();
				robot->position = posmod;
				robot->position->AddCallback(Model::CB_UPDATE, (model_callback_t)PositionUpdate, robot);
				robot->position->Subscribe();
				robot->ranger = ranger;
				robot->ranger->Subscribe();
				robot->lastVelocity = robot->position->GetVelocity();
				robots.push_back(robot);
			}

			world->AddUpdateCallback(MSRG::Callback, reinterpret_cast<void*>(this));
		}

		void ComputeWifiConnectivity(){
			for (unsigned int i = 0; i < robots.size(); i++){
				robots[i]->robotsInWifiRange.clear();
				for (unsigned int j = 0; j < robots.size(); j++){
					if (j != i){
						Pose iPose = robots[i]->position->GetPose();
						Pose jPose = robots[j]->position->GetPose();
						if (iPose.Distance2D(jPose) <= WIFI_RANGE){
							robots[i]->robotsInWifiRange.push_back(j);
						}
					}
				}
			}
		}

		void DebugInformation(){
			if (robots[0]->state == STOPPED){
				for (unsigned int i = 0; i < robots.size(); i++){
					std::cout << "Robot " << i << ":\nIn range: ";
					for (std::vector<int>::iterator iter = robots[i]->robotsInWifiRange.begin(); iter != robots[i]->robotsInWifiRange.end(); ++iter){
						std::cout << "msrg" << *iter << ", ";
					}
					std::cout << "\n";
					std::cout << "SRG:\n";
					SRGNode* current = robots[i]->srg;
					while ( current != NULL){
						std::cout << current->ToString();
						if (current->children.size() == 0){
							current = NULL;
						}else{
							current = current->children[0];
						}
					}
				}
			}
		}

		//actual logic
		void Tick(World* world){
		}

	protected:
		std::vector<Robot*> robots;
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
