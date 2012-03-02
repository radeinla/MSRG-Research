#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sstream>
#include <iostream>
#include <cmath>
#include <time.h>

#include "stage.hh"

using namespace Stg;

const double PI = 3.14159;
const double TWO_PI = PI*2;
const double RADIAN_PER_DEGREE = 0.01745;
const double MOVING_FORWARD_EPSILON = 0.000000001;

//Rp = sensor range
const double Rp = 1.0;
//Rc = communication range (must be a multiple of Rp)
const double Rc = 3*Rp;

const double ROBOT_MAP_RESOLUTION = 100;
const int ROBOT_MAP_HEIGHT = (int)ceil(Rp*ROBOT_MAP_RESOLUTION);
const int ROBOT_MAP_WIDTH = (int)ceil(Rp*ROBOT_MAP_RESOLUTION);

//Robot states
const int READY = 0;
const int TURNING = 1;
const int FINISHED_TURNING = 2;
const int MOVING = 3;
const int STORE_POSITION = 4;
const int STOPPED = 5;

const long MAX_NODES = 1;

double sqr(double x){
	return pow(x, 2);
}

double WorldAngle(double human_angle){
	return human_angle*RADIAN_PER_DEGREE;
}

double HumanAngle(double world_angle){
	return world_angle / RADIAN_PER_DEGREE;
}

double WorldAngle360(double angle){
	return fmod(angle + TWO_PI, TWO_PI);
}

//double frandom(){
//	double randomDenom = random() * random();
//	return (((double)random())/randomDenom)/randomDenom;
//}

//in radians..
bool WolrdAngle360Between(double n, double a, double b) {
        n = WorldAngle360(n);
        a = WorldAngle360(a);
        b = WorldAngle360(b);

	if (a > b){
		b = b + TWO_PI;
	}

        if (a < b)
                return a <= n && n <= b;
        return a <= n || n <= b;
}

bool InBetween(double n, double a, double b){
	return n >= a && n <= b;
}

double distance(double x1, double y1, double x2, double y2){
	double d = sqrt(sqr(x1-x2) + sqr(y1-y2));
	return d;
}

class SRGNode{
	public:
		SRGNode* parent;
		std::vector <SRGNode*> children;
		Pose pose;
		double fov;
		std::vector <double> bearings;
		std::vector <meters_t> ranges;
		double radius;

		bool inPerceptionRange(double globalX, double globalY){
			double dist = distance(pose.x, pose.y, globalX, globalY);
			std::cout << "distance from node: " << dist << "\n";
			return dist >= 0 && dist <= Rp;
		}

		bool inLSR(double globalX, double globalY){
			if (!inPerceptionRange(globalX, globalY)){
				return false;
			}
			double localPose = WorldAngle360(atan2(globalY-pose.y, globalX-pose.x));
			std::cout << "local pose: " << HumanAngle(localPose) << "\n";
			for (unsigned int i = 0; i < bearings.size(); i++){
				if (WolrdAngle360Between(localPose, bearings[i] - WorldAngle(fov/2), bearings[i] + WorldAngle(fov/2))){
					if (!InBetween(distance(pose.x, pose.y, globalX, globalY), radius, ranges[i])){
						return false;
					}
				}
			}
			return true;
		}

		bool inLRR(double globalX, double globalY){
			if (!inLSR(globalX, globalY)){
				return false;
			}
			return true;
		}

		bool inLF(double globalX, double globalY){
			return true;
		}

		bool inLIR(double globalX, double globalY){
			return true;
		}

		//radius is hard coded..
		SRGNode(SRGNode* parent, Pose pose) : parent(parent), pose(pose), radius(0.075){
		}

		std::string ToString(){
			std::stringstream ss;
			ss << "Node (" << pose.x << "," << pose.y << ")\nReadings:\n";
			for (unsigned int i = 0; i < bearings.size(); i++){
				ss << HumanAngle(bearings[i]) << ":" << ranges[i] << "\n";
			}

			return ss.str();
		}

};

class Robot{
	public:
		bool startup;
		long nodes;
		std::string name;
		ModelPosition* position;
		ModelRanger* ranger;
		std::vector <int> robotsInWifiRange;
		SRGNode* srg;
		SRGNode* current;
		int state;
		Velocity lastVelocity;

		//Global Pose..
		Pose targetPose;
		Robot() : srg(NULL), current(NULL), state(STORE_POSITION), startup(true), nodes(0){

		}


		void SetTarget(Pose target){
			targetPose = target;
		}


		void TurnToGlobalAngle(){
			Pose currentPose = position->GetGlobalPose();
			Pose odom = Pose(0,0,0,0);
			position->SetOdom(odom);
			double globalAngle = atan2(targetPose.y-currentPose.y, targetPose.x-currentPose.x);
			double angleToMove = fmod(globalAngle + WorldAngle(360)-WorldAngle360(currentPose.a), WorldAngle(360));
			position->GoTo(0, 0, angleToMove);
			state = TURNING;
		}

		void MoveForward(){
			Pose currentPose = position->GetGlobalPose();
			Pose odom = Pose(0,0,0,0);
			position->SetOdom(odom);
			position->GoTo(distance(currentPose.x, currentPose.y, targetPose.x, targetPose.y), 0, 0);
			state = MOVING;
		}

		bool IsMovingForward(){
			Velocity v = position->GetVelocity();
			return !((fabs(lastVelocity.x) > fabs(v.x)  && fabs(v.x) == 0));
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
				MoveForward();
			}
		}

		void UpdateMovingForwardState(){
			if (!IsMovingForward()){
				StopMoving();
				state = STORE_POSITION;
			}else{
				//std::cout << "still moving..\n";
			}
		}

		bool IsRotating(){
			Velocity v = position->GetVelocity();
			return !(fabs(lastVelocity.a) > fabs(v.a)  && fabs(v.a) <= 0.01);
		}

};

int PositionUpdate( ModelPosition* model, Robot* robot ){
	if (robot->state == STORE_POSITION){
		std::cout << "storing position..\n";
		Pose currentPose = robot->position->GetGlobalPose();
		std::vector <ModelRanger::Sensor> sensors = robot->ranger->GetSensorsMutable();
		std::vector <double> bearings;
		std::vector <meters_t> ranges;
	
		for (unsigned int i = 0; i < sensors.size(); i+=3){
			double perception = sensors[i].ranges[0];
			for (unsigned int j = i; j < i + 3; j++){
				if (sensors[j].ranges[0] < perception){
					perception = sensors[j].ranges[0];
				}
			}
			ranges.push_back(perception);
			bearings.push_back(WorldAngle((i/3)*30));
		}

		sensors.clear();
		
		if (robot->current == NULL){
			SRGNode* currentNode = new SRGNode(NULL, robot->position->GetPose());
			currentNode->bearings = bearings;
			currentNode->ranges = ranges;
			currentNode->fov = 30;
			std::cout << "in lsr: " << currentNode->inLSR(1.5, -0.5) << "\n";
			std::cout << "in lsr: " << currentNode->inLSR(1.5, 0.5) << "\n";
			robot->srg = currentNode;
			robot->current = currentNode;
		}else{
			SRGNode* currentNode = new SRGNode(robot->current, robot->position->GetPose());
			currentNode->bearings = bearings;
			currentNode->ranges = ranges;
			currentNode->fov = 30;
			std::cout << "in lsr: " << currentNode->inLSR(1.5, -0.5) << "\n";
			std::cout << "in lsr: " << currentNode->inLSR(1.5, 0.5) << "\n";
			robot->current->children.push_back(currentNode);
			robot->current = currentNode;
		}
		robot->nodes++;
	}

	switch(robot->state){
		case READY:
			//what's next?
			//dummy command..
			if (robot->startup || robot-> nodes <= MAX_NODES){
				//TODO: get a random configuration and set target..
				robot->SetTarget(Pose(1,1,0,0));
				robot->TurnToGlobalAngle();
				robot->startup = false;
			}else if (robot->nodes > MAX_NODES){
				robot->state = STOPPED;
			}
			break;
		case STORE_POSITION:
			robot->state =  READY;
			break;
		case STOPPED:
			break;
		//Go To Position.. Considered an atomic operation.. Cannot be stopped anywhere here..
		case TURNING:
			robot->UpdateRotationState();
			break;
		case MOVING:
			robot->UpdateMovingForwardState();
			break;
	}

	robot->UpdateLastVelocity();

	return 0;
}

class MSRG{
	public:
		bool displayedResults;
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
		MSRG(bool debug) : debug(debug), displayedResults(false){
			
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
						if (iPose.Distance2D(jPose) <= Rc){
							robots[i]->robotsInWifiRange.push_back(j);
						}
					}
				}
			}
		}

		void DebugInformation(){
			if (robots[0]->state == STOPPED && !displayedResults){
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
				displayedResults = true;
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

	srand( time(NULL) );

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
