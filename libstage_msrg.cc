#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sstream>
#include <iostream>
#include <cmath>
#include <time.h>

#include "stage.hh"
#include "CImg.h"

using namespace Stg;
using namespace cimg_library;

const long MAX_ITERATIONS = 1000;

//Robot states
const int READY = 0;
const int TURNING = 1;
const int FINISHED_TURNING = 2;
const int MOVING = 3;
const int STORE_POSITION = 4;
const int STOPPED = 5;
const unsigned char FREE = 255;
const unsigned char OBSTACLE = 0;
const unsigned char UNEXPLORED = 50;
const unsigned char UNSET = 50;
const unsigned char BACKGROUND = 255;
const unsigned char FOREGROUND = 0;
const unsigned char STREL_SET = 255;
const unsigned char STREL_UNSET = 0;
const unsigned char STREL_SET_COLOR[] = {STREL_SET};
const unsigned char STREL_UNSET_COLOR[] = {STREL_UNSET};

const double PI = 3.14159;
const double TWO_PI = PI*2;
const double RADIAN_PER_DEGREE = 0.01745;
const double ONE_DEGREE_RADIAN = 1 * RADIAN_PER_DEGREE;

//Rp = sensor range
const double Rp = 2.0;
//Rc = communication range (must be a multiple of Rp)
const double Rc = 3*Rp;

const double ROBOT_MAP_RESOLUTION = 100;
const int LRR_EROSION_EPSILON = 10;
const int ROBOT_MAP_HEIGHT = (int)ceil(Rp*ROBOT_MAP_RESOLUTION*2)+1;
const int ROBOT_MAP_WIDTH = (int)ceil(Rp*ROBOT_MAP_RESOLUTION*2)+1;
const int ROBOT_MAP_ORIGIN_X = ROBOT_MAP_WIDTH/2;
const int ROBOT_MAP_ORIGIN_Y = ROBOT_MAP_HEIGHT/2;
const int POLAR_UNIT = 1.41421;
const CImg <unsigned char> NO_BACKGROUND(ROBOT_MAP_WIDTH, ROBOT_MAP_HEIGHT, 1, 1, FOREGROUND);

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

double frandom(){
	return ((double)rand()) / RAND_MAX;
}

double frandom(double min, double max){
	return min + frandom()*(max-min);
}

//in radians..
bool WolrdAngle360Between(double n, double a, double b) {
        n = fmod(TWO_PI + fmod(n, TWO_PI), TWO_PI);
        a = WorldAngle360(a);
        b = WorldAngle360(b);

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
		std::vector <double> fov;
		std::vector <double> bearings;
		std::vector <meters_t> ranges;
		double radius;
		std::vector <std::vector<int> > mapData;
		CImg <unsigned char> lsr;
		CImg <unsigned char> lrr;
		CImg <unsigned char> lf;
		CImg <unsigned char> lir;
		CImg <unsigned char> robotCircular;
		CImg <unsigned char>* globalMap;
		int globalMapWidth;
		int globalMapHeight;
		std::vector <double*> lirRaffle;

		SRGNode(SRGNode* parent, Pose pose, CImg <unsigned char>* globalMap, int globalMapWidth, int globalMapHeight) : parent(parent), pose(pose), radius(0.30), globalMap(globalMap), globalMapWidth(globalMapWidth), globalMapHeight(globalMapHeight){
		}

		bool inPerceptionRange(double globalX, double globalY){
			double dist = distance(pose.x, pose.y, globalX, globalY);
			return InBetween(dist, 0, Rp);
		}

		bool inLSR(double globalX, double globalY){
			if (!inPerceptionRange(globalX, globalY)){
				return false;
			}
			double localPose = WorldAngle360(atan2(globalY-pose.y, globalX-pose.x));
			for (unsigned int i = 0; i < bearings.size(); i++){
				double halfSpan = (fov[i]/2)+1;
				double fromAngle360 = WorldAngle360(bearings[i]-WorldAngle(halfSpan));
				double toAngle360 = WorldAngle360(bearings[i]+WorldAngle(halfSpan));

				if (WolrdAngle360Between(localPose, fromAngle360, toAngle360)){
					if (!InBetween(distance(pose.x, pose.y, globalX, globalY), 0, ranges[i])){
						return false;
					}
				}
			}
			return true;
		}

		double toGlobalCoordinateX(int mapX){
			return pose.x + (((double)mapX-ROBOT_MAP_ORIGIN_X)/ROBOT_MAP_RESOLUTION);
		}

		double toGlobalCoordinateY(int mapY){
			return pose.y - (((double)mapY-ROBOT_MAP_ORIGIN_Y)/ROBOT_MAP_RESOLUTION);
		}

		int toLocalMapCoordinateX(double globalX){
			int deltaX = floor((pose.x - globalX) * ROBOT_MAP_RESOLUTION);
			return ROBOT_MAP_ORIGIN_X - deltaX;
		}

		int toLocalMapCoordinateY(double globalY){
			int deltaY = floor((pose.y - globalY) * ROBOT_MAP_RESOLUTION);
			return ROBOT_MAP_ORIGIN_Y + deltaY;
		}

		int toGlobalMapCoordinateX(double globalX){
			return (globalMapWidth/2)+floor(globalX * ROBOT_MAP_RESOLUTION);
		}

		int toGlobalMapCoordinateY(double globalY){
			return (globalMapHeight/2)-floor(globalY * ROBOT_MAP_RESOLUTION);
		}

		bool inMapLSR(int mapX, int mapY){
			return lsr(mapX, mapY) == FREE;
		}

		bool inMapLRR(int mapX, int mapY){
			return lrr(mapX, mapY) == BACKGROUND;
		}

		bool inMapLIR(int mapX, int mapY){
			return lir(mapX, mapY) == BACKGROUND;
		}

		bool inLRR(double globalX, double globalY){
			if (!inLSR(globalX, globalY)){
				return false;
			}
			return inMapLRR(toLocalMapCoordinateX(globalX), toLocalMapCoordinateY(globalY));
		}

		bool inLIR(double globalX, double globalY){
			if (!inLRR(globalX, globalY)){
				return false;
			}
			return inMapLIR(toLocalMapCoordinateX(globalX), toLocalMapCoordinateY(globalY));
		}

		std::string ToString(){
			std::stringstream ss;
			ss << "Node (" << pose.x << "," << pose.y << ")\nReadings:\n";
			for (unsigned int i = 0; i < bearings.size(); i++){
				ss << HumanAngle(bearings[i]) << "[" << fov[i] << "]" << ":" << ranges[i] << "\n";
			}

			return ss.str();
		}

		bool isValidXCoordinate(int globalMapX){
			if (globalMapX < 0){
				return false;
			}
			if (globalMapX >= globalMapWidth){
				return false;
			}
			return true;
		}

		bool isValidYCoordinate(int globalMapY){
			if (globalMapY < 0){
				return false;
			}
			if (globalMapY >= globalMapHeight){
				return false;
			}
			return true;
		}

		bool isLF(int globalMapX, int globalMapY){
			int delta[8][2] = {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
			for (int i = 0; i < 8; i++){
				int prospectX = globalMapX + delta[i][0];
				int prospectY = globalMapY + delta[i][1];
				if (isValidXCoordinate(prospectX) && isValidYCoordinate(prospectY)){
					if (globalMap->operator()(prospectX, prospectY) == UNEXPLORED){
						return true;
					}
				}
			}
			return false;
		}

		void update_lsr(){
			std::cout << "Updating LSR\n";

			int globalMapX = toGlobalMapCoordinateX(pose.x);
			int globalMapY = toGlobalMapCoordinateY(pose.y);
			std::cout << "Updating global map\n";
			int localTopLeftX = globalMapX-(ROBOT_MAP_WIDTH/2);
			int localTopLeftY = globalMapY-(ROBOT_MAP_HEIGHT/2);

//			std::cout << "local top left: (" << localTopLeftX << "," << localTopLeftY << ")\n";

			cimg_forXY(lsr, x, y){
				int prospectX = localTopLeftX+x;
				int prospectY = localTopLeftY+y;
				unsigned char currentValue = globalMap->operator()(prospectX, prospectY);
				if (currentValue == FREE){
					lsr(x,y) = FREE;
				}else if (currentValue == OBSTACLE){
					if (lsr(x,y) == UNEXPLORED){
						lsr(x,y) = OBSTACLE;
					}
				}
			}

			std::cout << "Finished updating lsr\n";

//			std::cout << "Showing updated lsr\n";
//			lsr.display();
		}

		void calculate_lsr(){
			std::cout << "Calculating lsr\n";

			lsr = CImg <unsigned char>(ROBOT_MAP_HEIGHT, ROBOT_MAP_WIDTH);
			lsr = UNEXPLORED;
			const unsigned char color_free[] = {FREE};
			const unsigned char color_obstacle[] = {OBSTACLE};

			for (unsigned int i = 0; i < bearings.size(); i++){
				double R = ranges[i] * ROBOT_MAP_RESOLUTION;
				double halfSpan = (fov[i]/2)+1;
				double theta1 = WorldAngle360(bearings[i]-WorldAngle(halfSpan)) - (ONE_DEGREE_RADIAN/2);
				double theta2 = WorldAngle360(bearings[i]+WorldAngle(halfSpan)) + (ONE_DEGREE_RADIAN/2);
				int deltaY1 = floor(R*sin(theta1));
				int deltaX1 = floor(R*cos(theta1));
				int deltaY2 = floor(R*sin(theta2));
				int deltaX2 = floor(R*cos(theta2));
				lsr.draw_triangle(ROBOT_MAP_ORIGIN_X, ROBOT_MAP_ORIGIN_Y,
					ROBOT_MAP_ORIGIN_X+deltaX1, ROBOT_MAP_ORIGIN_Y-deltaY1,
					ROBOT_MAP_ORIGIN_X+deltaX2, ROBOT_MAP_ORIGIN_Y-deltaY2,
					color_free);
				if (ranges[i] < Rp){
					int obstacleDeltaY1 = floor((R+POLAR_UNIT)*sin(theta1));
					int obstacleDeltaX1 = floor((R+POLAR_UNIT)*cos(theta1));
					int obstacleDeltaY2 = floor((R+POLAR_UNIT)*sin(theta2));
					int obstacleDeltaX2 = floor((R+POLAR_UNIT)*cos(theta2));
					lsr.draw_line(ROBOT_MAP_ORIGIN_X+obstacleDeltaX1, ROBOT_MAP_ORIGIN_Y-obstacleDeltaY1,
							ROBOT_MAP_ORIGIN_X+obstacleDeltaX2, ROBOT_MAP_ORIGIN_Y-obstacleDeltaY2,
							color_obstacle);
				}
			}

			std::cout << "Finished calculating lsr\n";

//			std::cout << "Showing lsr\n";
//			lsr.display();
		}

		void update_global_map(){
			int globalMapX = toGlobalMapCoordinateX(pose.x);
			int globalMapY = toGlobalMapCoordinateY(pose.y);
			std::cout << "Updating global map\n";
			int localTopLeftX = globalMapX-(ROBOT_MAP_WIDTH/2);
			int localTopLeftY = globalMapY-(ROBOT_MAP_HEIGHT/2);

//			std::cout << "local top left: (" << localTopLeftX << "," << localTopLeftY << ")\n";

			cimg_forXY(lsr, x, y){
				int prospectX = localTopLeftX+x;
				int prospectY = localTopLeftY+y;
				unsigned char currentValue = globalMap->operator()(prospectX, prospectY);
				if (currentValue == OBSTACLE){
					if (lsr(x, y) == FREE){
						globalMap->operator()(prospectX, prospectY) = lsr(x,y);
					}

				}else if (currentValue == UNEXPLORED){
					if (lsr(x, y) == OBSTACLE || lsr(x,y) == FREE){
						globalMap->operator()(prospectX, prospectY) = lsr(x,y);
					}
				}
			}
			std::cout << "Finished updating global map\n";

//			std::cout << "Showing global map\n";
//			globalMap->display();
		}

		//only emulates erosion using distance transform..
		void calculate_lrr(){
			std::cout << "Calculating lrr\n";
			lrr = CImg <unsigned char> (lsr);

			cimg_forXY(lrr, x, y){
				if (lrr(x, y) == UNSET){
					lrr(x, y) = STREL_UNSET;
				}
			}

			CImg <double> lrrDist(lrr);
			lrrDist.distance(STREL_UNSET);

			cimg_forXY(lrr, x, y){
				if (lrr(x,y) == STREL_SET){
					if (lrrDist(x,y) <= radius*ROBOT_MAP_RESOLUTION){
						lrr(x,y) = STREL_UNSET;
					}
				}
			}

			std::cout << "Finished calculating lrr\n";

//			std::cout << "Showing lrr\n";
//			lrr.display();
		}

		void calculate_lf(){
			std::cout << "Calculating lf\n";
			int globalMapX = toGlobalMapCoordinateX(pose.x);
			int globalMapY = toGlobalMapCoordinateY(pose.y);

			int localTopLeftX = globalMapX-(ROBOT_MAP_WIDTH/2);
			int localTopLeftY = globalMapY-(ROBOT_MAP_HEIGHT/2);

			lf = CImg <unsigned char>(ROBOT_MAP_HEIGHT, ROBOT_MAP_WIDTH);
			lf = FOREGROUND;

			cimg_forXY(lsr, x, y){
				if (lsr(x,y) == BACKGROUND && isLF(localTopLeftX+x, localTopLeftY+y)){
					lf(x, y) = BACKGROUND;
				}
			}
			std::cout << "Finished calculating lf\n";

//			std::cout << "Showing lf\n";
//			lf.display();
		}

		void calculate_lir(){
			std::cout << "Calculating lir\n";

			lir = CImg <unsigned char>(ROBOT_MAP_HEIGHT, ROBOT_MAP_WIDTH);
			lir = FOREGROUND;

			std::cout << "Calculating distance from lf matrix\n";

			CImg <double> distanceFromLF(lf);
			distanceFromLF.distance(BACKGROUND);

			lirRaffle.clear();

			cimg_forXY(distanceFromLF, x, y){
				if (lrr(x,y) == BACKGROUND && distanceFromLF(x,y) <= Rp*ROBOT_MAP_RESOLUTION && distanceFromLF(x,y) > 0){
					lir(x, y) = BACKGROUND;
					double globalCoordinateX = toGlobalCoordinateX(x);
					double globalCoordinateY = toGlobalCoordinateY(y);
					double* coordinates = new double[2];
					coordinates[0] = globalCoordinateX;
					coordinates[1] = globalCoordinateY;
					lirRaffle.push_back(coordinates);
				}
			}

			std::cout << "Finished calculating lir\n";

//			std::cout << "Showing lir\n";
//			lir.display();

		}

};

class Robot{
	public:
		bool backtracking;
		bool startup;
		long iterations;
		std::string name;
		ModelPosition* position;
		ModelRanger* ranger;
		std::vector <int> robotsInWifiRange;
		std::vector <SRGNode*> srgList;
		SRGNode* backtrackTarget;
		SRGNode* srg;
		SRGNode* current;
		int state;
		Velocity lastVelocity;
		CImg <unsigned char> map;
		int mapHeight;
		int mapWidth;

		//Global Pose..
		Pose targetPose;
		Robot(int mapHeight, int mapWidth) : srg(NULL), current(NULL), state(STORE_POSITION), startup(true), iterations(0), mapHeight(mapHeight), mapWidth(mapWidth), backtracking(false){
			map = CImg <unsigned char>(mapWidth, mapHeight);
			map = UNEXPLORED;
		}

		int currentMapX(){
			return toMapX(current->pose.x);
		}

		int currentMapY(){
			return toMapY(current->pose.y);
		}

		int toMapX(double globalX){
			return (mapWidth/2)+floor(globalX * ROBOT_MAP_RESOLUTION);
		}

		int toMapY(double globalY){
			return (mapHeight/2)-floor(globalY * ROBOT_MAP_RESOLUTION);
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
		std::vector <double> fov;

		std::cout << "Global Coordinates: " << currentPose.x << "," << currentPose.y << "\n";
	
		for (unsigned int i = 0; i < sensors.size(); i+=3){
			double perception = sensors[i].ranges[0];
			for (unsigned int j = i; j < i + 3; j++){
				if (sensors[j].ranges[0] < perception){
					perception = sensors[j].ranges[0];
				}
			}
			ranges.push_back(perception);
			bearings.push_back(WorldAngle360(currentPose.a+sensors[i].pose.a));
			fov.push_back(HumanAngle(sensors[i].fov));
		}

		SRGNode* currentNode;

		if (robot->backtracking){
			currentNode = robot->backtrackTarget;
		}else if (robot->iterations == 0){
			currentNode = new SRGNode(NULL, currentPose, &(robot->map), robot->mapWidth, robot->mapHeight);
		}else{
			currentNode = new SRGNode(robot->current, currentPose, &(robot->map), robot->mapWidth, robot->mapHeight);
		}

		currentNode->bearings = bearings;
		currentNode->ranges = ranges;
		currentNode->fov = fov;
		
		if (robot->backtracking){
			currentNode->update_lsr();
		}else{
			currentNode->calculate_lsr();
		}
		currentNode->update_global_map();
		currentNode->calculate_lrr();
		currentNode->calculate_lf();
		currentNode->calculate_lir();

		if (robot->iterations == 0){
			robot->srg = currentNode;
		}else{
			if (!robot->backtracking){
				robot->current->children.push_back(currentNode);
			}
		}

		robot->current = currentNode;

/*		if (!robot->backtracking){
			robot->srgList.push_back(currentNode);
		}*/

		robot->iterations++;

		std::cout << "Iterations done: " << robot->iterations << "\n";

//		if (robot->iterations%10 == 0){
//			robot->map.display();
//		}
		//robot->map.display();
	}

	switch(robot->state){
		case READY:
			//what's next?
			//dummy command..
			if (robot->startup || robot->iterations <= MAX_ITERATIONS){
				//TODO: get a random configuration and set target..
				if (robot->current->lirRaffle.size() == 0){
					if (robot->current->parent == NULL){
						std::cout << "Back at starting position, stopping because no more LIR!!!!\n";
						robot->backtracking = false;
						robot->state = STOPPED;
					}else{
						std::cout << "Backtracking to parent..\n";
						robot->backtracking = true;
						robot->backtrackTarget = robot->current->parent;
						std::cout << "(" << robot->backtrackTarget->pose.x << "," << robot->backtrackTarget->pose.y << ")\n";
						robot->SetTarget(Pose(robot->backtrackTarget->pose.x, robot->backtrackTarget->pose.y,0,0));
						robot->TurnToGlobalAngle();
					}
				}else{
					int randomIndex = random()%robot->current->lirRaffle.size();
					double* randomCoordinates = robot->current->lirRaffle[randomIndex];
					std::cout << "(" << randomCoordinates[0] << "," << randomCoordinates[1] << ")\n";
					robot->SetTarget(Pose(randomCoordinates[0], randomCoordinates[1],0,0));
					robot->TurnToGlobalAngle();
					robot->backtracking = false;
				}
				robot->startup = false;
			}else if (robot->iterations > MAX_ITERATIONS){
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
			if (msrg->robots[0]->state == STOPPED && msrg->displayedResults){
				return 1;
			}
			return 0;
		}

		//initializer
		MSRG(bool debug) : debug(debug), displayedResults(false){
			
		}

		//connect to world bots
		void connect(World* world){
			Model* floorplan = world->GetModel("msrg_floorplan");
			Geom floorplanGeom = floorplan->GetGeom();
			Size floorplanSize = floorplanGeom.size;
			int mapHeight = floorplanSize.y*ROBOT_MAP_RESOLUTION;
			int mapWidth = floorplanSize.x*ROBOT_MAP_RESOLUTION;
			for (unsigned int idx = 0; ; idx++){
				std::stringstream name;
				name << "msrg" << idx;
				
				std::cout << name.str() << "\n";

				if ((world->GetModel(name.str())) == NULL){
					break;
				}


				ModelPosition* posmod = reinterpret_cast<ModelPosition*>(world->GetModel(name.str()));

				ModelRanger* ranger = reinterpret_cast<ModelRanger*>(posmod->GetUnusedModelOfType( "ranger" ));

				Robot* robot = new Robot(mapHeight, mapWidth);
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
					robots[i]->map.display();
				}
				displayedResults = true;
			}
		}

		//actual logic
		void Tick(World* world){
		}

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
