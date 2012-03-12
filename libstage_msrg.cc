#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sstream>
#include <iostream>
#include <cmath>
#include <time.h>
#include <float.h>

#include "stage.hh"
#include "CImg.h"

using namespace Stg;
using namespace cimg_library;

const long MAX_ITERATIONS = 2000;

//Robot states
const int READY = 0;
const int TURNING = 1;
const int FINISHED_TURNING = 2;
const int MOVING = 3;
const int STORE_POSITION = 4;
const int STOPPED = 5;
const int SYNCHRONIZING = 6;
const int STARTUP = 7;
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
const double OBSTACLE_DILATION_MAX_DISTANCE = 20;
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

bool areCoordinatesSorted(int* coord1, int* coord2){
	if (coord1[0] < coord2[0]){
		return true;
	}else if (coord1[0] > coord2[0]){
		return false;
	}else if (coord1[1] < coord2[1]){
		return true;
	}else if (coord1[1] > coord2[1]){
		return false;
	}else{
		return false;
	}
}

class SRGNode{
	public:
		SRGNode* parent;
		std::vector <SRGNode*> children;
		std::vector <SRGNode*> bridges;
		bool bridge;
		Pose pose;
		std::vector <double> fov;
		std::vector <double> bearings;
		std::vector <meters_t> ranges;
		double radius;
		CImg <unsigned char> lsr;
		CImg <unsigned char> lrr;
		CImg <unsigned char> lf;
		CImg <unsigned char> lir;
		CImg <unsigned char>* globalMap;
		int globalMapWidth;
		int globalMapHeight;
		std::vector <double*> lirRaffle;
		std::vector <int*> lsrGlobalMapCoordinates;
		int fromRobot;
		int fromRobotIndex;

		SRGNode(SRGNode* parent, Pose pose, std::vector <double> bearings, std::vector <double> fov, std::vector <meters_t> ranges,
			CImg <unsigned char>* globalMap, int globalMapWidth, int globalMapHeight, int fromRobot, int fromRobotIndex, bool bridge) : 
			parent(parent), pose(pose), bearings(bearings), fov(fov), ranges(ranges),
			radius(0.40), globalMap(globalMap), globalMapWidth(globalMapWidth), globalMapHeight(globalMapHeight),
			fromRobot(fromRobot), fromRobotIndex(fromRobotIndex), bridge(bridge) {
			
		}

		bool isLSRCoupled(SRGNode* other){
			for (int i = 0; i < other->lsrGlobalMapCoordinates.size(); i++){
				if (binary_search(lsrGlobalMapCoordinates.begin(), lsrGlobalMapCoordinates.end(), other->lsrGlobalMapCoordinates[i], areCoordinatesSorted)){
					return true;
				}
			}
			return false;
		}

		bool isLIRSharedWith(SRGNode* other){
			cimg_forXY(lir, x, y){
				cimg_forXY(other->lir, x2, y2){
					if (lir(x,y) == BACKGROUND && other->lir(x2,y2) == BACKGROUND){
						double globalX1 = toGlobalCoordinateX(x);
						double globalX2 = other->toGlobalCoordinateX(x2);
						double globalY1 = toGlobalCoordinateY(y);
						double globalY2 = other->toGlobalCoordinateY(y2);
						int globalMapX1 = toGlobalMapCoordinateX(globalX1);
						int globalMapX2 = other->toGlobalMapCoordinateX(globalX2);
						int globalMapY1 = toGlobalMapCoordinateY(globalY1);
						int globalMapY2 = other->toGlobalMapCoordinateY(globalY2);
						
						if (globalMapX1 == globalMapX2 && globalMapY1 == globalMapY2){
							return true;
						}
					}
				}
			}
			return false;
		}

		bool inPerceptionRange(double globalX, double globalY){
			double dist = distance(pose.x, pose.y, globalX, globalY);
			return InBetween(dist, 0, Rp);
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
			int localMapCoordinateX = toLocalMapCoordinateX(globalX);
			int localMapCoordinateY = toLocalMapCoordinateY(globalY);

			if (!inMapLSR(localMapCoordinateX, localMapCoordinateY)){
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
			for (int i = 0; i < bearings.size(); i++){
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

		void store_lsr(){
			lsrGlobalMapCoordinates.clear();
			std::cout << "max bag size: " << ROBOT_MAP_HEIGHT*ROBOT_MAP_WIDTH << "\n";
			std::cout << "empty lsr? " << lsrGlobalMapCoordinates.size() << "\n";

			cimg_forXY(lsr, x, y){
				if (lsr(x,y) == BACKGROUND){
					double globalX = toGlobalCoordinateX(x);
					double globalY = toGlobalCoordinateY(y);
					int globalMapX = toGlobalMapCoordinateX(globalX);
					int globalMapY = toGlobalMapCoordinateY(globalY);
					int* coordinates = new int[2];
					coordinates[0] = globalMapX;
					coordinates[1] = globalMapY;
					lsrGlobalMapCoordinates.push_back(coordinates);
				}
			}

			std::cout << "filled lsr? " << lsrGlobalMapCoordinates.size() << "\n";

			sort(lsrGlobalMapCoordinates.begin(), lsrGlobalMapCoordinates.end(), areCoordinatesSorted);
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
				if (distance(ROBOT_MAP_ORIGIN_X, ROBOT_MAP_ORIGIN_Y, x, y) <= Rp*ROBOT_MAP_RESOLUTION){
					int prospectX = localTopLeftX+x;
					int prospectY = localTopLeftY+y;
					if (isValidXCoordinate(prospectX) && isValidYCoordinate(prospectY)){
						unsigned char currentValue = globalMap->operator()(prospectX, prospectY);
						/*if (currentValue != UNEXPLORED){
							if (lsr(x,y) != UNEXPLORED){
								lsr(x,y) = currentValue;
							}
						}*/
						if (currentValue == FREE){
							lsr(x,y) = FREE;
						}else if (currentValue == OBSTACLE){
							if (lsr(x,y) == UNEXPLORED){
								lsr(x,y) = OBSTACLE;
							}
						}
						//	if (lsr(x, y) == OBSTACLE || lsr(x,y) == FREE){
						//		globalMap->operator()(prospectX, prospectY) = lsr(x,y);
						//	}
					}
				}
			}

			CImg <double> distanceFromObstacles(lsr);
			distanceFromObstacles.distance(OBSTACLE);

			cimg_forXY(distanceFromObstacles, x, y){
				if (distanceFromObstacles(x, y) <= OBSTACLE_DILATION_MAX_DISTANCE){
					lsr(x,y) = OBSTACLE;
				}
			}

			store_lsr();

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

			for (int i = 0; i < bearings.size(); i++){
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

			CImg <double> distanceFromObstacles(lsr);
			distanceFromObstacles.distance(OBSTACLE);

			cimg_forXY(distanceFromObstacles, x, y){
				if (distanceFromObstacles(x, y) <= OBSTACLE_DILATION_MAX_DISTANCE){
					lsr(x,y) = OBSTACLE;
				}
			}

			store_lsr();

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
				if (isValidXCoordinate(prospectX) && isValidYCoordinate(prospectY)){
					unsigned char currentValue = globalMap->operator()(prospectX, prospectY);
					if (currentValue == OBSTACLE){
						if (lsr(x, y) == FREE){
							globalMap->operator()(prospectX, prospectY) = FREE;
						}

					}else if (currentValue == UNEXPLORED){
						if (lsr(x, y) == OBSTACLE || lsr(x,y) == FREE){
							globalMap->operator()(prospectX, prospectY) = lsr(x,y);
						}
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
					if (lrrDist(x,y) <= (radius)*ROBOT_MAP_RESOLUTION && distance(ROBOT_MAP_ORIGIN_X, ROBOT_MAP_ORIGIN_Y, x, y) > (radius)*ROBOT_MAP_RESOLUTION){
						lrr(x,y) = STREL_UNSET;
					}
				}
			}

			CImg <double> lrrConnected(lrr);
			lrrConnected.label();
			unsigned char origin_color = lrrConnected(ROBOT_MAP_ORIGIN_X, ROBOT_MAP_ORIGIN_Y);
			cimg_forXY(lrr, x, y){
				if (lrr(x,y) == STREL_SET){
					if (lrrConnected(x,y) != origin_color){
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

			CImg <double> distanceFromUnexplored(lsr);
			distanceFromUnexplored.distance(UNEXPLORED);

			CImg <double> distanceFromObstacle(lsr);
			distanceFromObstacle.distance(OBSTACLE);

			//distanceFromLF.display();
			//distanceFromUnexplored.display();

			lirRaffle.clear();

			cimg_forXY(distanceFromLF, x, y){
				if (lrr(x,y) == BACKGROUND && distanceFromLF(x,y) <= (Rp-radius)*ROBOT_MAP_RESOLUTION){
					lir(x, y) = BACKGROUND;
					double globalCoordinateX = toGlobalCoordinateX(x);
					double globalCoordinateY = toGlobalCoordinateY(y);
					double* coordinates = new double[2];
					coordinates[0] = globalCoordinateX;
					coordinates[1] = globalCoordinateY;
					double d = distance(x,y, ROBOT_MAP_ORIGIN_X, ROBOT_MAP_ORIGIN_Y);
					int freq = 1;
					if (distanceFromLF(x,y) <= (Rp*0.3)*ROBOT_MAP_RESOLUTION){
						freq = 10;
					}
					for (int i = 0; i < freq; i++){
						lirRaffle.push_back(coordinates);
					}
				}
			}

			std::cout << "Finished calculating lir\n";

//			std::cout << "Showing lir\n";
//			lir.display();

		}

};

class Robot{
	public:
		std::vector <Robot*>* allRobots;
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
		int gpaCoupling;
		int geaCoupling;
		int id;
		bool synchronized;

		//Global Pose..
		Pose targetPose;
		Robot(int id, int mapHeight, int mapWidth, std::vector <Robot*>* allRobots) :
			id(id), srg(NULL), current(NULL), state(STARTUP), startup(true), iterations(0), mapHeight(mapHeight), mapWidth(mapWidth),
			backtracking(false), allRobots(allRobots), gpaCoupling(-1), synchronized(false), geaCoupling(-1){
			map = CImg <unsigned char>(mapWidth, mapHeight);
			map = UNEXPLORED;
		}

		void updateGlobalMapFromOthers(CImg<unsigned char>* otherMap){
			cimg_forXY(map, x, y){
				unsigned char currentValue = map(x, y);
				unsigned char otherMapValue = otherMap->operator()(x,y);
				if (currentValue == OBSTACLE){
					if (otherMapValue == FREE){
						map(x, y) = FREE;
					}
				}else if (currentValue == UNEXPLORED){
					if (otherMapValue == OBSTACLE || otherMapValue == FREE){
						map(x, y) = otherMapValue;
					}
				}
			}
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
			return !((fabs(lastVelocity.x)+0.01 > fabs(v.x)  && fabs(v.x) == 0));
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
	/*std::cout << "GPA Couplings:\n";
	for (int i = 0; i < robot->allRobots->size(); i++){
		std::cout << i << ": " << robot->allRobots->operator[](i)->gpaCoupling << "\n";
	}*/
	if (robot->state == STORE_POSITION){
		std::cout << "storing position..\n";
		Pose currentPose = robot->position->GetGlobalPose();
		std::vector <ModelRanger::Sensor> sensors = robot->ranger->GetSensorsMutable();
		std::vector <double> bearings;
		std::vector <meters_t> ranges;
		std::vector <double> fov;

		std::cout << "Global Coordinates: " << currentPose.x << "," << currentPose.y << "\n";
		
		int q = 0;
		for (int i = 0; i < sensors.size(); i++){
			if (sensors[i].fov != 0){
				double perception = sensors[i].ranges[0];
				q++;
				int j = i+1;
				while (j < sensors.size() && sensors[j].fov == 0){
					if (sensors[j].ranges[0] < perception){
						perception = sensors[j].ranges[0];
					}
					j++;
					q++;
				}
				ranges.push_back(perception);
				bearings.push_back(WorldAngle360(currentPose.a+sensors[i].pose.a));
				fov.push_back(HumanAngle(sensors[i].fov));
			}
		}
		std::cout << "Processed " << q << " readings\n";

		SRGNode* currentNode;

		if (robot->backtracking){
			currentNode = robot->backtrackTarget;
		}else if (robot->iterations == 0){
			currentNode = new SRGNode(NULL, currentPose, bearings, fov, ranges, &(robot->map), robot->mapWidth, robot->mapHeight, robot->id, robot->srgList.size(), false);
			robot->srgList.push_back(currentNode);
			robot->targetPose = currentPose;
		}else{
			currentNode = new SRGNode(robot->current, currentPose, bearings, fov, ranges, &(robot->map), robot->mapWidth, robot->mapHeight, robot->id, robot->srgList.size(), false);
			robot->srgList.push_back(currentNode);
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
		std::cout << "Size of lsr bag: " << robot->current->lsrGlobalMapCoordinates.size() << "\n";

		robot->iterations++;

		std::cout << "Iterations done: " << robot->iterations << "\n";

		if (robot->iterations%100 == 0){
			robot->map.display();
		}
		//robot->map.display();
	}

	int geaCouplingId = 0;
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
		case STARTUP:
			robot->state = STORE_POSITION;
			break;
		case SYNCHRONIZING:
			if (robot->synchronized){
				return 0;
			}
			std::cout << "Synchronizing..\n";
			for (int i = 0; i < robot->allRobots->size(); i++){
				if (i != robot->id){
					Robot* r = robot->allRobots->operator[](i);
					if (r->gpaCoupling == robot->gpaCoupling){
						if (r->state != SYNCHRONIZING){
							return 0;
						}
					}
				}
			}
			
			std::cout << "GPA Couplings:\n";
			for (int i = 0; i < robot->allRobots->size(); i++){
				std::cout << i << ": " << robot->allRobots->operator[](i)->gpaCoupling << "\n";
			}

			for (int i = 0; i < robot->allRobots->size(); i++){
				for (int j = 0; j < robot->allRobots->size(); j++){
					if (i != j){
						Robot* r1 = robot->allRobots->operator[](i);
						Robot* r2 = robot->allRobots->operator[](j);
						r1->updateGlobalMapFromOthers(&(r2->map));
					}
				}
			}

			for (int i = 0; i < robot->allRobots->size(); i++){
				for (int j = 0; j < robot->allRobots->size(); j++){
					if (i != j){
						Robot* r1 = robot->allRobots->operator[](i);
						Robot* r2 = robot->allRobots->operator[](j);
						if (r1->gpaCoupling == r2->gpaCoupling && r1->gpaCoupling == robot->gpaCoupling){
//							std::cout << "Checking if LSR coupled: " << r1->name << " and " << r2->name << "\n";
							if (r1->current->isLSRCoupled(r2->current)){
//								std::cout << "LSR COUPLED: " << r1->name << " and " << r2->name << "\n";
								if (r1->geaCoupling == -1 && r2->geaCoupling == -1){
									r1->geaCoupling = geaCouplingId;
									r2->geaCoupling = geaCouplingId;
									geaCouplingId++;
								}else if (r1->geaCoupling != -1){
									r2->geaCoupling = r1->geaCoupling;
								}else if (r2->gpaCoupling != -1){
									r1->geaCoupling = r2->geaCoupling;
								}
							}
						}
					}
				}
			}

			std::cout << "GEA Couplings:\n";
			for (int i = 0; i < robot->allRobots->size(); i++){
				Robot* r = robot->allRobots->operator[](i);
				if (robot->gpaCoupling == r->gpaCoupling){
					std::cout << i << ": " << r->geaCoupling << "\n";
				}
			}
				
			/*std::cout << "Finding connection....\n";
			bool foundConnection = false;
			for (int i = 0; !foundConnection && i < robot->allRobots->size(); i++){
				for (int j = 0; !foundConnection && j < robot->allRobots->size(); j++){
					if (i != j){
						Robot* r1 = robot->allRobots->operator[](i);
						Robot* r2 = robot->allRobots->operator[](j);
						for (int k = 0; !foundConnection && k < r1->srgList.size();k++){
							for (int l = 0; !foundConnection && l < r2->srgList.size();l++){
								if (r1->srgList[k]->isLIRSharedWith(r2->srgList[l])){
									std::cout << "Found connection!!!!";
									//connect the two maps
									foundConnection = true;
								}
							}
						}
							
					}
				}
			}*/
			for (int i = 0; i < robot->allRobots->size(); i++){
				Robot* r = robot->allRobots->operator[](i);
//				r->map.display();
				r->synchronized = true;
			}
			break;
		case STORE_POSITION:
			std::cout << "Current Coupling of " << robot->name << ": " << robot->gpaCoupling << "\n";
			if (robot->gpaCoupling != -1){
				std::cout << "Stored position, trying to synchronize..\n";
				robot->synchronized = false;
				robot->geaCoupling = -1;
				robot->state = SYNCHRONIZING;
			}else{
				std::cout << "Is not coupled....\n";
				robot->state =  READY;
			}
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
			msrg->compute_gpa_coupling();
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

		void compute_gpa_coupling(){
			/*std::cout << "Computing couplings..\n";
			for (int i = 0; i < robots.size(); i++){
				std::cout << "current couplings for " << robots[i]->name << ": " << robots[i]->gpaCoupling << "\n";
				std::cout << "current position: " << robots[i]->targetPose.x << ", " << robots[i]->targetPose.y << "\n";
				robots[i]->gpaCoupling = -1;
			}*/
			int couplingId = 0;
			for (int i = 0; i < robots.size(); i++){
				for (int j = 0; j < robots.size(); j++){
					if (j != i){
						if (robots[i]->targetPose.Distance2D(robots[j]->targetPose) <= 2*Rp){
							if (robots[i]->gpaCoupling == -1 && robots[j]->gpaCoupling == -1){
								robots[i]->gpaCoupling = couplingId;
								robots[j]->gpaCoupling = couplingId;
								couplingId++;
							}else if (robots[i]->gpaCoupling != -1){
								robots[j]->gpaCoupling = robots[i]->gpaCoupling;
							}else if (robots[j]->gpaCoupling != -1){
								robots[i]->gpaCoupling = robots[j]->gpaCoupling;
							}
						}
					}
				}
			}
		}

		//connect to world bots
		void connect(World* world){
			Model* floorplan = world->GetModel("msrg_floorplan");
			Geom floorplanGeom = floorplan->GetGeom();
			Size floorplanSize = floorplanGeom.size;
			int mapHeight = floorplanSize.y*ROBOT_MAP_RESOLUTION;
			int mapWidth = floorplanSize.x*ROBOT_MAP_RESOLUTION;
			for (int idx = 0; ; idx++){
				std::stringstream name;
				name << "msrg" << idx;
				
				std::cout << name.str() << "\n";

				if ((world->GetModel(name.str())) == NULL){
					break;
				}


				ModelPosition* posmod = reinterpret_cast<ModelPosition*>(world->GetModel(name.str()));

				ModelRanger* ranger = reinterpret_cast<ModelRanger*>(posmod->GetUnusedModelOfType( "ranger" ));

				Robot* robot = new Robot(idx, mapHeight, mapWidth, &robots);
				robot->name = name.str();
				robot->position = posmod;
				robot->position->AddCallback(Model::CB_UPDATE, (model_callback_t)PositionUpdate, robot);
				robot->position->Subscribe();
				robot->ranger = ranger;
				robot->ranger->Subscribe();
				robot->lastVelocity = robot->position->GetVelocity();
				robot->targetPose = robot->position->GetGlobalPose();
				robots.push_back(robot);
			}

			world->AddUpdateCallback(MSRG::Callback, reinterpret_cast<void*>(this));
		}

		void ComputeWifiConnectivity(){
			/*for (int i = 0; i < robots.size(); i++){
				robots[i]->robotsInWifiRange.clear();
				for (int j = 0; j < robots.size(); j++){
					if (j != i){
						Pose iPose = robots[i]->position->GetPose();
						Pose jPose = robots[j]->position->GetPose();
						if (iPose.Distance2D(jPose) <= Rc){
							robots[i]->robotsInWifiRange.push_back(j);
						}
					}
				}
			}*/
		}

		void DebugInformation(){
			if (robots[0]->state == STOPPED && !displayedResults){
				for (int i = 0; i < robots.size(); i++){
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
