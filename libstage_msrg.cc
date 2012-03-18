#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <sstream>
#include <iostream>
#include <cmath>
#include <set>
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
const double OBSTACLE_DILATION_MAX_DISTANCE = 40;
const int LRR_EROSION_EPSILON = 10;
const int ROBOT_MAP_HEIGHT = (int)ceil(Rp*ROBOT_MAP_RESOLUTION*2)+1;
const int ROBOT_MAP_WIDTH = (int)ceil(Rp*ROBOT_MAP_RESOLUTION*2)+1;
const int ROBOT_MAP_ORIGIN_X = ROBOT_MAP_WIDTH/2;
const int ROBOT_MAP_ORIGIN_Y = ROBOT_MAP_HEIGHT/2;
const int POLAR_UNIT = 1.41421;
const CImg <unsigned char> NO_BACKGROUND(ROBOT_MAP_WIDTH, ROBOT_MAP_HEIGHT, 1, 1, FOREGROUND);
const CImg <unsigned char> NO_FOREGROUND(ROBOT_MAP_WIDTH, ROBOT_MAP_HEIGHT, 1, 1, BACKGROUND);

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

// Returns 1 if the lines intersect, otherwise 0. In addition, if the lines 
// intersect the intersection point may be stored in the floats i_x and i_y.
bool get_line_intersection(int p0_x, int p0_y, int p1_x, int p1_y, 
    int p2_x, int p2_y, int p3_x, int p3_y, int *i_x, int *i_y)
{
    float s1_x, s1_y, s2_x, s2_y;
    s1_x = p1_x - p0_x;     s1_y = p1_y - p0_y;
    s2_x = p3_x - p2_x;     s2_y = p3_y - p2_y;

    float s, t;
    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y);
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y);

    if (s >= 0 && s <= 1 && t >= 0 && t <= 1)
    {
        // Collision detected
        if (i_x != NULL)
            *i_x = p0_x + (t * s1_x);
        if (i_y != NULL)
            *i_y = p0_y + (t * s1_y);
        return true;
    }

    return false; // No collision
}

bool areCoordinatesSortedDouble(double* coord1, double* coord2){
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
		std::vector <int*> lsrObstacleGlobalMapCoordinates;
		std::vector <int*> lsrGlobalMapLinePoints;
		std::vector <int*> lsrObstacleGlobalMapLinePoints;
		std::vector <int*> lrrGlobalMapCoordinates;
		int fromRobot;
		int fromRobotIndex;

		SRGNode(SRGNode* parent, Pose pose, std::vector <double> bearings, std::vector <double> fov, std::vector <meters_t> ranges,
			CImg <unsigned char>* globalMap, int globalMapWidth, int globalMapHeight, int fromRobot, int fromRobotIndex, bool bridge) : 
			parent(parent), pose(pose), bearings(bearings), fov(fov), ranges(ranges),
			radius(0.40), globalMap(globalMap), globalMapWidth(globalMapWidth), globalMapHeight(globalMapHeight),
			fromRobot(fromRobot), fromRobotIndex(fromRobotIndex), bridge(bridge) {
			
		}

		//Copy constructor
		SRGNode(SRGNode* nodeToCopy, double newRadius, CImg<unsigned char>* newGlobalMap, int newGlobalMapWidth, int newGlobalMapHeight) : 
			parent(NULL), pose(nodeToCopy->pose), bearings(nodeToCopy->bearings), fov(nodeToCopy->fov), ranges(nodeToCopy->ranges),
			radius(nodeToCopy->radius), globalMap(newGlobalMap), globalMapWidth(newGlobalMapWidth), globalMapHeight(newGlobalMapHeight),
			fromRobot(nodeToCopy->fromRobot), fromRobotIndex(nodeToCopy->fromRobotIndex), bridge(false) {
			//calculate_lsr();
		}

		bool isLSRCoupled(SRGNode* other){
			for (int i = 0; i < other->lsrGlobalMapCoordinates.size(); i++){
				if (binary_search(lsrGlobalMapCoordinates.begin(), lsrGlobalMapCoordinates.end(), other->lsrGlobalMapCoordinates[i], areCoordinatesSorted)){
					return true;
				}
			}
			return false;
		}

		bool isLRRCoupled(SRGNode* other){
			for (int i = 0; i < other->lrrGlobalMapCoordinates.size(); i++){
				if (binary_search(lrrGlobalMapCoordinates.begin(), lrrGlobalMapCoordinates.end(), other->lrrGlobalMapCoordinates[i], areCoordinatesSorted)){
					return true;
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

		int localMapXToGlobalMapCoordinateX(int localMapX){
			int globalMapX = toGlobalMapCoordinateX(pose.x);
			int localTopLeftX = globalMapX-(ROBOT_MAP_WIDTH/2);
			return globalMapX + localMapX;
		}

		int localMapYToGlobalMapCoordinateY(int localMapY){
			int globalMapY = toGlobalMapCoordinateY(pose.y);
			int localTopLeftY = globalMapY-(ROBOT_MAP_HEIGHT/2);
			return globalMapY + localMapY;
		}

		int toGlobalMapCoordinateX(double globalX){
			return ((globalMapWidth/2)+floor(globalX * ROBOT_MAP_RESOLUTION)) - (ROBOT_MAP_RESOLUTION*2);
		}

		int toGlobalMapCoordinateY(double globalY){
			return ((globalMapHeight/2)-floor(globalY * ROBOT_MAP_RESOLUTION)) + (ROBOT_MAP_RESOLUTION*2);
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
					if (globalMap->atXY(prospectX, prospectY) == UNEXPLORED){
						return true;
					}
				}
			}
			return false;
		}

		void store_lsr(){
			lsrGlobalMapLinePoints.clear();

			for (int i = 0; i < ROBOT_MAP_HEIGHT; i++){
				for (int j = 0; j < ROBOT_MAP_WIDTH; j++){
					if (lsr(j,i) == BACKGROUND){
						int j_start = j;
						while (lsr(j,i) == BACKGROUND && j < ROBOT_MAP_WIDTH){
							j++;
						}
						int j_end = j-1;
						int* lineCoordinates = new int[4];
						lineCoordinates[0] = localMapXToGlobalMapCoordinateX(j_start);
						lineCoordinates[1] = localMapYToGlobalMapCoordinateY(i);
						lineCoordinates[2] = localMapXToGlobalMapCoordinateX(j_end);
						lineCoordinates[3] = localMapYToGlobalMapCoordinateY(i);
						lsrGlobalMapLinePoints.push_back(lineCoordinates);
					}
				}
			}

			/*lsrGlobalMapCoordinates.clear();

			cimg_forXY(lsr, x, y){
				if (lsr(x,y) == BACKGROUND){
					int globalMapX = localMapXToGlobalMapCoordinateX(x);
					int globalMapY = localMapYToGlobalMapCoordinateY(y);
					int* coordinates = new int[2];
					coordinates[0] = globalMapX;
					coordinates[1] = globalMapY;
					lsrGlobalMapCoordinates.push_back(coordinates);
				}
			}

			sort(lsrGlobalMapCoordinates.begin(), lsrGlobalMapCoordinates.end(), areCoordinatesSorted);*/
		}

		void store_lsr_obstacles(){
			lsrObstacleGlobalMapLinePoints.clear();

			for (int i = 0; i < ROBOT_MAP_HEIGHT; i++){
				for (int j = 0; j < ROBOT_MAP_WIDTH; j++){
					if (lsr(j,i) == FOREGROUND){
						int j_start = j;
						while (lsr(j,i) == FOREGROUND && j < ROBOT_MAP_WIDTH){
							j++;
						}
						int j_end = j-1;
						int* lineCoordinates = new int[4];
						lineCoordinates[0] = localMapXToGlobalMapCoordinateX(j_start);
						lineCoordinates[1] = localMapYToGlobalMapCoordinateY(i);
						lineCoordinates[2] = localMapXToGlobalMapCoordinateX(j_end);
						lineCoordinates[3] = localMapYToGlobalMapCoordinateY(i);
						lsrObstacleGlobalMapLinePoints.push_back(lineCoordinates);
					}
				}
			}
			/*lsrObstacleGlobalMapCoordinates.clear();

			cimg_forXY(lsr, x, y){
				if (lsr(x,y) == FOREGROUND){
					int globalMapX = localMapXToGlobalMapCoordinateX(x);
					int globalMapY = localMapYToGlobalMapCoordinateY(y);
					int* coordinates = new int[2];
					coordinates[0] = globalMapX;
					coordinates[1] = globalMapY;
					lsrObstacleGlobalMapCoordinates.push_back(coordinates);
				}
			}

			sort(lsrObstacleGlobalMapCoordinates.begin(), lsrObstacleGlobalMapCoordinates.end(), areCoordinatesSorted);*/
		}

		void store_lrr(){
			lrrGlobalMapCoordinates.clear();

			cimg_forXY(lrr, x, y){
				if (lrr(x,y) == BACKGROUND){
					int globalMapX = localMapXToGlobalMapCoordinateX(x);
					int globalMapY = localMapYToGlobalMapCoordinateY(y);
					int* coordinates = new int[2];
					coordinates[0] = globalMapX;
					coordinates[1] = globalMapY;
					lrrGlobalMapCoordinates.push_back(coordinates);
				}
			}

			sort(lrrGlobalMapCoordinates.begin(), lrrGlobalMapCoordinates.end(), areCoordinatesSorted);
		}

		void calculate_node_data(){
			calculate_lsr();
			update_lsr();
			update_global_map();
			calculate_lrr();
			calculate_lf();
			calculate_lir();
			lir.clear();
			lf.clear();
			lrr.clear();
			lsr.clear();
		}

		void update_lsr(){
//			std::cout << "Updating LSR\n";

			cimg_forXY(lsr, x, y){
				if (distance(ROBOT_MAP_ORIGIN_X, ROBOT_MAP_ORIGIN_Y, x, y) <= Rp*ROBOT_MAP_RESOLUTION){
					int globalMapX = localMapXToGlobalMapCoordinateX(x);
					int globalMapY = localMapYToGlobalMapCoordinateY(y);
					if (isValidXCoordinate(globalMapX) && isValidYCoordinate(globalMapY)){
						unsigned char currentValue = globalMap->atXY(globalMapX, globalMapY);
						if (currentValue == FREE){
							lsr(x,y) = FREE;
						}else if (currentValue == OBSTACLE){
							if (lsr(x,y) == UNEXPLORED){
								lsr(x,y) = OBSTACLE;
							}
						}
					}
				}
			}

			store_lsr();
			store_lsr_obstacles();

//			std::cout << "Finished updating lsr\n";

//			std::cout << "Showing updated lsr\n";
//			lsr.display();
		}

		void calculate_lsr(){
//			std::cout << "Calculating lsr\n";

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

			distanceFromObstacles.clear();

			store_lsr();
			store_lsr_obstacles();

//			std::cout << "Finished calculating lsr\n";

//			std::cout << "Showing lsr\n";
//			lsr.display();
		}

		void update_global_map(){
//			std::cout << "top left global map " << localMapXToGlobalMapCoordinateX(0) << ", " << localMapYToGlobalMapCoordinateY(0) << "\n";
//			std::cout << "top left global " << toGlobalCoordinateX(0) << ", " << toGlobalCoordinateY(0) << "\n";

			for (int i = 0; i < lsrGlobalMapLinePoints.size(); i++){
				int* h_points = lsrGlobalMapLinePoints[i];
				for (int j = h_points[0]; j <= h_points[2]; j++){
					globalMap->atXY(j, h_points[1]) = FREE;
				}
			}
			for (int i = 0; i < lsrObstacleGlobalMapLinePoints.size(); i++){
				int* h_points = lsrObstacleGlobalMapLinePoints[i];
				for (int j = h_points[0]; j <= h_points[2]; j++){
					if (globalMap->atXY(j, h_points[1]) == UNEXPLORED){
						globalMap->atXY(j, h_points[1]) = OBSTACLE;
					}
				}
			}

			/*for (int i = 0; i < lsrGlobalMapCoordinates.size(); i++){
				int* coordinates = lsrGlobalMapCoordinates[i];
				if (isValidXCoordinate(coordinates[0]) && isValidYCoordinate(coordinates[1])){
					globalMap->atXY(coordinates[0], coordinates[1]) = FREE;
				}
			}
			for (int i = 0; i < lsrObstacleGlobalMapCoordinates.size(); i++){
				int* coordinates = lsrObstacleGlobalMapCoordinates[i];
				if (isValidXCoordinate(coordinates[0]) && isValidYCoordinate(coordinates[1])){
					if (globalMap->atXY(coordinates[0], coordinates[1]) == UNEXPLORED){
						globalMap->atXY(coordinates[0], coordinates[1]) = OBSTACLE;
					}
				}
			}*/

			std::cout << "Finished updating global map\n";

			std::cout << "Showing global map\n";
			globalMap->display();
		}

		//only emulates erosion using distance transform..
		void calculate_lrr(){
//			std::cout << "Calculating lrr\n";
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
			lrrDist.clear();

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
			lrrConnected.clear();

			store_lrr();

//			std::cout << "Finished calculating lrr\n";

//			std::cout << "Showing lrr\n";
//			lrr.display();
		}

		void calculate_lf(){
//			std::cout << "Calculating lf\n";
			lf = CImg <unsigned char>(ROBOT_MAP_HEIGHT, ROBOT_MAP_WIDTH);
			lf = FOREGROUND;

			cimg_forXY(lsr, x, y){
				int globalMapX = localMapXToGlobalMapCoordinateX(x);
				int globalMapY = localMapYToGlobalMapCoordinateY(y);
				if (lsr(x,y) == BACKGROUND && isLF(globalMapX, globalMapY)){
					lf(x, y) = BACKGROUND;
				}
			}
//			std::cout << "Finished calculating lf\n";

//			std::cout << "Showing lf\n";
//			lf.display();
		}

		void calculate_lir(){
//			std::cout << "Calculating lir\n";

			lir = CImg <unsigned char>(ROBOT_MAP_HEIGHT, ROBOT_MAP_WIDTH);
			lir = FOREGROUND;

//			std::cout << "Calculating distance from lf matrix\n";

			CImg <double> distanceFromLF(lf);
			distanceFromLF.distance(BACKGROUND);

			//distanceFromLF.display();
			//distanceFromUnexplored.display();

			lirRaffle.clear();

			cimg_forXY(distanceFromLF, x, y){
				if (lrr(x,y) == BACKGROUND && distanceFromLF(x,y) <= Rp*ROBOT_MAP_RESOLUTION){
					lir(x, y) = BACKGROUND;
					double globalCoordinateX = toGlobalCoordinateX(x);
					double globalCoordinateY = toGlobalCoordinateY(y);
					double* coordinates = new double[2];
					coordinates[0] = globalCoordinateX;
					coordinates[1] = globalCoordinateY;
					double d = distance(x,y, ROBOT_MAP_ORIGIN_X, ROBOT_MAP_ORIGIN_Y);

					int freq = (10-(distanceFromLF(x,y)/((Rp*ROBOT_MAP_RESOLUTION)/10)));

					freq = ceil(pow(2,freq));

					for (int i = 0; i < freq; i++){
						lirRaffle.push_back(coordinates);
					}
				}
			}

			distanceFromLF.clear();

//			std::cout << "Finished calculating lir\n";

//			std::cout << "Showing lir\n";
//			lir.display();

		}

};

class SRGNodeCoordinates{
	public:
		double x;
		double y;
		SRGNode* node;

		SRGNodeCoordinates(SRGNode* node) :
			node(node), x(node->pose.x), y(node->pose.y){

		}

		SRGNodeCoordinates(double x, double y) :
			x(x), y(y){

		}
};

bool areCoordinatesSortedSRGNodeCoordinates(SRGNodeCoordinates coord1, SRGNodeCoordinates coord2){
	if (coord1.x < coord2.x){
		return true;
	}else if (coord1.x > coord2.x){
		return false;
	}else if (coord1.y < coord2.y){
		return true;
	}else if (coord1.y > coord2.y){
		return false;
	}else{
		return false;
	}
}

struct SRGNodeCoordinatesComparator{
	bool operator() (const SRGNodeCoordinates lhs, const SRGNodeCoordinates rhs) const {
		return areCoordinatesSortedSRGNodeCoordinates(lhs, rhs);
	}
};

class Robot{
	public:
		std::vector <Robot*>* allRobots;
		bool startup;
		long iterations;
		std::string name;
		ModelPosition* position;
		ModelRanger* ranger;
		std::vector <int> robotsInWifiRange;
		std::vector <SRGNode*> srgList;
		std::vector <SRGNode*> unconnectedNodes;
		std::set <SRGNodeCoordinates, SRGNodeCoordinatesComparator> allNodes;
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
		double radius;
		bool synchronized;
		bool master;

		//Global Pose..
		Pose targetPose;
		Robot(int id, int mapHeight, int mapWidth, std::vector <Robot*>* allRobots) :
			id(id), srg(NULL), current(NULL), state(STORE_POSITION), startup(true), iterations(0), mapHeight(mapHeight), mapWidth(mapWidth),
			allRobots(allRobots), gpaCoupling(-1), synchronized(false), geaCoupling(-1), master(false), radius(0.4){
			map = CImg <unsigned char>(mapWidth, mapHeight);
			map = UNEXPLORED;
		}

		void addNode(SRGNode* node){
			SRGNodeCoordinates coord(node);
			std::set<SRGNodeCoordinates, SRGNodeCoordinatesComparator>::iterator existing = allNodes.find(coord);
			if (existing == allNodes.end()){
				allNodes.insert(coord);
			}
		}

		void updateGlobalMapFromOthers(CImg<unsigned char>* otherMap){
			cimg_forXY(map, x, y){
				unsigned char currentValue = map(x, y);
				unsigned char otherMapValue = otherMap->atXY(x,y);
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

		void mergeNodesWith(Robot* r2){
			for (int i = 0; i < r2->srgList.size(); i++){
				SRGNodeCoordinates coord(r2->srgList[i]);
				std::set<SRGNodeCoordinates, SRGNodeCoordinatesComparator>::iterator existing = allNodes.find(coord);
				if (existing == allNodes.end()){
					SRGNode* toInsert = new SRGNode(coord.node, radius, &map, mapWidth, mapHeight);
					SRGNodeCoordinates coordToInsert(toInsert);
					unconnectedNodes.push_back(coordToInsert.node);
					allNodes.insert(coordToInsert);
				}else{
					/*(*existing).node->calculate_lrr();
					(*existing).node->calculate_lf();
					(*existing).node->calculate_lir();*/
				}
			}
			for (int i = 0; i < r2->unconnectedNodes.size(); i++){
				SRGNodeCoordinates coord(r2->unconnectedNodes[i]);
				std::set<SRGNodeCoordinates, SRGNodeCoordinatesComparator>::iterator existing = allNodes.find(coord);
				if (existing == allNodes.end()){
					SRGNode* toInsert = new SRGNode(coord.node, radius, &map, mapWidth, mapHeight);
					SRGNodeCoordinates coordToInsert(toInsert);
					unconnectedNodes.push_back(coordToInsert.node);
					allNodes.insert(coord);
				}else{
					/*(*existing).node->calculate_lrr();
					(*existing).node->calculate_lf();
					(*existing).node->calculate_lir();*/
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
	if (robot->state == STORE_POSITION){
		std::cout << "storing position..\n";
		Pose currentPose = robot->position->GetGlobalPose();
		std::vector <ModelRanger::Sensor> sensors = robot->ranger->GetSensorsMutable();
		std::vector <double> bearings;
		std::vector <meters_t> ranges;
		std::vector <double> fov;

//		std::cout << "Global Coordinates: " << currentPose.x << "," << currentPose.y << "\n";
		
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
//		std::cout << "Processed " << q << " readings\n";

		SRGNode* currentNode;

		bool backtracked = false;

		SRGNodeCoordinates coord(robot->targetPose.x, robot->targetPose.y);
		std::set<SRGNodeCoordinates, SRGNodeCoordinatesComparator>::iterator existing = robot->allNodes.find(coord);

		backtracked = existing != robot->allNodes.end();

		if (backtracked){
			currentNode = (*existing).node;
		}else if (robot->iterations == 0){
			currentNode = new SRGNode(NULL, currentPose, bearings, fov, ranges, &(robot->map), robot->mapWidth, robot->mapHeight, robot->id, robot->srgList.size(), false);
			robot->addNode(currentNode);
			robot->srgList.push_back(currentNode);
			robot->targetPose = currentPose;
		}else{
			currentNode = new SRGNode(robot->current, currentPose, bearings, fov, ranges, &(robot->map), robot->mapWidth, robot->mapHeight, robot->id, robot->srgList.size(), false);
			robot->addNode(currentNode);
			robot->srgList.push_back(currentNode);
		}

		/*TODO:
		//within bounding box
		//find node with LRR intersection with current node
		//check if directly bridgeable, create bridge..
		//if indirectly bridgeable, create intermediate node..
		//add bridges..
		*/

		currentNode->bearings = bearings;
		currentNode->ranges = ranges;
		currentNode->fov = fov;
	
		currentNode->calculate_node_data();	
		//try to deallocate imgs..
		
		if (robot->iterations == 0){
			robot->srg = currentNode;
		}else{
			if (!backtracked){
				robot->current->children.push_back(currentNode);
			}
		}

		robot->current = currentNode;

		robot->iterations++;

		std::cout << "Iterations done: " << robot->iterations << "\n";

		if (robot->iterations%100 == 0){
			robot->map.display();
		}
		//robot->map.display();
		robot->master = false;

		if (robot->current->lirRaffle.size() == 0){
			/**
			 * TODO: search space Rp-radius
			 * if node found with non-empty LIR, check if already connected
			 * if connected
			 *   check nodes within Rp-radius of node
			 *   find path to node, move one node closer to selected node
			 * if not connected, check if nodes within Rp-radius of unconnectedNode have intersection of LRR
			 *   after, plan a path to node with intersection of LRR with unconnectedNode
			 *   move one node closer to selected node
			**/
			if (robot->current->parent == NULL){
				std::cout << robot->name << " is back at starting position, stopping because no more LIR!!!!\n";
				robot->state = STOPPED;
				robot->map.display();
				return 0;
			}else{
				std::cout << "Backtracking to parent..\n";
				std::cout << "(" << robot->current->parent->pose.x << "," << robot->current->parent->pose.y << ")\n";
				robot->SetTarget(Pose(robot->current->parent->pose.x, robot->current->parent->pose.y,0,0));
			}
		}else{
			int randomIndex = random()%robot->current->lirRaffle.size();
			double* randomCoordinates = robot->current->lirRaffle[randomIndex];
			std::cout << "(" << randomCoordinates[0] << "," << randomCoordinates[1] << ")\n";
			robot->SetTarget(Pose(randomCoordinates[0], randomCoordinates[1],0,0));
		}
	}
	//std::cout << "Computing couplings..\n";
	for (int i = 0; i < robot->allRobots->size(); i++){
		Robot* r = robot->allRobots->at(i);
		//std::cout << "current couplings for " << r->name << ": " << r->gpaCoupling << "\n";
		//std::cout << "current targets: " << robot->allRobots->at(i)->targetPose.x << ", " << robot->allRobots->at(i)->targetPose.y << "\n";
		r->gpaCoupling = -1*(i+1);
	}
	int couplingId = 0;
	for (int i = 0; i < robot->allRobots->size(); i++){
		for (int j = 0; j < robot->allRobots->size(); j++){
			if (i!=j){
				Robot* r1 = robot->allRobots->at(i);
				Robot* r2 = robot->allRobots->at(j);
				if (r1->state != STOPPED && r2->state != STOPPED){ 
					//std::cout << "checking for coupling: " << r1->name << " and " << r2->name << "\n";
					if (r1->targetPose.Distance2D(r2->targetPose) <= 2*Rp){
						//std::cout << "less than 2 Rp.. Grouping together\n";
						if (r1->gpaCoupling < 0 && r2->gpaCoupling < 0){
							//std::cout << "both negative values!\n";
							r1->gpaCoupling = couplingId;
							r2->gpaCoupling = couplingId;
							couplingId++;
						}else if (r1->gpaCoupling > 0){
							r2->gpaCoupling = r1->gpaCoupling;
						}else if (r2->gpaCoupling > 0){
							r1->gpaCoupling = r2->gpaCoupling;
						}
						/*std::cout << "GPA Couplings:\n";
						for (int k = 0; k < robot->allRobots->size(); k++){
							std::cout << k << ": " << robot->allRobots->at(k)->gpaCoupling << "\n";
						}*/
					}
				}
			}
		}
	}
	/*std::cout << "GPA Couplings:\n";
	for (int i = 0; i < robot->allRobots->size(); i++){
		std::cout << i << ": " << robot->allRobots->at(i)->gpaCoupling << "\n";
	}*/

	int geaCouplingId = 0;
	switch(robot->state){
		case READY:
			if (robot->startup || robot->iterations <= MAX_ITERATIONS){
				robot->TurnToGlobalAngle();
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
				if (robot->gpaCoupling < 0){
					robot->synchronized = false;
					robot->state = READY;
				}else{
					bool hasMaster = false;
					for (int i = 0; !hasMaster && i < robot->allRobots->size(); i++){
						Robot* r1 = robot->allRobots->at(i);
						hasMaster = hasMaster | r1->master;
					}
					for (int i = 0; !hasMaster && i < robot->allRobots->size(); i++){
						Robot* r1 = robot->allRobots->at(i);
						//std::cout << robot->id << " " << r1->id << " " << r1->name  << " " << r1->gpaCoupling << "\n";
						if (r1->gpaCoupling == robot->gpaCoupling){
							if (robot->id == r1->id){
								std::cout << "GPA Couplings:\n";
								for (int j = 0; j < robot->allRobots->size(); j++){
									std::cout << j << ": " << robot->allRobots->at(j)->gpaCoupling << "\n";
								}
								std::cout << "Electing " << r1->name << " as master..\n";
								robot->state = READY;
								robot->synchronized = false;
								robot->master = true;
							}
							break;
						}
					}
				}
				return 0;
			}
			for (int i = 0; i < robot->allRobots->size(); i++){
				if (i != robot->id){
					Robot* r = robot->allRobots->at(i);
					if (r->state != STOPPED && r->gpaCoupling == robot->gpaCoupling){
						if (r->state != SYNCHRONIZING){
							return 0;
						}
					}
				}
			}
			
			for (int i = 0; i < robot->allRobots->size(); i++){
				for (int j = i+1; j < robot->allRobots->size(); j++){
					Robot* r1 = robot->allRobots->at(i);
					Robot* r2 = robot->allRobots->at(j);
					if (r1->gpaCoupling == r2->gpaCoupling && r1->gpaCoupling == robot->gpaCoupling){
						r1->updateGlobalMapFromOthers(&(r2->map));
						r2->updateGlobalMapFromOthers(&(r1->map));
					}
				}
			}

			std::cout << "Synchronizing nodes..\n";

			for (int i = 0; i < robot->allRobots->size(); i++){
				for (int j = i+1; j < robot->allRobots->size(); j++){
					Robot* r1 = robot->allRobots->at(i);
					Robot* r2 = robot->allRobots->at(j);
					if (r1->gpaCoupling == r2->gpaCoupling && r1->gpaCoupling == robot->gpaCoupling){
						//std::cout << "synchronizing srgs: " << r1->name << " and " << r2->name << "\n";

						r1->mergeNodesWith(r2);
						r2->mergeNodesWith(r1);

						/*std::cout << "node counts " << r1->name << "\n";
						std::cout << r1->srgList.size() << "\n"; 
						std::cout << r1->unconnectedNodes.size() << "\n";
						std::cout << "node counts " << r2->name << "\n";
						std::cout << r2->srgList.size() << "\n"; 
						std::cout << r2->unconnectedNodes.size() << "\n";*/
					}
				}
			}
		
			std::cout << "Finished synchronizing nodes!\n";

			for (int i = 0; i < robot->allRobots->size(); i++){
				for (int j = 0; j < robot->allRobots->size(); j++){
					if (i != j){
						Robot* r1 = robot->allRobots->at(i);
						Robot* r2 = robot->allRobots->at(j);
						if (r1->gpaCoupling == r2->gpaCoupling && r1->gpaCoupling == robot->gpaCoupling){
//							std::cout << "Checking if LSR coupled: " << r1->name << " and " << r2->name << "\n";
							if (r1->current->isLSRCoupled(r2->current)){
//								std::cout << "LSR COUPLED: " << r1->name << " and " << r2->name << "\n";
								if (r1->geaCoupling < 0 && r2->geaCoupling < 0){
									r1->geaCoupling = geaCouplingId;
									r2->geaCoupling = geaCouplingId;
									geaCouplingId++;
								}else if (r1->geaCoupling > 0){
									r2->geaCoupling = r1->geaCoupling;
								}else if (r2->gpaCoupling > 0){
									r1->geaCoupling = r2->geaCoupling;
								}
							}
						}
					}
				}
			}

			std::cout << "GEA Couplings:\n";
			for (int i = 0; i < robot->allRobots->size(); i++){
				Robot* r = robot->allRobots->at(i);
				if (robot->gpaCoupling == r->gpaCoupling){
					std::cout << i << ": " << r->geaCoupling << "\n";
				}
			}
				
			for (int i = 0; i < robot->allRobots->size(); i++){
				Robot* r = robot->allRobots->at(i);
				if (r->gpaCoupling == robot->gpaCoupling){
					//r->map.display();
					r->synchronized = true;
				}
			}
			//robot->map.display();
			break;
		case STORE_POSITION:
			//std::cout << "Current Coupling of " << robot->name << ": " << robot->gpaCoupling << "\n";
			if (robot->gpaCoupling != -1 && !robot->master){
				std::cout << "Stored position, trying to synchronize..\n";
				robot->synchronized = false;
				robot->geaCoupling = -1*((robot->id)+1);
				robot->state = SYNCHRONIZING;
			}else{
				//std::cout << "SETTING TO READY AGAIN!!\n";
				//std::cout << "Is not coupled....\n";
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

		void DebugInformation(){
			if (robots[0]->state == STOPPED && !displayedResults){
				for (int i = 0; i < robots.size(); i++){
					//std::cout << "Robot " << i << ":\nIn range: ";
					//for (std::vector<int>::iterator iter = robots[i]->robotsInWifiRange.begin(); iter != robots[i]->robotsInWifiRange.end(); ++iter){
					//	std::cout << "msrg" << *iter << ", ";
					//}
					//std::cout << "\n";
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

