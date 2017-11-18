/*
 * PathFinder.h
 *
 *  Created on: Nov 7, 2017
 *      Author: anilnayak
 */

#ifndef PATHFINDER_H_
#define PATHFINDER_H_
#define MAX_OBSTACLES 25
#define X_ARENA 3.66
#define Y_ARENA 3.05
#define ROBOT_X_BOUND 0.21 //0.196
#define ROBOT_Y_BOUND 0.21 //0.147
#define OBSTACLE_X_BOUND 0.315 //0.305
#define OBSTACLE_Y_BOUND 0.315
#define MOTOR_SPEED 20
//Configurations for the robot to move
#define MOVE 0.25
// #define epsilon 0.001
#define factor 2*100


#include <queue>
#include <list>
#include <vector>
#include <stack>
#include <deque>
#include <cmath>
#include <ev3.h>
#include <string>
#include <iostream>
#include <limits>

using namespace std;
	/* start location */
	static const float start[2] = {0.305, 0.61}; //2.135  1.524
	/* obstacle locations */
	static const float obstacle[MAX_OBSTACLES][2] = {
			{0.915, 0.305},
			{0.915, 0.61},
			{0.915, 0.915},
			{0.915, 1.219},
			{0.915, 1.524},
			{2.135, 1.219},
			{2.135, 1.524},
			{2.135, 1.829},
			{2.135, 2.134},
			{2.135, 2.439},
			{2.135, 2.744},
			{-1,-1},
			{-1,-1},
			{-1,-1},
			{-1,-1},
			{-1,-1},
			{-1,-1},
			{-1,-1},
			{-1,-1},
			{-1,-1},
			{-1,-1},
			{-1,-1},
			{-1,-1},
			{-1,-1},
			{-1,-1}
	};
	/* goal location */
	static const  float goal[2] = {3.05, 2.135};
	static const int num_obstacles = 13;

class PathFinder {
public:
	PathFinder();
	bool goal_found(struct Node node);
	void simplify_path();
	void run_into_path();
	void find_last_goal_center(float a,float b,int c);
	bool check_boundary(struct Node node);
	bool check_obstacle(struct Node node);
	bool contact_with_object(float x,float y,float x1,float y1);
	int find_path();
	void complete_path(struct Node *node);
	void move(float distance);
	void turn(int direction);
	void computepath();
	void path_find_from_goal();
	int goal_node_detail(struct Node* node);
	struct Node createNode(float x, float y, int direction, float px, float py, int pdirection, struct Node* parent,int start);
	struct Node createMapNode(float x, float y,float px, float py, int pdirection, int start);
	void find_parent_of_diff_node(float x,float y);
	struct Node createRootNode(float x, float y,int start);
	bool isNodeCreatedBefore(struct Node node);
	bool checkEqual(float a,float b);
	bool checkValidMove(struct Node node,float px, float py);
	bool isNodeSafeToCreate(float px,float py,struct Node node);
	void post_processing_goal_found();
	queue<Node> map_learning_queue;
	deque<Node> map_learning_queue_d;
	queue<Node> goal_queue;
	stack<Node> path_queue;
	stack<Node> additional;
	queue<Node> follow_path;
	list<Node> map_coordinates;
};

#endif /* PATHFINDER_H_ */
