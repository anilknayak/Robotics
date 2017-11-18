/*
 * PathFinder.cpp
 *
 *  Created on: Nov 7, 2017
 *      Author: anilnayak
 */

#include "PathFinder.h"


using namespace std;

struct Node{
	float x;
	float y;
	int direction; //3-forward,1-left,0-right,4-back
	float px;
	float py;
	int pdirection; //9-root,0-left,1-top,2-right,3-bottom
	Node *parent;
	int start;
};



PathFinder::PathFinder() {
	// TODO Auto-generated constructor stub
}


struct Node PathFinder::createNode(float x, float y, int direction,float px, float py, int pdirection,struct Node *parent,int start){
	
	struct Node newnode;
	newnode.x = x;
	newnode.y = y;
	newnode.direction = direction;
	newnode.parent = parent;
	newnode.px = px;
	newnode.start = start;
	newnode.py = py;
	newnode.pdirection = pdirection;

	return newnode;
	
}

struct Node PathFinder::createMapNode(float x, float y,float px,float py,int pdirection,int start){
	
	struct Node newnode;
	newnode.x = x;
	newnode.y = y;
	newnode.direction = -1;
	newnode.parent = NULL;
	newnode.px = px;
	newnode.start = start;
	newnode.py = py;
	newnode.pdirection = pdirection;

	return newnode;
	
}

struct Node PathFinder::createRootNode(float x, float y,int start){
	struct Node newnode;
	newnode.x = x;
	newnode.y = y;
	newnode.direction = -1;
	newnode.parent = NULL;
	newnode.start = start;
	newnode.px = 0;
	newnode.py = 0;
	newnode.pdirection = 0;
	return newnode;
}

int PathFinder::find_path(){
	if(map_learning_queue.empty()){
		return 0;
	}

	//Take front element from the queue to search for the neighbour elements
	struct Node node = map_learning_queue.front();
	map_learning_queue.pop();
	float oldx = node.x;
	float oldy = node.y;

	//For Right Move Coordinates
	float r_c[2] = { oldx + MOVE , oldy };
	//3-forward,1-left,0-right,4-back
	struct Node right = createNode(r_c[0],r_c[1],3,node.px,node.py,node.pdirection,&node,99);
	if(!isNodeCreatedBefore(right)){
		
		bool isValidMoveForRobot_right = checkValidMove(right,node.x,node.y);
		bool isGoalFound = goal_found(right);
		if(isGoalFound){
			struct Node right1 = createMapNode(r_c[0],r_c[1],node.x,node.y,3,node.start);
			map_coordinates.push_back(right1);
			isValidMoveForRobot_right = false;
			goal_queue.push(right);
			post_processing_goal_found();
		}

		if(isValidMoveForRobot_right){
			struct Node right1 = createMapNode(r_c[0],r_c[1],node.x,node.y,3,node.start);
			map_coordinates.push_back(right1);
			map_learning_queue.push(right);
		}
	}

	//For Right Move Coordinates
	float l_c[2] = { oldx - MOVE , oldy };
	//3-forward,1-left,0-right,4-back
	struct Node left = createNode(l_c[0],l_c[1],4,node.px,node.py,node.pdirection,&node,99);
	if(!isNodeCreatedBefore(left)){
		
		bool isValidMoveForRobot_left = checkValidMove(left,node.x,node.y);
		bool isGoalFound = goal_found(left);
		if(isGoalFound){
			struct Node left1 = createMapNode(l_c[0],l_c[1],node.x,node.y,4,node.start);
		map_coordinates.push_back(left1);
			isValidMoveForRobot_left = false;
			goal_queue.push(left);
			post_processing_goal_found();
		}

		if(isValidMoveForRobot_left){
			struct Node left1 = createMapNode(l_c[0],l_c[1],node.x,node.y,4,node.start);
		map_coordinates.push_back(left1);
			
			map_learning_queue.push(left);
			
		}
	}

	//For Right Move Coordinates
	//3-forward,[1-left],0-right,4-back
	float t_c[2] = { oldx , oldy + MOVE };
	struct Node top = createNode(t_c[0],t_c[1],1,node.px,node.py,node.pdirection,&node,99);
	if(!isNodeCreatedBefore(top)){
		
		bool isValidMoveForRobot_top = checkValidMove(top,node.x,node.y);
		bool isGoalFound = goal_found(top);
		if(isGoalFound){
			struct Node top1 = createMapNode(t_c[0],t_c[1],node.x,node.y,1,node.start);
		map_coordinates.push_back(top1);
			isValidMoveForRobot_top = false;
			goal_queue.push(top);
			post_processing_goal_found();
		}
		if(isValidMoveForRobot_top){
			struct Node top1 = createMapNode(t_c[0],t_c[1],node.x,node.y,1,node.start);
		map_coordinates.push_back(top1);
			
			map_learning_queue.push(top);
			
		}

	}
 
	//For Right Move Coordinates
	float b_c[2] = { oldx , oldy - MOVE };
	//3-forward,1-left,[0-right],4-back
	struct Node bottom = createNode(b_c[0],b_c[1],0,node.px,node.py,node.pdirection,&node,99);
	if(!isNodeCreatedBefore(bottom)){
		
		bool isValidMoveForRobot_bottom = checkValidMove(bottom,node.x,node.y);
		bool isGoalFound = goal_found(bottom);
		if(isGoalFound){
			// MODIFIED
			struct Node bottom1 = createMapNode(b_c[0],b_c[1],node.x,node.y,0,node.start);
		map_coordinates.push_back(bottom1);
			isValidMoveForRobot_bottom = false;
			goal_queue.push(bottom);
			post_processing_goal_found();
		}

		if(isValidMoveForRobot_bottom){
			struct Node bottom1 = createMapNode(b_c[0],b_c[1],node.x,node.y,0,node.start);
		map_coordinates.push_back(bottom1);
		
			map_learning_queue.push(bottom);
		
		}

	}

	find_path();
	return 0;
}

void PathFinder::post_processing_goal_found(){
	while(!map_learning_queue.empty()){
		map_learning_queue.pop();
	}
}

bool PathFinder::checkEqual(float a,float b){
	
	if(fabs(a - b) <= numeric_limits<float>::epsilon()){
		return true;
	}else{
		return false;
	}

}

bool PathFinder::isNodeSafeToCreate(float px,float py,struct Node node){
	if(px == -1.00 && py == -1.00){
		return true;
	}

	if(checkEqual(px,node.x) && checkEqual(py, node.y)){
		return false;
	}
	else{
		return true;
	}
}

bool PathFinder::check_boundary(struct Node node){
	if((node.x > X_ARENA) || (checkEqual(node.x,X_ARENA)) || (node.x < 0.0) ||  (checkEqual(node.x,0.0))){
		return true;
	}
	else if((node.y > Y_ARENA) || (checkEqual(node.y,Y_ARENA)) || (node.y < 0.0) ||  (checkEqual(node.y,0.0))){
		return true;
	}
	else{
		return false;
	}
}

bool PathFinder::goal_found(struct Node node){
	return contact_with_object(node.x,node.y,goal[0],goal[1]);
}

bool PathFinder::check_obstacle(struct Node node){
	bool isObstacle = false;
	int i = 0;
	int size_obs = 25;
	
	for(i=0;i<size_obs;i++){
		float x1 = obstacle[i][0];
		float y1 = obstacle[i][1];

		if(x1==-1.0){
			continue;
		}

		isObstacle = contact_with_object(node.x,node.y,x1,y1);

		if(isObstacle){
			break;
		}
	}

	return isObstacle;
}

bool PathFinder::contact_with_object(float x, float y, float x1, float y1){

	x = x - (ROBOT_X_BOUND/2);
	y = y - (ROBOT_Y_BOUND/2);

	x1 = x1 - (OBSTACLE_X_BOUND/2);
	y1 = y1 - (OBSTACLE_Y_BOUND/2);

	bool isObstacle = false;
	if (    ((x1>=x) && (x1<=(x+ROBOT_X_BOUND)) )
		&& (   (y+ROBOT_Y_BOUND >= y1) &&
				((y <= y1+OBSTACLE_Y_BOUND) ||
				(y >= y1 && y+ROBOT_Y_BOUND <= y1+OBSTACLE_Y_BOUND))
			) 
		){
		
		isObstacle = true;
	}
	else if ( ((x1+OBSTACLE_X_BOUND>=x) && (x1+OBSTACLE_X_BOUND<=x+ROBOT_X_BOUND) )
		   &&  (
					(y+ROBOT_Y_BOUND >= y1) &&
					((y <= y1+OBSTACLE_Y_BOUND) ||
					(y >= y1 && y+ROBOT_Y_BOUND <= y1+OBSTACLE_Y_BOUND))
				)
		  
		  ){
		isObstacle = true;
	}
	else if ( (y1>=y) && (y1<=y+ROBOT_Y_BOUND) && (
				(x+ROBOT_X_BOUND >= x1) &&
				((x <= x1+OBSTACLE_X_BOUND) ||
				(x >= x1 && x+ROBOT_X_BOUND <= x1+OBSTACLE_X_BOUND))
			)
		){
		isObstacle = true;
	}
	else if (	((y1+OBSTACLE_Y_BOUND>=y) && (y1+OBSTACLE_Y_BOUND<=y+ROBOT_Y_BOUND)) 
		&& (
				(x+ROBOT_X_BOUND >= x1) &&
				((x <= x1+OBSTACLE_X_BOUND) ||
				(x >= x1 && x+ROBOT_X_BOUND <= x1+OBSTACLE_X_BOUND))
			)
		){
		isObstacle = true;
	}
	else if ((x >= x1) && 
		  (x+ROBOT_X_BOUND <= x1+OBSTACLE_X_BOUND) && 
		  (y >= y1) &&
		  (y+ROBOT_Y_BOUND <= y1+OBSTACLE_Y_BOUND)){
		isObstacle = true;
	}

	
	return isObstacle;
}

bool PathFinder::checkValidMove(struct Node node,float px1,float py1){
	bool isValidMoveForRobot = false;

	float px = px1;
	float py = py1;
	bool isSafeToCreate = isNodeSafeToCreate( px, py,node);

	if(isSafeToCreate)
	{
		isValidMoveForRobot = true;
		bool isBoundary = check_boundary(node);
		if(isBoundary){
			isValidMoveForRobot = false;
		}
		else{
			bool isObstacle = check_obstacle(node);
			if(isObstacle){
				isValidMoveForRobot = false;
			}
		}
	}
	return isValidMoveForRobot;
}


bool PathFinder::isNodeCreatedBefore(struct Node node){
	bool isCreatedEarlier = false;
	
	list<Node>::const_iterator iterator;
	for (iterator = map_coordinates.begin(); iterator != map_coordinates.end(); ++iterator) {
		float x1 = iterator->x;
		float y1 = iterator->y;

		if(checkEqual(node.x,x1) && checkEqual(node.y,y1)){
			isCreatedEarlier = true;
			break;
		}
	}

	return isCreatedEarlier;
}



//ADD
void PathFinder::complete_path(struct Node *node){
		struct Node path_node;
		path_node.x = node->x;
		path_node.y = node->y;
		path_node.direction = node->direction;
		find_last_goal_center(node->x,node->y,node->direction);
		while(!additional.empty()){
			struct Node n = additional.top();
			additional.pop();
			path_queue.push(n);
		}
		path_queue.push(path_node);
		find_parent_of_diff_node(node->x,node->y);
}

//ADD
void PathFinder::find_last_goal_center(float x,float y,int direction){
		string dir = "";
		if(direction==3){
			dir = "X+";
		}
		if(direction==1){
			dir = "Y+";
		}
		if(direction==0){
			dir = "Y-";
		}
		if(direction==4){
			dir = "X-";
		}

		//Check Where is the Goal
	if((x - goal[0])<0){
		//Goal is front if head is X+
		//move front by  dist
		struct Node path_node;

		if(dir=="X+"){
			path_node.x = goal[0];
			path_node.y = y;
			path_node.direction = direction;

		}
		//Goal is front if head is Y-
		//move left and travel by dist
		if(dir=="Y-"){
			path_node.x = goal[0];
			path_node.y = y;
			path_node.direction = 1;
		}
		//Goal is front if head is Y+
		//move right and travel by dist
		if(dir=="Y+"){
			path_node.x = goal[0];
			path_node.y = y;
			path_node.direction = 0;
		}

		additional.push(path_node);


	}else if((x - goal[0])>0){
		//Goal is front if head is X+
		//move front by  dist
		if(dir=="X+"){
			struct Node path_node;
			path_node.x = x;
			path_node.y = y;
			path_node.direction = 0;
			additional.push(path_node);

			struct Node path_node1;
			path_node1.x = goal[0];
			path_node1.y = y;
			path_node1.direction = 0;
			additional.push(path_node1);
		}

		//Goal is front if head is Y-
		//move left and travel by dist
		if(dir=="Y-"){
			struct Node path_node;
			path_node.x = goal[0];
			path_node.y = y;
			path_node.direction = 0;
			additional.push(path_node);
		}
		//Goal is front if head is Y+
		//move right and travel by dist
		if(dir=="Y+"){
			struct Node path_node;
			path_node.x = goal[0];
			path_node.y = y;
			path_node.direction = 1;
			additional.push(path_node);
		}

	}


	if((y - goal[1])<0){
		//Goal is front if head is X+
		//move front by  dist
		if(dir=="X+"){
			struct Node path_node;
			path_node.x = goal[0];
			path_node.y = goal[1];
			path_node.direction = 1;
			additional.push(path_node);
		}

		//Goal is front if head is Y-
		//move left and travel by dist
		if(dir=="Y-"){
			struct Node path_node;
			path_node.x = goal[0];
			path_node.y = y;
			path_node.direction = 1;
			additional.push(path_node);
			
			struct Node path_node1;
			path_node1.x = goal[0];
			path_node1.y = goal[1];
			path_node1.direction = 1;
			additional.push(path_node1);
			
		}
		//Goal is front if head is Y+
		//move right and travel by dist
		if(dir=="Y+"){
			struct Node path_node;
			path_node.x = goal[0];
			path_node.y = goal[1];
			path_node.direction = 3;
			additional.push(path_node);
		}
	}else if((y - goal[1])>0){
		//Goal is front if head is X+
		//move front by  dist
		if(dir=="X+"){
			struct Node path_node;
			path_node.x = goal[0];
			path_node.y = goal[1];
			path_node.direction = 0;
			additional.push(path_node);
		}

		//Goal is front if head is Y-
		//move left and travel by dist
		if(dir=="Y-"){
			struct Node path_node;
			path_node.x = goal[0];
			path_node.y = goal[1];
			path_node.direction = 3;
			additional.push(path_node);
		}
		//Goal is front if head is Y+
		//move right and travel by dist
		if(dir=="Y+"){
			struct Node path_node;
			path_node.x = goal[0];
			path_node.y = y;
			path_node.direction = 0;
			additional.push(path_node);

			struct Node path_node1;
			path_node1.x = goal[0];
			path_node1.y = goal[1];
			path_node1.direction = 0;
			additional.push(path_node1);
		}
	}
}

//ADD
void PathFinder::find_parent_of_diff_node(float x,float y){
	
	float x2 = 0.0;
	float y2 = 0.0;
	int s2 = -1;

	list<Node>::const_iterator iterator;
	for (iterator = map_coordinates.begin(); iterator != map_coordinates.end(); ++iterator) {
		float x1 = iterator->x;
		float y1 = iterator->y;
		if(checkEqual(x,x1) && checkEqual(y,y1)){
			x2 = iterator->px;
			y2 = iterator->py;
			int d2 = iterator->pdirection;
			s2 = iterator->start;
			struct Node path_node;
			path_node.x = x2;
			path_node.y = y2;
			path_node.direction = d2;
			path_queue.push(path_node);
			cout << "Node  : " <<  x2 << "\t" << y2 << "\t" << d2 << "\n" ;
			break;

		}
	}
	if(s2!=10){
		find_parent_of_diff_node(x2,y2);
	}
}

//ADD
void PathFinder::path_find_from_goal(){
	string head = "X+";
	int previousDirection = 3;
	float previousx = start[0];
	float previousy = start[1];
	int count = 0;
	bool firstturn = false;

	float lastX = 0.0;
	float lastY = 0.0;
	string lasthead = "";

	while(!path_queue.empty()){
		struct Node node = path_queue.top();
		path_queue.pop();
		int direction = node.direction;

		lastX = node.x;
		lastY = node.y;
		string dir = "";
		if(direction==3){
			dir = "right";
		}
		if(direction==1){
			dir = "top";
		}
		if(direction==0){
			dir = "bottom";
		}
		if(direction==4){
			dir = "left";
		}



		cout << "Node  : " <<  node.x << "\t" << node.y << "\t" << dir << "\n" ;

		if(count==0){

			if(previousDirection!=direction){
				// previousx = node.x;
				// previousy = node.y;
				// previousDirection = direction;
				int turnto = 99;
				if(direction==0){
					head = "Y-";
					turnto = direction;
				}else if(direction==1){
					head = "Y+";
					turnto = direction;
				}else if(direction==4){
					head = "X-";
					turnto = 1;
				}

				if(turnto!=99){
					turn(turnto);
					if (head == "X-"){
						turn(turnto);
					}
				}
				previousDirection = direction;
				previousx = node.x;
				previousy = node.y;
			}
			count++;
		}else{

			int turnto = 99;
			float distance = 0.0;

			if(firstturn){
				if(head == "X+"){
					distance = fabs(node.x - previousx);
				}else if(head == "Y+"){
					distance = fabs(node.y - previousy);
				}else if(head == "Y-"){
					distance = fabs(previousy - node.y);
				}else if(head == "X-"){
					distance = fabs(previousx - node.x);
				}

				previousy = node.y;
				previousx = node.x;

				if(fabs(distance)>0.0){
					move(distance);
				}
				firstturn = false;
			}else{



				if(direction==0 && head == "X+"){
					if (previousDirection == 0){
						head = "X+";
					}else{
						head = "Y-";
						turnto = direction;
					}
				}else if(direction==0 && head == "Y-"){
					if (previousDirection == 0){
						head = "Y-";
					}else{
						head = "X-";
						turnto = direction;
					}
				}else if(direction==0 && head == "X-"){
					if (previousDirection == 0){
						head = "X-";
					}else{
						head = "Y+";
						turnto = direction;
					}
				}else if(direction==0 && head == "Y+"){
					if (previousDirection == 0){
						head = "Y+";
					}else{
						head = "X+";
						turnto = direction;
					}
				}else if(direction==1 && head == "X+"){
					if (previousDirection == 1){
						head = "X+";
					}else{
						head = "Y+";
						turnto = direction;
					}
				}else if(direction==1 && head == "Y+"){
					if (previousDirection == 1){
						head = "Y+";
					}else{
						head = "X-";
						turnto = direction;
					}
				}else if(direction==1 && head == "Y-"){
					if (previousDirection == 1){
						head = "Y-";
					}else{
						head = "X+";
						turnto = direction;
					}
				}else if(direction==1 && head == "X-"){
					if (previousDirection == 1){
						head = "X-";
					}else{
						head = "Y-";
						turnto = direction;
					}
				}else if(direction==3 && head == "Y-"){
					head = "X+";
					turnto = 1;
				}else if(direction==3 && head == "Y+"){
					head = "X+";
					turnto = 0;
				}
				previousDirection = direction;
				if(turnto!=99){
					turn(turnto);
				}
				if(head == "X+"){
					distance = fabs(node.x - previousx);
				}else if(head == "Y+"){
					distance = fabs(node.y - previousy);
				}else if(head == "Y-"){
					distance = fabs(previousy - node.y);
				}else if(head == "X-"){
					distance = fabs(previousx - node.x);
				}

				previousy = node.y;
				previousx = node.x;

				if(fabs(distance)>0.0){
					move(distance);
				}
			}

			lasthead = head;
		}
	}


	


}


void PathFinder::move(float dist){


	int actang,reqsp;
	int Kp=1;
	float fx;
	float err;
	ResetRotationCount(OUT_A);
	OnFwdSync(OUT_AB, MOTOR_SPEED);
	while (TRUE)
	{
		Wait(20);
		actang= (MotorRotationCount(OUT_A));
		float distact= ((float)actang/360)*0.176;
		err=(dist-distact);
		if ((dist-distact)<0.00001)
		{Off(OUT_AB);break;}
		fx= Kp*(dist-distact);
		reqsp= floor((30931*pow(fx,4))-(21229*pow(fx,3))+(4746.9*pow(fx,2))-(184.15*fx)+8.4551);
		if (reqsp>MOTOR_SPEED)
		{
			reqsp=MOTOR_SPEED;
		}
		else if(reqsp<1)
		{
			reqsp=1;
		}
		OnFwdSync(OUT_AB, reqsp);
	}

	
}

void PathFinder::turn(int lr){

	//float dist=0.0785;
	float dist=0.0953;
	int actang,reqsp;
	float distact;
	int Kp=1;
	float fx;
	if (lr==0)
	{
		ResetRotationCount(OUT_A);
		ResetRotationCount(OUT_B);
		OnFwdReg(OUT_A, MOTOR_SPEED);
		OnRevReg(OUT_B, MOTOR_SPEED);

		while (TRUE)
		{
			Wait(20);
			actang= (MotorRotationCount(OUT_A));
			distact= ((float)actang/360)*0.176;
			if ((dist-distact)<0.00001)
			{Off(OUT_A);Off(OUT_B);break;}
			fx= Kp*(dist-distact);
			reqsp= floor((30931*pow(fx,4))-(21229*pow(fx,3))+(4746.9*pow(fx,2))-(184.15*fx)+8.4551);
			if (reqsp>MOTOR_SPEED)
			{
				reqsp=MOTOR_SPEED;
			}
			else if(reqsp<1)
			{
				reqsp=1;
			}


			OnFwdReg(OUT_A, reqsp);
			OnRevReg(OUT_B, reqsp);
		}
		if ((dist-distact)<0)
		{
			OnFwdReg(OUT_B, 5);
			Wait(1100);Off(OUT_B);


		}

	}
	else
	{
		ResetRotationCount(OUT_B);
		OnFwdReg(OUT_B, MOTOR_SPEED);
		OnRevReg(OUT_A, MOTOR_SPEED);

		while (TRUE)
		{
			Wait(20);
			actang= (MotorRotationCount(OUT_B));
			distact= ((float)actang/360)*0.176;
			if ((dist-distact)<0.00001)
			{Off(OUT_B);Off(OUT_A);break;}
			fx= Kp*(dist-distact);
			reqsp= floor((30931*pow(fx,4))-(21229*pow(fx,3))+(4746.9*pow(fx,2))-(184.15*fx)+8.4551);
			if (reqsp>MOTOR_SPEED)
			{
				reqsp=MOTOR_SPEED;
			}
			else if(reqsp<1)
			{
				reqsp=1;
			}


			OnFwdReg(OUT_B, reqsp);
			OnRevReg(OUT_A, reqsp);
		}
		if ((dist-distact)<0.)
		{

			OnFwdReg(OUT_A, 5);
			Wait(1100);Off(OUT_A);
		}
	}


}

void PathFinder::simplify_path(){
	struct Node Previous;
	int count = 0 ;
	while(!path_queue.empty()){
		struct Node node = path_queue.top();
		follow_path.push(node);
		path_queue.pop();
		// // cout << "Node Popped : " << node.x << "\t" << node.y << "\t" << node.direction << "\n";
		// if(count==0){
		// 	follow_path.push(node);
		// 	Previous = node;
		// 	path_queue.pop();
		// 	count++;
		// }else if((Previous.x==node.x || Previous.y==node.y) and (Previous.direction==node.direction)){
		// 	Previous = node;
		// 	path_queue.pop();
		// }else{
		// 	follow_path.push(node);
		// 	path_queue.pop();
		// 	Previous = node;
		// }
	}

	
}


void PathFinder::run_into_path(){
	string head = "X+";
	int previousDirection = 3;
	float previousx = start[0];
	float previousy = start[1];
	int count = 0;
	bool firstturn = false;

	float lastX = 0.0;
	float lastY = 0.0;
	string lasthead = "";

	while(!follow_path.empty()){
		struct Node node = follow_path.front();
		follow_path.pop();
		int direction = node.direction;

		lastX = node.x;
		lastY = node.y;
		string dir = "";
		if(direction==3){
			dir = "right";
		}
		if(direction==1){
			dir = "top";
		}
		if(direction==0){
			dir = "bottom";
		}
		if(direction==4){
			dir = "left";
		}



		cout << "Node  : " <<  node.x << "\t" << node.y << "\t" << dir << "\n" ;


		if(count==0){

			if(previousDirection!=direction){
				// previousx = node.x;
				// previousy = node.y;
				// previousDirection = direction;
				int turnto = 99;
				if(direction==0){
					head = "Y-";
					turnto = direction;
				}else if(direction==1){
					head = "Y+";
					turnto = direction;
				}else if(direction==4){
					head = "X-";
					turnto = 1;
				}

				if(turnto!=99){
					turn(turnto);
					if (head == "X-"){
						turn(turnto);
					}
				}
				previousDirection = direction;
				previousx = node.x;
				previousy = node.y;
			}
			count++;
		}else{

			int turnto = 99;
			float distance = 0.0;

			if(firstturn){
				if(head == "X+"){
					distance = fabs(node.x - previousx);
				}else if(head == "Y+"){
					distance = fabs(node.y - previousy);
				}else if(head == "Y-"){
					distance = fabs(previousy - node.y);
				}else if(head == "X-"){
					distance = fabs(previousx - node.x);
				}

				previousy = node.y;
				previousx = node.x;

				if(fabs(distance)>0.0){
					move(distance);
				}
				firstturn = false;
			}else{



				if(direction==0 && head == "X+"){
					if (previousDirection == 0){
						head = "X+";
					}else{
						head = "Y-";
						turnto = direction;
					}
				}else if(direction==0 && head == "Y-"){
					if (previousDirection == 0){
						head = "Y-";
					}else{
						head = "X-";
						turnto = direction;
					}
				}else if(direction==0 && head == "X-"){
					if (previousDirection == 0){
						head = "X-";
					}else{
						head = "Y+";
						turnto = direction;
					}
				}else if(direction==0 && head == "Y+"){
					if (previousDirection == 0){
						head = "Y+";
					}else{
						head = "X+";
						turnto = direction;
					}
				}else if(direction==1 && head == "X+"){
					if (previousDirection == 1){
						head = "X+";
					}else{
						head = "Y+";
						turnto = direction;
					}
				}else if(direction==1 && head == "Y+"){
					if (previousDirection == 1){
						head = "Y+";
					}else{
						head = "X-";
						turnto = direction;
					}
				}else if(direction==1 && head == "Y-"){
					if (previousDirection == 1){
						head = "Y-";
					}else{
						head = "X+";
						turnto = direction;
					}
				}else if(direction==1 && head == "X-"){
					if (previousDirection == 1){
						head = "X-";
					}else{
						head = "Y-";
						turnto = direction;
					}
				}else if(direction==3 && head == "Y-"){
					head = "X+";
					turnto = 1;
				}else if(direction==3 && head == "Y+"){
					head = "X+";
					turnto = 0;
				}
				previousDirection = direction;
				if(turnto!=99){
					turn(turnto);
				}
				if(head == "X+"){
					distance = fabs(node.x - previousx);
				}else if(head == "Y+"){
					distance = fabs(node.y - previousy);
				}else if(head == "Y-"){
					distance = fabs(previousy - node.y);
				}else if(head == "X-"){
					distance = fabs(previousx - node.x);
				}

				previousy = node.y;
				previousx = node.x;

				if(fabs(distance)>0.0){
					move(distance);
				}
			}

			lasthead = head;
		}
	}


}

void PathFinder::computepath(){
	struct Node root = createRootNode(start[0],start[1],10);
	map_learning_queue.push(root);
	

	map_coordinates.push_back(root);
	find_path();
	goal_queue.pop();
	struct Node node1 = goal_queue.front();
	goal_queue.pop();


	//ADD
	complete_path(&node1);
	
	simplify_path();

	run_into_path();
	
	
}

int main()
{
  InitEV3();
  PathFinder pf ;
  pf.computepath();
  Wait(2000);
  FreeEV3();
}
