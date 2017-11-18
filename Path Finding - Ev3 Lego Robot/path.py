from tkinter import *
import time


class Queue:
	def __init__(self):
		self.items = []

	def isEmpty(self):
		return self.items == []

	def enqueue(self, item):
		self.items.insert(0,item)

	def dequeue(self):
		return self.items.pop()

	def size(self):
		return len(self.items)

class Node:
	def __init__(self,x,y,parent,direction):
		self.x = x
		self.y = y
		self.parent = parent
		self.direction = direction


data = [
			[0.915, 0.305],
			[0.915, 0.61],
			[0.915, 0.915],
			[0.915, 1.219],
			[0.915, 1.524],
			[2.135, 1.219],
			[2.135, 1.524],
			[2.135, 1.829],
			[2.135, 2.134],
			[2.135, 2.439],
			[2.135, 2.744],
			[-1,-1],
			[-1,-1],
			[-1,-1],
			[-1,-1],
			[-1,-1],
			[-1,-1],
			[-1,-1],
			[-1,-1],
			[-1,-1],
			[-1,-1],
			[-1,-1],
			[-1,-1],
			[-1,-1],
			[-1,-1],
			[-1,-1],
			[-1,-1],
			[-1,-1]
		];
factor = 2*100
obstacles = []

master = Tk()


X_BOUND = 3.66*factor
Y_BOUND = 3.05*factor

THRESHOLD = 0.2 * factor

OBSTACLE_X_BOUND = 0.315*factor #0.305
OBSTACLE_Y_BOUND = 0.315*factor

ROBOT_X_BOUND = 0.21*factor # 0.196
ROBOT_Y_BOUND = 0.21*factor # 0.147



move = 0.25*factor

goal = [int(3.05*factor), int(2.135*factor)];
start = [int(0.305*factor), int(0.61*factor)];


w = Canvas(master, width=X_BOUND, height=Y_BOUND)
# w.canvasx(screenx, gridspacing=None)
# w.canvasy(Y_BOUND, gridspacing=None)
w.pack()

map_coor = []

# Designing Obstacles
for rec in data:
	if int(rec[0]) != -1:
		# print(x,y)
		x = int((rec[0]-(OBSTACLE_X_BOUND/(2*factor)))*factor)
		y = int((rec[1]-(OBSTACLE_Y_BOUND/(2*factor)))*factor)
		# print(x,y)
		obstacles.append([x,y])
		w.create_rectangle(x, y, (x+OBSTACLE_X_BOUND), (y+OBSTACLE_Y_BOUND), fill="gray")
# Designing Obstacles

# Designing Goal
w.create_rectangle(goal[0], goal[1], (goal[0]+OBSTACLE_X_BOUND), (goal[1]+OBSTACLE_Y_BOUND), fill="red")
# Designing Goal

# Designing Start
w.create_rectangle(start[0], start[1], (start[0]+ROBOT_X_BOUND), (start[1]+ROBOT_Y_BOUND), fill="blue")
root1 = Node(start[0],start[1],None,None)
root = Node(start[0],start[1],root1,None)
map_coor.append([start[0],start[1]])
q = Queue()
goal_q = Queue()
q.enqueue(root)
# Designing Start



def computePath():
	goal123 = False
	if q.isEmpty():
		return
	
	if not goal_q.isEmpty():
		return

	node = q.dequeue()
	oldx = node.x
	oldy = node.y

	#Right Node
	r_c = [oldx+move,oldy]
	right = Node(oldx+move,oldy,node,"right")
	if not r_c in map_coor:
		map_coor.append(r_c)
		isValidMoveForRobot_Right = checkValidMove(right,node.parent)
		isGoalFound = goal_found(right)
		if isGoalFound :
			print("Goal r")
			goal123 = True
			isValidMoveForRobot_Right = False
			goal_q.enqueue(right)
			post_processing_goal_found()

		# print("Right ",isValidMoveForRobot_Right)
		if isValidMoveForRobot_Right:
			q.enqueue(right)
			w.create_rectangle(right.x, right.y, (right.x+ROBOT_X_BOUND), (right.y+ROBOT_Y_BOUND), fill="green")

	#Left
	l_c = [oldx-move,oldy]
	left = Node(oldx-move,oldy,node,"left")
	if not l_c in map_coor:
		map_coor.append(l_c)
		isValidMoveForRobot_Left = checkValidMove(left,node.parent)
		isGoalFound = goal_found(left)
		if isGoalFound :
			print("Goal l")
			goal123 = True
			isValidMoveForRobot_Left = False
			goal_q.enqueue(left)
			post_processing_goal_found()
		if isValidMoveForRobot_Left:
			q.enqueue(left)
			w.create_rectangle(left.x, left.y, (left.x+ROBOT_X_BOUND), (left.y+ROBOT_Y_BOUND), fill="green")
	

	# #Top
	t_c = [oldx,oldy+move]
	top = Node(oldx,oldy+move,node,"top")
	if not t_c in map_coor:
		map_coor.append(t_c)
		isValidMoveForRobot_Top = checkValidMove(top,node.parent)
		isGoalFound = goal_found(top)
		if isGoalFound :
			print("Goal t")
			goal123 = True
			isValidMoveForRobot_Top = False
			goal_q.enqueue(top)
			post_processing_goal_found()
		if isValidMoveForRobot_Top:
			q.enqueue(top)
			w.create_rectangle(top.x, top.y, (top.x+ROBOT_X_BOUND), (top.y+ROBOT_Y_BOUND), fill="green")
	

	#Bottom
	b_c = [oldx,oldy-move]
	bottom = Node(oldx,oldy-move,node,"bottom")
	if not b_c in map_coor:
		map_coor.append(b_c)
		isValidMoveForRobot_Bottom = checkValidMove(bottom,node.parent)
		isGoalFound = goal_found(bottom)
		if isGoalFound :
			print("Goal b")
			goal123 = True
			isValidMoveForRobot_Bottom = False
			goal_q.enqueue(bottom)
			post_processing_goal_found()

		if isValidMoveForRobot_Bottom:
			q.enqueue(bottom)
			w.create_rectangle(bottom.x, bottom.y, (bottom.x+ROBOT_X_BOUND), (bottom.y+ROBOT_Y_BOUND), fill="green")

	computePath()


def post_processing_goal_found():
	for i in range(q.size()):
		q.dequeue()


def checkValidMove(node,parent):
	isValidMoveForRobot = False
	isSafeToCreate = isNodeSafeToCreate(parent,node)
	if isSafeToCreate:
		isValidMoveForRobot = True
		isBoundary = check_boundary(node)
		if isBoundary:
			# print("isBoundary : ",isBoundary)
			isValidMoveForRobot = False
		else:
			isObstacle = check_obstacle(node)
			# print("isObstacle : ",isObstacle)
			if isObstacle:
				isValidMoveForRobot = False

	return isValidMoveForRobot 
	
def isNodeSafeToCreate(parent,node):
	if parent == None:
		return True
	if parent.x == node.x and parent.y == node.y:
		return False
	else:
		return True

def check_boundary(node):
	if int(node.x)>= int(X_BOUND) or int(node.x) <= 0:
		return True
	elif int(node.y)>= int(Y_BOUND) or int(node.y) <= 0:
		return True
	else:
		return False

def goal_found(node):
	isGoalFound = contact_with_object(node.x,node.y,goal,"orange")
	return isGoalFound

def check_obstacle(node):
	isObstacle = False
	x_co = node.x
	y_co = node.y
	for obstacle in obstacles:
		if int(obstacle[0])==-1:
			continue
		isObstacle = contact_with_object(x_co,y_co,obstacle,"black")
		if isObstacle:
			break
	return isObstacle

def contact_with_object(x_co,y_co,obstacle,color_impact):
	isObstacle = False
	
	x1 = obstacle[0] 
	y1 = obstacle[1] 

	x_co1 = x_co;
	y_co1 = y_co;

	# x_co = x_co - (ROBOT_X_BOUND/2)
	# y_co = y_co - (ROBOT_Y_BOUND/2)

	x_co1 = x_co

	# Obstacle at right side of Robot
	if (    ((x1>=x_co) and (x1<=(x_co+ROBOT_X_BOUND)) )
		and (   (y_co+ROBOT_Y_BOUND >= y1) and
				((y_co <= y1+OBSTACLE_Y_BOUND) or
				(y_co >= y1 and y_co+ROBOT_Y_BOUND <= y1+OBSTACLE_Y_BOUND))
			) 
		):
		
		w.create_rectangle(x_co1, y_co1, (x_co1+ROBOT_X_BOUND), (y_co1+ROBOT_Y_BOUND), fill=color_impact)
		isObstacle = True
	# Obstacle at Left side of Robot
	elif (      ((x1+OBSTACLE_X_BOUND>=x_co) and (x1+OBSTACLE_X_BOUND<=x_co+ROBOT_X_BOUND) )
		   and  (
					(y_co+ROBOT_Y_BOUND >= y1) and
					((y_co <= y1+OBSTACLE_Y_BOUND) or
					(y_co >= y1 and y_co+ROBOT_Y_BOUND <= y1+OBSTACLE_Y_BOUND))
				)
		  
		  ):
		w.create_rectangle(x_co1, y_co1, (x_co1+ROBOT_X_BOUND), (y_co1+ROBOT_Y_BOUND), fill=color_impact)
		isObstacle = True
	# Obstacle at top side of Robot
	elif ( (y1>=y_co) and (y1<=y_co+ROBOT_Y_BOUND) and (
				(x_co+ROBOT_X_BOUND >= x1) and
				((x_co <= x1+OBSTACLE_X_BOUND) or
				(x_co >= x1 and x_co+ROBOT_X_BOUND <= x1+OBSTACLE_X_BOUND))
			)
		):
		w.create_rectangle(x_co1, y_co1, (x_co1+ROBOT_X_BOUND), (y_co1+ROBOT_Y_BOUND), fill=color_impact)
		isObstacle = True
	
	# Obstacle at bottom side of Robot
	elif (	((y1+OBSTACLE_Y_BOUND>=y_co) and (y1+OBSTACLE_Y_BOUND<=y_co+ROBOT_Y_BOUND)) 
		and (
				(x_co+ROBOT_X_BOUND >= x1) and
				((x_co <= x1+OBSTACLE_X_BOUND) or
				(x_co >= x1 and x_co+ROBOT_X_BOUND <= x1+OBSTACLE_X_BOUND))
			)
		):
		w.create_rectangle(x_co1, y_co1, (x_co1+ROBOT_X_BOUND), (y_co1+ROBOT_Y_BOUND), fill=color_impact)
		isObstacle = True

	elif ((x_co >= x1) and 
		  (x_co+ROBOT_X_BOUND <= x1+OBSTACLE_X_BOUND) and 
		  (y_co >= y1) and
		  (y_co+ROBOT_Y_BOUND <= y1+OBSTACLE_Y_BOUND)):
		w.create_rectangle(x_co1, y_co1, (x_co1+ROBOT_X_BOUND), (y_co1+ROBOT_Y_BOUND), fill=color_impact)
		isObstacle = True
	return isObstacle

computePath()
g_node = goal_q.dequeue()

# print(g_node.x/factor,":",g_node.y/factor)

path_q = Queue()
path = []
def find_goal(node):
	if node.parent == None:
		return
	
	n = Node(node.x,node.y,None,node.direction)
	path_q.enqueue(n)
	path.append(n)
	find_goal(node.parent)

find_goal(g_node)

n_path_n = path_q.dequeue()
def follow(n_path):
	if path_q.isEmpty():
		return

	print (n_path.x/factor,n_path.y/factor,n_path.direction)
	# print (int(n_path.x), int(n_path.y))
	w.create_rectangle(int(n_path.x), int(n_path.y), (int(n_path.x)+ROBOT_X_BOUND), (int(n_path.y)+ROBOT_Y_BOUND), fill="yellow")
	w.create_text((int(n_path.x+ROBOT_X_BOUND/2), int(n_path.y+ROBOT_Y_BOUND/2)), text=n_path.direction[0].upper())
	n_path = path_q.dequeue()
	follow(n_path)

def follow1():
	l = len(path)
	
	for i in range(l-1, -1, -1):
		n_path = path[i]
		print (n_path.x/factor,n_path.y/factor,n_path.direction)
		# print (int(n_path.x), int(n_path.y))
		if not n_path.direction==None:
			w.create_rectangle(int(n_path.x), int(n_path.y), (int(n_path.x)+ROBOT_X_BOUND), (int(n_path.y)+ROBOT_Y_BOUND), fill="yellow")
			w.create_text((int(n_path.x+ROBOT_X_BOUND/2), int(n_path.y+ROBOT_Y_BOUND/2)), text=n_path.direction[0].upper())
		# n_path = path_q.dequeue()

# follow(n_path_n)
follow1()

mainloop()







