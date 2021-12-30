#!/usr/bin/env python3

import heapq as hq
from buildMap import Map
from node import Node
import math
import math


def radians(degree):
	return (degree * math.pi / 180)


ROBOT_SIZE = 0.1
G_MULTIPLIER = 0.2
MOVES = [   (0.2, radians(0)),  # move ahead
            (-0.2, radians(0)), # move backward
            (0,radians(90)), # turn left
            (0, -radians(90))] # turn right


TOLERANCE = 0.2


class PathPlanner:
    def __init__(self,start,theta,goal):
        print("building map...")
        # map remains constant
        self.image_path = "./map1.png"
        self.map = Map(image_path=self.image_path).grid_map
        self.start = start
        self.theta = theta
        self.goal = goal

    def plan(self):
        final = a_star(self.start, self.goal, self.map)
        if final == None:
            print("Path not found.")
        else:
            print("Constructing path..")
            path = self.construct_path(final)
            #print("path: ",path)
            points = []
            for step in path:
                points.append((step.x, step.y))

            # publish this path - safegoto for each of path components
            points.reverse()
            points = points[1:]
            points.append((self.goal.x, self.goal.y))
            for p in range(len(points)):
                print("x: ",points[p][0], "y: ",points[p][1])
            
            # visualization
            Map.visualize(self.image_path,points)

            #first process the points
            translate_x = points[0][0]
            translate_y = points[0][1]
            for p in range(len(points)):
                new_x = points[p][0] - translate_x
                new_y = points[p][1] - translate_y
                if self.theta == math.pi/2:
                    points[p] = [-new_y,new_x]
                elif self.theta == math.pi:
                    points[p] = [-new_x, -new_y]
                elif self.theta == -math.pi/2:
                    points[p] = [new_y, -new_x]
                else:
                    points[p] = [new_x,new_y]
            #translate coordinates for theta

            print("Call local_planner...")
           

    def construct_path(self,end):
        """
        backtrack  from end to construct path
        """
        current = end
        path = []
        while current != None:
            path.append(current)
            current = current.parent
        
        return path

def a_star(start, end, grid_map):
	# start,end are in world coordinates and Node objects

	# before starting A star, check if goal is reachable - ie, in an obstacle free zone - if not, directly reject
	if not end.is_valid(grid_map):
		print("Goal is invalid.")
		return None
	print("Goal is valid.")
	opened = []
	closed = []
	final = None
	hq.heappush(opened, (0.0, start))
    


	while (final == None) and opened:
		# q is a Node object with x, y, theta
		q = hq.heappop(opened)[1]
		for move in MOVES:		# move is in world coordinates
			if (q.is_move_valid(grid_map, move)):
				next_node = q.apply_move(move)	# Node is returned in world coordinates
			else:
				next_node = None
			#print("next node is : ", next_node) 
			if next_node != None:
				if next_node.euclidean_distance(end) < TOLERANCE:
					next_node.parent = q					
					final = next_node
					break
				# update heuristics h(n) and g(n)
				next_node.h = next_node.euclidean_distance(end)
				next_node.g = q.g + next_node.euclidean_distance(q)
				# f(n) = h(n) + g(n)
				next_node.f = G_MULTIPLIER * next_node.g + next_node.h
				next_node.parent = q

				# other candidate locations to put in the heap
				potential_open = any(other_f <= next_node.f and other_next.is_similar(next_node) for other_f, other_next in opened)
				
				if not potential_open:
					potential_closed = any(other_next.is_similar(next_node) and other_next.f <= next_node.f for other_next in closed)
					if not potential_closed:
						hq.heappush(opened, (next_node.f, next_node))
		closed.append(q)	

	return final			
    

if __name__ == "__main__":
    start = Node(-74.35,-74.2, math.pi)# pixel(124,245)
    goal = Node(-75.45,-77.9, math.pi) # pixel(594,155)
    planner = PathPlanner(start,math.pi,goal)
    planner.plan()