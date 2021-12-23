#!/usr/bin/env python3



import heapq as hq
from os import close, path

from PIL.Image import NONE, new
from buildMap import Map
from node import Node
import math
from geometry_msgs.msg import Twist,Pose
#from safegoto import SafeGoTo
import rospy
import copy
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
        self.map = Map().grid_map
        self.start = start
        self.theta = theta
        self.goal = goal
        print("map built.planner initalized.")

    def plan(self):
        final = a_star(self.start, self.goal, self.map)
        if final == None:
            print("Path not found.")
        else:
            print("Constructing path..")
            #print(final)

            #exit()
            path = self.construct_path(final)
            #print("path: ",path)
            points = []
            for step in path:
                points.append((step.x, step.y))

            # publish this path - safegoto for each of path components
            points.reverse() # listeyi append ile oluşturduğumuz için en başta son nokta, en sonda ise ilk nokta var o yüzden ters çeviriyoruz.
            points = points[1:] # ilk eleman start noktası olduğu için alma
            points.append((self.goal.x, self.goal.y)) # tolerans ile aldığımız için son nokta hedef nokta değil, o toleransın altında olan nokta, o yüzden hedef noktayı da ekliyoruz.
            # show all path points
            # for p in range(len(points)):
            #     print("x: ",points[p][0], "y: ",points[p][1])
            
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


            print("Safegoto will call...")
            # run safegoto on the translated coordinates
			#robot = SafeGoTo()
			#robot.travel(points)

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
    # start and end are in world coordianates and Node objects

    #before starting A star, check if goal is reachable -ie, in an obstacle free zone - if not, directly reject
    if not end.is_valid(grid_map):
        print("Goal invalid.")
        return None
    print("Goal valid.")
    opened = []
    closed = []
    final = None
    # heap is a tree-based data structure.
    hq.heappush(opened,(0.0,start))#Heap elements can be tuples. This is useful for assigning comparison values (such as task priorities).Normalde heappush direk en sona ekliyor.

    # for other_1,other_2 in opened:
    #     print("other_1: ",other_1)
    #     print("other_2: ",other_2)

    c = 0
    while (final == None) and opened:
        c += 1
        # q is a Node object with x,y,theta
        q = hq.heappop(opened)[1] #heappop delete only first index. [1] means that second element of tuple, so node object.
     
        # if c < 4: ##(for debug)
        #     print("\nMOVES for'una {}. girisi".format(c))
        for move in MOVES:
            print("***********************")
            print("Move: ",move)
            print("q object name: ",q)
            print("Nokta(q)--> x:{}  y: {}".format(q.x,q.y))
            if (q.is_move_valid(grid_map,move)):
                print("-> Noktaya move eklendi ve valid.")
                next_node = q.apply_move(move)
                print("Next_node--> x:{} y:{}".format(next_node.x,next_node.y))
                #print("opened: ",opened)
            else:
                print("-> Noktaya move eklendi ama valid degil.")
                next_node = None
            
            if next_node != None:
                if next_node.euclidean_distance(end) < TOLERANCE:
                    print("-> Next node ile goal arasindaki fark toleranstan az.")
                    next_node.parent = q
                    final = next_node
                    print("-> Ve final next_node'a esitlendi.Dongu kirildi.")
                    break
                #update heuristic h(n) and g(n)
                next_node.h = next_node.euclidean_distance(end)
                next_node.g = q.g + next_node.euclidean_distance(q)
                #f(n) = h(n) + g(n)
                next_node.f = G_MULTIPLIER * next_node.g + next_node.h
                next_node.parent = q
                print("Next_node object name: ",next_node)
                print("Current next_node.f: ",next_node.f)
                print("Parent of q: ",q.parent)


                # other candidate locations to put in the heap
                potential_open = any(other_f <= next_node.f and other_next.is_similar(next_node) for other_f, other_next in opened)
                print("Potential open : ",potential_open)

                if not potential_open:
                    potential_closed = any(other_next.is_similar(next_node) and other_next.f <= next_node.f for other_next in closed)
                    print("Potential closed: ",potential_closed)
                    if not potential_closed:
                        hq.heappush(opened,(next_node.f, next_node))
                        print("Opened'a pushlandi: ",opened)
        
        
            closed.append(q)

            print("\n*****Closed******")
            for other in closed:
                print("Node name: {} -- Node.x: {} -- Node.y: {}".format(other,other.x,other.y))


    return final



if __name__ == "__main__":
    start = Node(-54.0, 7.5, math.pi)
    goal = Node(5.9375, 12.437500000000002,math.pi)
    planner = PathPlanner(start,math.pi,goal)
    planner.plan()