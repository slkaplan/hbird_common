from hbird_msgs.msg import Waypoint
from queue import PriorityQueue
import math
class PathPlanner():
    """
    This class computes a waypoint-based path from the start to the goal poses.

    You are to implement the __init__ and plan methods. Feel free to add any more methods.
    """
    def __init__(self, env):
        """
        Inputs:
        - env (dict):    dictionary storing relevant environment parameters 
                         (see Environment class in planner_utils.py)
        """
        self.env = env
        self.start = (round(env.start_pose.position.x/env.grid_size)*env.grid_size,round(env.start_pose.position.y/env.grid_size)*env.grid_size)
        self.goal = (round(env.goal_pose.position.x/env.grid_size)*env.grid_size,round(env.goal_pose.position.y/env.grid_size)*env.grid_size)
        # your code here
        print(self.start)

    def neighbors(self, current):
        neighbors = []
        for x in [-self.env.grid_size, 0, self.env.grid_size]:
            for y in [-self.env.grid_size, 0, self.env.grid_size]:
                if x==y==0:
                    continue
                next_pos = (current[0]+x, current[1]+y)
                if next_pos[0] < self.env.x_min or next_pos[0] > self.env.x_max:
                    continue
                if next_pos[1] < self.env.y_min or next_pos[1] > self.env.y_max:
                    continue
                neighbors.append(next_pos)
        return neighbors

    def cost(self, current, next):
        return math.sqrt((next[0]-current[0])**2 + (next[1]-current[1])**2)

    def heuristic(self, goal, next):
        return math.sqrt((goal[0]-next[0])**2 + (goal[1]-next[1])**2)
    
    def plan(self):
        """
        Main method that computes and returns the path

        Returns:
        - path (list of Waypoint objects)      
        """
        
        path = []

        # your code here
        frontier = PriorityQueue()
        frontier.put(self.start, 0)
        came_from = dict()
        cost_so_far = dict()
        came_from[self.start] = None
        cost_so_far[self.start] = 0

        while not frontier.empty():
            current = frontier.get()

            if current == self.goal:
                break
            
            for next in self.neighbors(current):
                new_cost = cost_so_far[current] + self.cost(current, next)
                if next not in cost_so_far or new_cost < cost_so_far[next]:
                    cost_so_far[next] = new_cost
                    priority = new_cost + self.heuristic(self.goal, next)
                    frontier.put(next, priority)
                    came_from[next] = current
        # Iterate backward through poses
        pos = self.goal
        while pos != None:
            waypoint = Waypoint()
            waypoint.position.x = pos[0]
            waypoint.position.y = pos[1]
            waypoint.position.z = self.env.start_pose.position.z
            waypoint.heading = self.env.start_pose.heading
            path.append(waypoint)
            pos = came_from[pos]
        return path


