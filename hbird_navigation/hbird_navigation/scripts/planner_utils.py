""" catch-all script for the planner utility data structures (structs) and functions """

from hbird_msgs.msg import Waypoint
import numpy as np
from itertools import product
import matplotlib.pyplot as plt



class Environment():

    def __init__(self, config):
        """
        This class parses the configuration files to create an environment object with all relevant params

        You do not need to edit this class
        """
        
        # create start_pose waypoint
        self.start_pose = Waypoint()
        self.start_pose.position.x = config["poses"]["start_x"]
        self.start_pose.position.y = config["poses"]["start_y"]
        self.start_pose.position.z = config["poses"]["start_z"]
        self.start_pose.heading = config["poses"]["start_yaw"]

        # create goal_pose waypoint
        self.goal_pose = Waypoint()
        self.goal_pose.position.x = config["poses"]["goal_x"]
        self.goal_pose.position.y = config["poses"]["goal_y"]
        self.goal_pose.position.z = config["poses"]["goal_z"]
        self.goal_pose.heading= config["poses"]["goal_yaw"]

        # set map dimensions
        self.x_min = config["map"]["dimensions"]["x_min"]
        self.y_min = config["map"]["dimensions"]["y_min"]
        self.x_max = config["map"]["dimensions"]["x_max"]
        self.y_max = config["map"]["dimensions"]["y_max"]

        # set environment params
        self.grid_size = config["map"]["grid_size"]
        self.robot_radius = config["map"]["robot_radius"]

        # create obstacle dictionary
        self.obstacles = self.create_obstacles(config["obstacles"])

        # create gridlines dictionary
        self.gridlines = self.create_gridlines()

    
    def create_obstacles(self, obs_corners):
        """
        Method takes in the two corners of rectangles from the yaml file
        and define all the points as obstacles
        """
        obstacles = dict()
        obst_val = []
        x_vals_obst = []
        y_vals_obst = []

        for pair in obs_corners:
            x_vals_obst += list(np.arange(pair[0][0], pair[1][0], 0.1))
            y_vals_obst += list(np.arange(pair[0][1], pair[1][1], 0.1))

            obst_val += product(x_vals_obst, y_vals_obst)

        obstacles["x"] = [xy[0] for xy in obst_val]
        obstacles["y"] = [xy[1] for xy in obst_val] 
        
        return obstacles
        

    def create_gridlines(self):
        """
        Method creates gridlines as a grid discretization of the map environment
        """

        grid_x, grid_y = [], []
        gx, gy = 0, 0
        gridlines = dict()
        
        while gx < self.x_max:
            gx += self.grid_size
            grid_x.append(gx)
        while gy < self.y_max:
            gy += self.grid_size
            grid_y.append(gy)

        gridlines["x"] = grid_x
        gridlines["y"] = grid_y

        return gridlines



class Visualizer():
    """
    This class handles the 2D visualization of the planner

    You do not need to edit this class
    """
    def __init__(self, env):
        """
        Inputs:
        - env (dict):    dictionary storing relevant environment parameters 
                         (see Environment class in planner_utils.py)
        """
        self.env = env
        
        # create plot and figure object
        self.fig, self.ax = plt.subplots()
        # set figure size
        self.fig.set_size_inches(10, 8)
        # set marker parameters
        self.marker_scale_factor = 1000
        self.marker_size = self.scale_marker_size()


    def scale_marker_size(self):
        """Calculate the marker size relative to the axes limits"""
        x_range = self.env.x_max - self.env.x_min
        y_range = self.env.y_max - self.env.y_min
        max_range = max(x_range, y_range)
        return (self.env.robot_radius * self.marker_scale_factor) / max_range      
    

    def plot(self, path):
        """
        Function handles plotting of the map, start/goal locations and path
        
        Inputs:
        - path (list of Waypoint objects)
        """

        # obstacle plot
        self.ax.plot(self.env.obstacles["x"], self.env.obstacles["y"], ".k")

        # start and goal position plots
        self.ax.plot(self.env.start_pose.position.x, 
                self.env.start_pose.position.y,
                marker='s', 
                markersize=self.marker_size, 
                color='red', alpha=0.5)
        self.ax.plot(self.env.start_pose.position.x, 
                self.env.start_pose.position.y, 
                "o", color='black')
        self.ax.plot(self.env.goal_pose.position.x, 
                self.env.goal_pose.position.y, 
                marker='s', 
                markersize=self.marker_size, 
                color='green', alpha=0.5)
        self.ax.plot(self.env.goal_pose.position.x, 
                self.env.goal_pose.position.y, 
                "o", color='black')

        # path plots
        path_x, path_y = [], []
        for waypoint in path:
            path_x.append(waypoint.position.x)
            path_y.append(waypoint.position.y)
        self.ax.plot(path_x, path_y, "o-")

        # plotter parameter setting
        self.ax.set_xticks(self.env.gridlines["x"])
        self.ax.set_yticks(self.env.gridlines["y"])
        self.ax.set_xlim([self.env.x_min, self.env.x_max])
        self.ax.set_ylim([self.env.y_min, self.env.y_max])
        plt.grid(True)
        plt.show()
 