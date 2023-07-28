import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor

import time
import math
import numpy as np 
from scipy.stats import norm

from hbird_msgs.action import AgentTask
from hbird_msgs.msg import Waypoint, State
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from .scripts.utils import State as StatePy


#TODO:



class AgentControlNode(Node):

    def __init__(self):
        super().__init__('agent_control_node')

        self.get_logger().info('Starting up the Agent Control Node...')
        
        # TODO: Add the parameter getter and the parameter declaration
        param_descriptor = ParameterDescriptor(description='Defines agent ID.')
        self.declare_parameter('agent_id', 'HB0', param_descriptor)
        self._agent_id = self.get_parameter('agent_id')._value

        # topics:
        scan_topic = '/'+self._agent_id+'/laser_scan'
        front_camera_topic = '/'+self._agent_id+'/front_camera/color/image_raw'
        rear_camera_topic = '/'+self._agent_id+'/rear_camera/color/image_raw'
        agent_state_topic = '/'+self._agent_id+'/agent_state'
        cmd_vel_topic = '/'+self._agent_id+'/cmd_vel'

        # subscribers:
        self._laser_scan_subscriber = self.create_subscription(LaserScan, scan_topic,
				            self.laser_scan_callback, 10)
        self._front_camera_subscriber = self.create_subscription(Image, front_camera_topic,
				            self.front_camera_callback, 10)
        self._rear_camera_subscriber = self.create_subscription(Image, rear_camera_topic,
				            self.rear_camera_callback, 10)
        self._state_subscriber = self.create_subscription(State, agent_state_topic,
				            self.state_update_callback, 10)
        # self._status_subscriber = self.create_subscription(MessageType, '/agent_state',
		# 		            self.state_update_callback, 10)


        # publishers:
        self._cmd_vel_publisher = self.create_publisher(Twist, cmd_vel_topic, 10)

        # timer:
        self._publish_rate = 0.5  # cycle/sec
        self._publish_timer = self.create_timer(self._publish_rate, self.control_cycle)


        # action server:       
        self._action_server = ActionServer(self, 
                                           AgentTask, 
                                           "task_action", 
                                           self.action_execution_callback)


        # attributes:
        self._state = State() # TODO: This is in ROS message format
        # self._status = ...
        self._task_handle = None
        self._last_scan = None
        self._last_front_camera_image = None
        self._last_rear_camera_image = None
        self._current_path = None

        self._cycle_duration = 0.1
        self._Kp = 1.0
        self._landing_height_threshold = 0.2
        self._v_max = 0.2
        self._vz_max = 0.13 # maximum velocity for take-off and landing
        self._lift_height = 1.2


        # represents the state as in a finite state diagram/flowchart
        self._stage = 0 # 0 is before takeoff and after landing, drone is not actively completing a task
        self._behavior = "waiting for task"
        self._cancelled = False
        self._curr_waypoint = None
        self._next_waypoint = None
        self._task_complete = False
        self.nxt_waypoint_idx = 0
        self._stall_threshold = 120  # 2 minutes

        self.get_logger().info('Waiting for a task from Ground Control...')


    def laser_scan_callback(self, scan_msg):
        self._last_scan = scan_msg.ranges  #TODO Check this


    def front_camera_callback(self, camera_msg):
        self._last_front_camera_image = camera_msg.data


    def rear_camera_callback(self, camera_msg):
        self._last_rear_camera_image = camera_msg.data


    def state_update_callback(self, state_msg):
        self._state = state_msg    # TODO: This is in ROS message format

    
    def action_execution_callback(self, task_handle):
        """Execute a goal."""
        self.get_logger().info('Task received! Executing task...')
        
        # update task handle
        self._task_handle = task_handle

        # set the desired path
        self._current_path = self._task_handle.request.path.data
        # self.get_logger().info(self._current_path)
        self.get_logger().info('Length of path: {}'.format(len(self._current_path)))

        # find current waypoint
        # self._curr_waypoint = self.find_current_waypoint()
        self._curr_waypoint = self._current_path[self.nxt_waypoint_idx]
        self.get_logger().info('Current Waypoint: {}'.format(self._curr_waypoint))

        # create a feedback message instance
        self._feedback_msg = AgentTask.Feedback()

        # initiate take off and update stage and behavior
        self._stage = 1
        self._behavior = "taking off"

        # execute the action
        while not self._task_complete:

            # if self.nxt_waypoint_idx > len(self._current_path)-4:
            #     self.get_logger().info('In main execution loop...')


            # check if ground control has cancelled this task
            if task_handle.is_cancel_requested:
                task_handle.canceled()
                self.get_logger().info('Task canceled')
                # it seems to me that the GCS has to give a path back to base with the cancel order

                # perform a return to home location maneuver??
                # get a new path 

                return AgentTask.Result()
            
            
            match self._stage:
                case 1: # drone takes off...
                    self.take_off()
                    self._stage = 2
                    self._behavior = "path following"
                case 2: # follow path by tracking waypoints
                    while self._stage == 2:
                         # get next waypoint
                        self.get_logger().info('Getting next waypoint...')
                        self.nxt_waypoint_idx += 1
                        if self.nxt_waypoint_idx >= len(self._current_path):
                            self._stage = 3
                            self._behavior = "landing"
                        else:
                            self._next_waypoint = self.find_next_waypoint()
                            
                            # start timer to track how long agent has been traveling to next waypoint
                            t1 = time.time()

                            # set state to traveling between waypoints
                            in_btwn_waypoints = True
                            elapsed = 0.0
                            while in_btwn_waypoints:
                                # check the task has not been cancelled
                                if self.task_cancelled(task_handle):
                                    return AgentTask.Result()
                                

                                cmd_vel = self.compute_control_cmds()
                                
                                # send velocity commands to Crazyflie to move drone
                                self.move(cmd_vel)

                                # self.get_logger().info('State: {}, {}, {}'.format(self._state.position.x,
                                #                                                 self._state.position.y,
                                #                                                 self._state.position.z))

                                # check whether drone is within range of new way point or if it's on a pick/drop waypoint
                                if self.reached_waypoint() or (self._next_waypoint.id == self._curr_waypoint.id):
                                    # update current waypoint
                                    self._curr_waypoint = self._next_waypoint
                                    if self._curr_waypoint.type == 'pick':
                                        # self._stage = 3
                                        self._behavior = "pick operation: rising to bin"
                                        in_btwn_waypoints = False
                                    elif self._curr_waypoint.type == 'drop':
                                        # self._stage = 4
                                        self._behavior = "drop operation"
                                        in_btwn_waypoints = False
                                    elif self._curr_waypoint.type == 'end':
                                    #   self._stage = 5
                                        self._behavior = "landing"
                                        in_btwn_waypoints = False
                                    else:
                                        in_btwn_waypoints = False          
                                else:
                                    # if traveling between waypoints is taking too long, consider the task cancelled or get a new path
                                    t2 = time.time()
                                    elapsed = t2 - t1
                                    if (elapsed > self._stall_threshold):
                                        self.get_logger().info("Elapsed time too long! Setting cancelled to True")
                                        self._cancelled = True
                                        self._stage = 4
                                        in_btwn_waypoints = False
                            elapsed = time.time() - t1
                            self.get_logger().info("Elapsed time between waypoint {} and waypoint {}: {}".format(self.nxt_waypoint_idx-1, 
                                                                                                                         self.nxt_waypoint_idx, elapsed))
                case 3:
                    self.land()
                    self._task_complete = True
                    # status = success
                    if self._cancelled:
                        # status = fail
                        self.get_logger().info("Failed to complete task.")
                    else: 
                        self.get_logger().info("Task completed successfully.")
                    self._stage = 0
                case 4:
                    # recalculate path with no pick or drop waypoints
                    self._stage = 2
                case _:
                    # default case
                    return

            # set a cycle time/ mayb not need anymore
            time.sleep(self._cycle_duration)

        
        # indicate that the action has succeeded
        task_handle.succeed()

        # Populate result message
        result = AgentTask.Result()
        result.status = 'done' # some sort of message to indicate completion

        self.get_logger().info('Returning result: {0}'.format(result.status))

        return result

    
    def compute_control_cmds(self):

        final_cmd_vel = Twist()
        # final_cmd_vel.linear.x = 0.1
        # final_cmd_vel.angular.z = 0.1 
        # self.get_logger().info('behavior: {}'.format(self._behavior))
        match self._behavior:
            case "path following":
                # get path following velocity
                path_follow_vel_cmd = self.follow_path()

                # get obstacle avoidance velocity
                obst_avoid_vel_cmd = self.avoid_obstacle()

                pick_align_vel_cmd = None
                drop_align_vel_cmd = None
            case "pick operation: rising to bin" | "pick operation: aligning to bin" | "pick operation: picking parcel" | "pick operation: returning to highway" :
                # get path following velocity
                path_follow_vel_cmd = self.follow_path()

                # get obstacle avoidance velocity
                obst_avoid_vel_cmd = self.avoid_obstacle()

                # get bin alignment velocity
                pick_align_vel_cmd = self.align_to_bin()

                drop_align_vel_cmd = None
            case "drop operation":
                # get path following velocity
                path_follow_vel_cmd = self.follow_path()

                # get obstacle avoidance velocity
                obst_avoid_vel_cmd = self.avoid_obstacle()

                # get dropoff alignment velocity
                drop_align_vel_cmd = self.align_to_drop_off()

                pick_align_vel_cmd = None
            case _:
                self.get_logger().info("unknown arbiter mode")

        # arbiter:
        final_cmd_vel = self.blend_commands(path_follow_vel_cmd, 
                                            obst_avoid_vel_cmd,
                                            pick_align_vel_cmd,
                                            drop_align_vel_cmd)

        
        return final_cmd_vel
    
    # TODO check if the brainwaves are merged properly
    # to test out just the path following take a look at comments above avoid_obstacle function 
    def blend_commands(self, path_follow_brainwave,
                       obs_avoid_brainwave, pick_align_brainwave, drop_align_brainwave):
        merged_brainwave_x = [0] * 37
        merged_brainwave_y = [0] * 37
        merged_brainwave_z = [0] * 51
        # merges the brainwaves to determine the best x, y, and z velocities
        if path_follow_brainwave != None:
            # self.get_logger().info("path following not null!")
            merged_brainwave_x = list(np.add(merged_brainwave_x, path_follow_brainwave[0]))
            merged_brainwave_y = list(np.add(merged_brainwave_y, path_follow_brainwave[1]))
            merged_brainwave_z = list(np.add(merged_brainwave_z, path_follow_brainwave[2]))

        if obs_avoid_brainwave != None:
            # self.get_logger().info("obstacle avoidance not null!")
            merged_brainwave_x = list(np.add(merged_brainwave_x, obs_avoid_brainwave[0]))
            merged_brainwave_y = list(np.add(merged_brainwave_y, obs_avoid_brainwave[1]))
            merged_brainwave_z = list(np.add(merged_brainwave_z, obs_avoid_brainwave[2]))

        
        if pick_align_brainwave != None:
            # self.get_logger().info("pick align not null!")
            merged_brainwave_x = list(np.add(merged_brainwave_x, pick_align_brainwave[0]))
            merged_brainwave_y = list(np.add(merged_brainwave_y, pick_align_brainwave[1]))
            merged_brainwave_z = list(np.add(merged_brainwave_z, pick_align_brainwave[2]))


        if drop_align_brainwave != None:
            self.get_logger().info("drop align not null!")
            merged_brainwave_x = list(np.add(merged_brainwave_x, drop_align_brainwave[0]))
            merged_brainwave_y  = list(np.add(merged_brainwave_y, drop_align_brainwave[1]))
            merged_brainwave_z  = list(np.add(merged_brainwave_z, drop_align_brainwave[2]))

        # get final z velocity
        # merged_brainwave_z = list(merged_brainwave_z)
        # self.get_logger().info("merged brainwave_z: {}".format(merged_brainwave_z))
        # self.get_logger().info("max of z-brainwave: {}".format(max(merged_brainwave_z)))
        final_x_vel_index = merged_brainwave_x.index(max(merged_brainwave_x))
        final_y_vel_index = merged_brainwave_y.index(max(merged_brainwave_y))
        final_z_vel_index = merged_brainwave_z.index(max(merged_brainwave_z))
        # self.get_logger().info("final_z_vel index: {}".format(final_z_vel_index))
        xy_velocities = list(np.linspace(-self._v_max, self._v_max, 37))
        z_velocities = np.linspace(-self._vz_max, self._vz_max, 51)
        # self.get_logger().info("final z velocities: {}".format(list(z_velocities)))
        final_x_vel= xy_velocities[final_x_vel_index]
        final_y_vel= xy_velocities[final_y_vel_index]
        final_z_vel= z_velocities[final_z_vel_index]
        # self.get_logger().info("final z velocity: {}".format(final_z_vel))

        # self.get_logger().info("Final Heading {}".format(heading_angle))      
        
        final_cmd = Twist()
        if self._behavior == "path following":
            delta_x = self._next_waypoint.position.x - self._state.position.x
            delta_y = self._next_waypoint.position.y - self._state.position.y
        else:
            delta_x = 1.0
            delta_y = 1.0
        Kp_z = 0.5

        final_cmd.linear.x = self.clamp(final_x_vel * self._Kp * abs(delta_x), -self._v_max, self._v_max)
        final_cmd.linear.y = self.clamp(final_y_vel * self._Kp * abs(delta_y), -self._v_max, self._v_max)
        final_cmd.linear.z = self.clamp(final_z_vel, -self._vz_max, self._vz_max)
        
        # self.get_logger().info("Linear X: {}, Linear Y: {}, Linear Z: {}".format(final_cmd.linear.x, final_cmd.linear.y, final_cmd.linear.z))
        final_cmd.angular.z = 0.0
        return final_cmd



    # TODO figure out how to convert waypoint ID to coordinates
    # TODO check how function works 
    def follow_path(self):
        # relative next waypoint ID  = next waypoint relative to the current waypoint 
        global_delta_y = self._next_waypoint.position.y - self._curr_waypoint.position.y 
        global_delta_x = self._next_waypoint.position.x - self._curr_waypoint.position.x
        global_delta_z = self._next_waypoint.position.z - self._curr_waypoint.position.z
        # self.get_logger().info("Current Waypoint X: {}, Y: {}, Z: {}".format(self._curr_waypoint.position.x, self._curr_waypoint.position.y, self._curr_waypoint.position.z))
        # self.get_logger().info("Next Waypoint X: {}, Y: {}, Z: {}".format(self._next_waypoint.position.x, self._next_waypoint.position.y, self._next_waypoint.position.z))
        # self.get_logger().info("Change in X: {}, Y: {}, Z: {}".format(global_delta_x, global_delta_y, global_delta_z))

        ### Get index for xy-brainwave (37 possibilities) ###
        head_angle = self.convert_to_angle(global_delta_x, global_delta_y)
        
        if self._behavior == "path following":
            # normalize x_velocity and y_velocity
            x_vel = math.cos(math.radians(head_angle)) * self._v_max
            y_vel = math.sin(math.radians(head_angle)) * self._v_max
        else:
            x_vel = 0.0
            y_vel = 0.0
        
        xy_velocities = list(np.linspace(-self._v_max, self._v_max, 37))
        x_vel_index = xy_velocities.index(self.closest(xy_velocities, x_vel))
        y_vel_index = xy_velocities.index(self.closest(xy_velocities, y_vel))


        ### Get index for z-brainwave (51 possibilities) ###
        z_velocities = list(np.linspace(-self._vz_max, self._vz_max, 51))
        z_vel_index = z_velocities.index(self.closest(z_velocities, global_delta_z))
        
        # use generate_normalized_pdf to generate the brainwave 
        path_follow_brainwave = []
        # path_follow_brainwave.append(self.generate_normalized_pdf(head_angle_index, 0.7, 37))
        path_follow_brainwave.append(self.generate_normalized_pdf(x_vel_index, 0.7, 37))
        path_follow_brainwave.append(self.generate_normalized_pdf(y_vel_index, 0.7, 37))
        path_follow_brainwave.append(self.generate_normalized_pdf(z_vel_index, 0.7, 51))
        return path_follow_brainwave
    
    def align_to_bin(self):
        bin_pose = self.localize_bin()
        
        xy_velocities = list(np.linspace(-self._v_max, self._v_max, 37))
        z_velocities = list(np.linspace(-self._vz_max, self._vz_max, 51))
        match self._behavior:
            case "pick operation: rising to bin":
                self.get_logger().info('Pick Operation: Rising to Bin...')
                
                # set x and y vel to 0.0
                delta_x = 0.0
                delta_y = 0.0
                # find distance from bin
                delta_z = self._curr_waypoint.position.z - self._state.position.z
                # self.get_logger().info('Current Z is {}'.format(self._state.position.z))
                # self.get_logger().info('Waypoint Z is {}'.format(self._curr_waypoint.position.z))
                # self.get_logger().info('Bin_delta_z is {}'.format(delta_z))

                # if reached bin, stop moving in z-direction
                if abs(delta_z) <= 0.28:
                    self.get_logger().info('Pick Operation: Bin Reached!')
                    self._behavior = "pick operation: aligning to bin"
                    delta_z = 0.0
                
                x_vel_index = xy_velocities.index(self.closest(xy_velocities, delta_x))
                y_vel_index = xy_velocities.index(self.closest(xy_velocities, delta_y))               
                z_vel_index = z_velocities.index(self.closest(z_velocities, delta_z))

            case "pick operation: aligning to bin":
                self.get_logger().info('Pick Operation: Aligning to bin...')
                
                # call function get bin pose and calculate necessary delta_x, delta_y and delta_z

                # set x and y vel to 0.0
                delta_x = 0.0
                delta_y = 0.0
                delta_z = 0.0
                
                x_vel_index = xy_velocities.index(self.closest(xy_velocities, delta_x))
                y_vel_index = xy_velocities.index(self.closest(xy_velocities, delta_y))               
                z_vel_index = z_velocities.index(self.closest(z_velocities, delta_z))
                self.get_logger().info('Pick Operation: Aligned to bin!')
                self._behavior = "pick operation: picking parcel"

            case "pick operation: picking parcel":
                self.get_logger().info('Pick Operation: Picking Parcel...')
                
                # call function to pick parcel and set parcel_picked to True

                # set x and y vel to 0.0
                delta_x = 0.0
                delta_y = 0.0
                delta_z = 0.0
                
                x_vel_index = xy_velocities.index(self.closest(xy_velocities, delta_x))
                y_vel_index = xy_velocities.index(self.closest(xy_velocities, delta_y))               
                z_vel_index = z_velocities.index(self.closest(z_velocities, delta_z))
                self.get_logger().info('Pick Operation: Parcel Picked!')
                self._behavior = "pick operation: returning to highway"

            case "pick operation: returning to highway": 
                self.get_logger().info('Pick Operation: Returning to highway...')

                # set x and y vel to 0.0
                delta_x = 0.0
                delta_y = 0.0
                # find distance from bin
                delta_z = self._lift_height - self._state.position.z
                # self.get_logger().info('Current Z is {}'.format(self._state.position.z))
                # self.get_logger().info('Waypoint Z is {}'.format(self._curr_waypoint.position.z))
                # self.get_logger().info('Bin_delta_z is {}'.format(delta_z))

                # if reached bin, stop moving in z-direction
                if abs(delta_z) <= 0.45:
                    self.get_logger().info('Pick Operation: Reached highway!')
                    self._behavior = "path following"
                    delta_z = 0.0
                
                x_vel_index = xy_velocities.index(self.closest(xy_velocities, delta_x))
                y_vel_index = xy_velocities.index(self.closest(xy_velocities, delta_y))               
                z_vel_index = z_velocities.index(self.closest(z_velocities, delta_z))
            case _:
                self.get_logger().info('Pick Operation: Do Nothing....')

        # use generate_normalized_pdf to generate the brainwave 
        bin_align_brainwave = []
        bin_align_brainwave.append(self.generate_normalized_pdf(x_vel_index, 0.7, 37))
        bin_align_brainwave.append(self.generate_normalized_pdf(y_vel_index, 0.7, 37))
        bin_align_brainwave.append(self.generate_normalized_pdf(z_vel_index, 1.0, 51))           
        return bin_align_brainwave

    def align_to_drop_off(self, bin_pose):
        drop_pose = self.localize_drop()
        drop_align_brainwave = None
        # ....
        return drop_align_brainwave

    
    #TODO right now the function just replces the value if it gets new reading from the lidar. Since tere is one value for eaxh 10 degrees might want to optimize it 
    # to test just path following comment out the "for loop"
    def avoid_obstacle(self): #,lidar_scan
        # function creates brainwave for obstacle avoidance
        # assumes the zero alighs with the 0 of the drone heading direction
        # what to expect from lidar [{ "x": 1.23, "y": 4.56, "distance": 5.67, "intensity": 10 },  { "x": -2.34, "y": 0.12, "distance": 3.45, "intensity": 5 },  { "x": 6.78, "y": -9.87, "distance": 10.11, "intensity": 8 }]
        # obst_avoid_brainwave = np.zeros(37)
        # for point in lidar_scan:
        #     obstacle_coordinate_x = point["x"]
        #     obstacle_coordinate_y = point["y"]
        #     # converts to degree plus rounds it to the place
        #     obstacle_place = round(self.convert_to_angle(obstacle_coordinate_x, obstacle_coordinate_y) / 10) 
        #     obst_avoid_brainwave[obstacle_place] = point["distace"] * -1 
        # obst_avoid_brainwave = [0]*37
        obst_avoid_brainwave = None
        return obst_avoid_brainwave

    def move(self, cmd_vel):
        # publish control command
        self._cmd_vel_publisher.publish(cmd_vel)
        # self.get_logger().info("Moving agent...")
        # update the state by getting location data

        # publish the feedback (state of the agent)
        self._feedback_msg.state = self._state
        self._feedback_msg.agent_id = self._agent_id
        self._task_handle.publish_feedback(self._feedback_msg)   
    
    def convert_to_angle(self, x, y):
        # find angle between delta_x vector and delta_y vector
        angle = math.degrees(math.atan2(y, x)) + 180
        # self.get_logger().info("Angle of Attack: {}".format(angle))
        # outputs angle from 0 to 370
        return angle
    
    def generate_normalized_pdf(self, center_index, std_deviation, size):
        # Calculate the positions for the array based on the center index
        positions = np.arange(size) - center_index

        # Calculate the Gaussian distribution values using the positions, mean=0
        gaussian_values = np.exp(-0.5 * (positions / std_deviation) ** 2)

        # Normalize the Gaussian values to create a PDF (Probability Density Function)
        normalized_pdf = gaussian_values / np.sum(gaussian_values)

        return list(normalized_pdf)

    def hover(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0

        self.move(cmd_vel)



    def reached_waypoint(self):
        # check whether we have reached the next waypoint
        distance_threshold = 0.5
        delta_x = self._next_waypoint.position.x - self._state.position.x
        delta_y = self._next_waypoint.position.y - self._state.position.y
        wypt_distance = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))

        if wypt_distance <= distance_threshold:
            self.get_logger().info("Drone has reached next waypoint!")
            return True
        else:
            return False
    
    def localize_bin(self):
        bin_pose = None
        # ....
        return bin_pose
    
    def localize_drop(self):
        drop_pose = None
        # ....
        return drop_pose
    

    def find_next_waypoint(self):
    # intakes the current node and finds the next one
    # self.nxt_waypoint_idx = self._current_path.index(self._curr_waypoint) + 1
        self.get_logger().info('Next Waypoint Index {}:'.format(self.nxt_waypoint_idx))

        nxt_waypoint = self._current_path[self.nxt_waypoint_idx]
        # calculate next waypoint
        return nxt_waypoint


    # def find_current_waypoint(self):
    #     """Returns the corresponding (or closest) node/waypoint to the given position"""
    #     curr_waypoint = None
    #     min_dist = 999

    #     # iterate over all the nodes/waypoints in the path to find the one closest to the [x,y,z] point
    #     for wp in self._current_path:
    #         dist = self.calculate_euclidean_dist(wp.position, self._state.position)

    #         if dist < min_dist:
    #             curr_waypoint = wp
    #             min_dist = dist
    #             self.get_logger().info('Min node is {} and distance is {}'.format(curr_waypoint.id, min_dist))
    #     print("-------------------")

    #     return curr_waypoint
    
    

    def control_cycle(self, ):
        pass

    
    def convert_to_ros_state(self, state_py):
        ros_state = State()
        ros_state.position.x = state_py.x_pos
        ros_state.position.y = state_py.y_pos
        ros_state.position.z = state_py.z_pos
        ros_state.orientation.z = state_py.psi


    def convert_from_ros_state(self, ros_state):
        state = StatePy()
        state.x_pos = ros_state.position.x
        state.y_pos = ros_state.position.y
        state.z_pos = ros_state.position.z
        state.psi = ros_state.orientation.z


    def calculate_euclidean_dist(self, pos1, pos2):
        """Returns the euclidean distance between two geometry_msg type positions"""
        return np.sqrt((pos1.x - pos2.x)**2 + 
                       (pos1.y - pos2.y)**2 +
                       (pos1.z - pos2.z)**2)
    
    def closest(self, lst, K):
        return lst[min(range(len(lst)), key = lambda i: abs(lst[i]-K))]

    # def task_complete(self):
    #     return False
    
    def task_cancelled(self, task_handle):
        # check if ground control has cancelled this task
        if task_handle.is_cancel_requested:
            task_handle.canceled()
            self.get_logger().info('Task canceled')
            self._cancelled = True
    
    def take_off(self):
        self.get_logger().info('Taking Off...')

        takeoff_cmd_vel = Twist()
        takeoff_cmd_vel.linear.x = 0.0
        takeoff_cmd_vel.linear.y = 0.0
        while (self._state.position.z < self._lift_height):
            # self.get_logger().info('Taking Off...')
            delta_z = self._lift_height - self._state.position.z
            takeoff_cmd_vel.linear.z = self._vz_max * np.clip(delta_z, 0.0, 1.0)
            self.move(takeoff_cmd_vel)

    def land(self):
        self.get_logger().info('Landing...')

        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        self.get_logger().info('Land command velocity created...')
        while (self._state.position.z**2) > self._landing_height_threshold:
            self.get_logger().info('Height above threshold,...')
            Kp_z = 0.8
            cmd_vel.linear.z = np.clip(Kp_z * (-self._state.position.z), -self._vz_max, self._vz_max)

            # send velocity to agent using move 
            self.move(cmd_vel)
            time.sleep(0.1) #TODO: How to standardize the publishing rate??
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        cmd_vel.linear.z = 0.0
        self.move(cmd_vel)
        self.get_logger().info('Landing complete!')


    def check_bin_alignment(self):
        well_aligned = False
        #using camera and infrared, determine if agent is well_aligned and ready to grab parcel

    
    def clamp(self, value, value_min, value_max):
        return min(max(value, value_min), value_max)

    

def main(args=None):
    rclpy.init(args=args)

    agent_control = AgentControlNode()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()

    rclpy.spin(agent_control, executor=executor)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    agent_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

