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

        # represents the state as in a finite state diagram/flowchart
        self._stage = 0 # 0 is before takeoff and after landing, drone is not actively completing a task
        self._cancelled = False
        self._curr_waypoint = None
        self._next_waypoint = None
        self._task_complete = False

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
        self._curr_waypoint = self._current_path[0]
        self.get_logger().info('Current Waypoint: {}'.format(self._curr_waypoint))

        # create a feedback message instance
        feedback_msg = AgentTask.Feedback()

        # initiate take off and update stage
        self._stage = 1
        stall_threshold = 120  # 2 minutes

        # execute the action
        while not self._task_complete:

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
                case 2: # follow path by tracking waypoints
                    while self._stage == 2:
                         # get next waypoint
                         self._next_waypoint = self.find_next_waypoint()

                         # start timer to track how long agent has been traveling to next waypoint
                         t1 = time.time()

                         # set state to traveling between waypoints
                         in_btwn_waypoints = True
                         
                         while in_btwn_waypoints:

                            # check the task has not been cancelled
                            if self.task_cancelled(task_handle):
                                return AgentTask.Result()
                            
                            # proceed if it has not

                            cmd_vel = self.compute_control_cmds("path following")
                            
                            # send velocity commands to Crazyflie to move drone
                            self.move(cmd_vel, task_handle, feedback_msg)

                            # check whether drone is within range of new way point
                            if self.reached_waypoint():
                                # update current waypoint
                                self._curr_waypoint = self._next_waypoint
                                if self._curr_waypoint.type == 'pick':
                                    self._stage = 3
                                    in_btwn_waypoints = False

                                elif self._curr_waypoint.type == 'drop':
                                    self._stage = 4
                                    in_btwn_waypoints = False

                                elif self._curr_waypoint.type == 'end':
                                  self._stage = 5
                                  in_btwn_waypoints = False

                                else:
                                  in_btwn_waypoints = False
                                  
                            else:
                                # if traveling between waypoints is taking too long, consider the task cancelled or get a new path
                                t2 = time.time()
                                elapsed = t2 - t1
                                if elapsed > stall_threshold:
                                    self._cancelled = True
                                    self._stage = 6
                                    in_btwn_waypoints = False
                case 3:
                    # bin_aligned = False
                    # parcel_picked = False
                    # while self._stage == 3:
                    #     if bin_aligned:
                    #         # call function to pick parcel and set parcel_picked to True

                    #         # check that parcel has been successfully picked up (this check 
                    #         # could be in drop function tho eliminating need for parcel_picked variable 
                    #         # and directly updating the stage after picking the parcel)
                    #         if parcel_picked:  
                    #             # prepare to continue path following
                    #             self._stage = 2
                    #             break
                    #     else:
                        
                    #         cmd_vel = self.compute_control_cmds("picking up")
                            
                    #         # send velocity commands to Crazyflie to move drone until aligned with bin
                    #         self.move(cmd_vel, task_handle, feedback_msg)
                            
                    #         # call function to check when drone is aligned to bin and set bin_aligned to True

                    #         # NOTE: compute_control_cmds should maybe be called in a loop inside of a new function
                    #         # that only exits when the drone is sufficiently aligned to the pick up bin rather 
                    #         # than checking for alignment after every motor command
                    self.get_logger().info('Package picked!! Woohoo...')
                    self._stage = 2

                        
                case 4:
                    # parcel_dropped = False
                    # drop_aligned = False
                    # while self._stage == 4:
                    #     if drop_aligned:
                    #         # call function to drop parcel and set parcel_dropped to True

                    #         # check that parcel has been successfully dropped down (this check 
                    #         # could be in drop function tho eliminating need for parcel_dropped variable 
                    #         # and directly updating the stage after dropping the parcel)
                    #         if parcel_dropped:  
                    #             # prepare to continue path following
                    #             self._stage = 2
                    #             break
                    #     else:
                    #         cmd_vel = self.compute_control_cmds("dropping off")
                            
                    #         # send velocity commands to Crazyflie to move drone until aligned with bin
                    #         self.move(cmd_vel, task_handle, feedback_msg)
                            
                    #         # call function to know when drone is aligned to drop_off bin and set drop_aligned to True

                    #         # NOTE: compute_control_cmds should maybe be called in a loop inside of a new function
                    #         # that only exits when the drone is sufficiently aligned to the drop off bin rather 
                    #         # than checking for alignment after every motor command
                    self.get_logger().info('Package dropped!! Phowww...')
                    self._stage = 2               
                case 5:
                    self.land()
                    self._task_complete = True
                    # status = success
                    if self._cancelled:
                        # status = fail
                        self.get_logger().info("Failed to complete task.")
                    
                    self.get_logger().info("Task completed successfully.")
                    self._stage = 0
                case 6:
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

    
    def compute_control_cmds(self, mode):

        final_cmd_vel = Twist()
        final_cmd_vel.linear.x = 0.1
        final_cmd_vel.angular.z = 0.1 

        match mode:
            case "path following":
                # get path following velocity
                path_follow_vel_cmd = self.follow_path()

                # get obstacle avoidance velocity
                obst_avoid_vel_cmd = self.avoid_obstacle()

                bin_align_vel_cmd = None
                drop_align_vel_cmd = None
            case "picking up":
                # get obstacle avoidance velocity
                obst_avoid_vel_cmd = self.avoid_obstacle()

                # get bin alignment velocity
                bin_pose = self.localize_bin()
                bin_align_vel_cmd = self.align_to_bin(bin_pose)

                path_follow_vel_cmd = None
                drop_align_vel_cmd = None
            case "dropping off":
                # get obstacle avoidance velocity
                obst_avoid_vel_cmd = self.avoid_obstacle()

                # get dropoff alignment velocity
                drop_pose = self.localize_drop()
                drop_align_vel_cmd = self.align_to_drop_off(drop_pose)

                path_follow_vel_cmd = None
                bin_align_vel_cmd = None
            case _:
                self.get_logger().info("unknown arbiter mode")

        # arbiter:
        final_cmd_vel = self.blend_commands(path_follow_vel_cmd, 
                                            obst_avoid_vel_cmd)
                                            #bin_align_vel_cmd,
                                            #drop_align_vel_cmd)

        
        return final_cmd_vel


    
    def find_next_waypoint(self) :
        # intakes the current node and finds the next one
        nxt_waypoint_indx = self._current_path.index(self._curr_waypoint) + 1
        self.get_logger().info('Next Waypoint Index {}:'.format(nxt_waypoint_indx))
        nxt_waypoint = self._current_path[nxt_waypoint_indx]
        # calculate next waypoint
        return nxt_waypoint
    

    def find_current_waypoint(self):
        """Returns the corresponding (or closest) node/waypoint to the given position"""
        curr_waypoint = None
        min_dist = 999

        # iterate over all the nodes/wapoints in the path to find the one closest to the [x,y,z] point
        for wp in self._current_path:
            dist = self.calculate_euclidean_dist(wp.position, self._state.position)

            if dist < min_dist:
                curr_waypoint = wp
                min_dist = dist
                self.get_logger().info('Min node is {} and distance is {}'.format(curr_waypoint.id, min_dist))
        print("-------------------")

        return curr_waypoint


    # TODO figure out how to convert waypoint ID to coordinates
    # TODO check how funciton works 
    def follow_path(self):
        # relative next waypoint ID  = next waypoint relative to the current waypoint 
        global_delta_y = self._next_waypoint.position.y - self._curr_waypoint.position.y 
        global_delta_x = self._next_waypoint.position.x - self._curr_waypoint.position.x
        global_delta_z = self._next_waypoint.position.z - self._curr_waypoint.position.z
        # self.get_logger().info("Current Waypoint X: {}".format(self._curr_waypoint.position.x))
        # self.get_logger().info("Current Waypoint Y: {}".format(self._curr_waypoint.position.y))
        # self.get_logger().info("Current Waypoint Z: {}".format(self._curr_waypoint.position.z))
        # self.get_logger().info("Next Waypoint X: {}".format(self._next_waypoint.position.x))
        # self.get_logger().info("Next Waypoint Y: {}".format(self._next_waypoint.position.y))
        # self.get_logger().info("Next Waypoint Z: {}".format(self._next_waypoint.position.z))
        # self.get_logger().info("Change in X: {}".format(global_delta_x))
        # self.get_logger().info("Change in Y: {}".format(global_delta_y))
        # self.get_logger().info("Change in Z: {}".format(global_delta_z))

        # convert to angle (relative next waypoint id)
        head_angle = self.convert_to_angle(global_delta_x, global_delta_y)
        # angle of heading converted to a position in list of brainwave
        if head_angle == 360: 
            head_angle_index = 0 
        else: 
            head_angle_index = round(head_angle/10)
        # xy direction represent distance between drone location and waypoint in xy plane
        xy_distance = math.sqrt(pow(global_delta_x, 2) + pow(global_delta_y, 2))
        if xy_distance == 0: # waypoint is directly above drone
            angle_of_elevation_rad = math.pi/2
        else: 
            # angle_of_elevation_rad = math.atan2(global_delta_z/xy_distance)
            angle_of_elevation_rad = math.asin(global_delta_z/xy_distance)
        # use create_normal_curve to generate the brainwave 
        path_follow_brainwave = list(self.create_normal_curve(head_angle_index))
        path_follow_brainwave.append(angle_of_elevation_rad)
        # self.get_logger().info("Pathfollowing Brainwave {}".format(path_follow_brainwave))
        return path_follow_brainwave
    
    
    
    #TODO right now the function just replces the value if it gets new reading from the lidar. Since tere is one value for eaxh 10 degrees might want to optimize it 
    # to test just path following comment out the "for loop"
    def avoid_obstacle(self): #,lidar_scan
        # function creates brainwave for obstacle avoidance
        # assumes the zero alighs with the 0 of the drone heading direction
        # what to expect from lidar [{ "x": 1.23, "y": 4.56, "distance": 5.67, "intensity": 10 },  { "x": -2.34, "y": 0.12, "distance": 3.45, "intensity": 5 },  { "x": 6.78, "y": -9.87, "distance": 10.11, "intensity": 8 }]
        # obst_avoid_brainwave = np.zeros(36)
        # for point in lidar_scan:
        #     obstacle_coordinate_x = point["x"]
        #     obstacle_coordinate_y = point["y"]
        #     # converts to degree plus rounds it to the place
        #     obstacle_place = round(self.convert_to_angle(obstacle_coordinate_x, obstacle_coordinate_y) / 10) 
        #     obst_avoid_brainwave[obstacle_place] = point["distace"] * -1 
        obst_avoid_brainwave = [0]*36
        return obst_avoid_brainwave
    
    
    def convert_to_angle(self, x, y):
        # find angle between delta_x vector and delta_y vector
        angle = math.degrees(math.atan2(y, x)) + 180
        # self.get_logger().info("Angle of Attack: {}".format(angle))
        # outputs angle from 0 to 360
        return angle
    
    #TODO might give out an error when center of the wave is placed at index of "0"
    def create_normal_curve(self, position):
    # Generate a list of 36 values with a normal curve at specified position 
    # used to generate a brainwave for path_following
        values = np.zeros(36)
        curve_start = position - 2
        curve_end = position + 3
        x = np.linspace(-2, 2, 5)
        curve = norm.pdf(x)
        if position > 1: 
            values[curve_start:curve_end] = curve
        elif position == 1:
            values[-1] = curve[1]
            values[0:4] = curve[1:]
        elif position == 0:
            values[-2:] = curve[0:2]
            values[0:3] = curve[2:]
        return values

    def move(self, cmd_vel, task_handle, feedback_msg):
        # publish control command
        self._cmd_vel_publisher.publish(cmd_vel)
        self.get_logger().info("Moving agent...")
        # update the state by getting location data

        # publish the feedback (state of the agent)
        feedback_msg.state = self._state
        feedback_msg.agent_id = self._agent_id
        task_handle.publish_feedback(feedback_msg)

    def reached_waypoint(self):
        # code to check whether we have reached the next waypoint
        # get current location
        # comp current location to waypoint location
        # if close, return True, else return False
        distance_threshold = 0.1
        delta_x = self._next_waypoint.position.x - self._state.position.x
        delta_y = self._next_waypoint.position.y - self._state.position.y
        wypt_distance = math.sqrt(pow(delta_x, 2) + pow(delta_y, 2))
        if wypt_distance <= distance_threshold:
            self.get_logger().info("Drone has reached next waypoint!")
            return True
        else:
            return False


        

        return False
    
    def localize_bin(self):
        bin_pose = None
        # ....
        return bin_pose
    
    def localize_drop(self):
        drop_pose = None
        # ....
        return drop_pose
    

    def align_to_bin(self, bin_pose):
        bin_align_cmd = None
        # ....
        return bin_align_cmd

    def align_to_drop_off(self, bin_pose):
        drop_align_cmd = None
        # ....
        return drop_align_cmd

    # TODO check if the barinwaves are merged properly
    # to test out just the path following take a look at comments above avoid_obstacle function 
    def blend_commands(self, path_follow_brainwave,
                       obs_avoid_brainwave):
        # merges the brainwaves to determine the best directio of heading 
        z_angle_rad = path_follow_brainwave[-1]
        path_follow_brainwave.pop(-1)
        merged_brainwave = list(np.array(path_follow_brainwave) + np.array(obs_avoid_brainwave))
        heading_angle = ((merged_brainwave.index(max(merged_brainwave)))*10)
        heading_angle_rad = math.radians(heading_angle)
        # self.get_logger().info("Final Heading {}".format(heading_angle))
        velocity = 1
        final_cmd = Twist()
        # instead of multiplying by '-1' try to go to follow path and switch substraction of current and previous (might work, needs to be tried out)
        final_cmd.linear.x = math.cos(heading_angle_rad) * velocity
        final_cmd.linear.y = math.sin(heading_angle_rad) * velocity
        final_cmd.linear.z = math.sin(z_angle_rad) * velocity
        self.get_logger().info("Linear X: {}, Linear Y: {}, Linear Z: {}".format(final_cmd.linear.x, final_cmd.linear.y, final_cmd.linear.z))
        final_cmd.angular.z = 0.0
        #final_cmd = None 
        return final_cmd
    

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
        #crazyflie method to initiate takeoff
        # update stage to start path following

    def land(self):
        self.get_logger().info('Landing...')
        # crazyflie method to initiate landing


    def check_bin_alignment(self):
        well_aligned = False
        #using camera and infrared, determine if agent is well_aligned and ready to grab parcel

        

    

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

