import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import ParameterDescriptor

import time

from hbird_msgs.action import AgentTask
from hbird_msgs.msg import Waypoint, State
from sensor_msgs.msg import LaserScan, Image
from geometry_msgs.msg import Twist
from .scripts.utils import State as StatePy


#TODO:
# 1. Add all the right message types to the pub/subs
# 2. Define the state message type (would be custom one). Align with the State data class?
# 3. Do we need a control_cycle??
# 4. Fix the action server... ActionType, etc.
# 5. Set the initial states, etc.
# 6. Write out all callbacks to store (and process sensor data)
# 7. How to handle cancellations?? line 94-98
# 8. Figure out cycle time for the main execution loop



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

        self._cycle_duration = 0.1

        # represents the state as in a finite state diagram/flowchart
        self.stage = 0 # 0 is before takeoff and after landing, drone is not actively completing a task
        self.cancelled = False
        self.curr_waypoint_id = "A1"
        self.task_complete

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

        # create a feedback message instance
        feedback_msg = AgentTask.Feedback()

        # initiate take off and update stage
        self.stage = 1
        stall_threshold = 120  # 2 minutes

        # execute the action
        while not self.task_complete:

            # check if ground control has cancelled this task
            if task_handle.is_cancel_requested:
                task_handle.canceled()
                self.get_logger().info('Task canceled')
                # it seems to me that the GCS has to give a path back to base with the cancel order

                # perform a return to home location maneuver??
                # get a new path 

                return AgentTask.Result()
            

            match self.stage:
                case 1:
                    self.takeoff()
                    self.stage = 2
                case 2:
                    while self.stage == 2:
                         # get next waypoint
                         nxt_waypoint_id = self.find_next_waypoint(self.curr_waypoint_id)

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
                            self.move(self, cmd_vel, task_handle, feedback_msg)

                            # check whether drone is within range of new way point
                            if self.reached_waypoint(nxt_waypoint_id):
                                # update current waypoint
                                self.curr_waypoint_id = nxt_waypoint_id
                                # if "pick"
                                #   self.stage = 3
                                #   in_btwn_waypoints = False

                                # else if "drop"
                                #   self.stage = 4
                                #   in_btwn_waypoints = False

                                # else if "end"
                                #   self.stage = 5
                                #   in_btwn_waypoints = False

                                # else:
                                #   in_btwn_waypoints = False
                            else:
                                # if traveling between waypoints is taking too long, consider the task cancelled or get a new path
                                t2 = time.time()
                                elapsed = t2 - t1
                                if elapsed > stall_threshold:
                                    self.cancelled = True
                                    self.stage = 6
                                    in_btwn_waypoints = False
                case 3:
                    bin_aligned = False
                    parcel_picked = False
                    while self.stage == 3:
                        if bin_aligned:
                            # call function to pick parcel and set parcel_picked to True

                            # check that parcel has been successfully picked up (this check 
                            # could be in drop function tho eliminating need for parcel_picked variable 
                            # and directly updating the stage after picking the parcel)
                            if parcel_picked:  
                                # prepare to continue path following
                                self.stage = 2
                                break
                        else:
                        
                            cmd_vel = self.compute_control_cmds("picking up")
                            
                            # send velocity commands to Crazyflie to move drone until aligned with bin
                            self.move(self, cmd_vel, task_handle, feedback_msg)
                            
                            # call function to check when drone is aligned to bin and set bin_aligned to True

                            # NOTE: compute_control_cmds should maybe be called in a loop inside of a new function
                            # that only exits when the drone is sufficiently aligned to the pick up bin rather 
                            # than checking for alignment after every motor command

                        
                case 4:
                    parcel_dropped = False
                    drop_aligned = False
                    while self.stage == 4:
                        if drop_aligned:
                            # call function to drop parcel and set parcel_dropped to True

                            # check that parcel has been successfully dropped down (this check 
                            # could be in drop function tho eliminating need for parcel_dropped variable 
                            # and directly updating the stage after dropping the parcel)
                            if parcel_dropped:  
                                # prepare to continue path following
                                self.stage = 2
                                break
                        else:
                            cmd_vel = self.compute_control_cmds("dropping off")
                            
                            # send velocity commands to Crazyflie to move drone until aligned with bin
                            self.move(self, cmd_vel, task_handle, feedback_msg)
                            
                            # call function to know when drone is aligned to drop_off bin and set drop_aligned to True

                            # NOTE: compute_control_cmds should maybe be called in a loop inside of a new function
                            # that only exits when the drone is sufficiently aligned to the drop off bin rather 
                            # than checking for alignment after every motor command
                case 5:
                    self.land()
                    self.task_complete = True
                    # status = success
                    if self.cancelled:
                        # status = fail
                        self.get_logger().info("Failed to complete task.")
                    
                    self.get_logger().info("Task completed successfully.")
                    self.stage = 0
                case 6:
                    # recalculate path with no pick or drop waypoints
                    self.stage = 2
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
                path_follow_vel_cmd = self.follow_path(nxt_waypoint_id)

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
                                            obst_avoid_vel_cmd,
                                            bin_align_vel_cmd,
                                            drop_align_vel_cmd)

        
        return final_cmd_vel


    
    def find_next_waypoint(self, current_waypoint_id):
        nxt_waypoint_id = None

        # calculate next waypoint
        return nxt_waypoint_id
    

    def follow_path(self, nxt_waypoint_id):
        path_follow_cmd = None
        # ...
        return path_follow_cmd
    

    def avoid_obstacle(self):
        obs_avoid_cmd = None
        # ....
        return obs_avoid_cmd
    
    def move(self, cmd_vel, task_handle, feedback_msg):
        # publish control command
        self._cmd_vel_publisher.publish(cmd_vel)

        # update the state

        # publish the feedback (state of the agent)
        feedback_msg.state = self._state
        feedback_msg.agent_id = self._agent_id
        task_handle.publish_feedback(feedback_msg)

    def reached_waypoint(self, nxt_waypoint_id):
        # code to check whether we have reached the next waypoint
        # get current location
        # comp current location to waypoint location
        # if close, return True, else return False
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


    def blend_commands(self, path_follow_cmd, 
                       obs_avoid_cmd,
                       bin_align_cmd):
        final_cmd = None
        # ....
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


    def task_complete(self):
        return False
    
    def task_cancelled(self, task_handle):
        # check if ground control has cancelled this task
        if task_handle.is_cancel_requested:
            task_handle.canceled()
            self.get_logger().info('Task canceled')

        self.cancelled = True
    
    def take_off(self):
        self.get_logger().info('Taking Off...')
        #crazyflie method to initiate takeoff
        # update stage to start path following

    def land(self):
        self.get_logger().info('Landing...')
        # crazyflie method to initiate landing


    def check_bin_alignment(self):
        well_aligned = false
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
