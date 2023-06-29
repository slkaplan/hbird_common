import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import time
import yaml
import pathlib

from hbird_interfaces.action import AgentTask
from hbird_interfaces.msg import Waypoint, Path

from .scripts.Quadrotor import Quadrotor
from .scripts.utils import Task, Position, State, define_env
from .scripts.utils import WaypointPy as WaypointPy


#TODO:
# 1. Make sure that initialization works well--using the right yaml file
# [DONE] 2. (Action) Create the ActionType
# [DONE] 3. (Action) Use the ID from the config file to address action clients?
# [DONE] 4. How to handle URI in this context?
# 5. Is the Quadrotor class fine as is? Do we need to use it?
# [DONE] 6. Implement a simple task assignment... it could just be random assignment
# 7. In "generate_agent_paths", we'll need the a-star and others...? For now, just make something up
# [DONE] 8. (Action) Understand how Actions work and update the send_paths function
# [DONE] 9. Add a way to take in the feedback message and update trace of the agents




class GroundControlNode(Node):
    
    def __init__(self, config):
        super().__init__("ground_control_node")

        self.get_logger().info('Starting up the Ground Control Node...\n')

        self._mode = config['mode']
        self._agent_init = config['agent_init']
        self._time_delta = config['time_delta']
        self._env_simplified = config['map'].copy()
        self._env = define_env(config['map'])
        self._num_agents = len(self._agent_init)

        # initialize variables
        self._agent_list = dict()
        self._task_list = dict()
        self._task_assignment = dict()
        self._agent_paths = dict()
        # self._available_agents = list(self._agent_list.keys()) #TODO: Set after init_agent_list
        # self._task_queue = list(self._task_list.keys())
        self._completed_tasks = dict()
        self._agents_active = False


        # initialize agent list
        self.init_agent_list()

        # initialize task list
        self.init_task_list(config)


        # initialize the action clients for each HB agent
        self._action_clients = dict()
        for agent in self._agent_list.values():
            self._action_clients[agent._id] = ActionClient(self, 
                                                           AgentTask, 
                                                           agent._id+"_action")  # e.g., "HB1_action"
      
    
    def init_agent_list(self):
        # Define the agent list

        # Plotly colors
        colors = [
            '#1f77b4',  # muted blue
            '#ff7f0e',  # safety orange
            '#2ca02c',  # cooked asparagus green
            '#d62728',  # brick red
            '#9467bd',  # muted purple
            '#8c564b',  # chestnut brown
            '#e377c2',  # raspberry yogurt pink
            '#7f7f7f',  # middle gray
            '#bcbd22',  # curry yellow-green
            '#17becf'   # blue-teal
        ]

        for i in range(self._num_agents):
            # define initial state
            start = State(x_pos=self._agent_init[i][1][0],
                        y_pos=self._agent_init[i][1][1])
            # define agent as Quadrotor
            agent = Quadrotor(init_state=start,
                            color=colors[i],
                            id=self._agent_init[i][0],
                            take_off_height=self._agent_init[i][2],
                            mode=self._mode,
                            dt=self._time_delta)
            self._agent_list[agent._id] = agent

        
            print(f'Agent {agent._id} | Mode [{self._mode}] | Position  \
                  {self._agent_list[agent._id].get_pos().x, self._agent_list[agent._id].get_pos().y} ')
        print('----------------------------------')

    
    def init_task_list(self, config):
        
        pick_locations = config['pick_loc']
        drop_locations = config['drop_loc']

        print('-----------------------')
        print('Task List')
        print('-----------------------')

        for i in range(len(pick_locations)):
            t = Task(pick_loc=Position(x=pick_locations[i][1][0], y=pick_locations[i][1][1], z=pick_locations[i][1][2]),
                    drop_loc=Position(
                        x=drop_locations[i][1][0], y=drop_locations[i][1][1], z=0.0),
                    pick_id=pick_locations[i][0],
                    drop_id=drop_locations[i][0],
                    id='T'+str(i),
                    priority=i,
                    time_input=time.time())
            self._task_list['T'+str(i)] = t

            print(f'Task {t.id}: {t.pick_id} -> {t.drop_id}')

        print('-----------------------')


    def create_task_assignment(self):
        # Temporary implementation where number of tasks must be equal to number of agents
        if self._num_agents != len(self._task_list):
            raise ValueError("Scenario must ensure number of agents is equal to number of tasks!")
        
        for i, agent in enumerate(self._agent_list.values()):
            self._task_assignment[agent] = self._task_list['T'+str(i)]   # key is agent object and value is task object

        # % PRINT DEBUG
        for agent, task in self._task_assignment.items():
            print(f'Agent {agent._id} : {task.pick_id} -> {task.drop_id}')


    def generate_agent_paths(self):
        """generate path/trajectory for each agent based on assignment"""
        
        # your code here...

        # each path should be a list of WaypointPy instances

        self.get_logger().info('Generating agent paths...')

        # create fake paths
        paths = []
        paths.append([WaypointPy(id='pick', x=1.67, y=0.45),
                      WaypointPy(id=' ', x=1.67, y=1.67),
                      WaypointPy(id='drop', x=1.67, y=2.28)])
        paths.append([WaypointPy(id='pick', x=3.81, y=0.45),
                      WaypointPy(id=' ', x=3.81, y=1.67),
                      WaypointPy(id='drop', x=3.81, y=2.28)])

        # add fake paths to agents
        for i, agent in enumerate(self._agent_list.values()):
            agent.set_path(paths[i])


        self.get_logger().info('Agent path generation is complete!!')

    
    def send_paths(self):

        for agent in self._agent_list.values():

            # % PRINT DEBUG
            self.get_logger().info('Task for Agent [{0}] is trying to send...'.format(agent._id))

            task_msg = AgentTask.Goal()     # create an instance AgentTask action
            task_msg.path = self.convert_to_ros_path(agent.get_path())    # convert from list to ROS message type (Waypoint)
            
            ac = self._action_clients[agent._id]
            ac.wait_for_server() # Waits for server to be available, then sends goal/paths

            # returns a future handle, client runs feedback_callback after sending the goal/task
            future_handle = ac.send_goal_async(task_msg, feedback_callback=self.feedback_callback)
            # register a callback for when the task is 'complete' (i.e., either server accepts or rejects goal request)
            future_handle.add_done_callback(self.goal_response_callback)

            # % PRINT DEBUG
            self.get_logger().info('Task for Agent [{0}] is sent'.format(agent._id)) #TODO: Create a getter for ID


    def feedback_callback(self, feedback_msg):
        """ Run when client sends goal"""
        agent_state = self.convert_from_ros_state(feedback_msg.feedback.state)   # this converts from ROS to python State datatype
        agent_id = feedback_msg.feedback.agent_id

        # update the agent state and state trace
        self._agent_list[agent_id].update_state(agent_state)

        # % PRINT DEBUG
        self.get_logger().info('Agent [{}] | Updating state [x: {}, y: {}, z: {}]'.format(agent_id,
                                                                                           agent_state.x_pos,
                                                                                           agent_state.y_pos,
                                                                                           agent_state.z_pos))


    def goal_response_callback(self, future):
        # Get handle for the goal we just sent
        goal_handle = future.result()

        # Return early if goal is rejected
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        # Use goal handle to request the result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    def get_result_callback(self, future):
        result = future.result().result

        # Log result and shut down ROS 2 cleanly
        self.get_logger().info('Result: {0}'.format(result.status))
        
        #TODO: Not sure we want to shutdown here
        rclpy.shutdown()


    def convert_to_ros_path(self, path):

        ros_path = Path()

        for path_wp in path:
            ros_wp = Waypoint()
            ros_wp.position.x = path_wp.x
            ros_wp.position.y = path_wp.y
            ros_wp.position.z = path_wp.z
            ros_wp.id = path_wp.id
            ros_wp.heading = path_wp.heading
            
            ros_path.data.append(ros_wp)

        return ros_path


    def convert_from_ros_state(self, ros_state):
        state = State()
        state.x_pos = ros_state.position.x
        state.y_pos = ros_state.position.y
        state.z_pos = ros_state.position.z
        state.psi = ros_state.orientation.z

        return state


def main(args=None):
    rclpy.init(args=args)

    # load configuration file from YAML file
    config_path = str(pathlib.Path().parent.resolve())+"/src/hbird_software_system/hbird_navigation/hbird_navigation/config/"
    config_file = 'scenario_1.yaml'
    with open(config_path+config_file, 'r') as file:
        config = yaml.load(file, Loader=yaml.FullLoader)

    gcs = GroundControlNode(config)

    # create task assignment
    gcs.create_task_assignment()

    # generate agent paths
    gcs.generate_agent_paths()

    # send paths to agents
    future = gcs.send_paths()
    # # Sends goal and waits until it’s completed
    # future = gcs.send_goal(10)

    rclpy.spin(gcs, future)

if __name__ == '__main__':
    main()