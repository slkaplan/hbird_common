
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

from hbird_msgs.msg import Waypoint, State


class AgentControlNode(Node):
    
    def __init__(self):
        super().__init__('agent_control_node')

        self.get_logger().info('Starting up the Agent Control Node...')

        # TODO: Add the parameter getter and the parameter declaration
        param_descriptor = ParameterDescriptor(description='Defines agent ID.')
        self.declare_parameter('agent_id', 'HB1', param_descriptor)
        self._agent_id = self.get_parameter('agent_id')._value
        
        # define ROS2 topics
        vehicle_state_topic = '/'+self._agent_id+'/agent_state'
        pos_setpoint_topic = '/'+self._agent_id+'/position_setpoint'

        # initialize subscriber and publisher
        self._state_subscriber = self.create_subscription(State, 
                                                          vehicle_state_topic,
                                                          self.state_update_callback, 10)

        self._pos_setpoint_publisher = self.create_publisher(Waypoint, 
                                                             pos_setpoint_topic, 10)
        
        # initialize timer
        self._publish_rate = 0.5  # sec/cycle
        self._publish_timer = self.create_timer(self._publish_rate, self.control_cycle)

        # define stage
        self.stage = "ground"

        # set desired position setpoints
        self.x_des = 1.0
        self.y_des = 1.5
        self.z_des = 3.0
        self.psi_des = 6.283


    def state_update_callback(self, state_msg):
        self._state = state_msg    # TODO: This is in ROS message format


    def control_cycle(self):

        pos_setpoint = Waypoint()

        # your code here

        
        # publish the setpoint
        self._pos_setpoint_publisher.publish(pos_setpoint)

    



def main(args=None):
    rclpy.init(args=args)

    agent_control = AgentControlNode()

    rclpy.spin(agent_control)

    # Destroy the node explicitly
    agent_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()