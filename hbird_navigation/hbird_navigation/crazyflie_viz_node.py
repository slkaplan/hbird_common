import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import Waypoint, State
from geometry_msgs.msg import Twist

from .scripts.utils import State as StatePy



class CrazyflieVizNode(Node):

    def __init__(self):
        super().__init__('crazyflie_viz_node')
        
        # subscribers:
        self._control_subscriber = self.create_subscription(Twist, '/cmd_vel',
				            self.control_callback, 10)
        self._control_subscriber  # prevent unused variable warning

        # publishers:
        self._state_publisher = self.create_publisher(State, '/agent_state', 10)
        # self._status_publisher = self.create_publisher(MessageType, '/agent_status', 10)

        # timer:
        self._publish_rate = 0.5  # cycle/sec
        self._publish_timer = self.create_timer(self._publish_rate, self.timer_callback)

        # attributes:
        self._state = State()
        # self._status = ...


    def control_callback(self, cmd_msg):
		# advance the state using a simple kinematic model
        self.update_state(cmd_msg)
        #self.update_status(...)


    def update_state(self, cmd):
        # advance state using vel_cmd
        self._state.position.x += cmd.linear.x * self._time_delta
        self._state.position.y += cmd.linear.y * self._time_delta
        self._state.position.z += cmd.linear.z * self._time_delta
        self._state.orientation.z += cmd.angular.z * self._time_delta # TODO: In radians??


    def update_status(self, ):
        pass


    def timer_callback(self, ):
		# regulates publishing of agent state and status
		
		# arrange agent state message
        state_msg = Twist()
        state_msg = self._state
        # ...
        self._state_publisher.publish(state_msg)
        # self._status_publisher.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)

    cf_viz = CrazyflieVizNode()

    rclpy.spin(cf_viz)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    cf_viz.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()