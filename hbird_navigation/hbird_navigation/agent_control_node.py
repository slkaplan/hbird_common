import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor

import pathlib
import yaml

import math
from hbird_msgs.msg import Waypoint, State
from time import perf_counter
from enum import Enum, auto

from .scripts.path_planner import (
    PathPlanner,
)
from .scripts.planner_utils import *


class MovementState(Enum):
    GROUND = auto()
    TAKEOFF = auto()
    FOLLOW_PATH = auto()
    LAND = auto()
    END = auto()


class AgentControlNode(Node):
    def __init__(self):
        super().__init__("agent_control_node")

        self.get_logger().info("Starting up the Agent Control Node...")

        # TODO: Add the parameter getter and the parameter declaration
        param_descriptor = ParameterDescriptor(description="Defines agent ID.")
        self.declare_parameter("agent_id", "HB1", param_descriptor)
        self._agent_id = self.get_parameter("agent_id")._value

        # define ROS2 topics
        vehicle_state_topic = "/" + self._agent_id + "/agent_state"
        pos_setpoint_topic = "/" + self._agent_id + "/position_setpoint"

        # initialize subscriber and publisher
        self._state_subscriber = self.create_subscription(
            State, vehicle_state_topic, self.state_update_callback, 10
        )

        self._pos_setpoint_publisher = self.create_publisher(
            Waypoint, pos_setpoint_topic, 10
        )

        # initialize timer
        self._publish_rate = 0.2  # sec/cycle
        self._publish_timer = self.create_timer(self._publish_rate, self.control_cycle)

        # get planner config file
        config_path = (
            str(pathlib.Path().parent.resolve())
            + "/src/hbird_common/hbird_navigation/hbird_navigation/config/"
        )
        config_file = "planner_config.yaml"
        try:
            with open(config_path + config_file, "r") as file:
                config = yaml.load(file, Loader=yaml.FullLoader)
        except FileNotFoundError:
            print(f"The file '{config_file}' does not exist.")
        except yaml.YAMLError as e:
            print(f"Error parsing the YAML file: {e}")

        env = Environment(config)
        path_planner = PathPlanner(env)
        self.path = path_planner.plan()
        self.current_target_index = 0
        self.target_height = 1.2
        # set thresholds
        self.pos_threshold = 0.1
        self.orient_threshold = 0.05
        self.state_time = perf_counter()

        self.stage = MovementState.GROUND

        self.start_x = env.start_pose.position.x - 4.5
        self.start_y = env.start_pose.position.y - 5.0
        self.goal_x = env.goal_pose.position.x - 4.5
        self.goal_y = env.goal_pose.position.y - 5.0

    def angle_wrap(self, angle : float):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def state_update_callback(self, state_msg):
        self._state = state_msg

    def reached_pos(self, target_pos) -> bool:
        pos_error = math.sqrt(
            (self._state.position.x - target_pos.position.x) ** 2
            + (self._state.position.y - target_pos.position.y) ** 2
            + (self._state.position.z - target_pos.position.z) ** 2
        )

        orient_error = abs(self.angle_wrap(self._state.orientation.z - target_pos.heading))

        return pos_error < self.pos_threshold and orient_error < self.orient_threshold


    def control_cycle(self):
        pos_setpoint = Waypoint()

        self.get_logger().info(
            f"State: {self.stage.name}, Time {(perf_counter() - self.state_time):.2f}, {self.current_target_index}"
        )

        match self.stage:
            case MovementState.GROUND:
                pos_setpoint.position.x = self.start_x
                pos_setpoint.position.y = self.start_y
                pos_setpoint.position.z = 0.0
                pos_setpoint.heading = 0.0
                if perf_counter() - self.state_time > 10:
                    self.state_time = perf_counter()
                    self.stage = MovementState.TAKEOFF
            case MovementState.TAKEOFF:
                pos_setpoint.position.x = self.start_x
                pos_setpoint.position.y = self.start_y
                pos_setpoint.position.z = self.target_height
                pos_setpoint.heading = 0.0
                if self.reached_pos(pos_setpoint):
                    self.state_time = perf_counter()
                    self.stage = MovementState.FOLLOW_PATH
            case MovementState.FOLLOW_PATH:
                pos_setpoint = self.map_to_webots_transform(
                    self.path[self.current_target_index]
                )
                pos_setpoint.position.z = self.target_height
                pos_setpoint.heading = 0.0
                self.get_logger().info(f"Cur Pos {self._state.position.x:.2f},{self._state.position.y:.2f}, Target Pos {pos_setpoint.position.x:.2f},{pos_setpoint.position.y:.2f}, Path pos {self.path[self.current_target_index].position.x:.2f}, {self.path[self.current_target_index].position.y:.2f}")
                if self.reached_pos(pos_setpoint):
                    self.current_target_index += 1
                    if self.current_target_index == len(self.path):
                        self.state_time = perf_counter()
                        self.stage = MovementState.LAND
            case MovementState.LAND:
                pos_setpoint.position.x = self.goal_x
                pos_setpoint.position.y = self.goal_y
                pos_setpoint.position.z = 0.0
                pos_setpoint.heading = 0.0
                if perf_counter() - self.state_time > 5:
                    self.stage = MovementState.END
            case MovementState.END:
                while True:
                    pass

        # publish the setpoint
        self._pos_setpoint_publisher.publish(pos_setpoint)

    def map_to_webots_transform(self, waypoint):
        """Helper function that returns a Waypoint adjusted to the webots reference frame"""
        webots_wp = Waypoint()
        webots_wp.position.x = waypoint.position.x - 4.5
        webots_wp.position.y = waypoint.position.y - 5.0
        return webots_wp


def main(args=None):
    rclpy.init(args=args)

    agent_control = AgentControlNode()

    rclpy.spin(agent_control)

    # Destroy the node explicitly
    agent_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
