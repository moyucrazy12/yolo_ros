#!/usr/bin/env python3

from enum import Enum

import rclpy
from rclpy.node import Node, Optional
from rclpy.executors import Future, MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from yolo_msgs.srv import SetClasses
from rcl_interfaces.srv import SetParameters
from rclpy.parameter import Parameter

from smarc_action_base.gentler_action_server import GentlerActionServer


class YoloActionServer:
    def __init__(self, node: Node):
        self._node = node

        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        self._node.declare_parameter('set_classes_serivce', "/yolo/set_classes")
        set_classes_serivce_name = self._node.get_parameter('set_classes_serivce').value

        self._node.declare_parameter('set_parameter_serivce', "/yolo/yolo_node/set_parameters")
        set_parameter_serivce_name = self._node.get_parameter('set_parameter_serivce').value

        #Service clients
        self.set_classes_client = self._node.create_client(SetClasses, set_classes_serivce_name, callback_group=self.service_callback_group)
        self.set_threshold_client = self._node.create_client(SetParameters,set_parameter_serivce_name, callback_group=self.service_callback_group)

        # Wait for service
        while not self.set_classes_client.wait_for_service(timeout_sec=1.0): #Does this waiting cause problems?
            self._node.get_logger().info('set classes service not available, waiting...')

        # Wait for service
        while not self.set_threshold_client.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info('set param service not available, waiting...')
        
        # The timer callback function will make the service call if the request if not None
        # (for thread reasons) 
        self.set_classes_request = None
        self.set_threshold_request = None
        
        #Futures for keeping track of service calls
        self.set_classes_future = None
        self.set_threshold_future = None


        self._classes_as = GentlerActionServer(
            self._node,
            "yolo_set_classes",
            self._on_goal_received_classes,
            lambda: True,
            lambda: None,
            lambda: True,
            lambda: "No feedback",
            loop_frequency = 1.0
        )

        self._classes_as = GentlerActionServer(
            self._node,
            "yolo_set_threshold",
            self._on_goal_received_threshold,
            lambda: True,
            lambda: None,
            lambda: True,
            lambda: "No feedback",
            loop_frequency = 1.0
        )

        self._tracking_as = GentlerActionServer(
            self._node,
            "yolo_set_tracking",
            lambda goal_request: self._node.get_logger().warn("tracking action not implemented yet") and False,
            lambda: True,
            lambda: None,
            lambda: True,
            lambda: "No feedback",
            loop_frequency = 1.0
        )

        timer = node.create_timer(1.0 , self.timer_cb, callback_group=self.timer_callback_group)

        self._node.get_logger().warn(f"YoloServer initialized.")

    #Callback server for printing the result of a service call
    def service_callback_response(self, future):
        try:
            response = future.result()
            self._node.get_logger().info(f'Result: {response}')
        except Exception as e:
            self._node.get_logger().error(f'Service call failed: {e}')



    def timer_cb(self):
        self._node.get_logger().warn(f"Timer callback")
        
        #Set classes 
        if(self.set_classes_request is not None):
            self._node.get_logger().info(f"Calling set classes service")
            #Check if we are currently trying to do a service call. Anc cancel it in that case
            if not (self.set_classes_future is None or self.set_classes_future.done):
                self._node.get_logger().error(f'Service call was not finnished before next call. Canceling service call')
                self.set_classes_future.cancel()

            # Make async call
            self.set_classes_future = self.set_classes_client.call_async(self.set_classes_request)

            # Attach callback
            self.set_classes_future.add_done_callback(self.service_callback_response)
            
            # Clear request so we donn't call the service next time
            self.set_classes_request = None

        #Set threshold 
        if(self.set_threshold_request is not None):
            self._node.get_logger().info(f"Calling threshold param service")

            #Check if we are currently trying to do a service call. Anc cancel it in that case
            if not (self.set_classes_future is None or self.set_classes_future.done):
                self._node.get_logger().error(f'Service call was not finnished before next call. Canceling service call')
                self.set_classes_future.cancel()
            
            # Make async call
            self.set_threshold_future = self.set_threshold_client.call_async(self.set_threshold_request)

            # Attach callback
            self.set_threshold_future.add_done_callback(self.service_callback_response)
            
            # Clear request so we donn't call the service next time
            self.set_threshold_request = None
           

    def _on_goal_received_classes(self, goal_request: dict) -> bool:
        """
        # classes = ['person', 'car', 'etc]
        """
        self._node.get_logger().info(f"Received new classes to track: {goal_request}")
        try:            
            self.set_classes_request = SetClasses.Request()
            self.set_classes_request.classes = goal_request['classes']
            return True
        except KeyError as e:
            self._node.get_logger().info("Missing key in goal request")
            return False
        except ValueError as e:
            self._node.get_logger().info("Invalid value in goal request")
            return False
        
    def _on_goal_received_threshold(self, goal_request: dict) -> bool:
        """
        # threshold = 0.5
        """
        self._node.get_logger().info(f"Received new threshold parameter: {goal_request}")
        try:
            param = Parameter( 'threshold', Parameter.Type.DOUBLE, float(goal_request['threshold']))
            self.set_threshold_request = SetParameters.Request()
            self.set_threshold_request.parameters = [param.to_parameter_msg()]
            return True
        except KeyError as e:
            self._node.get_logger().info("Missing key in goal request")
            return False
        except ValueError as e:
            self._node.get_logger().info("Invalid value in goal request")
            return False
        
    def _on_goal_received_tracking(self, goal_request: dict) -> bool:
        """
        # tracking = 'AUTO' / id
        """
        self._node.get_logger().info(f"Received new tracking parameters: {goal_request}")
        try:
            #TODO service call?
            return True
        except KeyError as e:
            self._node.get_logger().info("Missing key in goal request")
            return False
        except ValueError as e:
            self._node.get_logger().info("Invalid value in goal request")
            return False

def main():
    rclpy.init()
    
    node = Node("yolo_action_server_node")
    action_server = YoloActionServer(node)
    
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()