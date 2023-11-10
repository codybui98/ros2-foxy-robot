#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified by AutomaticAddison.com
 
import time  # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2
import serial
from rclpy.node import Node
from robot_navigator import BasicNavigator, NavigationResult # Helper module

class ledControlNode(Node):
  def __init__(self):
    super().__init__('led_control_node')
    navigator = BasicNavigator()
    self.result = navigator.getResult()
    self.declare_parameter('led_port', value="/dev/ttyACM0")
    self.port = self.get_parameter('led_port').get_parameter_value().string_value
    self.ser = serial.Serial(self.port, 
                             baudrate = 9600, 
                             bytesize = serial.EIGHTBITS,
                             parity = serial.PARITY_NONE,
                             stopbits = serial.STOPBITS_ONE,
                             timeout=0.1,
                             rtscts=False)
    if self.ser:
        self.get_logger().info(f'LED Connected')    
    self.get_logger().info(f'Using serial port {self.ser.name}')
    self.ser.flushInput()
    self.ser.flushOutput()
    # time.sleep(2)
    navigator.waitUntilNav2Active()
    # set timer
    # while navigator.isNavComplete():
    self.pub_period = 2 #0.024
    self.pub_timer = self.create_timer(self.pub_period, self.pub_callback)

    # if self.result == NavigationResult.SUCCEEDED:
    #   self.get_logger().info('Goal succeeded!')
    # elif self.result == NavigationResult.CANCELED:
    #   self.get_logger().info('Goal was canceled!')
    # elif self.result == NavigationResult.FAILED:
    #   self.get_logger().info('Goal failed!')
    # else:
    #   self.get_logger().info('Goal has an invalid return status!')

    # self.pub_period = 0.03 #0.024
    # self.pub_timer = self.create_timer(self.pub_period, self.pub_callback)

  def pub_callback (self):
      if self.result == NavigationResult.SUCCEEDED:
        self.get_logger().info('Goal succeeded!')
        self.x = 'p'
        # self.send_command(self.x)
      elif self.result == NavigationResult.FAILED:
        self.get_logger().info('Goal was failed!')
        self.x = 'w'
        # self.send_command(self.x)
      # elif self.result == NavigationResult.EXECUTING:
      #   self.get_logger().info('Goal executing!')
      #   self.x = 'p'
      #   self.send_command(self.x)
      else:
        self.x = 'p'
        self.get_logger().info('Goal has an invalid return status!')
      self.send_command(self.x)

  def send_command(self, x: str):
      self.get_logger().debug(f'Data to send: {x}')
      command = f'{x}'.encode('UTF-8')
      self.get_logger().info(f'Sending command: "{command}"')
      self.ser.write(command)

def main(args=None):
 
  # Start the ROS 2 Python Client Library
  rclpy.init(args=args)
 
  # Launch the ROS 2 Navigation Stack
  # navigator = BasicNavigator()

#   navigator.waitUntilNav2Active()
 
  # Do something depending on the return code
  # result = navigator.getResult()
  # while True:
  #   if result == NavigationResult.SUCCEEDED:
  #       print('Goal succeeded!')
  #   elif result == NavigationResult.CANCELED:
  #       print('Goal was canceled!')
  #   elif result == NavigationResult.FAILED:
  #       print('Goal failed!')
  #   else:
  #       print('Goal has an invalid return status!')
 
  # Shut down the ROS 2 Navigation Stack
#   navigator.lifecycleShutdown()
 
  # exit(0)
  led_control_node = ledControlNode()
  while rclpy.ok():
      rclpy.spin_once(led_control_node)
      
  led_control_node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()