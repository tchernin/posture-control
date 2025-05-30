#!/usr/bin/env python3

import numpy as np
import tensorflow as tf
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ros2_hc_msgs.msg import Pressure
from pathlib import Path

DELTA = 3

class ModelInferenceNode(Node):
    def __init__(self):
        super().__init__('model_inference_node')
        self.get_logger().info("Setting up Pressure Inference ROS 2 Node...")

        model_path = Path(f'trained_models/Delta{DELTA}.keras')  # This is actually an h5 file
        assert model_path.exists()
        self.model = tf.keras.models.load_model(model_path, compile=False)
        self.get_logger().info(f"Model loaded from {model_path}")

        output_topic = self.declare_parameter('output_topic', '/cmd_vel').value
        self.publisher = self.create_publisher(Twist, output_topic, 10)

        self.last_velocity = np.zeros(2)
        self.last_bottom_pressure = np.zeros(12)
        self.last_back_pressure = np.zeros(12)
        self.bottom_pressure_diff = np.zeros(12)
        self.back_pressure_diff = np.zeros(12)
        self.bottom_pressure_hist = np.zeros((10, 12))
        self.back_pressure_hist = np.zeros((10, 12))

        self.create_subscription(Pressure, '/pressure1', self.bottompressure_callback, 10)
        self.create_subscription(Int64MultiArray, '/backpressuremat_array', self.backpressure_callback, 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        self.create_timer(0.2, self.timer_callback)
        self.get_logger().info("Model inference node initialized successfully.")

    def bottompressure_callback(self, msg):
        self.last_bottom_pressure = np.array(msg.pressure)
        self.bottom_pressure_hist = np.roll(self.bottom_pressure_hist, -1, axis=0)
        self.bottom_pressure_hist[-1] = self.last_bottom_pressure
        self.bottom_pressure_diff = self.last_bottom_pressure - self.bottom_pressure_hist[-DELTA]

    def backpressure_callback(self, msg):
        self.last_back_pressure = np.array(msg.data)
        self.back_pressure_hist = np.roll(self.back_pressure_hist, -1, axis=0)
        self.back_pressure_hist[-1] = self.last_back_pressure
        self.back_pressure_diff = self.last_back_pressure - self.back_pressure_hist[-DELTA]

    def odom_callback(self, msg):
        self.last_velocity[0] = msg.twist.twist.linear.x
        self.last_velocity[1] = msg.twist.twist.angular.z

    def timer_callback(self):
        input_data = np.concatenate([
            self.last_bottom_pressure, self.last_back_pressure,
            self.bottom_pressure_diff, self.back_pressure_diff,
            self.last_velocity
        ], axis=0)
        x = np.array(input_data).reshape(-1, self.input_dim)

        myscaler = np.concatenate([
            np.full(len(self.last_bottom_pressure), 255),
            np.full(len(self.last_back_pressure), 255),
            np.full(len(self.bottom_pressure_diff), 255),
            np.full(len(self.back_pressure_diff), 255),
            np.full(len(self.last_velocity), 1.3)
        ], axis=0)
        x = x / myscaler

        predicted_velocities = self.model.predict(x)[0]
        self.get_logger().info(f"Predicted joystick: {predicted_velocities}")

        twist = Twist()
        if sum(self.last_bottom_pressure) > 100:
            twist.angular.z = float(predicted_velocities[0])
            twist.linear.x = float(predicted_velocities[1])
        else:
            self.get_logger().info("Nobody's here!")
            twist.angular.z = 0.0
            twist.linear.x = 0.0

        self.publisher.publish(twist)
        self.get_logger().info(f"Published linear and angular velocities: {twist}")

def main(args=None):
    rclpy.init(args=args)
    node = ModelInferenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
