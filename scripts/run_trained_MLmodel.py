#!/usr/bin/env python3

from sklearn.preprocessing import StandardScaler, MinMaxScaler

import rospy
import numpy
import joblib
import tensorflow as tf

from std_msgs.msg import Int64MultiArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# ----- parameters ----------------
DELTA = 3 # time step used to compute the differencial pressure in the trained model
INPUT_DIM = 50 # number of features used in the trained model
# ----- ---------- ----------------

class ModelInferenceNode:
    """
    This Class aims at infering the joystick values from the input data.

    Create a subscriber for the topics: /pressuremat_array, /backpressuremat_array and /whill/odom (for the pressure mats on the seat and backrest and the odometry)
    Define the callback functions
    Create the publisher: the infer the velocity commmand
    """
    def __init__(self):
        """
        Initializes the ROS node, loads the TensorFlow model, and sets up the subscribers and publishers.
        """
        # Initialize the ROS node
        rospy.init_node('model_inference_node')
        rospy.loginfo("Setting up Pressure Inference ROS Node...")

        # Load TensorFlow model
        model_path = rospy.get_param('~model_checkpoint_path', 'Delta%s_FeaturesNumber%s.keras'%(DELTA, INPUT_DIM))#
        self.model = tf.keras.models.load_model(model_path)
        rospy.loginfo("Model loaded from {}".format(model_path))

        # Set up publisher for cmd_vel
        output_topic = rospy.get_param('~output_topic', '/pressuremat_cmd_vel_acc')
        self.publisher = rospy.Publisher(output_topic, Twist, queue_size=10)
        rospy.loginfo("Publishing cmd_vel commands to {}".format(output_topic))
        rospy.loginfo("Model inference node initialized successfully.")

        self.input_dim = INPUT_DIM
        self.last_velocity = numpy.zeros(2)
        self.last_bottom_pressure = numpy.zeros(12)
        self.last_back_pressure = numpy.zeros(12)
        self.bottom_pressure_diff = numpy.zeros(12)
        self.back_pressure_diff = numpy.zeros(12)
        self.bottom_pressure_hist = numpy.zeros((10, 12))
        self.back_pressure_hist = numpy.zeros((10, 12))

        self.subscription = rospy.Subscriber(
            '/pressuremat_array',
            Int64MultiArray,
            self.bottompressure_callback)

        rospy.Subscriber(
            '/backpressuremat_array',
            Int64MultiArray,
            self.backpressure_callback)

        rospy.Subscriber(
            '/whill/odom',
            Odometry,
            self.odom_callback)

        self.timer = rospy.Timer(rospy.Duration(0.2), self.callback)

    def bottompressure_callback(self, msg):
        """
        Subscribes to the topic: /pressuremat_array, 'pressure data from the seat pressure mat'
        It computes the differential pressure and updates the pressure values at each of the 12 sensors
        """
        self.last_bottom_pressure = numpy.array(msg.data)
        self.bottom_pressure_hist = numpy.roll(self.bottom_pressure_hist, -1)
        self.bottom_pressure_hist[-1] = self.last_bottom_pressure
        self.bottom_pressure_diff = self.last_bottom_pressure - self.bottom_pressure_hist[-DELTA]

    def backpressure_callback(self, msg):
        """
        Subscribes to the topic: /backpressuremat_array, 'pressure data from the backrest pressure mat'
        It computes the differential pressure and updates the pressure values at each of the 12 sensors
        """
        self.last_back_pressure = numpy.array(msg.data)
        self.back_pressure_hist = numpy.roll(self.back_pressure_hist, -1)
        self.back_pressure_hist[-1] = self.last_back_pressure
        self.back_pressure_diff = self.last_back_pressure - self.back_pressure_hist[-DELTA]

    def odom_callback(self, msg):
        """
        Subscribes to the topic: /whill/odom, 'wheelchair odometry (angular and linear)'
        It updates the velocity values
        """
        self.last_velocity[0] = msg.twist.twist.linear.x
        self.last_velocity[1] = msg.twist.twist.angular.z


    def callback(self, event):
        """
        Callback function: gets executed upon receiving a message on the subscribed topic.
        It scales the data, performs model inference and publishes the result. For safety reasons, if the seat pressure mat is empty, the wheelchair doesn't move (even if the pressure sensors measure non zero values).
        Note that the model infers the joystick values. At this stage no velocity mapping is done. So the velocity values are currently restricted to be in the range [-1, 1] m/s.
        """
        # Prepare data for model inference
        # this is an example in the case where the set of input features is (columns_backbottompressure + columns_backbottompressure_diff + columns_odom). It can be easily modified for a different set. In this case both the variables input_data and mscaler need to be modified. 
        input_data = numpy.concatenate((self.last_bottom_pressure, self.last_back_pressure, self.bottom_pressure_diff, self.back_pressure_diff, self.last_velocity), axis=0)
        
        x = numpy.array(input_data).reshape(-1, self.input_dim)

        myscaler = numpy.concatenate((numpy.full(len(self.last_bottom_pressure), 255), numpy.full(len(self.last_back_pressure), 255),
                                      numpy.full(len(self.bottom_pressure_diff), 255), numpy.full(len(self.back_pressure_diff), 255),
                                      numpy.full(len(self.last_velocity), 1.3)), axis=0)
        x = x/myscaler

        # Perform inference
        predicted_velocities = self.model.predict(x)[0]

        rospy.loginfo("Predicted joystick: {}".format(predicted_velocities))

        # Prepare and publish the Twist message
        twist = Twist()
        if (sum(self.last_bottom_pressure))> 100:
          twist.angular.z = predicted_velocities[0]
          twist.linear.x = predicted_velocities[1] 
          self.publisher.publish(twist)  

        else:  
          print("nobody's here!")
          twist.angular.z = 0.
          twist.linear.x = 0.
          self.publisher.publish(twist)

        rospy.loginfo("Published linear and angular velocities: {}.".format(twist))

if __name__ == '__main__':
    try:
        node = ModelInferenceNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
