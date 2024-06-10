import rclpy
from rclpy.node import Node
from ros_healthcare_msg.msg import Pressure
from geometry_msgs.msg import Twist
from std_msgs.msg import Int64MultiArray, Float64MultiArray, Float64
import numpy as np


POSITIONWEIGHT_X =  [-127.5,-127.5,-127.5, -67.5, -47.5, -47.5, 47.5, 47.5, 67.5, 127.5, 127.5, 127.5]
POSITIONWEIGHT_Y = [-120,-120,-120,-120,-25,-25, 25, 25, 85, 85, 135, 135]
SENSORS_PROJECTION_X = [0, 2, 4, 1, 3, 5, 6, 8, 10, 11, 7, 9]
SENSORS_PROJECTION_Y = [4, 5, 6, 7, 3, 8, 2, 9, 1, 10, 0, 11]
 
# ----- parameters ----------------
THRESHOLDX = 0.4 # for the COP
THRESHOLDY = 0.2 # for the COP
KAPPA_W = 1 # for the ang velocity, proportional part
KAPPA_V = 1 # for the lin velocity, proportional part
KAPPA_W_D = 0.1  # for the ang velocity, differential part
KAPPA_V_D = 0.1  # for the lin velocity, differential part
THRESHOLD_ACC_LIN_FW = 0.05 # acceleration limit for the fw linear velocity
# ----- ---------- ----------------



class PressureSubscriber(Node):
    """
    This Class aims at computing the COP velocity commands from the pressure data.

    Create a subscriber for the topics: pressure sensors;
    Define the callback functions
    Create the publisher: the computed command velocity, the raw pressure data and the computed COP values
    """
    def __init__(self):
        super().__init__('subscriber_pressure_accumulating')
        self.subscription = self.create_subscription(
            Pressure,
            '/pressure1',
           self.listener_callback,
           10)



        self.vx = 0.
        self.w = 0.
        self.w_prev = 0.
        self.v_prev = 0.
        self.a0 = np.zeros(shape=(100,12)) # for calibration, to accumulate over 10 secondes at 10Hz
        self.a0_cal = np.zeros(shape=(100, 12))  # only defined for calibration
        self.COPx_arr = []
        self.COPy_arr = []
        self.p0_last = np.array([])
        self.pmax0 = np.array([])
        self.cmd_vel = Twist()
        self.pressure_values = Int64MultiArray()
        self.COP_values = Float64MultiArray()
        self.publisher_ = self.create_publisher(Twist, 'pressuremat_cmd_vel_acc', 10)
        self.publisher_array = self.create_publisher(Int64MultiArray, 'pressuremat_array', 10)
        self.publisher_COParray = self.create_publisher(Float64MultiArray, 'COP_array', 10)


    def listener_callback(self, msg):
        """
        Callback function: called at the frequency rate of the pressure data (10Hz)
        Returns: publish the processed data: the computed command velocity, the raw pressure data and the computed COP values
        """
        # Do the calibration
        if (sum(self.a0_cal[0,:]) == 0):
            self.pmax0, self.p0_last = self.calibration_neutral(msg)
        self.a0_cal = np.roll(self.a0_cal,-1, axis=0)
        self.a0_cal[-1] = msg.pressure

        # Store the pressure values
        val = []
        for i in range(len(msg.pressure)):
            val.append(msg.pressure[i])
        self.pressure_values.data = val
        self.get_logger().info('Received pressure readings: "%s"' % msg.pressure)


        # COP computation
        calibrationweight = max(self.pmax0)/self.pmax0
        COPx, COPy = self.get_COP(val, calibrationweight)
        COP = [COPx, COPy]
        self.COP_values.data = COP

        # Velocity mapping
        self.w = 0.
        self.vx = 0.
        if not (sum(self.a0_cal[0, :]) == 0):
            # FINISHED CALIBRATING
            self.compute_velocities(COPx, COPy)

        
        # Save and Publish the topics
        self.cmd_vel.linear.x = self.vx
        self.cmd_vel.linear.y = 0.
        self.cmd_vel.linear.z = 0.

        self.cmd_vel.angular.x = 0.
        self.cmd_vel.angular.y = 0.
        self.cmd_vel.angular.z = self.w
        
          
        self.publisher_.publish(self.cmd_vel)
        self.publisher_array.publish(self.pressure_values)
        self.publisher_COParray.publish(self.COP_values)


    def calibration_neutral(self, msg):
        """
        Contains the calibration step required for the use of the COP algorithm as end-to-end posture control.
        Returns the pressure values that will be used to calibrate the COP values.
        """
        # pressure
        self.a0 = np.roll(self.a0, -1, axis=0)
        self.a0[-1] = msg.pressure

        p0_last = np.array([])
        if sum(self.a0[int(self.a0.shape[0]/5),:]) != 0:
            print("Stay neutral and lean back-> this will be your stop position")
        else:
            print("move it in the way you want to control the wheelchair")

        # p_last stores the last values of measured pressure (accumulated over 10% of the calibration time)
        # used for the neutral calibration
        for i in range(len(msg.pressure)):
            p0_last = np.append(p0_last, sum(self.a0[-int(self.a0.shape[0]/10):,i]))

        # get the maximal value reached by each of the sensors averaged over 10% of the calibration time
        pmax0 = np.array([])
        for i in range(len(msg.pressure)):#
            pmax0 = np.append(pmax0, np.mean(self.a0[np.argsort(self.a0[:,i]),i][-int(self.a0.shape[0]/10):]))

        return pmax0, p0_last

    def f_COPscale(self, COP, COP_min, COP_max):
        if COP < 0:
            return max(-COP / COP_min , -1.)
        elif COP > 0:
            return min(COP / COP_max , 1.)
        else:
            return COP

    def get_COP(self, val, calibrationweight):
        """
        Returns the COP values from the given pressure data and the calibration weights
        """

        calibrationweight_x = [calibrationweight[i] for i in SENSORS_PROJECTION_X]
        calibrationweight_y = [calibrationweight[i] for i in SENSORS_PROJECTION_Y]

        projected_x = [val[i] for i in SENSORS_PROJECTION_X]
        projected_y = [val[i] for i in SENSORS_PROJECTION_Y]

        projected_x0 = [self.p0_last[i] for i in SENSORS_PROJECTION_X]
        projected_y0 = [self.p0_last[i] for i in SENSORS_PROJECTION_Y]

        COPx = 0
        COPy = 0
        COPx0 = 0
        COPy0 = 0
        for i in range(12):
            COPx = COPx + projected_x[i] * POSITIONWEIGHT_X[i] * calibrationweight_x[i]
            COPy = COPy + projected_y[i] * POSITIONWEIGHT_Y[i] * calibrationweight_y[i]
            COPx0 = COPx0 + projected_x0[i] * POSITIONWEIGHT_X[i] * calibrationweight_x[i]
            COPy0 = COPy0 + projected_y0[i] * POSITIONWEIGHT_Y[i] * calibrationweight_y[i]

        COPx = COPx / (sum(val) * max(POSITIONWEIGHT_X))
        COPy = COPy / (sum(val) * max(POSITIONWEIGHT_Y))
        COPx0 = COPx0 / (sum(self.p0_last) * max(POSITIONWEIGHT_X))
        COPy0 = COPy0 / (sum(self.p0_last) * max(POSITIONWEIGHT_Y))
        COPx = COPx - COPx0
        COPy = COPy - COPy0

        # Normalise COP to be [-1,1] for both x and y
        if (sum(self.a0_cal[0, :]) == 0):
            self.COPx_arr.append(COPx)
            self.COPy_arr.append(COPy)

        COPx_min = np.nanmin(self.COPx_arr)
        COPx_max = np.nanmax(self.COPx_arr)
        COPy_min = np.nanmin(self.COPy_arr)
        COPy_max = np.nanmax(self.COPy_arr)

        # to ensure to get a COP smaller than 1: would be
        # the case if COP is larger than the COPmax value obtained during calibration
        COPx = self.f_COPscale(COPx, COPx_min, COPx_max)
        COPy = self.f_COPscale(COPy, COPy_min, COPy_max)

        return COPx, COPy

    def compute_velocities(self, COPx, COPy):
        """
        Returns the COP-velocity mapping.
        It relies on a proportional and derivative controller.
        It also set a limit for the forward acceleration.
        """
        if (sum(self.pressure_values.data))> 100:
         # ANGULAR VELOCITY
        	if (np.abs(COPx) > THRESHOLDX):
        		self.w = - KAPPA_W * (COPx - np.sign(COPx) * THRESHOLDX)
        		print("turning")

         # LINEAR VELOCITY
        	if COPy > THRESHOLDY:
        		self.vx = KAPPA_V * (COPy - THRESHOLDY)
        		print("Fw")
        	else:
        		self.vx = 0.
        		print("Stop")

        	Dv = KAPPA_V_D * (- self.vx + self.v_prev)
        	Dw = KAPPA_W_D * (- self.w + self.w_prev)

        	self.vx = self.vx + Dv
        	self.w = self.w + Dw

         #check if the new velocity is below the acceleration limit
        	if (self.vx - self.v_prev) > THRESHOLD_ACC_LIN_FW:
        		self.vx = self.v_prev + THRESHOLD_ACC_LIN_FW
        		print("limiting the fw lin acc")

        	self.w_prev = self.w
        	self.v_prev = self.vx
        else:
        	print("nobody's here!")
        	self.w = 0.
        	self.vx = 0.



def main(args=None):

    print("Init...")

    rclpy.init(args=args)

    subscriber_pressure = PressureSubscriber()

    rclpy.spin(subscriber_pressure)

    subscriber_pressure.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
