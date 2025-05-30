import argparse
from dataclasses import dataclass
from geometry_msgs.msg import Twist
import logging as log
import numpy as np
from pathlib import Path
import rclpy
from rclpy.node import Node
from ros2_hc_msgs.msg import Pressure
from std_msgs.msg import Int64MultiArray, Float64MultiArray
from typing import List, Optional
import yaml


@dataclass
class CopParams:
    POSITIONWEIGHT_X: List[float]
    POSITIONWEIGHT_Y: List[float]
    SENSORS_PROJECTION_X: List[float]
    SENSORS_PROJECTION_Y: List[float]

    THRESHOLDX: float
    THRESHOLDY: float
    KAPPA_W: float  # for the ang velocity, proportional part
    KAPPA_V: float  # for the lin velocity, proportional part
    KAPPA_W_D: float  # for the ang velocity, differential part
    KAPPA_V_D: float  # for the lin velocity, differential part
    THRESHOLD_ACC_LIN_FW: float  # acceleration limit for the fw linear velocity

    MAX_LIN_VEL_MPS: float
    MAX_ANG_VEL_RADPS: float
    THRESHOLD_MIN_SUM_PRESSURE: (
        float  # minimum summed pressure to consider someone is sitting on it
    )

    @staticmethod
    def from_yaml(filepath: str) -> "CopParams":
        assert Path(filepath).exists(), f"The file {filepath} does not exist."
        with open(filepath, "r") as f:
            data = yaml.safe_load(f)
            return CopParams(
                POSITIONWEIGHT_X=data["POSITIONWEIGHT_X"],
                POSITIONWEIGHT_Y=data["POSITIONWEIGHT_Y"],
                SENSORS_PROJECTION_X=data["SENSORS_PROJECTION_X"],
                SENSORS_PROJECTION_Y=data["SENSORS_PROJECTION_Y"],
                THRESHOLDX=data["THRESHOLDX"],
                THRESHOLDY=data["THRESHOLDY"],
                KAPPA_W=data["KAPPA_W"],
                KAPPA_V=data["KAPPA_V"],
                KAPPA_W_D=data["KAPPA_W_D"],
                KAPPA_V_D=data["KAPPA_V_D"],
                THRESHOLD_ACC_LIN_FW=data["THRESHOLD_ACC_LIN_FW"],
                MAX_LIN_VEL_MPS=data["MAX_LIN_VEL_MPS"],
                MAX_ANG_VEL_RADPS=data["MAX_ANG_VEL_RADPS"],
                THRESHOLD_MIN_SUM_PRESSURE=data["THRESHOLD_MIN_SUM_PRESSURE"],
            )


@dataclass
class VelocityCmd:
    vx_mps: float
    wz_radps: float


@dataclass
class CopCalibration:
    max_avg_pressure: np.ndarray
    baseline_pressure_sum: np.ndarray
    COPx_min: float
    COPy_min: float
    COPx_max: float
    COPy_max: float

    @staticmethod
    def from_yaml(filepath: str) -> Optional["CopCalibration"]:
        if filepath is None:
            return None
        assert Path(filepath).exists(), f"The file {filepath} does not exist."
        with open(filepath, "r") as f:
            data = yaml.safe_load(f)
        calib_data = data["calibration"]
        return CopCalibration(
            max_avg_pressure=np.array(calib_data["max_avg_pressure"]),
            baseline_pressure_sum=np.array(calib_data["baseline_pressure_sum"]),
            COPx_min=float(calib_data['COPx_min']),
            COPy_min=float(calib_data['COPy_min']),
            COPx_max=float(calib_data['COPx_max']),
            COPy_max=float(calib_data['COPy_max']),
        )


@dataclass
class CenterOfPressure:
    x: float
    y: float


class CopPostureControl(Node):
    """
    This Class aims at computing the COP velocity commands from the pressure data.

    Create a subscriber for the topics: pressure sensors;
    Define the callback functions
    Create the publisher: the computed command velocity, the raw pressure data and the computed COP values
    """

    def __init__(self, params: CopParams, calibration: Optional[CopCalibration] = None):
        super().__init__("pressure_based_control")

        self.params = params
        self.calibration = calibration
        self.w_prev = 0.0
        self.v_prev = 0.0
        self.pressure_buffer = np.zeros(
            shape=(100, 12)
        )  # for calibration, to accumulate over 10 secondes at 10Hz
        self.pressure_buffer_calibration = np.zeros(
            shape=(100, 12)
        )  # only defined for calibration
        self.COPx_calib: np.ndarray = []
        self.COPy_calib: np.ndarray = []

        self.pressure_subscriber = self.create_subscription(
            Pressure, "/pressure1", self.pressure_callback, 10
        )

        self.cmd_velocity_publisher = self.create_publisher(Twist, "cmd_vel", 10)
        self.pressure_array_publisher = self.create_publisher(
            Int64MultiArray, "pressuremat_array", 10
        )
        self.cop_array_publisher = self.create_publisher(
            Float64MultiArray, "COP_array", 10
        )

        log.debug(f"Input calibration {self.calibration}")

    def pressure_callback(self, msg: Pressure):
        """
        Called when new pressure data arrives (~10Hz).
        Processes pressure data, computes COP, velocity commands, and publishes them.
        """
        current_pressure = msg.pressure
        if (sum(current_pressure)) < self.params.THRESHOLD_MIN_SUM_PRESSURE:
            log.warning(
                f"Not enough pressure detected: {current_pressure}. Is someone sitting on the mat?"
            )
            return

        # Run calibration if none exists and buffer is empty
        if self.calibration is None:
            self.calibration_neutral(msg.pressure)
            return
        
        # Update rolling calibration buffer
        self.pressure_buffer_calibration = np.roll(
            self.pressure_buffer_calibration, -1, axis=0
        )
        self.pressure_buffer_calibration[-1] = current_pressure

        # Store raw pressure values for publishing
        self.pressure_values: Int64MultiArray = Int64MultiArray(
            data=list(current_pressure)
        )
        self.get_logger().info(f'Received pressure readings: "{current_pressure}"')

        # Compute Center of Pressure (COP)
        cop = self.get_scaled_center_of_pressure(
            input_pressure=self.pressure_values.data,
            calibration=self.calibration,
        )
        cop_values_msg = Float64MultiArray(data=[cop.x, cop.y])

        # Compute velocity commands only if calibration buffer is not empty
        velocity_command = self.compute_velocities(cop)

        # Prepare Twist message with velocity limits
        cmd_velocity_msg = Twist()
        cmd_velocity_msg.linear.x = min(
            velocity_command.vx_mps, self.params.MAX_LIN_VEL_MPS
        )
        cmd_velocity_msg.angular.z = (
            min(velocity_command.wz_radps, self.params.MAX_ANG_VEL_RADPS)
            if velocity_command.wz_radps > 0
            else max(velocity_command.wz_radps, -self.params.MAX_ANG_VEL_RADPS)
        )

        # Publish velocity, pressure, and COP data
        self.cmd_velocity_publisher.publish(cmd_velocity_msg)
        self.pressure_array_publisher.publish(self.pressure_values)
        self.cop_array_publisher.publish(cop_values_msg)

    def calibration_neutral(self, pressure: List[int]):
        """
        Calibrates the pressure sensor data for Center of Pressure (COP) control.
        Records baseline and maximum pressure values used for interpreting user posture.
        """

        # Store incoming pressure data in a rolling buffer
        self.pressure_buffer = np.roll(self.pressure_buffer, -1, axis=0)
        self.pressure_buffer[-1] = pressure

        # User guidance based on pressure early in the buffer
        if sum(self.pressure_buffer[int(self.pressure_buffer.shape[0] / 5), :]) != 0:
            log.warning("Stay neutral and lean back -> this will be your stop position")
        else:
            log.warning("Move in the way you want to control the wheelchair")

        # Calculate baseline pressure over the most recent 10% of buffer
        baseline_pressure_sum = np.array(
            [
                sum(self.pressure_buffer[-int(self.pressure_buffer.shape[0] / 10) :, i])
                for i in range(len(pressure))
            ]
        )

        # Calculate max average pressure values (top 10% readings per sensor)
        max_avg_pressure = np.array(
            [
                np.mean(
                    self.pressure_buffer[np.argsort(self.pressure_buffer[:, i]), i][
                        -int(self.pressure_buffer.shape[0] / 10) :
                    ]
                )
                for i in range(len(pressure))
            ]
        )
        cop = self.get_center_of_pressure(
            pressure=pressure,
            max_avg_pressure=max_avg_pressure,
            baseline_pressure_sum=baseline_pressure_sum,
        )
        self.COPx_calib.append(cop.x)
        self.COPy_calib.append(cop.y)

        if np.any(
            self.pressure_buffer[0] != 0.0
        ):  # Check that the calib buffer is full
            self.calibration = CopCalibration(
                max_avg_pressure=max_avg_pressure,
                baseline_pressure_sum=baseline_pressure_sum,
                COPx_min = np.nanmin(self.COPx_calib),
                COPx_max = np.nanmax(self.COPx_calib),
                COPy_min = np.nanmin(self.COPy_calib),
                COPy_max = np.nanmax(self.COPy_calib),
            )
            log.info(f"Calibrated: {self.calibration}")

    def scale_value(
        self, input_value: float, min_value: float, max_value: float
    ) -> float:
        scaled_value = (
            max(-input_value / min_value, -1.0)
            if input_value < 0
            else min(input_value / max_value, 1.0) if input_value > 0 else input_value
        )
        return scaled_value

    def get_center_of_pressure(
        self,
        pressure: List[int],
        max_avg_pressure: np.ndarray,
        baseline_pressure_sum: np.ndarray,
    ) -> CenterOfPressure:
        """
        Returns the COP values from the given pressure data and the calibration weights
        """
        # Calculate calibration weights (normalize by max pressure per sensor)
        epsilon = 1e-6  # small value to avoid division by zero
        max_avg_pressure_safe = np.where(max_avg_pressure == 0, epsilon, max_avg_pressure)
        calibration_weight = max(max_avg_pressure) / max_avg_pressure_safe

        calibration_weight = np.nan_to_num(calibration_weight, nan=0.0)

        calibration_weight_x = [
            calibration_weight[i] for i in self.params.SENSORS_PROJECTION_X
        ]
        calibration_weight_y = [
            calibration_weight[i] for i in self.params.SENSORS_PROJECTION_Y
        ]

        projected_x = [pressure[i] for i in self.params.SENSORS_PROJECTION_X]
        projected_y = [pressure[i] for i in self.params.SENSORS_PROJECTION_Y]

        projected_x0 = [
            baseline_pressure_sum[i] for i in self.params.SENSORS_PROJECTION_X
        ]
        projected_y0 = [
            baseline_pressure_sum[i] for i in self.params.SENSORS_PROJECTION_Y
        ]

        COPx = COPy = COPx0 = COPy0 = 0.0

        for i in range(12):
            COPx = (
                COPx
                + projected_x[i]
                * self.params.POSITIONWEIGHT_X[i]
                * calibration_weight_x[i]
            )
            COPy = (
                COPy
                + projected_y[i]
                * self.params.POSITIONWEIGHT_Y[i]
                * calibration_weight_y[i]
            )
            COPx0 = (
                COPx0
                + projected_x0[i]
                * self.params.POSITIONWEIGHT_X[i]
                * calibration_weight_x[i]
            )
            COPy0 = (
                COPy0
                + projected_y0[i]
                * self.params.POSITIONWEIGHT_Y[i]
                * calibration_weight_y[i]
            )

        COPx = COPx / (sum(pressure) * max(self.params.POSITIONWEIGHT_X))
        COPy = COPy / (sum(pressure) * max(self.params.POSITIONWEIGHT_Y))
        COPx0 = COPx0 / (sum(baseline_pressure_sum) * max(self.params.POSITIONWEIGHT_X))
        COPy0 = COPy0 / (sum(baseline_pressure_sum) * max(self.params.POSITIONWEIGHT_Y))
        COPx = COPx - COPx0
        COPy = COPy - COPy0

        return CenterOfPressure(x=COPx, y=COPy)

    def get_scaled_center_of_pressure(
        self, input_pressure: List[int], calibration: CopCalibration
    ) -> CenterOfPressure:
        cop = self.get_center_of_pressure(
            pressure=input_pressure,
            max_avg_pressure=calibration.max_avg_pressure,
            baseline_pressure_sum=calibration.baseline_pressure_sum,
        )

        # to ensure to get a COP smaller than 1: would be
        # the case if COP is larger than the COPmax value obtained during calibration
        COPx = self.scale_value(cop.x, self.calibration.COPx_min, self.calibration.COPx_max)
        COPy = self.scale_value(cop.y, self.calibration.COPy_min, self.calibration.COPy_max)

        return CenterOfPressure(x=COPx, y=COPy)

    def compute_velocities(self, center_of_pressure: CenterOfPressure) -> VelocityCmd:
        """
        Returns the COP-velocity mapping.
        It relies on a proportional and derivative controller.
        It also set a limit for the forward acceleration.
        """
        vx_mps: float = 0.0
        wz_radps: float = 0.0

        # ANGULAR VELOCITY
        if np.abs(center_of_pressure.x) > self.params.THRESHOLDX:
            wz_radps = -self.params.KAPPA_W * (
                center_of_pressure.x
                - np.sign(center_of_pressure.x) * self.params.THRESHOLDX
            )
            log.debug("Turning")

        # LINEAR VELOCITY
        if center_of_pressure.y > self.params.THRESHOLDY:
            vx_mps = self.params.KAPPA_V * (
                center_of_pressure.y - self.params.THRESHOLDY
            )
            log.debug("Moving Forward")
        else:
            log.debug("Stopping")

        Dv = self.params.KAPPA_V_D * (-vx_mps + self.v_prev)
        Dw = self.params.KAPPA_W_D * (-wz_radps + self.w_prev)

        vx_mps = vx_mps + Dv
        wz_radps = wz_radps + Dw

        # check if the new velocity is below the acceleration limit

        if (vx_mps - self.v_prev) > self.params.THRESHOLD_ACC_LIN_FW:
            vx_mps = self.v_prev + self.params.THRESHOLD_ACC_LIN_FW
            log.info("Limiting the forward linear acceleration.")

        self.v_prev = vx_mps
        self.w_prev = wz_radps

        return VelocityCmd(vx_mps=vx_mps, wz_radps=wz_radps)


def main():

    parser = argparse.ArgumentParser(
        description="Pressure-based control calibration loader"
    )
    parser.add_argument(
        "--params-file",
        type=str,
        required=True,
        help="Path to the YAML file containing COP parameters",
    )
    parser.add_argument(
        "--calibration-file",
        type=str,
        required=False,
        help="Path to the YAML file containing COP calibration data",
    )

    parsed_args = parser.parse_args()

    log.info(f"Using params file path: {parsed_args.params_file}")

    calibration_file = parsed_args.calibration_file
    if calibration_file:
        log.info(f"Using calibration file path: {parsed_args.calibration_file}")

    rclpy.init()

    cop_pressure_control = CopPostureControl(
        params=CopParams.from_yaml(parsed_args.params_file),
        calibration=CopCalibration.from_yaml(calibration_file),
    )

    rclpy.spin(cop_pressure_control)

    cop_pressure_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
