"""
IMU Data Processor
Handles CSV file loading, serial streaming, and data processing for dual IMU system.
"""

import pandas as pd
import numpy as np
import serial
import serial.tools.list_ports
from collections import deque
from typing import Optional, Tuple, Dict
from PyQt5.QtCore import QThread, pyqtSignal
import time
import threading
from scipy import signal


class IMUData:
    """Container for IMU data with statistics."""

    def __init__(self, max_samples=10000):
        """
        Initialize IMU data container.

        Args:
            max_samples: Maximum number of samples to keep in memory
        """
        self.max_samples = max_samples

        # Thread synchronization lock to prevent race conditions between
        # serial reader thread (writing) and GUI thread (reading)
        self.data_lock = threading.Lock()

        # Track which IMUs have provided data
        self.lsm6dsv_available = False
        self.lsm6dsox_available = False
        self.lis2dw12_available = False
        self.adxl362_available = False
        self.lsm6dsv_sflp_available = False
        self.lsm6dsox_fusion_available = False

        # Separate timestamps for each IMU (they may sample at different rates)
        self.lsm_timestamp = deque(maxlen=max_samples)
        self.lsmox_timestamp = deque(maxlen=max_samples)
        self.lis_timestamp = deque(maxlen=max_samples)
        self.adxl_timestamp = deque(maxlen=max_samples)
        self.lsm_sflp_timestamp = deque(maxlen=max_samples)

        # LSM6DSV (6-axis) data
        self.lsm_accel_x = deque(maxlen=max_samples)
        self.lsm_accel_y = deque(maxlen=max_samples)
        self.lsm_accel_z = deque(maxlen=max_samples)
        self.lsm_gyro_x = deque(maxlen=max_samples)
        self.lsm_gyro_y = deque(maxlen=max_samples)
        self.lsm_gyro_z = deque(maxlen=max_samples)

        # LSM6DSOX (6-axis) data
        self.lsmox_accel_x = deque(maxlen=max_samples)
        self.lsmox_accel_y = deque(maxlen=max_samples)
        self.lsmox_accel_z = deque(maxlen=max_samples)
        self.lsmox_gyro_x = deque(maxlen=max_samples)
        self.lsmox_gyro_y = deque(maxlen=max_samples)
        self.lsmox_gyro_z = deque(maxlen=max_samples)

        # LIS2DW12 (3-axis) data
        self.lis_accel_x = deque(maxlen=max_samples)
        self.lis_accel_y = deque(maxlen=max_samples)
        self.lis_accel_z = deque(maxlen=max_samples)

        # ADXL362 (3-axis) data
        self.adxl_accel_x = deque(maxlen=max_samples)
        self.adxl_accel_y = deque(maxlen=max_samples)
        self.adxl_accel_z = deque(maxlen=max_samples)

        # LSM6DSV SFLP (Sensor Fusion) quaternion data
        self.lsm_quat_w = deque(maxlen=max_samples)
        self.lsm_quat_x = deque(maxlen=max_samples)
        self.lsm_quat_y = deque(maxlen=max_samples)
        self.lsm_quat_z = deque(maxlen=max_samples)

        # LSM6DSOX Fusion library data
        self.lsmox_fusion_timestamp = deque(maxlen=max_samples)
        # Quaternion
        self.lsmox_fusion_quat_w = deque(maxlen=max_samples)
        self.lsmox_fusion_quat_x = deque(maxlen=max_samples)
        self.lsmox_fusion_quat_y = deque(maxlen=max_samples)
        self.lsmox_fusion_quat_z = deque(maxlen=max_samples)
        # Euler angles
        self.lsmox_fusion_roll = deque(maxlen=max_samples)
        self.lsmox_fusion_pitch = deque(maxlen=max_samples)
        self.lsmox_fusion_yaw = deque(maxlen=max_samples)
        # Linear acceleration (gravity removed)
        self.lsmox_fusion_linacc_x = deque(maxlen=max_samples)
        self.lsmox_fusion_linacc_y = deque(maxlen=max_samples)
        self.lsmox_fusion_linacc_z = deque(maxlen=max_samples)
        # Earth frame acceleration
        self.lsmox_fusion_earthacc_x = deque(maxlen=max_samples)
        self.lsmox_fusion_earthacc_y = deque(maxlen=max_samples)
        self.lsmox_fusion_earthacc_z = deque(maxlen=max_samples)
        # Gravity vector
        self.lsmox_fusion_gravity_x = deque(maxlen=max_samples)
        self.lsmox_fusion_gravity_y = deque(maxlen=max_samples)
        self.lsmox_fusion_gravity_z = deque(maxlen=max_samples)

        self.sample_count = 0
        self.lsm_sample_count = 0
        self.lsmox_sample_count = 0
        self.lis_sample_count = 0
        self.adxl_sample_count = 0
        self.lsm_sflp_sample_count = 0
        self.lsmox_fusion_sample_count = 0
        self.start_time = None

        # Calibration data - stores bias for each sensor
        self.calibration = {
            'LSM6DSV': {'accel': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                       'gyro': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                       'calibrated': False},
            'LSM6DSOX': {'accel': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                        'gyro': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                        'calibrated': False},
            'LIS2DW12': {'accel': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                        'calibrated': False},
            'ADXL362': {'accel': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                       'calibrated': False}
        }

    def add_lsm6dsv_sample(self, timestamp, ax, ay, az, gx, gy, gz):
        """Add a new LSM6DSV data sample."""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = timestamp

            self.lsm6dsv_available = True
            self.lsm_timestamp.append(timestamp)
            self.lsm_accel_x.append(ax)
            self.lsm_accel_y.append(ay)
            self.lsm_accel_z.append(az)
            self.lsm_gyro_x.append(gx)
            self.lsm_gyro_y.append(gy)
            self.lsm_gyro_z.append(gz)

            self.lsm_sample_count += 1
            self.sample_count += 1

    def add_lsm6dsox_sample(self, timestamp, ax, ay, az, gx, gy, gz):
        """Add a new LSM6DSOX data sample."""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = timestamp

            self.lsm6dsox_available = True
            self.lsmox_timestamp.append(timestamp)
            self.lsmox_accel_x.append(ax)
            self.lsmox_accel_y.append(ay)
            self.lsmox_accel_z.append(az)
            self.lsmox_gyro_x.append(gx)
            self.lsmox_gyro_y.append(gy)
            self.lsmox_gyro_z.append(gz)

            self.lsmox_sample_count += 1
            self.sample_count += 1

    def add_lis2dw12_sample(self, timestamp, ax, ay, az):
        """Add a new LIS2DW12 data sample."""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = timestamp

            self.lis2dw12_available = True
            self.lis_timestamp.append(timestamp)
            self.lis_accel_x.append(ax)
            self.lis_accel_y.append(ay)
            self.lis_accel_z.append(az)

            self.lis_sample_count += 1
            self.sample_count += 1

    def add_adxl362_sample(self, timestamp, ax, ay, az):
        """Add a new ADXL362 data sample."""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = timestamp

            self.adxl362_available = True
            self.adxl_timestamp.append(timestamp)
            self.adxl_accel_x.append(ax)
            self.adxl_accel_y.append(ay)
            self.adxl_accel_z.append(az)

            self.adxl_sample_count += 1
            self.sample_count += 1

    def add_lsm6dsv_sflp_sample(self, timestamp, qw, qx, qy, qz):
        """Add a new LSM6DSV SFLP (sensor fusion) quaternion sample."""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = timestamp

            self.lsm6dsv_sflp_available = True
            self.lsm_sflp_timestamp.append(timestamp)
            self.lsm_quat_w.append(qw)
            self.lsm_quat_x.append(qx)
            self.lsm_quat_y.append(qy)
            self.lsm_quat_z.append(qz)

            self.lsm_sflp_sample_count += 1
            self.sample_count += 1

    def add_lsm6dsox_fusion_quat_sample(self, timestamp, qw, qx, qy, qz):
        """Add a new LSM6DSOX Fusion quaternion sample."""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = timestamp

            self.lsm6dsox_fusion_available = True
            self.lsmox_fusion_timestamp.append(timestamp)
            self.lsmox_fusion_quat_w.append(qw)
            self.lsmox_fusion_quat_x.append(qx)
            self.lsmox_fusion_quat_y.append(qy)
            self.lsmox_fusion_quat_z.append(qz)

            self.lsmox_fusion_sample_count += 1
            self.sample_count += 1

    def add_lsm6dsox_fusion_euler_sample(self, timestamp, roll, pitch, yaw):
        """Add a new LSM6DSOX Fusion Euler angles sample."""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = timestamp

            self.lsm6dsox_fusion_available = True
            # Reuse timestamp if already added
            if len(self.lsmox_fusion_timestamp) == 0 or self.lsmox_fusion_timestamp[-1] != timestamp:
                self.lsmox_fusion_timestamp.append(timestamp)

            self.lsmox_fusion_roll.append(roll)
            self.lsmox_fusion_pitch.append(pitch)
            self.lsmox_fusion_yaw.append(yaw)

            # Only increment if this is a new timestamp
            if len(self.lsmox_fusion_timestamp) > 0 and self.lsmox_fusion_timestamp[-1] == timestamp:
                pass  # Same timestamp as quaternion, don't double-count
            else:
                self.lsmox_fusion_sample_count += 1
                self.sample_count += 1

    def add_lsm6dsox_fusion_linacc_sample(self, timestamp, x, y, z):
        """Add a new LSM6DSOX Fusion linear acceleration sample."""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = timestamp

            self.lsm6dsox_fusion_available = True
            self.lsmox_fusion_linacc_x.append(x)
            self.lsmox_fusion_linacc_y.append(y)
            self.lsmox_fusion_linacc_z.append(z)

    def add_lsm6dsox_fusion_earthacc_sample(self, timestamp, x, y, z):
        """Add a new LSM6DSOX Fusion earth acceleration sample."""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = timestamp

            self.lsm6dsox_fusion_available = True
            self.lsmox_fusion_earthacc_x.append(x)
            self.lsmox_fusion_earthacc_y.append(y)
            self.lsmox_fusion_earthacc_z.append(z)

    def add_lsm6dsox_fusion_gravity_sample(self, timestamp, x, y, z):
        """Add a new LSM6DSOX Fusion gravity vector sample."""
        with self.data_lock:
            if self.start_time is None:
                self.start_time = timestamp

            self.lsm6dsox_fusion_available = True
            self.lsmox_fusion_gravity_x.append(x)
            self.lsmox_fusion_gravity_y.append(y)
            self.lsmox_fusion_gravity_z.append(z)

    def get_lsm_accel_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get LSM6DSV accelerometer data."""
        with self.data_lock:
            return (np.array(self.lsm_timestamp),
                    np.array(self.lsm_accel_x),
                    np.array(self.lsm_accel_y),
                    np.array(self.lsm_accel_z))

    def get_lsm_gyro_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get LSM6DSV gyroscope data."""
        with self.data_lock:
            return (np.array(self.lsm_timestamp),
                    np.array(self.lsm_gyro_x),
                    np.array(self.lsm_gyro_y),
                    np.array(self.lsm_gyro_z))

    def get_lsmox_accel_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get LSM6DSOX accelerometer data."""
        with self.data_lock:
            return (np.array(self.lsmox_timestamp),
                    np.array(self.lsmox_accel_x),
                    np.array(self.lsmox_accel_y),
                    np.array(self.lsmox_accel_z))

    def get_lsmox_gyro_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get LSM6DSOX gyroscope data."""
        with self.data_lock:
            return (np.array(self.lsmox_timestamp),
                    np.array(self.lsmox_gyro_x),
                    np.array(self.lsmox_gyro_y),
                    np.array(self.lsmox_gyro_z))

    def get_lis_accel_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get LIS2DW12 accelerometer data."""
        with self.data_lock:
            return (np.array(self.lis_timestamp),
                    np.array(self.lis_accel_x),
                    np.array(self.lis_accel_y),
                    np.array(self.lis_accel_z))

    def get_adxl362_accel_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get ADXL362 accelerometer data."""
        with self.data_lock:
            return (np.array(self.adxl_timestamp),
                    np.array(self.adxl_accel_x),
                    np.array(self.adxl_accel_y),
                    np.array(self.adxl_accel_z))

    def get_lsm_sflp_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get LSM6DSV SFLP quaternion data."""
        with self.data_lock:
            return (np.array(self.lsm_sflp_timestamp),
                    np.array(self.lsm_quat_w),
                    np.array(self.lsm_quat_x),
                    np.array(self.lsm_quat_y),
                    np.array(self.lsm_quat_z))

    def get_lsm6dsox_fusion_quat_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get LSM6DSOX Fusion quaternion data."""
        with self.data_lock:
            return (np.array(self.lsmox_fusion_timestamp),
                    np.array(self.lsmox_fusion_quat_w),
                    np.array(self.lsmox_fusion_quat_x),
                    np.array(self.lsmox_fusion_quat_y),
                    np.array(self.lsmox_fusion_quat_z))

    def get_lsm6dsox_fusion_euler_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get LSM6DSOX Fusion Euler angles data."""
        with self.data_lock:
            return (np.array(self.lsmox_fusion_timestamp),
                    np.array(self.lsmox_fusion_roll),
                    np.array(self.lsmox_fusion_pitch),
                    np.array(self.lsmox_fusion_yaw))

    def get_lsm6dsox_fusion_linacc_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get LSM6DSOX Fusion linear acceleration data."""
        with self.data_lock:
            return (np.array(self.lsmox_fusion_timestamp),
                    np.array(self.lsmox_fusion_linacc_x),
                    np.array(self.lsmox_fusion_linacc_y),
                    np.array(self.lsmox_fusion_linacc_z))

    def get_lsm6dsox_fusion_earthacc_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get LSM6DSOX Fusion earth acceleration data."""
        with self.data_lock:
            return (np.array(self.lsmox_fusion_timestamp),
                    np.array(self.lsmox_fusion_earthacc_x),
                    np.array(self.lsmox_fusion_earthacc_y),
                    np.array(self.lsmox_fusion_earthacc_z))

    def get_lsm6dsox_fusion_gravity_data(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Get LSM6DSOX Fusion gravity vector data."""
        with self.data_lock:
            return (np.array(self.lsmox_fusion_timestamp),
                    np.array(self.lsmox_fusion_gravity_x),
                    np.array(self.lsmox_fusion_gravity_y),
                    np.array(self.lsmox_fusion_gravity_z))

    @staticmethod
    def quaternion_to_euler(qw, qx, qy, qz) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Convert quaternion to Euler angles (roll, pitch, yaw) in degrees.

        Args:
            qw, qx, qy, qz: Quaternion components (can be arrays or scalars)

        Returns:
            Tuple of (roll, pitch, yaw) in degrees
        """
        # Ensure we're working with numpy arrays
        qw = np.asarray(qw)
        qx = np.asarray(qx)
        qy = np.asarray(qy)
        qz = np.asarray(qz)

        # Roll (x-axis rotation)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (qw * qy - qz * qx)
        # Clamp to avoid numerical issues with arcsin
        sinp = np.clip(sinp, -1.0, 1.0)
        pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Convert to degrees
        roll = np.degrees(roll)
        pitch = np.degrees(pitch)
        yaw = np.degrees(yaw)

        return roll, pitch, yaw

    def set_calibration(self, sensor_name: str, accel_bias: Tuple[float, float, float],
                       gyro_bias: Optional[Tuple[float, float, float]] = None):
        """
        Set calibration bias for a sensor.

        Args:
            sensor_name: Sensor name (LSM6DSV, LSM6DSOX, LIS2DW12, ADXL362)
            accel_bias: Tuple of (x, y, z) accelerometer bias in mg
            gyro_bias: Optional tuple of (x, y, z) gyroscope bias in mdps (for 6-axis sensors)
        """
        if sensor_name not in self.calibration:
            return

        self.calibration[sensor_name]['accel']['x'] = accel_bias[0]
        self.calibration[sensor_name]['accel']['y'] = accel_bias[1]
        self.calibration[sensor_name]['accel']['z'] = accel_bias[2]

        if gyro_bias is not None and 'gyro' in self.calibration[sensor_name]:
            self.calibration[sensor_name]['gyro']['x'] = gyro_bias[0]
            self.calibration[sensor_name]['gyro']['y'] = gyro_bias[1]
            self.calibration[sensor_name]['gyro']['z'] = gyro_bias[2]

        self.calibration[sensor_name]['calibrated'] = True

    def get_calibration(self, sensor_name: str) -> Dict:
        """Get calibration data for a sensor."""
        if sensor_name in self.calibration:
            return self.calibration[sensor_name]
        return None

    def clear_calibration(self, sensor_name: Optional[str] = None):
        """
        Clear calibration for a specific sensor or all sensors.

        Args:
            sensor_name: Sensor to clear, or None to clear all
        """
        if sensor_name is None:
            # Clear all calibrations
            for sensor in self.calibration:
                self.calibration[sensor]['accel'] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
                if 'gyro' in self.calibration[sensor]:
                    self.calibration[sensor]['gyro'] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
                self.calibration[sensor]['calibrated'] = False
        elif sensor_name in self.calibration:
            # Clear specific sensor
            self.calibration[sensor_name]['accel'] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            if 'gyro' in self.calibration[sensor_name]:
                self.calibration[sensor_name]['gyro'] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
            self.calibration[sensor_name]['calibrated'] = False

    def is_calibrated(self, sensor_name: str) -> bool:
        """Check if a sensor has been calibrated."""
        if sensor_name in self.calibration:
            return self.calibration[sensor_name]['calibrated']
        return False

    def get_calibrated_sensors(self) -> list:
        """Get list of calibrated sensor names."""
        return [name for name, data in self.calibration.items() if data['calibrated']]

    def clear(self):
        """Clear all data."""
        self.lsm_timestamp.clear()
        self.lsmox_timestamp.clear()
        self.lis_timestamp.clear()
        self.adxl_timestamp.clear()
        self.lsm_sflp_timestamp.clear()
        self.lsm_accel_x.clear()
        self.lsm_accel_y.clear()
        self.lsm_accel_z.clear()
        self.lsm_gyro_x.clear()
        self.lsm_gyro_y.clear()
        self.lsm_gyro_z.clear()
        self.lsmox_accel_x.clear()
        self.lsmox_accel_y.clear()
        self.lsmox_accel_z.clear()
        self.lsmox_gyro_x.clear()
        self.lsmox_gyro_y.clear()
        self.lsmox_gyro_z.clear()
        self.lis_accel_x.clear()
        self.lis_accel_y.clear()
        self.lis_accel_z.clear()
        self.adxl_accel_x.clear()
        self.adxl_accel_y.clear()
        self.adxl_accel_z.clear()
        self.lsm_quat_w.clear()
        self.lsm_quat_x.clear()
        self.lsm_quat_y.clear()
        self.lsm_quat_z.clear()
        self.lsmox_fusion_timestamp.clear()
        self.lsmox_fusion_quat_w.clear()
        self.lsmox_fusion_quat_x.clear()
        self.lsmox_fusion_quat_y.clear()
        self.lsmox_fusion_quat_z.clear()
        self.lsmox_fusion_roll.clear()
        self.lsmox_fusion_pitch.clear()
        self.lsmox_fusion_yaw.clear()
        self.lsmox_fusion_linacc_x.clear()
        self.lsmox_fusion_linacc_y.clear()
        self.lsmox_fusion_linacc_z.clear()
        self.lsmox_fusion_earthacc_x.clear()
        self.lsmox_fusion_earthacc_y.clear()
        self.lsmox_fusion_earthacc_z.clear()
        self.lsmox_fusion_gravity_x.clear()
        self.lsmox_fusion_gravity_y.clear()
        self.lsmox_fusion_gravity_z.clear()
        self.sample_count = 0
        self.lsm_sample_count = 0
        self.lsmox_sample_count = 0
        self.lis_sample_count = 0
        self.adxl_sample_count = 0
        self.lsm_sflp_sample_count = 0
        self.lsmox_fusion_sample_count = 0
        self.lsm6dsv_available = False
        self.lsm6dsox_available = False
        self.lis2dw12_available = False
        self.adxl362_available = False
        self.lsm6dsv_sflp_available = False
        self.lsm6dsox_fusion_available = False
        self.start_time = None


class DataIntegrator:
    """Numerical integration utilities for IMU data."""

    @staticmethod
    def detrend(data: np.ndarray) -> np.ndarray:
        """
        Remove DC offset (mean) from data.

        Args:
            data: Input array

        Returns:
            Detrended data
        """
        if len(data) == 0:
            return data
        return data - np.mean(data)

    @staticmethod
    def integrate_once(timestamps: np.ndarray, x: np.ndarray, y: np.ndarray, z: np.ndarray,
                      is_accel: bool = True, detrend: bool = False) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Perform first integration using cumulative trapezoidal rule.

        Args:
            timestamps: Time array in microseconds
            x, y, z: Data arrays
            is_accel: True for accelerometer (mg), False for gyroscope (mdps)
            detrend: Whether to remove DC offset before integration

        Returns:
            Integrated x, y, z arrays (velocity in m/s or angle in degrees)
        """
        if len(timestamps) < 2:
            return np.array([]), np.array([]), np.array([])

        # Convert timestamps to seconds
        time_sec = timestamps / 1e6

        # Apply detrending if requested
        if detrend:
            x = DataIntegrator.detrend(x)
            y = DataIntegrator.detrend(y)
            z = DataIntegrator.detrend(z)

        if is_accel:
            # Convert mg to m/s²
            x_ms2 = x / 1000.0 * 9.81
            y_ms2 = y / 1000.0 * 9.81
            z_ms2 = z / 1000.0 * 9.81

            # Integrate to get velocity (m/s)
            vx = np.concatenate(([0], np.cumsum(np.diff(time_sec) * (x_ms2[:-1] + x_ms2[1:]) / 2)))
            vy = np.concatenate(([0], np.cumsum(np.diff(time_sec) * (y_ms2[:-1] + y_ms2[1:]) / 2)))
            vz = np.concatenate(([0], np.cumsum(np.diff(time_sec) * (z_ms2[:-1] + z_ms2[1:]) / 2)))
        else:
            # Convert mdps to deg/s
            x_degs = x / 1000.0
            y_degs = y / 1000.0
            z_degs = z / 1000.0

            # Integrate to get angle (degrees)
            vx = np.concatenate(([0], np.cumsum(np.diff(time_sec) * (x_degs[:-1] + x_degs[1:]) / 2)))
            vy = np.concatenate(([0], np.cumsum(np.diff(time_sec) * (y_degs[:-1] + y_degs[1:]) / 2)))
            vz = np.concatenate(([0], np.cumsum(np.diff(time_sec) * (z_degs[:-1] + z_degs[1:]) / 2)))

        return vx, vy, vz

    @staticmethod
    def integrate_twice(timestamps: np.ndarray, x: np.ndarray, y: np.ndarray, z: np.ndarray,
                       detrend: bool = False) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Perform double integration (acceleration -> position).

        Args:
            timestamps: Time array in microseconds
            x, y, z: Acceleration data in mg
            detrend: Whether to remove DC offset before integration

        Returns:
            Position x, y, z arrays in meters
        """
        if len(timestamps) < 2:
            return np.array([]), np.array([]), np.array([])

        # First integration: acceleration -> velocity
        vx, vy, vz = DataIntegrator.integrate_once(timestamps, x, y, z, is_accel=True, detrend=detrend)

        # Second integration: velocity -> position
        time_sec = timestamps / 1e6

        px = np.concatenate(([0], np.cumsum(np.diff(time_sec) * (vx[:-1] + vx[1:]) / 2)))
        py = np.concatenate(([0], np.cumsum(np.diff(time_sec) * (vy[:-1] + vy[1:]) / 2)))
        pz = np.concatenate(([0], np.cumsum(np.diff(time_sec) * (vz[:-1] + vz[1:]) / 2)))

        return px, py, pz


class DataFilter:
    """Signal filtering utilities for IMU data."""

    @staticmethod
    def lowpass_butterworth(data_x: np.ndarray, data_y: np.ndarray, data_z: np.ndarray,
                           sample_rate: float, cutoff_freq: float, order: int = 4) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Apply low-pass Butterworth filter to 3-axis data.

        Args:
            data_x, data_y, data_z: Input data arrays
            sample_rate: Sample rate in Hz
            cutoff_freq: Cutoff frequency in Hz
            order: Filter order (default 4)

        Returns:
            Filtered x, y, z arrays
        """
        if len(data_x) < 2 or sample_rate <= 0 or cutoff_freq <= 0:
            return data_x, data_y, data_z

        # Ensure cutoff is less than Nyquist frequency
        nyquist = sample_rate / 2.0
        if cutoff_freq >= nyquist:
            cutoff_freq = nyquist * 0.95

        # Design Butterworth filter
        sos = signal.butter(order, cutoff_freq, btype='low', fs=sample_rate, output='sos')

        # Apply filter to each axis
        filtered_x = signal.sosfiltfilt(sos, data_x)
        filtered_y = signal.sosfiltfilt(sos, data_y)
        filtered_z = signal.sosfiltfilt(sos, data_z)

        return filtered_x, filtered_y, filtered_z

    @staticmethod
    def highpass_butterworth(data_x: np.ndarray, data_y: np.ndarray, data_z: np.ndarray,
                            sample_rate: float, cutoff_freq: float, order: int = 4) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Apply high-pass Butterworth filter to 3-axis data.

        Args:
            data_x, data_y, data_z: Input data arrays
            sample_rate: Sample rate in Hz
            cutoff_freq: Cutoff frequency in Hz
            order: Filter order (default 4)

        Returns:
            Filtered x, y, z arrays
        """
        if len(data_x) < 2 or sample_rate <= 0 or cutoff_freq <= 0:
            return data_x, data_y, data_z

        # Ensure cutoff is less than Nyquist frequency
        nyquist = sample_rate / 2.0
        if cutoff_freq >= nyquist:
            cutoff_freq = nyquist * 0.95

        # Design Butterworth filter
        sos = signal.butter(order, cutoff_freq, btype='high', fs=sample_rate, output='sos')

        # Apply filter to each axis
        filtered_x = signal.sosfiltfilt(sos, data_x)
        filtered_y = signal.sosfiltfilt(sos, data_y)
        filtered_z = signal.sosfiltfilt(sos, data_z)

        return filtered_x, filtered_y, filtered_z

    @staticmethod
    def moving_average(data_x: np.ndarray, data_y: np.ndarray, data_z: np.ndarray,
                      window_size: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Apply moving average filter to 3-axis data with proper edge handling.

        Args:
            data_x, data_y, data_z: Input data arrays
            window_size: Window size (number of samples, must be odd)

        Returns:
            Filtered x, y, z arrays

        Note:
            Uses symmetric padding to avoid edge effects from zero-padding.
        """
        if len(data_x) < window_size or window_size < 3:
            return data_x, data_y, data_z

        # Ensure window size is odd
        if window_size % 2 == 0:
            window_size += 1

        # Create uniform window
        window = np.ones(window_size) / window_size

        # Calculate padding size (half window on each side)
        pad_size = window_size // 2

        # Apply symmetric padding to avoid edge effects
        # 'reflect' mode mirrors the data at boundaries
        padded_x = np.pad(data_x, pad_size, mode='reflect')
        padded_y = np.pad(data_y, pad_size, mode='reflect')
        padded_z = np.pad(data_z, pad_size, mode='reflect')

        # Apply convolution with 'valid' mode (no additional padding)
        filtered_x = np.convolve(padded_x, window, mode='valid')
        filtered_y = np.convolve(padded_y, window, mode='valid')
        filtered_z = np.convolve(padded_z, window, mode='valid')

        return filtered_x, filtered_y, filtered_z

    @staticmethod
    def savgol_filter(data_x: np.ndarray, data_y: np.ndarray, data_z: np.ndarray,
                     window_size: int, poly_order: int = 2) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Apply Savitzky-Golay filter to 3-axis data.

        Args:
            data_x, data_y, data_z: Input data arrays
            window_size: Window size (number of samples, must be odd)
            poly_order: Polynomial order (must be less than window_size)

        Returns:
            Filtered x, y, z arrays
        """
        if len(data_x) < window_size or window_size < 3:
            return data_x, data_y, data_z

        # Ensure window size is odd
        if window_size % 2 == 0:
            window_size += 1

        # Ensure poly_order is valid
        if poly_order >= window_size:
            poly_order = window_size - 1

        # Apply Savitzky-Golay filter
        filtered_x = signal.savgol_filter(data_x, window_size, poly_order)
        filtered_y = signal.savgol_filter(data_y, window_size, poly_order)
        filtered_z = signal.savgol_filter(data_z, window_size, poly_order)

        return filtered_x, filtered_y, filtered_z

    @staticmethod
    def apply_filter_to_quaternion(qw: np.ndarray, qx: np.ndarray, qy: np.ndarray, qz: np.ndarray,
                                   filter_type: str, **kwargs) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Apply filter to quaternion data (4-axis).

        Args:
            qw, qx, qy, qz: Quaternion components
            filter_type: Type of filter ('lowpass', 'highpass', 'moving_average', 'savgol')
            **kwargs: Filter-specific parameters

        Returns:
            Filtered quaternion components (w, x, y, z)
        """
        if filter_type == 'lowpass':
            sample_rate = kwargs.get('sample_rate', 100.0)
            cutoff_freq = kwargs.get('cutoff_freq', 10.0)
            order = kwargs.get('order', 4)

            # Filter first 3 components together, then w separately
            filtered_x, filtered_y, filtered_z = DataFilter.lowpass_butterworth(qx, qy, qz, sample_rate, cutoff_freq, order)
            filtered_w, _, _ = DataFilter.lowpass_butterworth(qw, qw, qw, sample_rate, cutoff_freq, order)

        elif filter_type == 'highpass':
            sample_rate = kwargs.get('sample_rate', 100.0)
            cutoff_freq = kwargs.get('cutoff_freq', 0.5)
            order = kwargs.get('order', 4)

            filtered_x, filtered_y, filtered_z = DataFilter.highpass_butterworth(qx, qy, qz, sample_rate, cutoff_freq, order)
            filtered_w, _, _ = DataFilter.highpass_butterworth(qw, qw, qw, sample_rate, cutoff_freq, order)

        elif filter_type == 'moving_average':
            window_size = kwargs.get('window_size', 5)

            filtered_x, filtered_y, filtered_z = DataFilter.moving_average(qx, qy, qz, window_size)
            filtered_w, _, _ = DataFilter.moving_average(qw, qw, qw, window_size)

        elif filter_type == 'savgol':
            window_size = kwargs.get('window_size', 5)
            poly_order = kwargs.get('poly_order', 2)

            filtered_x, filtered_y, filtered_z = DataFilter.savgol_filter(qx, qy, qz, window_size, poly_order)
            filtered_w, _, _ = DataFilter.savgol_filter(qw, qw, qw, window_size, poly_order)

        else:
            return qw, qx, qy, qz

        # Renormalize quaternion to maintain unit length
        magnitude = np.sqrt(filtered_w**2 + filtered_x**2 + filtered_y**2 + filtered_z**2)
        magnitude = np.where(magnitude == 0, 1.0, magnitude)  # Avoid division by zero

        filtered_w = filtered_w / magnitude
        filtered_x = filtered_x / magnitude
        filtered_y = filtered_y / magnitude
        filtered_z = filtered_z / magnitude

        return filtered_w, filtered_x, filtered_y, filtered_z


class DataStatistics:
    """Calculate statistics for IMU data."""

    @staticmethod
    def calculate_stats(x: np.ndarray, y: np.ndarray, z: np.ndarray) -> Dict:
        """
        Calculate statistics for 3-axis data.

        Returns:
            Dictionary containing min, max, mean, std, and RMS for each axis
        """
        if len(x) == 0:
            return {
                'x': {'min': 0, 'max': 0, 'mean': 0, 'std': 0, 'rms': 0},
                'y': {'min': 0, 'max': 0, 'mean': 0, 'std': 0, 'rms': 0},
                'z': {'min': 0, 'max': 0, 'mean': 0, 'std': 0, 'rms': 0},
                'magnitude': {'min': 0, 'max': 0, 'mean': 0, 'std': 0, 'rms': 0}
            }

        # Calculate magnitude
        magnitude = np.sqrt(x**2 + y**2 + z**2)

        stats = {
            'x': {
                'min': float(np.min(x)),
                'max': float(np.max(x)),
                'mean': float(np.mean(x)),
                'std': float(np.std(x)),
                'rms': float(np.sqrt(np.mean(x**2)))
            },
            'y': {
                'min': float(np.min(y)),
                'max': float(np.max(y)),
                'mean': float(np.mean(y)),
                'std': float(np.std(y)),
                'rms': float(np.sqrt(np.mean(y**2)))
            },
            'z': {
                'min': float(np.min(z)),
                'max': float(np.max(z)),
                'mean': float(np.mean(z)),
                'std': float(np.std(z)),
                'rms': float(np.sqrt(np.mean(z**2)))
            },
            'magnitude': {
                'min': float(np.min(magnitude)),
                'max': float(np.max(magnitude)),
                'mean': float(np.mean(magnitude)),
                'std': float(np.std(magnitude)),
                'rms': float(np.sqrt(np.mean(magnitude**2)))
            }
        }

        return stats

    @staticmethod
    def calculate_sample_rate(timestamps: np.ndarray) -> float:
        """Calculate average sample rate from timestamps in microseconds."""
        if len(timestamps) < 2:
            return 0.0

        # Calculate time differences in microseconds
        time_diffs = np.diff(timestamps)

        # Filter out zero or negative differences
        valid_diffs = time_diffs[time_diffs > 0]

        if len(valid_diffs) == 0:
            return 0.0

        # Average time per sample in seconds
        avg_time_per_sample = np.mean(valid_diffs) / 1e6

        # Sample rate in Hz
        return 1.0 / avg_time_per_sample if avg_time_per_sample > 0 else 0.0


class CSVLoader:
    """Load IMU data from CSV files."""

    @staticmethod
    def load_csv(filepath: str, imu_data: IMUData) -> bool:
        """
        Load CSV file into IMUData container.
        Supports new format with IMU identifier as first column.

        Args:
            filepath: Path to CSV file
            imu_data: IMUData container to populate

        Returns:
            True if successful, False otherwise
        """
        try:
            # Clear existing data
            imu_data.clear()

            # Read file line by line to handle variable column format
            with open(filepath, 'r') as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue

                    parts = line.split(',')

                    # Check if this is the new format with IMU identifier
                    if len(parts) >= 4:
                        imu_type = parts[0].strip()

                        if imu_type == 'LSM6DSV' and len(parts) >= 8:
                            # Format: LSM6DSV,timestamp,ax,ay,az,gx,gy,gz
                            try:
                                timestamp = float(parts[1])
                                ax = float(parts[2])
                                ay = float(parts[3])
                                az = float(parts[4])
                                gx = float(parts[5])
                                gy = float(parts[6])
                                gz = float(parts[7])
                                imu_data.add_lsm6dsv_sample(timestamp, ax, ay, az, gx, gy, gz)
                            except ValueError:
                                continue  # Skip invalid lines

                        elif imu_type == 'LSM6DSOX' and len(parts) >= 8:
                            # Format: LSM6DSOX,timestamp,ax,ay,az,gx,gy,gz
                            try:
                                timestamp = float(parts[1])
                                ax = float(parts[2])
                                ay = float(parts[3])
                                az = float(parts[4])
                                gx = float(parts[5])
                                gy = float(parts[6])
                                gz = float(parts[7])
                                imu_data.add_lsm6dsox_sample(timestamp, ax, ay, az, gx, gy, gz)
                            except ValueError:
                                continue  # Skip invalid lines

                        elif imu_type == 'LIS2DW12' and len(parts) >= 5:
                            # Format: LIS2DW12,timestamp,ax,ay,az
                            try:
                                timestamp = float(parts[1])
                                ax = float(parts[2])
                                ay = float(parts[3])
                                az = float(parts[4])
                                imu_data.add_lis2dw12_sample(timestamp, ax, ay, az)
                            except ValueError:
                                continue  # Skip invalid lines

                        elif imu_type == 'ADXL362' and len(parts) >= 5:
                            # Format: ADXL362,timestamp,ax,ay,az
                            try:
                                timestamp = float(parts[1])
                                ax = float(parts[2])
                                ay = float(parts[3])
                                az = float(parts[4])
                                imu_data.add_adxl362_sample(timestamp, ax, ay, az)
                            except ValueError:
                                continue  # Skip invalid lines

                        elif imu_type == 'LSM6DSV_SFLP' and len(parts) >= 6:
                            # Format: LSM6DSV_SFLP,timestamp,qw,qx,qy,qz
                            try:
                                timestamp = float(parts[1])
                                qw = float(parts[2])
                                qx = float(parts[3])
                                qy = float(parts[4])
                                qz = float(parts[5])
                                imu_data.add_lsm6dsv_sflp_sample(timestamp, qw, qx, qy, qz)
                            except ValueError:
                                continue  # Skip invalid lines

                        elif imu_type == 'LSM6DSOX_FUSION_QUAT' and len(parts) >= 6:
                            # Format: LSM6DSOX_FUSION_QUAT,timestamp,qw,qx,qy,qz
                            try:
                                timestamp = float(parts[1])
                                qw = float(parts[2])
                                qx = float(parts[3])
                                qy = float(parts[4])
                                qz = float(parts[5])
                                imu_data.add_lsm6dsox_fusion_quat_sample(timestamp, qw, qx, qy, qz)
                            except ValueError:
                                continue  # Skip invalid lines

                        elif imu_type == 'LSM6DSOX_FUSION_EULER' and len(parts) >= 5:
                            # Format: LSM6DSOX_FUSION_EULER,timestamp,roll,pitch,yaw
                            try:
                                timestamp = float(parts[1])
                                roll = float(parts[2])
                                pitch = float(parts[3])
                                yaw = float(parts[4])
                                imu_data.add_lsm6dsox_fusion_euler_sample(timestamp, roll, pitch, yaw)
                            except ValueError:
                                continue  # Skip invalid lines

                        elif imu_type == 'LSM6DSOX_FUSION_LINACC' and len(parts) >= 5:
                            # Format: LSM6DSOX_FUSION_LINACC,timestamp,x,y,z
                            try:
                                timestamp = float(parts[1])
                                x = float(parts[2])
                                y = float(parts[3])
                                z = float(parts[4])
                                imu_data.add_lsm6dsox_fusion_linacc_sample(timestamp, x, y, z)
                            except ValueError:
                                continue  # Skip invalid lines

                        elif imu_type == 'LSM6DSOX_FUSION_EARTHACC' and len(parts) >= 5:
                            # Format: LSM6DSOX_FUSION_EARTHACC,timestamp,x,y,z
                            try:
                                timestamp = float(parts[1])
                                x = float(parts[2])
                                y = float(parts[3])
                                z = float(parts[4])
                                imu_data.add_lsm6dsox_fusion_earthacc_sample(timestamp, x, y, z)
                            except ValueError:
                                continue  # Skip invalid lines

                        elif imu_type == 'LSM6DSOX_FUSION_GRAVITY' and len(parts) >= 5:
                            # Format: LSM6DSOX_FUSION_GRAVITY,timestamp,x,y,z
                            try:
                                timestamp = float(parts[1])
                                x = float(parts[2])
                                y = float(parts[3])
                                z = float(parts[4])
                                imu_data.add_lsm6dsox_fusion_gravity_sample(timestamp, x, y, z)
                            except ValueError:
                                continue  # Skip invalid lines

            return imu_data.sample_count > 0

        except Exception as e:
            print(f"Error loading CSV: {e}")
            return False


class SerialReader(QThread):
    """Thread for reading serial data in real-time."""

    # Signals - separate signals for each IMU type
    lsm6dsv_data_received = pyqtSignal(float, float, float, float, float, float, float)  # timestamp, ax, ay, az, gx, gy, gz
    lsm6dsox_data_received = pyqtSignal(float, float, float, float, float, float, float)  # timestamp, ax, ay, az, gx, gy, gz
    lis2dw12_data_received = pyqtSignal(float, float, float, float)  # timestamp, ax, ay, az
    adxl362_data_received = pyqtSignal(float, float, float, float)  # timestamp, ax, ay, az
    lsm6dsv_sflp_data_received = pyqtSignal(float, float, float, float, float)  # timestamp, qw, qx, qy, qz
    lsm6dsox_fusion_quat_received = pyqtSignal(float, float, float, float, float)  # timestamp, qw, qx, qy, qz
    lsm6dsox_fusion_euler_received = pyqtSignal(float, float, float, float)  # timestamp, roll, pitch, yaw
    lsm6dsox_fusion_linacc_received = pyqtSignal(float, float, float, float)  # timestamp, x, y, z
    lsm6dsox_fusion_earthacc_received = pyqtSignal(float, float, float, float)  # timestamp, x, y, z
    lsm6dsox_fusion_gravity_received = pyqtSignal(float, float, float, float)  # timestamp, x, y, z
    error_occurred = pyqtSignal(str)
    connection_status = pyqtSignal(bool)
    imu_detected = pyqtSignal(str, bool)  # imu_type, is_present

    def __init__(self):
        super().__init__()
        self.serial_port: Optional[serial.Serial] = None
        self.running = False
        self.port_name = ""
        self.baud_rate = 115200

    def set_port(self, port_name: str, baud_rate: int = 115200):
        """Set serial port parameters."""
        self.port_name = port_name
        self.baud_rate = baud_rate

    def run(self):
        """Main thread loop for reading serial data."""
        try:
            # Open serial port
            self.serial_port = serial.Serial(
                port=self.port_name,
                baudrate=self.baud_rate,
                timeout=1.0
            )

            self.running = True
            self.connection_status.emit(True)

            # Main read loop
            while self.running:
                try:
                    line = self.serial_port.readline().decode('utf-8', errors='ignore').strip()

                    if not line:
                        continue

                    # Skip informational messages (lines containing "===", "Detected", etc.)
                    if '===' in line or 'Detected' in line or 'NOT FOUND' in line or 'Starting' in line:
                        continue

                    # Parse CSV line with IMU identifier
                    parts = line.split(',')

                    if len(parts) >= 4:
                        imu_type = parts[0].strip()

                        if imu_type == 'LSM6DSV' and len(parts) >= 8:
                            # Format: LSM6DSV,timestamp,ax,ay,az,gx,gy,gz
                            timestamp = float(parts[1])
                            ax = float(parts[2])
                            ay = float(parts[3])
                            az = float(parts[4])
                            gx = float(parts[5])
                            gy = float(parts[6])
                            gz = float(parts[7])

                            # Emit LSM6DSV data
                            self.lsm6dsv_data_received.emit(timestamp, ax, ay, az, gx, gy, gz)

                        elif imu_type == 'LSM6DSOX' and len(parts) >= 8:
                            # Format: LSM6DSOX,timestamp,ax,ay,az,gx,gy,gz
                            timestamp = float(parts[1])
                            ax = float(parts[2])
                            ay = float(parts[3])
                            az = float(parts[4])
                            gx = float(parts[5])
                            gy = float(parts[6])
                            gz = float(parts[7])

                            # Emit LSM6DSOX data
                            self.lsm6dsox_data_received.emit(timestamp, ax, ay, az, gx, gy, gz)

                        elif imu_type == 'LIS2DW12' and len(parts) >= 5:
                            # Format: LIS2DW12,timestamp,ax,ay,az
                            timestamp = float(parts[1])
                            ax = float(parts[2])
                            ay = float(parts[3])
                            az = float(parts[4])

                            # Emit LIS2DW12 data
                            self.lis2dw12_data_received.emit(timestamp, ax, ay, az)

                        elif imu_type == 'ADXL362' and len(parts) >= 5:
                            # Format: ADXL362,timestamp,ax,ay,az
                            timestamp = float(parts[1])
                            ax = float(parts[2])
                            ay = float(parts[3])
                            az = float(parts[4])

                            # Emit ADXL362 data
                            self.adxl362_data_received.emit(timestamp, ax, ay, az)

                        elif imu_type == 'LSM6DSV_SFLP' and len(parts) >= 6:
                            # Format: LSM6DSV_SFLP,timestamp,qw,qx,qy,qz
                            timestamp = float(parts[1])
                            qw = float(parts[2])
                            qx = float(parts[3])
                            qy = float(parts[4])
                            qz = float(parts[5])

                            # Emit SFLP data
                            self.lsm6dsv_sflp_data_received.emit(timestamp, qw, qx, qy, qz)

                        elif imu_type == 'LSM6DSOX_FUSION_QUAT' and len(parts) >= 6:
                            # Format: LSM6DSOX_FUSION_QUAT,timestamp,qw,qx,qy,qz
                            timestamp = float(parts[1])
                            qw = float(parts[2])
                            qx = float(parts[3])
                            qy = float(parts[4])
                            qz = float(parts[5])

                            # Emit fusion quaternion data
                            self.lsm6dsox_fusion_quat_received.emit(timestamp, qw, qx, qy, qz)

                        elif imu_type == 'LSM6DSOX_FUSION_EULER' and len(parts) >= 5:
                            # Format: LSM6DSOX_FUSION_EULER,timestamp,roll,pitch,yaw
                            timestamp = float(parts[1])
                            roll = float(parts[2])
                            pitch = float(parts[3])
                            yaw = float(parts[4])

                            # Emit fusion Euler data
                            self.lsm6dsox_fusion_euler_received.emit(timestamp, roll, pitch, yaw)

                        elif imu_type == 'LSM6DSOX_FUSION_LINACC' and len(parts) >= 5:
                            # Format: LSM6DSOX_FUSION_LINACC,timestamp,x,y,z
                            timestamp = float(parts[1])
                            x = float(parts[2])
                            y = float(parts[3])
                            z = float(parts[4])

                            # Emit fusion linear accel data
                            self.lsm6dsox_fusion_linacc_received.emit(timestamp, x, y, z)

                        elif imu_type == 'LSM6DSOX_FUSION_EARTHACC' and len(parts) >= 5:
                            # Format: LSM6DSOX_FUSION_EARTHACC,timestamp,x,y,z
                            timestamp = float(parts[1])
                            x = float(parts[2])
                            y = float(parts[3])
                            z = float(parts[4])

                            # Emit fusion earth accel data
                            self.lsm6dsox_fusion_earthacc_received.emit(timestamp, x, y, z)

                        elif imu_type == 'LSM6DSOX_FUSION_GRAVITY' and len(parts) >= 5:
                            # Format: LSM6DSOX_FUSION_GRAVITY,timestamp,x,y,z
                            timestamp = float(parts[1])
                            x = float(parts[2])
                            y = float(parts[3])
                            z = float(parts[4])

                            # Emit fusion gravity data
                            self.lsm6dsox_fusion_gravity_received.emit(timestamp, x, y, z)

                except ValueError:
                    # Skip invalid lines
                    continue
                except Exception as e:
                    self.error_occurred.emit(f"Read error: {str(e)}")

        except serial.SerialException as e:
            self.error_occurred.emit(f"Serial port error: {str(e)}")
            self.connection_status.emit(False)
        except Exception as e:
            self.error_occurred.emit(f"Unexpected error: {str(e)}")
            self.connection_status.emit(False)
        finally:
            self.stop()

    def stop(self):
        """Stop reading and close serial port."""
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except:
                pass
        self.connection_status.emit(False)
        # Wait for thread to finish
        self.wait(1000)

    @staticmethod
    def get_available_ports():
        """Get list of available serial ports."""
        ports = serial.tools.list_ports.comports()
        return [port.device for port in ports]


class CalibrationCapture(QThread):
    """Thread for capturing calibration samples over 1 second."""

    # Signals
    calibration_complete = pyqtSignal(str, tuple, tuple)  # sensor_name, accel_bias (x,y,z), gyro_bias (x,y,z) or None
    calibration_progress = pyqtSignal(int)  # Progress percentage 0-100
    calibration_error = pyqtSignal(str)  # Error message

    def __init__(self, imu_data: IMUData, sensor_name: str, sensor_type: str, capture_duration: float = 1.0):
        """
        Initialize calibration capture.

        Args:
            imu_data: IMUData instance to read samples from
            sensor_name: Name of sensor (LSM6DSV, LSM6DSOX, LIS2DW12, ADXL362)
            sensor_type: Type (LSM6DSV_ACCEL, LSM6DSV_GYRO, etc.)
            capture_duration: How long to capture samples in seconds (default 1.0)
        """
        super().__init__()
        self.imu_data = imu_data
        self.sensor_name = sensor_name
        self.sensor_type = sensor_type
        self.capture_duration = capture_duration
        self.sample_interval = 0.01  # Check every 10ms

    def run(self):
        """Capture samples for calibration over 1 second."""
        try:
            # Lists to accumulate samples
            accel_x_samples = []
            accel_y_samples = []
            accel_z_samples = []
            gyro_x_samples = []
            gyro_y_samples = []
            gyro_z_samples = []

            # Determine which data to collect based on sensor
            has_gyro = self.sensor_name in ['LSM6DSV', 'LSM6DSOX']

            start_time = time.time()
            last_sample_count = 0

            # Get initial sample count
            if self.sensor_name == 'LSM6DSV':
                last_sample_count = self.imu_data.lsm_sample_count
            elif self.sensor_name == 'LSM6DSOX':
                last_sample_count = self.imu_data.lsmox_sample_count
            elif self.sensor_name == 'LIS2DW12':
                last_sample_count = self.imu_data.lis_sample_count
            elif self.sensor_name == 'ADXL362':
                last_sample_count = self.imu_data.adxl_sample_count

            # Collect samples for 1 second
            while time.time() - start_time < self.capture_duration:
                # Get current data
                if self.sensor_name == 'LSM6DSV':
                    if self.imu_data.lsm_sample_count > last_sample_count:
                        # New samples available
                        accel_x_samples.append(self.imu_data.lsm_accel_x[-1])
                        accel_y_samples.append(self.imu_data.lsm_accel_y[-1])
                        accel_z_samples.append(self.imu_data.lsm_accel_z[-1])
                        gyro_x_samples.append(self.imu_data.lsm_gyro_x[-1])
                        gyro_y_samples.append(self.imu_data.lsm_gyro_y[-1])
                        gyro_z_samples.append(self.imu_data.lsm_gyro_z[-1])
                        last_sample_count = self.imu_data.lsm_sample_count

                elif self.sensor_name == 'LSM6DSOX':
                    if self.imu_data.lsmox_sample_count > last_sample_count:
                        accel_x_samples.append(self.imu_data.lsmox_accel_x[-1])
                        accel_y_samples.append(self.imu_data.lsmox_accel_y[-1])
                        accel_z_samples.append(self.imu_data.lsmox_accel_z[-1])
                        gyro_x_samples.append(self.imu_data.lsmox_gyro_x[-1])
                        gyro_y_samples.append(self.imu_data.lsmox_gyro_y[-1])
                        gyro_z_samples.append(self.imu_data.lsmox_gyro_z[-1])
                        last_sample_count = self.imu_data.lsmox_sample_count

                elif self.sensor_name == 'LIS2DW12':
                    if self.imu_data.lis_sample_count > last_sample_count:
                        accel_x_samples.append(self.imu_data.lis_accel_x[-1])
                        accel_y_samples.append(self.imu_data.lis_accel_y[-1])
                        accel_z_samples.append(self.imu_data.lis_accel_z[-1])
                        last_sample_count = self.imu_data.lis_sample_count

                elif self.sensor_name == 'ADXL362':
                    if self.imu_data.adxl_sample_count > last_sample_count:
                        accel_x_samples.append(self.imu_data.adxl_accel_x[-1])
                        accel_y_samples.append(self.imu_data.adxl_accel_y[-1])
                        accel_z_samples.append(self.imu_data.adxl_accel_z[-1])
                        last_sample_count = self.imu_data.adxl_sample_count

                # Update progress
                progress = int((time.time() - start_time) / self.capture_duration * 100)
                self.calibration_progress.emit(min(progress, 100))

                # Sleep briefly
                time.sleep(self.sample_interval)

            # Check if we collected enough samples
            if len(accel_x_samples) < 10:
                self.calibration_error.emit(f"Insufficient samples collected ({len(accel_x_samples)}). Ensure sensor is streaming data.")
                return

            # Calculate average bias
            accel_bias = (
                float(np.mean(accel_x_samples)),
                float(np.mean(accel_y_samples)),
                float(np.mean(accel_z_samples))
            )

            gyro_bias = None
            if has_gyro and len(gyro_x_samples) > 0:
                gyro_bias = (
                    float(np.mean(gyro_x_samples)),
                    float(np.mean(gyro_y_samples)),
                    float(np.mean(gyro_z_samples))
                )

            # Emit completion signal
            self.calibration_complete.emit(self.sensor_name, accel_bias, gyro_bias)

        except Exception as e:
            self.calibration_error.emit(f"Calibration error: {str(e)}")
