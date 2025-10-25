"""
IMU Data Visualizer
GUI application for visualizing multi-IMU data (LSM6DSV, LIS2DW12, and ADXL362)
"""

import sys
import numpy as np
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout,
                             QHBoxLayout, QPushButton, QLabel, QComboBox,
                             QGroupBox, QRadioButton, QFileDialog, QGridLayout,
                             QButtonGroup, QTextEdit, QSplitter, QCheckBox,
                             QSpinBox, QDoubleSpinBox, QSlider)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QFont
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D

from data_processor import IMUData, DataStatistics, DataIntegrator, DataFilter, CSVLoader, SerialReader, CalibrationCapture


class IMUVisualizer(QMainWindow):
    """Main window for IMU data visualization."""

    def __init__(self):
        super().__init__()

        # Data
        self.imu_data = IMUData(max_samples=50000)
        self.serial_reader = SerialReader()

        # State
        self.baud_rate = 1000000  # Default to 1Mbit to match firmware
        self.current_imu = "LSM6DSV_ACCEL"  # LSM6DSV_ACCEL, LSM6DSV_GYRO, LSM6DSOX_ACCEL, LSM6DSOX_GYRO, LIS2DW12, ADXL362, LSM6DSV_SFLP
        self.current_graph = "XYZ"  # XYZ, MAGNITUDE, 3D
        self.integration_level = 0  # 0=raw, 1=first integration, 2=second integration
        self.detrend_enabled = False
        self.is_streaming = False
        self.sflp_view_mode = "QUATERNION"  # QUATERNION or EULER

        # Filtering state
        self.filter_type = "None"  # None, Low-pass, High-pass, Moving Average, Savitzky-Golay
        self.lowpass_cutoff = 10.0  # Hz
        self.lowpass_order = 4
        self.highpass_cutoff = 0.5  # Hz
        self.highpass_order = 4
        self.moving_avg_window = 5
        self.savgol_window = 5
        self.savgol_order = 2

        # Calibration state
        self.calibration_enabled = False
        self.calibration_thread = None
        self.calibration_duration = 1.0  # seconds
        self.calibration_mode = "capture"  # "capture" or "existing"

        # Setup UI
        self.init_ui()

        # Connect signals
        self.setup_connections()

        # Initialize integration options based on default IMU selection
        self.update_integration_options()

        # Setup update timer for plots (reduced frequency for performance)
        self.plot_timer = QTimer()
        self.plot_timer.timeout.connect(self.update_plots)
        self.plot_timer.start(200)  # Update every 200ms

        # Setup stats timer
        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self.update_statistics)
        self.stats_timer.start(1000)  # Update every 1 second

        # Performance settings
        self.max_plot_points = 5000  # Limit points for plotting
        self.downsample_threshold = 10000  # Downsample if more points

    def init_ui(self):
        """Initialize the user interface."""
        self.setWindowTitle("Dual IMU Data Visualizer")
        self.setGeometry(100, 100, 1400, 900)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QVBoxLayout(central_widget)

        # Top control panel
        control_panel = self.create_control_panel()
        main_layout.addWidget(control_panel)

        # Splitter for plots and statistics
        splitter = QSplitter(Qt.Horizontal)

        # Left side: Plot area
        plot_widget = self.create_plot_area()
        splitter.addWidget(plot_widget)

        # Right side: Statistics panel
        stats_widget = self.create_statistics_panel()
        splitter.addWidget(stats_widget)

        # Set splitter proportions (70% plots, 30% stats)
        splitter.setSizes([1000, 400])

        main_layout.addWidget(splitter)

    def create_control_panel(self) -> QWidget:
        """Create the top control panel."""
        panel = QWidget()
        layout = QHBoxLayout(panel)

        # Data Source Group
        data_source_group = QGroupBox("Data Source")
        data_source_layout = QHBoxLayout()

        self.load_file_btn = QPushButton("Load CSV File")
        self.load_file_btn.clicked.connect(self.load_csv_file)
        data_source_layout.addWidget(self.load_file_btn)

        self.port_combo = QComboBox()
        self.refresh_ports()
        data_source_layout.addWidget(QLabel("Port:"))
        data_source_layout.addWidget(self.port_combo)

        self.refresh_ports_btn = QPushButton("Refresh")
        self.refresh_ports_btn.clicked.connect(self.refresh_ports)
        data_source_layout.addWidget(self.refresh_ports_btn)

        data_source_layout.addWidget(QLabel("Baud Rate:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["9600", "19200", "38400", "57600", "115200", "230400", "460800", "921600", "1000000"])
        self.baud_combo.setCurrentText("1000000")  # Default to 1Mbit
        self.baud_combo.currentTextChanged.connect(self.on_baud_rate_changed)
        data_source_layout.addWidget(self.baud_combo)

        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.toggle_serial_connection)
        data_source_layout.addWidget(self.connect_btn)

        data_source_group.setLayout(data_source_layout)
        layout.addWidget(data_source_group)

        # IMU Selection Group
        imu_group = QGroupBox("IMU Selection")
        imu_layout = QVBoxLayout()

        self.imu_button_group = QButtonGroup()

        self.lsm_accel_radio = QRadioButton("LSM6DSV - Accelerometer")
        self.lsm_accel_radio.setChecked(True)
        self.imu_button_group.addButton(self.lsm_accel_radio, 0)
        imu_layout.addWidget(self.lsm_accel_radio)

        self.lsm_gyro_radio = QRadioButton("LSM6DSV - Gyroscope")
        self.imu_button_group.addButton(self.lsm_gyro_radio, 1)
        imu_layout.addWidget(self.lsm_gyro_radio)

        self.lsm_both_radio = QRadioButton("LSM6DSV - Both (Accel + Gyro)")
        self.imu_button_group.addButton(self.lsm_both_radio, 2)
        imu_layout.addWidget(self.lsm_both_radio)

        self.lsm_sflp_radio = QRadioButton("LSM6DSV - Sensor Fusion (SFLP)")
        self.imu_button_group.addButton(self.lsm_sflp_radio, 3)
        imu_layout.addWidget(self.lsm_sflp_radio)

        self.lsmox_accel_radio = QRadioButton("LSM6DSOX - Accelerometer")
        self.imu_button_group.addButton(self.lsmox_accel_radio, 4)
        imu_layout.addWidget(self.lsmox_accel_radio)

        self.lsmox_gyro_radio = QRadioButton("LSM6DSOX - Gyroscope")
        self.imu_button_group.addButton(self.lsmox_gyro_radio, 5)
        imu_layout.addWidget(self.lsmox_gyro_radio)

        self.lis_radio = QRadioButton("LIS2DW12 - Accelerometer")
        self.imu_button_group.addButton(self.lis_radio, 6)
        imu_layout.addWidget(self.lis_radio)

        self.adxl_radio = QRadioButton("ADXL362 - Accelerometer")
        self.imu_button_group.addButton(self.adxl_radio, 7)
        imu_layout.addWidget(self.adxl_radio)

        self.imu_button_group.buttonClicked.connect(self.on_imu_changed)

        imu_group.setLayout(imu_layout)
        layout.addWidget(imu_group)

        # Graph Type Group
        graph_group = QGroupBox("Graph Type")
        graph_layout = QVBoxLayout()

        self.graph_combo = QComboBox()
        self.graph_combo.addItems(["X, Y, Z vs Time", "Magnitude vs Time", "3D Trajectory"])
        self.graph_combo.currentTextChanged.connect(self.on_graph_changed)
        graph_layout.addWidget(self.graph_combo)

        # Integration Level Selection
        graph_layout.addWidget(QLabel("Integration Level:"))
        self.integration_combo = QComboBox()
        self.integration_combo.addItems(["Raw (Acceleration)", "Velocity", "Position"])
        self.integration_combo.currentIndexChanged.connect(self.on_integration_changed)
        graph_layout.addWidget(self.integration_combo)

        # Detrend option
        self.detrend_checkbox = QCheckBox("Remove DC Offset")
        self.detrend_checkbox.setChecked(False)
        self.detrend_checkbox.stateChanged.connect(self.on_detrend_changed)
        graph_layout.addWidget(self.detrend_checkbox)

        # SFLP View Mode Selection (only visible when SFLP is selected)
        graph_layout.addWidget(QLabel("SFLP View Mode:"))
        self.sflp_view_combo = QComboBox()
        self.sflp_view_combo.addItems(["Quaternion (w,x,y,z)", "Euler Angles (roll,pitch,yaw)"])
        self.sflp_view_combo.currentIndexChanged.connect(self.on_sflp_view_changed)
        graph_layout.addWidget(self.sflp_view_combo)
        self.sflp_view_combo.setVisible(False)  # Hidden by default

        graph_group.setLayout(graph_layout)
        layout.addWidget(graph_group)

        # Filtering Group
        filtering_group = self.create_filtering_controls()
        layout.addWidget(filtering_group)

        # Calibration Group
        calibration_group = self.create_calibration_controls()
        layout.addWidget(calibration_group)

        # Control Buttons Group
        controls_group = QGroupBox("Controls")
        controls_layout = QVBoxLayout()

        self.clear_btn = QPushButton("Clear Data")
        self.clear_btn.clicked.connect(self.clear_data)
        controls_layout.addWidget(self.clear_btn)

        controls_group.setLayout(controls_layout)
        layout.addWidget(controls_group)

        layout.addStretch()

        return panel

    def create_plot_area(self) -> QWidget:
        """Create the plotting area."""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Create matplotlib figure
        self.figure = Figure(figsize=(10, 8))
        self.canvas = FigureCanvas(self.figure)
        self.toolbar = NavigationToolbar(self.canvas, widget)

        layout.addWidget(self.toolbar)
        layout.addWidget(self.canvas)

        # Initialize plot axes
        self.init_plots()

        return widget

    def create_statistics_panel(self) -> QWidget:
        """Create the statistics display panel."""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # IMU Status Group
        status_group = QGroupBox("IMU Status")
        status_layout = QVBoxLayout()

        self.lsm6dsv_status_label = QLabel("LSM6DSV: Unknown")
        self.lsm6dsox_status_label = QLabel("LSM6DSOX: Unknown")
        self.lis2dw12_status_label = QLabel("LIS2DW12: Unknown")
        self.adxl362_status_label = QLabel("ADXL362: Unknown")
        self.lsm6dsv_sflp_status_label = QLabel("LSM6DSV SFLP: Unknown")

        font = QFont()
        font.setBold(True)
        self.lsm6dsv_status_label.setFont(font)
        self.lsm6dsox_status_label.setFont(font)
        self.lis2dw12_status_label.setFont(font)
        self.adxl362_status_label.setFont(font)
        self.lsm6dsv_sflp_status_label.setFont(font)

        status_layout.addWidget(self.lsm6dsv_status_label)
        status_layout.addWidget(self.lsm6dsv_sflp_status_label)
        status_layout.addWidget(self.lsm6dsox_status_label)
        status_layout.addWidget(self.lis2dw12_status_label)
        status_layout.addWidget(self.adxl362_status_label)
        status_group.setLayout(status_layout)

        layout.addWidget(status_group)

        # Statistics Group
        stats_group = QGroupBox("Statistics")
        stats_layout = QVBoxLayout()

        # Create text display for statistics
        self.stats_text = QTextEdit()
        self.stats_text.setReadOnly(True)
        self.stats_text.setMaximumHeight(600)
        font = QFont("Courier", 9)
        self.stats_text.setFont(font)

        stats_layout.addWidget(self.stats_text)
        stats_group.setLayout(stats_layout)

        layout.addWidget(stats_group)

        return widget

    def create_filtering_controls(self) -> QGroupBox:
        """Create filtering control group."""
        filtering_group = QGroupBox("Signal Filtering")
        filtering_layout = QVBoxLayout()

        # Filter type selection
        filtering_layout.addWidget(QLabel("Filter Type:"))
        self.filter_combo = QComboBox()
        self.filter_combo.addItems(["None", "Low-pass", "High-pass", "Moving Average", "Savitzky-Golay"])
        self.filter_combo.currentTextChanged.connect(self.on_filter_changed)
        filtering_layout.addWidget(self.filter_combo)

        # Low-pass filter controls
        self.lowpass_widget = QWidget()
        lowpass_layout = QVBoxLayout()
        lowpass_layout.setContentsMargins(0, 0, 0, 0)

        lowpass_layout.addWidget(QLabel("Cutoff Frequency (Hz):"))
        self.lowpass_cutoff_spin = QDoubleSpinBox()
        self.lowpass_cutoff_spin.setRange(0.1, 100.0)
        self.lowpass_cutoff_spin.setValue(10.0)
        self.lowpass_cutoff_spin.setSingleStep(0.5)
        self.lowpass_cutoff_spin.valueChanged.connect(self.on_lowpass_params_changed)
        lowpass_layout.addWidget(self.lowpass_cutoff_spin)

        lowpass_layout.addWidget(QLabel("Filter Order:"))
        self.lowpass_order_spin = QSpinBox()
        self.lowpass_order_spin.setRange(2, 8)
        self.lowpass_order_spin.setValue(4)
        self.lowpass_order_spin.valueChanged.connect(self.on_lowpass_params_changed)
        lowpass_layout.addWidget(self.lowpass_order_spin)

        self.lowpass_widget.setLayout(lowpass_layout)
        self.lowpass_widget.setVisible(False)
        filtering_layout.addWidget(self.lowpass_widget)

        # High-pass filter controls
        self.highpass_widget = QWidget()
        highpass_layout = QVBoxLayout()
        highpass_layout.setContentsMargins(0, 0, 0, 0)

        highpass_layout.addWidget(QLabel("Cutoff Frequency (Hz):"))
        self.highpass_cutoff_spin = QDoubleSpinBox()
        self.highpass_cutoff_spin.setRange(0.01, 10.0)
        self.highpass_cutoff_spin.setValue(0.5)
        self.highpass_cutoff_spin.setSingleStep(0.1)
        self.highpass_cutoff_spin.valueChanged.connect(self.on_highpass_params_changed)
        highpass_layout.addWidget(self.highpass_cutoff_spin)

        highpass_layout.addWidget(QLabel("Filter Order:"))
        self.highpass_order_spin = QSpinBox()
        self.highpass_order_spin.setRange(2, 8)
        self.highpass_order_spin.setValue(4)
        self.highpass_order_spin.valueChanged.connect(self.on_highpass_params_changed)
        highpass_layout.addWidget(self.highpass_order_spin)

        self.highpass_widget.setLayout(highpass_layout)
        self.highpass_widget.setVisible(False)
        filtering_layout.addWidget(self.highpass_widget)

        # Moving average filter controls
        self.moving_avg_widget = QWidget()
        moving_avg_layout = QVBoxLayout()
        moving_avg_layout.setContentsMargins(0, 0, 0, 0)

        moving_avg_layout.addWidget(QLabel("Window Size:"))
        self.moving_avg_slider = QSlider(Qt.Horizontal)
        self.moving_avg_slider.setRange(3, 51)
        self.moving_avg_slider.setValue(5)
        self.moving_avg_slider.setSingleStep(2)
        self.moving_avg_slider.valueChanged.connect(self.on_moving_avg_params_changed)
        moving_avg_layout.addWidget(self.moving_avg_slider)

        self.moving_avg_label = QLabel("5 samples")
        moving_avg_layout.addWidget(self.moving_avg_label)

        self.moving_avg_widget.setLayout(moving_avg_layout)
        self.moving_avg_widget.setVisible(False)
        filtering_layout.addWidget(self.moving_avg_widget)

        # Savitzky-Golay filter controls
        self.savgol_widget = QWidget()
        savgol_layout = QVBoxLayout()
        savgol_layout.setContentsMargins(0, 0, 0, 0)

        savgol_layout.addWidget(QLabel("Window Size:"))
        self.savgol_window_slider = QSlider(Qt.Horizontal)
        self.savgol_window_slider.setRange(3, 51)
        self.savgol_window_slider.setValue(5)
        self.savgol_window_slider.setSingleStep(2)
        self.savgol_window_slider.valueChanged.connect(self.on_savgol_params_changed)
        savgol_layout.addWidget(self.savgol_window_slider)

        self.savgol_window_label = QLabel("5 samples")
        savgol_layout.addWidget(self.savgol_window_label)

        savgol_layout.addWidget(QLabel("Polynomial Order:"))
        self.savgol_order_spin = QSpinBox()
        self.savgol_order_spin.setRange(1, 5)
        self.savgol_order_spin.setValue(2)
        self.savgol_order_spin.valueChanged.connect(self.on_savgol_params_changed)
        savgol_layout.addWidget(self.savgol_order_spin)

        self.savgol_widget.setLayout(savgol_layout)
        self.savgol_widget.setVisible(False)
        filtering_layout.addWidget(self.savgol_widget)

        filtering_group.setLayout(filtering_layout)
        return filtering_group

    def create_calibration_controls(self) -> QGroupBox:
        """Create calibration control group."""
        calibration_group = QGroupBox("Calibration")
        calibration_layout = QVBoxLayout()

        # Calibration duration control
        duration_layout = QHBoxLayout()
        duration_layout.addWidget(QLabel("Duration:"))
        self.calibration_duration_spin = QDoubleSpinBox()
        self.calibration_duration_spin.setRange(0.5, 10.0)
        self.calibration_duration_spin.setValue(1.0)
        self.calibration_duration_spin.setSingleStep(0.5)
        self.calibration_duration_spin.setSuffix(" sec")
        self.calibration_duration_spin.setToolTip("How long to capture samples for calibration")
        self.calibration_duration_spin.valueChanged.connect(self.on_calibration_duration_changed)
        duration_layout.addWidget(self.calibration_duration_spin)
        duration_layout.addStretch()
        calibration_layout.addLayout(duration_layout)

        # Calibration mode selection
        calibration_layout.addWidget(QLabel("Mode:"))
        self.calibration_mode_group = QButtonGroup()

        self.capture_mode_radio = QRadioButton("Capture new samples")
        self.capture_mode_radio.setChecked(True)
        self.capture_mode_radio.setToolTip("Collect samples over time for calibration")
        self.calibration_mode_group.addButton(self.capture_mode_radio, 0)
        calibration_layout.addWidget(self.capture_mode_radio)

        self.existing_mode_radio = QRadioButton("Use all existing data")
        self.existing_mode_radio.setToolTip("Calculate calibration from all samples already in memory (instant)")
        self.calibration_mode_group.addButton(self.existing_mode_radio, 1)
        calibration_layout.addWidget(self.existing_mode_radio)

        self.calibration_mode_group.buttonClicked.connect(self.on_calibration_mode_changed)

        # Calibrate button
        self.calibrate_btn = QPushButton("Calibrate Current Sensor")
        self.calibrate_btn.setToolTip("Hold device still and click to calibrate")
        self.calibrate_btn.clicked.connect(self.start_calibration)
        calibration_layout.addWidget(self.calibrate_btn)

        # Apply calibration checkbox
        self.apply_calibration_checkbox = QCheckBox("Apply Calibration")
        self.apply_calibration_checkbox.setChecked(False)
        self.apply_calibration_checkbox.setToolTip("Enable/disable bias removal from calibrated sensors")
        self.apply_calibration_checkbox.stateChanged.connect(self.on_calibration_toggle)
        calibration_layout.addWidget(self.apply_calibration_checkbox)

        # Calibration status label
        self.calibration_status_label = QLabel("No sensors calibrated")
        self.calibration_status_label.setWordWrap(True)
        font = QFont()
        font.setPointSize(8)
        self.calibration_status_label.setFont(font)
        calibration_layout.addWidget(self.calibration_status_label)

        # Progress label (hidden by default)
        self.calibration_progress_label = QLabel("Calibrating... 0%")
        self.calibration_progress_label.setVisible(False)
        calibration_layout.addWidget(self.calibration_progress_label)

        # Clear calibration button
        self.clear_calibration_btn = QPushButton("Clear All Calibrations")
        self.clear_calibration_btn.clicked.connect(self.clear_all_calibrations)
        calibration_layout.addWidget(self.clear_calibration_btn)

        calibration_group.setLayout(calibration_layout)
        return calibration_group

    def init_plots(self):
        """Initialize plot axes based on current graph type."""
        self.figure.clear()

        if self.current_graph == "XYZ":
            # Three stacked subplots for X, Y, Z with shared x-axis
            self.ax1 = self.figure.add_subplot(3, 1, 1)
            self.ax2 = self.figure.add_subplot(3, 1, 2, sharex=self.ax1)
            self.ax3 = self.figure.add_subplot(3, 1, 3, sharex=self.ax1)

            self.ax1.set_ylabel('X')
            self.ax2.set_ylabel('Y')
            self.ax3.set_ylabel('Z')
            self.ax3.set_xlabel('Time (μs)')

            self.ax1.grid(True, alpha=0.3)
            self.ax2.grid(True, alpha=0.3)
            self.ax3.grid(True, alpha=0.3)

        elif self.current_graph == "MAGNITUDE":
            # Single plot for magnitude
            self.ax_mag = self.figure.add_subplot(1, 1, 1)
            self.ax_mag.set_xlabel('Time (μs)')
            self.ax_mag.set_ylabel('Magnitude')
            self.ax_mag.grid(True, alpha=0.3)

        elif self.current_graph == "3D":
            # 3D trajectory plot
            self.ax_3d = self.figure.add_subplot(1, 1, 1, projection='3d')
            self.ax_3d.set_xlabel('X')
            self.ax_3d.set_ylabel('Y')
            self.ax_3d.set_zlabel('Z')

        self.figure.tight_layout()
        self.canvas.draw()

    def setup_connections(self):
        """Setup signal/slot connections."""
        self.serial_reader.lsm6dsv_data_received.connect(self.on_lsm6dsv_data_received)
        self.serial_reader.lsm6dsox_data_received.connect(self.on_lsm6dsox_data_received)
        self.serial_reader.lis2dw12_data_received.connect(self.on_lis2dw12_data_received)
        self.serial_reader.adxl362_data_received.connect(self.on_adxl362_data_received)
        self.serial_reader.lsm6dsv_sflp_data_received.connect(self.on_lsm6dsv_sflp_data_received)
        self.serial_reader.error_occurred.connect(self.on_serial_error)
        self.serial_reader.connection_status.connect(self.on_connection_status_changed)

    def refresh_ports(self):
        """Refresh the list of available serial ports."""
        ports = SerialReader.get_available_ports()
        self.port_combo.clear()
        self.port_combo.addItems(ports)

    def on_baud_rate_changed(self, baud_str: str):
        """Handle baud rate selection change."""
        self.baud_rate = int(baud_str)

    def load_csv_file(self):
        """Load data from a CSV file."""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "Open CSV File", "", "CSV Files (*.csv);;All Files (*)"
        )

        if file_path:
            success = CSVLoader.load_csv(file_path, self.imu_data)
            if success:
                self.statusBar().showMessage(f"Loaded {self.imu_data.sample_count} samples from {file_path}", 3000)
                self.update_imu_status()
                self.update_plots()
                self.update_statistics()
            else:
                self.statusBar().showMessage("Error loading CSV file", 3000)

    def toggle_serial_connection(self):
        """Toggle serial port connection."""
        if not self.is_streaming:
            # Start streaming
            port = self.port_combo.currentText()
            if port:
                self.serial_reader.set_port(port, self.baud_rate)
                self.serial_reader.start()
                self.is_streaming = True
                self.connect_btn.setText("Disconnect")
                self.connect_btn.setEnabled(False)  # Disable during connection
                self.statusBar().showMessage(f"Connecting to {port} at {self.baud_rate} baud...", 2000)
        else:
            # Stop streaming
            self.connect_btn.setEnabled(False)
            self.statusBar().showMessage("Disconnecting...", 2000)
            if self.serial_reader.isRunning():
                self.serial_reader.stop()
            self.is_streaming = False
            self.connect_btn.setText("Connect")
            self.connect_btn.setEnabled(True)

    def on_lsm6dsv_data_received(self, timestamp, ax, ay, az, gx, gy, gz):
        """Handle received LSM6DSV serial data."""
        self.imu_data.add_lsm6dsv_sample(timestamp, ax, ay, az, gx, gy, gz)

    def on_lsm6dsox_data_received(self, timestamp, ax, ay, az, gx, gy, gz):
        """Handle received LSM6DSOX serial data."""
        self.imu_data.add_lsm6dsox_sample(timestamp, ax, ay, az, gx, gy, gz)

    def on_lis2dw12_data_received(self, timestamp, ax, ay, az):
        """Handle received LIS2DW12 serial data."""
        self.imu_data.add_lis2dw12_sample(timestamp, ax, ay, az)

    def on_adxl362_data_received(self, timestamp, ax, ay, az):
        """Handle received ADXL362 serial data."""
        self.imu_data.add_adxl362_sample(timestamp, ax, ay, az)

    def on_lsm6dsv_sflp_data_received(self, timestamp, qw, qx, qy, qz):
        """Handle received LSM6DSV SFLP serial data."""
        self.imu_data.add_lsm6dsv_sflp_sample(timestamp, qw, qx, qy, qz)

    def update_imu_status(self):
        """Update IMU status labels based on available data."""
        # Update LSM6DSV status
        if self.imu_data.lsm6dsv_available:
            self.lsm6dsv_status_label.setText(f"LSM6DSV: Active ({self.imu_data.lsm_sample_count} samples)")
            self.lsm6dsv_status_label.setStyleSheet("color: green;")
            self.lsm_accel_radio.setEnabled(True)
            self.lsm_gyro_radio.setEnabled(True)
            self.lsm_both_radio.setEnabled(True)
        else:
            self.lsm6dsv_status_label.setText("LSM6DSV: Not Detected")
            self.lsm6dsv_status_label.setStyleSheet("color: gray;")
            self.lsm_accel_radio.setEnabled(False)
            self.lsm_gyro_radio.setEnabled(False)
            self.lsm_both_radio.setEnabled(False)

        # Update LSM6DSOX status
        if self.imu_data.lsm6dsox_available:
            self.lsm6dsox_status_label.setText(f"LSM6DSOX: Active ({self.imu_data.lsmox_sample_count} samples)")
            self.lsm6dsox_status_label.setStyleSheet("color: green;")
            self.lsmox_accel_radio.setEnabled(True)
            self.lsmox_gyro_radio.setEnabled(True)
        else:
            self.lsm6dsox_status_label.setText("LSM6DSOX: Not Detected")
            self.lsm6dsox_status_label.setStyleSheet("color: gray;")
            self.lsmox_accel_radio.setEnabled(False)
            self.lsmox_gyro_radio.setEnabled(False)

        # Update LIS2DW12 status
        if self.imu_data.lis2dw12_available:
            self.lis2dw12_status_label.setText(f"LIS2DW12: Active ({self.imu_data.lis_sample_count} samples)")
            self.lis2dw12_status_label.setStyleSheet("color: green;")
            self.lis_radio.setEnabled(True)
        else:
            self.lis2dw12_status_label.setText("LIS2DW12: Not Detected")
            self.lis2dw12_status_label.setStyleSheet("color: gray;")
            self.lis_radio.setEnabled(False)

        # Update ADXL362 status
        if self.imu_data.adxl362_available:
            self.adxl362_status_label.setText(f"ADXL362: Active ({self.imu_data.adxl_sample_count} samples)")
            self.adxl362_status_label.setStyleSheet("color: green;")
            self.adxl_radio.setEnabled(True)
        else:
            self.adxl362_status_label.setText("ADXL362: Not Detected")
            self.adxl362_status_label.setStyleSheet("color: gray;")
            self.adxl_radio.setEnabled(False)

        # Update LSM6DSV SFLP status
        if self.imu_data.lsm6dsv_sflp_available:
            self.lsm6dsv_sflp_status_label.setText(f"LSM6DSV SFLP: Active ({self.imu_data.lsm_sflp_sample_count} samples)")
            self.lsm6dsv_sflp_status_label.setStyleSheet("color: green;")
            self.lsm_sflp_radio.setEnabled(True)
        else:
            self.lsm6dsv_sflp_status_label.setText("LSM6DSV SFLP: Not Detected")
            self.lsm6dsv_sflp_status_label.setStyleSheet("color: gray;")
            self.lsm_sflp_radio.setEnabled(False)

    def on_serial_error(self, error_msg: str):
        """Handle serial errors."""
        self.statusBar().showMessage(f"Error: {error_msg}", 5000)

    def on_connection_status_changed(self, connected: bool):
        """Handle connection status changes."""
        if connected:
            self.connect_btn.setText("Disconnect")
            self.connect_btn.setEnabled(True)
            self.statusBar().showMessage("Connected", 2000)
        else:
            self.connect_btn.setText("Connect")
            self.connect_btn.setEnabled(True)
            if self.is_streaming:
                self.statusBar().showMessage("Connection lost", 3000)
            self.is_streaming = False

    def on_imu_changed(self):
        """Handle IMU selection change."""
        if self.lsm_accel_radio.isChecked():
            self.current_imu = "LSM6DSV_ACCEL"
        elif self.lsm_gyro_radio.isChecked():
            self.current_imu = "LSM6DSV_GYRO"
        elif self.lsm_both_radio.isChecked():
            self.current_imu = "LSM6DSV_BOTH"
        elif self.lsm_sflp_radio.isChecked():
            self.current_imu = "LSM6DSV_SFLP"
        elif self.lsmox_accel_radio.isChecked():
            self.current_imu = "LSM6DSOX_ACCEL"
        elif self.lsmox_gyro_radio.isChecked():
            self.current_imu = "LSM6DSOX_GYRO"
        elif self.lis_radio.isChecked():
            self.current_imu = "LIS2DW12"
        else:
            self.current_imu = "ADXL362"

        # Show/hide SFLP view mode selector
        if self.current_imu == "LSM6DSV_SFLP":
            self.sflp_view_combo.setVisible(True)
        else:
            self.sflp_view_combo.setVisible(False)

        # Update integration options based on sensor type
        self.update_integration_options()

        self.update_plots()
        self.update_statistics()

    def on_graph_changed(self, graph_type: str):
        """Handle graph type change."""
        if graph_type == "X, Y, Z vs Time":
            self.current_graph = "XYZ"
        elif graph_type == "Magnitude vs Time":
            self.current_graph = "MAGNITUDE"
        else:
            self.current_graph = "3D"

        self.init_plots()
        self.update_plots()

    def on_integration_changed(self, index: int):
        """Handle integration level change."""
        self.integration_level = index
        self.update_plots()
        self.update_statistics()

    def on_detrend_changed(self, state):
        """Handle detrend checkbox change."""
        self.detrend_enabled = (state == Qt.Checked)
        self.update_plots()
        self.update_statistics()

    def on_sflp_view_changed(self, index):
        """Handle SFLP view mode change."""
        if index == 0:
            self.sflp_view_mode = "QUATERNION"
        else:
            self.sflp_view_mode = "EULER"
        self.update_plots()
        self.update_statistics()

    def on_filter_changed(self, filter_name: str):
        """Handle filter type change."""
        self.filter_type = filter_name

        # Show/hide appropriate filter controls
        self.lowpass_widget.setVisible(filter_name == "Low-pass")
        self.highpass_widget.setVisible(filter_name == "High-pass")
        self.moving_avg_widget.setVisible(filter_name == "Moving Average")
        self.savgol_widget.setVisible(filter_name == "Savitzky-Golay")

        # Update plots with new filter
        self.update_plots()
        self.update_statistics()

    def on_lowpass_params_changed(self):
        """Handle low-pass filter parameter changes."""
        self.lowpass_cutoff = self.lowpass_cutoff_spin.value()
        self.lowpass_order = self.lowpass_order_spin.value()
        if self.filter_type == "Low-pass":
            self.update_plots()
            self.update_statistics()

    def on_highpass_params_changed(self):
        """Handle high-pass filter parameter changes."""
        self.highpass_cutoff = self.highpass_cutoff_spin.value()
        self.highpass_order = self.highpass_order_spin.value()
        if self.filter_type == "High-pass":
            self.update_plots()
            self.update_statistics()

    def on_moving_avg_params_changed(self):
        """Handle moving average filter parameter changes."""
        window = self.moving_avg_slider.value()
        # Ensure odd window size
        if window % 2 == 0:
            window += 1
            self.moving_avg_slider.setValue(window)
        self.moving_avg_window = window
        self.moving_avg_label.setText(f"{window} samples")
        if self.filter_type == "Moving Average":
            self.update_plots()
            self.update_statistics()

    def on_savgol_params_changed(self):
        """Handle Savitzky-Golay filter parameter changes."""
        window = self.savgol_window_slider.value()
        # Ensure odd window size
        if window % 2 == 0:
            window += 1
            self.savgol_window_slider.setValue(window)
        self.savgol_window = window
        self.savgol_order = self.savgol_order_spin.value()
        self.savgol_window_label.setText(f"{window} samples")

        # Ensure poly order is less than window size
        if self.savgol_order >= self.savgol_window:
            self.savgol_order = self.savgol_window - 1
            self.savgol_order_spin.setValue(self.savgol_order)

        if self.filter_type == "Savitzky-Golay":
            self.update_plots()
            self.update_statistics()

    def start_calibration(self):
        """Start calibration for the current sensor."""
        # Determine sensor name from current selection
        sensor_map = {
            "LSM6DSV_ACCEL": "LSM6DSV",
            "LSM6DSV_GYRO": "LSM6DSV",
            "LSM6DSV_BOTH": "LSM6DSV",
            "LSM6DSOX_ACCEL": "LSM6DSOX",
            "LSM6DSOX_GYRO": "LSM6DSOX",
            "LIS2DW12": "LIS2DW12",
            "ADXL362": "ADXL362"
        }

        # Don't calibrate sensor fusion - it doesn't need it
        if self.current_imu == "LSM6DSV_SFLP":
            self.statusBar().showMessage("Sensor fusion doesn't require calibration", 3000)
            return

        sensor_name = sensor_map.get(self.current_imu)
        if not sensor_name:
            self.statusBar().showMessage("Cannot calibrate this sensor", 3000)
            return

        # Check if sensor has data
        if sensor_name == "LSM6DSV" and self.imu_data.lsm_sample_count < 10:
            self.statusBar().showMessage("LSM6DSV: Insufficient data. Ensure sensor is streaming.", 3000)
            return
        elif sensor_name == "LSM6DSOX" and self.imu_data.lsmox_sample_count < 10:
            self.statusBar().showMessage("LSM6DSOX: Insufficient data. Ensure sensor is streaming.", 3000)
            return
        elif sensor_name == "LIS2DW12" and self.imu_data.lis_sample_count < 10:
            self.statusBar().showMessage("LIS2DW12: Insufficient data. Ensure sensor is streaming.", 3000)
            return
        elif sensor_name == "ADXL362" and self.imu_data.adxl_sample_count < 10:
            self.statusBar().showMessage("ADXL362: Insufficient data. Ensure sensor is streaming.", 3000)
            return

        # Handle based on calibration mode
        if self.calibration_mode == "existing":
            # Use all existing data - instant calibration
            self.calibrate_from_existing_data(sensor_name)
        else:
            # Capture new samples over time
            # Disable button during calibration
            self.calibrate_btn.setEnabled(False)
            self.calibration_progress_label.setVisible(True)
            self.calibration_progress_label.setText("Calibrating... 0%")
            self.statusBar().showMessage(f"Calibrating for {self.calibration_duration}s... Keep device still!", 5000)

            # Create and start calibration thread with configurable duration
            self.calibration_thread = CalibrationCapture(
                self.imu_data,
                sensor_name,
                self.current_imu,
                capture_duration=self.calibration_duration
            )
            self.calibration_thread.calibration_complete.connect(self.on_calibration_complete)
            self.calibration_thread.calibration_progress.connect(self.on_calibration_progress)
            self.calibration_thread.calibration_error.connect(self.on_calibration_error)
            self.calibration_thread.start()

    def on_calibration_complete(self, sensor_name: str, accel_bias: tuple, gyro_bias: tuple):
        """Handle calibration completion."""
        # Store calibration data
        self.imu_data.set_calibration(sensor_name, accel_bias, gyro_bias)

        # Update UI
        self.calibrate_btn.setEnabled(True)
        self.calibration_progress_label.setVisible(False)
        self.update_calibration_status()

        # Show completion message
        msg = f"{sensor_name} calibrated:\n"
        msg += f"  Accel bias: X={accel_bias[0]:.1f}, Y={accel_bias[1]:.1f}, Z={accel_bias[2]:.1f} mg"
        if gyro_bias:
            msg += f"\n  Gyro bias: X={gyro_bias[0]:.1f}, Y={gyro_bias[1]:.1f}, Z={gyro_bias[2]:.1f} mdps"
        self.statusBar().showMessage(msg, 5000)

        # Update plots if calibration is enabled
        if self.calibration_enabled:
            self.update_plots()
            self.update_statistics()

    def on_calibration_progress(self, progress: int):
        """Handle calibration progress updates."""
        self.calibration_progress_label.setText(f"Calibrating... {progress}%")

    def on_calibration_error(self, error_msg: str):
        """Handle calibration errors."""
        self.calibrate_btn.setEnabled(True)
        self.calibration_progress_label.setVisible(False)
        self.statusBar().showMessage(f"Calibration error: {error_msg}", 5000)

    def on_calibration_toggle(self, state):
        """Handle calibration enable/disable toggle."""
        self.calibration_enabled = (state == Qt.Checked)
        self.update_plots()
        self.update_statistics()

    def clear_all_calibrations(self):
        """Clear all calibration data."""
        self.imu_data.clear_calibration()
        self.update_calibration_status()
        self.statusBar().showMessage("All calibrations cleared", 3000)
        if self.calibration_enabled:
            self.update_plots()
            self.update_statistics()

    def update_calibration_status(self):
        """Update the calibration status label."""
        calibrated = self.imu_data.get_calibrated_sensors()
        if calibrated:
            status = "Calibrated: " + ", ".join(calibrated)
        else:
            status = "No sensors calibrated"
        self.calibration_status_label.setText(status)

    def on_calibration_duration_changed(self, value):
        """Handle calibration duration change."""
        self.calibration_duration = value

    def on_calibration_mode_changed(self):
        """Handle calibration mode change."""
        if self.capture_mode_radio.isChecked():
            self.calibration_mode = "capture"
            self.calibration_duration_spin.setEnabled(True)
        else:
            self.calibration_mode = "existing"
            self.calibration_duration_spin.setEnabled(False)

    def calibrate_from_existing_data(self, sensor_name: str):
        """
        Calibrate using all existing data in memory.

        Args:
            sensor_name: Sensor name (LSM6DSV, LSM6DSOX, LIS2DW12, ADXL362)
        """
        import numpy as np

        try:
            # Get all data for the sensor
            accel_x = []
            accel_y = []
            accel_z = []
            gyro_x = []
            gyro_y = []
            gyro_z = []

            if sensor_name == 'LSM6DSV':
                if len(self.imu_data.lsm_accel_x) < 10:
                    self.statusBar().showMessage(f"Insufficient data: {len(self.imu_data.lsm_accel_x)} samples (need at least 10)", 5000)
                    return
                accel_x = list(self.imu_data.lsm_accel_x)
                accel_y = list(self.imu_data.lsm_accel_y)
                accel_z = list(self.imu_data.lsm_accel_z)
                gyro_x = list(self.imu_data.lsm_gyro_x)
                gyro_y = list(self.imu_data.lsm_gyro_y)
                gyro_z = list(self.imu_data.lsm_gyro_z)

            elif sensor_name == 'LSM6DSOX':
                if len(self.imu_data.lsmox_accel_x) < 10:
                    self.statusBar().showMessage(f"Insufficient data: {len(self.imu_data.lsmox_accel_x)} samples (need at least 10)", 5000)
                    return
                accel_x = list(self.imu_data.lsmox_accel_x)
                accel_y = list(self.imu_data.lsmox_accel_y)
                accel_z = list(self.imu_data.lsmox_accel_z)
                gyro_x = list(self.imu_data.lsmox_gyro_x)
                gyro_y = list(self.imu_data.lsmox_gyro_y)
                gyro_z = list(self.imu_data.lsmox_gyro_z)

            elif sensor_name == 'LIS2DW12':
                if len(self.imu_data.lis_accel_x) < 10:
                    self.statusBar().showMessage(f"Insufficient data: {len(self.imu_data.lis_accel_x)} samples (need at least 10)", 5000)
                    return
                accel_x = list(self.imu_data.lis_accel_x)
                accel_y = list(self.imu_data.lis_accel_y)
                accel_z = list(self.imu_data.lis_accel_z)

            elif sensor_name == 'ADXL362':
                if len(self.imu_data.adxl_accel_x) < 10:
                    self.statusBar().showMessage(f"Insufficient data: {len(self.imu_data.adxl_accel_x)} samples (need at least 10)", 5000)
                    return
                accel_x = list(self.imu_data.adxl_accel_x)
                accel_y = list(self.imu_data.adxl_accel_y)
                accel_z = list(self.imu_data.adxl_accel_z)

            # Calculate average bias
            accel_bias = (
                float(np.mean(accel_x)),
                float(np.mean(accel_y)),
                float(np.mean(accel_z))
            )

            gyro_bias = None
            if len(gyro_x) > 0:
                gyro_bias = (
                    float(np.mean(gyro_x)),
                    float(np.mean(gyro_y)),
                    float(np.mean(gyro_z))
                )

            # Store calibration
            self.imu_data.set_calibration(sensor_name, accel_bias, gyro_bias)

            # Update UI
            self.update_calibration_status()

            # Show completion message
            num_samples = len(accel_x)
            msg = f"{sensor_name} calibrated using {num_samples} samples:\n"
            msg += f"  Accel bias: X={accel_bias[0]:.1f}, Y={accel_bias[1]:.1f}, Z={accel_bias[2]:.1f} mg"
            if gyro_bias:
                msg += f"\n  Gyro bias: X={gyro_bias[0]:.1f}, Y={gyro_bias[1]:.1f}, Z={gyro_bias[2]:.1f} mdps"
            self.statusBar().showMessage(msg, 5000)

            # Update plots if calibration is enabled
            if self.calibration_enabled:
                self.update_plots()
                self.update_statistics()

        except Exception as e:
            self.statusBar().showMessage(f"Calibration error: {str(e)}", 5000)

    def apply_calibration_to_data(self, sensor_name: str, x, y, z, is_gyro=False):
        """
        Apply calibration bias removal to data.

        Args:
            sensor_name: Sensor name (LSM6DSV, LSM6DSOX, LIS2DW12, ADXL362)
            x, y, z: Data arrays
            is_gyro: True if data is gyroscope, False if accelerometer

        Returns:
            Calibrated x, y, z arrays
        """
        if not self.calibration_enabled:
            return x, y, z

        cal_data = self.imu_data.get_calibration(sensor_name)
        if not cal_data or not cal_data['calibrated']:
            return x, y, z

        # Get appropriate bias (accel or gyro)
        if is_gyro and 'gyro' in cal_data:
            bias = cal_data['gyro']
        else:
            bias = cal_data['accel']

        # Subtract bias
        import numpy as np
        x_cal = x - bias['x']
        y_cal = y - bias['y']
        z_cal = z - bias['z']

        return x_cal, y_cal, z_cal

    def update_integration_options(self):
        """Update integration combo box options based on selected IMU."""
        # Block signals to avoid triggering updates
        self.integration_combo.blockSignals(True)

        # Clear existing items
        self.integration_combo.clear()

        # Check if current IMU is SFLP (no integration for sensor fusion data)
        if self.current_imu == "LSM6DSV_SFLP":
            # SFLP: Raw quaternion/Euler only
            self.integration_combo.addItems(["Raw (Orientation)"])
            self.integration_combo.setEnabled(False)
            self.integration_level = 0
        elif self.current_imu in ["LSM6DSV_ACCEL", "LSM6DSOX_ACCEL", "LIS2DW12", "ADXL362", "LSM6DSV_BOTH"]:
            # Accelerometer or BOTH: Raw, Velocity, Position
            # For BOTH mode: Level 0 = Raw (Accel + AngVel), Level 1 = Velocity + Angle, Level 2 = Position + Angle
            if self.current_imu == "LSM6DSV_BOTH":
                self.integration_combo.addItems(["Raw (Accel + AngVel)", "Velocity + Angle", "Position + Angle"])
            else:
                self.integration_combo.addItems(["Raw (Acceleration)", "Velocity", "Position"])
            self.integration_combo.setEnabled(True)
        else:
            # Gyroscope: Raw, Angle
            self.integration_combo.addItems(["Raw (Angular Velocity)", "Angle"])
            self.integration_combo.setEnabled(True)

        # Reset to raw if current level is out of bounds
        if self.integration_level >= self.integration_combo.count():
            self.integration_level = 0

        self.integration_combo.setCurrentIndex(self.integration_level)

        # Re-enable signals
        self.integration_combo.blockSignals(False)

    def clear_data(self):
        """Clear all data."""
        self.imu_data.clear()
        self.update_imu_status()
        self.update_plots()
        self.update_statistics()
        self.statusBar().showMessage("Data cleared", 2000)

    def apply_current_filter(self, timestamps, x, y, z, is_quaternion=False):
        """
        Apply the currently selected filter to data.

        Args:
            timestamps: Timestamp array
            x, y, z: Data arrays (or qx, qy, qz for quaternions)
            is_quaternion: If True, expects 4th parameter qw

        Returns:
            Filtered x, y, z arrays (or qw, qx, qy, qz for quaternions)
        """
        if self.filter_type == "None" or len(timestamps) < 2:
            return x, y, z

        # Calculate sample rate
        sample_rate = DataStatistics.calculate_sample_rate(timestamps)
        if sample_rate <= 0:
            return x, y, z

        # Apply appropriate filter
        try:
            if self.filter_type == "Low-pass":
                filtered_x, filtered_y, filtered_z = DataFilter.lowpass_butterworth(
                    x, y, z, sample_rate, self.lowpass_cutoff, self.lowpass_order
                )
            elif self.filter_type == "High-pass":
                filtered_x, filtered_y, filtered_z = DataFilter.highpass_butterworth(
                    x, y, z, sample_rate, self.highpass_cutoff, self.highpass_order
                )
            elif self.filter_type == "Moving Average":
                filtered_x, filtered_y, filtered_z = DataFilter.moving_average(
                    x, y, z, self.moving_avg_window
                )
            elif self.filter_type == "Savitzky-Golay":
                filtered_x, filtered_y, filtered_z = DataFilter.savgol_filter(
                    x, y, z, self.savgol_window, self.savgol_order
                )
            else:
                return x, y, z

            return filtered_x, filtered_y, filtered_z

        except Exception as e:
            # If filtering fails, return original data
            print(f"Filter error: {e}")
            return x, y, z

    def apply_current_filter_to_quaternion(self, timestamps, qw, qx, qy, qz):
        """
        Apply the currently selected filter to quaternion data.

        Args:
            timestamps: Timestamp array
            qw, qx, qy, qz: Quaternion components

        Returns:
            Filtered qw, qx, qy, qz arrays
        """
        if self.filter_type == "None" or len(timestamps) < 2:
            return qw, qx, qy, qz

        # Calculate sample rate
        sample_rate = DataStatistics.calculate_sample_rate(timestamps)
        if sample_rate <= 0:
            return qw, qx, qy, qz

        # Apply appropriate filter using the quaternion helper
        try:
            if self.filter_type == "Low-pass":
                filtered_qw, filtered_qx, filtered_qy, filtered_qz = DataFilter.apply_filter_to_quaternion(
                    qw, qx, qy, qz, 'lowpass',
                    sample_rate=sample_rate, cutoff_freq=self.lowpass_cutoff, order=self.lowpass_order
                )
            elif self.filter_type == "High-pass":
                filtered_qw, filtered_qx, filtered_qy, filtered_qz = DataFilter.apply_filter_to_quaternion(
                    qw, qx, qy, qz, 'highpass',
                    sample_rate=sample_rate, cutoff_freq=self.highpass_cutoff, order=self.highpass_order
                )
            elif self.filter_type == "Moving Average":
                filtered_qw, filtered_qx, filtered_qy, filtered_qz = DataFilter.apply_filter_to_quaternion(
                    qw, qx, qy, qz, 'moving_average',
                    window_size=self.moving_avg_window
                )
            elif self.filter_type == "Savitzky-Golay":
                filtered_qw, filtered_qx, filtered_qy, filtered_qz = DataFilter.apply_filter_to_quaternion(
                    qw, qx, qy, qz, 'savgol',
                    window_size=self.savgol_window, poly_order=self.savgol_order
                )
            else:
                return qw, qx, qy, qz

            return filtered_qw, filtered_qx, filtered_qy, filtered_qz

        except Exception as e:
            # If filtering fails, return original data
            print(f"Filter error: {e}")
            return qw, qx, qy, qz

    def downsample_data(self, timestamps, x, y, z):
        """Downsample data for plotting performance."""
        n = len(timestamps)

        if n <= self.downsample_threshold:
            return timestamps, x, y, z

        # Calculate downsampling factor
        factor = int(np.ceil(n / self.max_plot_points))

        # Downsample using slicing
        return timestamps[::factor], x[::factor], y[::factor], z[::factor]

    def update_plots(self):
        """Update the plots with current data."""
        if self.imu_data.sample_count == 0:
            return

        # Determine if current sensor is accelerometer or gyroscope
        is_accel = self.current_imu in ["LSM6DSV_ACCEL", "LSM6DSOX_ACCEL", "LIS2DW12", "ADXL362"]

        # Get data based on selected IMU
        if self.current_imu == "LSM6DSV_ACCEL":
            if not self.imu_data.lsm6dsv_available:
                return  # No data for this IMU
            timestamps, x, y, z = self.imu_data.get_lsm_accel_data()
            x, y, z = self.apply_calibration_to_data("LSM6DSV", x, y, z, is_gyro=False)
            sensor_name = "LSM6DSV Accelerometer"
        elif self.current_imu == "LSM6DSV_GYRO":
            if not self.imu_data.lsm6dsv_available:
                return  # No data for this IMU
            timestamps, x, y, z = self.imu_data.get_lsm_gyro_data()
            x, y, z = self.apply_calibration_to_data("LSM6DSV", x, y, z, is_gyro=True)
            sensor_name = "LSM6DSV Gyroscope"
        elif self.current_imu == "LSM6DSV_BOTH":
            if not self.imu_data.lsm6dsv_available:
                return  # No data for this IMU
            # Get both accelerometer and gyroscope data
            timestamps_accel, accel_x, accel_y, accel_z = self.imu_data.get_lsm_accel_data()
            timestamps_gyro, gyro_x, gyro_y, gyro_z = self.imu_data.get_lsm_gyro_data()

            # Apply calibration separately
            accel_x, accel_y, accel_z = self.apply_calibration_to_data("LSM6DSV", accel_x, accel_y, accel_z, is_gyro=False)
            gyro_x, gyro_y, gyro_z = self.apply_calibration_to_data("LSM6DSV", gyro_x, gyro_y, gyro_z, is_gyro=True)

            # Apply filtering separately
            accel_x, accel_y, accel_z = self.apply_current_filter(timestamps_accel, accel_x, accel_y, accel_z)
            gyro_x, gyro_y, gyro_z = self.apply_current_filter(timestamps_gyro, gyro_x, gyro_y, gyro_z)

            sensor_name = "LSM6DSV - Both"
        elif self.current_imu == "LSM6DSOX_ACCEL":
            if not self.imu_data.lsm6dsox_available:
                return  # No data for this IMU
            timestamps, x, y, z = self.imu_data.get_lsmox_accel_data()
            x, y, z = self.apply_calibration_to_data("LSM6DSOX", x, y, z, is_gyro=False)
            sensor_name = "LSM6DSOX Accelerometer"
        elif self.current_imu == "LSM6DSOX_GYRO":
            if not self.imu_data.lsm6dsox_available:
                return  # No data for this IMU
            timestamps, x, y, z = self.imu_data.get_lsmox_gyro_data()
            x, y, z = self.apply_calibration_to_data("LSM6DSOX", x, y, z, is_gyro=True)
            sensor_name = "LSM6DSOX Gyroscope"
        elif self.current_imu == "LIS2DW12":
            if not self.imu_data.lis2dw12_available:
                return  # No data for this IMU
            timestamps, x, y, z = self.imu_data.get_lis_accel_data()
            x, y, z = self.apply_calibration_to_data("LIS2DW12", x, y, z, is_gyro=False)
            sensor_name = "LIS2DW12 Accelerometer"
        elif self.current_imu == "LSM6DSV_SFLP":
            if not self.imu_data.lsm6dsv_sflp_available:
                return  # No data for this IMU
            timestamps, qw, qx, qy, qz = self.imu_data.get_lsm_sflp_data()

            # Apply filtering to quaternion data
            qw, qx, qy, qz = self.apply_current_filter_to_quaternion(timestamps, qw, qx, qy, qz)

            # Convert to Euler angles if needed
            if self.sflp_view_mode == "EULER":
                x, y, z = IMUData.quaternion_to_euler(qw, qx, qy, qz)
                sensor_name = "LSM6DSV SFLP (Euler Angles)"
                data_type = "Orientation"
                units = "deg"
            else:
                # For quaternion mode, we'll handle this specially
                sensor_name = "LSM6DSV SFLP (Quaternion)"
                data_type = "Quaternion"
                units = ""

        else:  # ADXL362
            if not self.imu_data.adxl362_available:
                return  # No data for this IMU
            timestamps, x, y, z = self.imu_data.get_adxl362_accel_data()
            x, y, z = self.apply_calibration_to_data("ADXL362", x, y, z, is_gyro=False)
            sensor_name = "ADXL362 Accelerometer"

        # Apply filtering to 3-axis data (before integration)
        # Skip for LSM6DSV_BOTH (already filtered above) and SFLP quaternion mode
        if self.current_imu != "LSM6DSV_SFLP" and self.current_imu != "LSM6DSV_BOTH":
            x, y, z = self.apply_current_filter(timestamps, x, y, z)
        elif self.current_imu == "LSM6DSV_SFLP" and self.sflp_view_mode == "EULER":
            x, y, z = self.apply_current_filter(timestamps, x, y, z)

        # Apply integration based on integration level (skip for SFLP and BOTH)
        if self.current_imu != "LSM6DSV_SFLP" and self.current_imu != "LSM6DSV_BOTH":
            if self.integration_level == 0:
                # Raw data
                if self.current_imu == "LSM6DSV_SFLP":
                    # Already set above for Euler
                    pass
                elif is_accel:
                    data_type = "Acceleration"
                    units = "mg"
                else:
                    data_type = "Angular Velocity"
                    units = "mdps"
            elif self.integration_level == 1:
                # First integration
                if len(timestamps) < 2:
                    return  # Need at least 2 points for integration
                x, y, z = DataIntegrator.integrate_once(timestamps, x, y, z, is_accel=is_accel, detrend=self.detrend_enabled)
                if is_accel:
                    data_type = "Velocity"
                    units = "m/s"
                else:
                    data_type = "Angle"
                    units = "deg"
            else:  # integration_level == 2 (only for accelerometers)
                # Second integration (position)
                if len(timestamps) < 2:
                    return  # Need at least 2 points for integration
                x, y, z = DataIntegrator.integrate_twice(timestamps, x, y, z, detrend=self.detrend_enabled)
                data_type = "Position"
                units = "m"
        elif self.current_imu == "LSM6DSV_SFLP" and self.sflp_view_mode == "EULER":
            # Integration already applied for SFLP Euler mode
            pass

        # Apply integration for LSM6DSV_BOTH mode (separately for accel and gyro)
        if self.current_imu == "LSM6DSV_BOTH":
            if self.integration_level == 0:
                # Raw data
                accel_data_type = "Acceleration"
                accel_units = "mg"
                gyro_data_type = "Angular Velocity"
                gyro_units = "mdps"
            elif self.integration_level == 1:
                # First integration
                if len(timestamps_accel) >= 2:
                    accel_x, accel_y, accel_z = DataIntegrator.integrate_once(
                        timestamps_accel, accel_x, accel_y, accel_z, is_accel=True, detrend=self.detrend_enabled)
                accel_data_type = "Velocity"
                accel_units = "m/s"

                if len(timestamps_gyro) >= 2:
                    gyro_x, gyro_y, gyro_z = DataIntegrator.integrate_once(
                        timestamps_gyro, gyro_x, gyro_y, gyro_z, is_accel=False, detrend=self.detrend_enabled)
                gyro_data_type = "Angle"
                gyro_units = "deg"
            else:  # integration_level == 2
                # Second integration (only for accel)
                if len(timestamps_accel) >= 2:
                    accel_x, accel_y, accel_z = DataIntegrator.integrate_twice(
                        timestamps_accel, accel_x, accel_y, accel_z, detrend=self.detrend_enabled)
                accel_data_type = "Position"
                accel_units = "m"

                # Gyro can only do first integration
                if len(timestamps_gyro) >= 2:
                    gyro_x, gyro_y, gyro_z = DataIntegrator.integrate_once(
                        timestamps_gyro, gyro_x, gyro_y, gyro_z, is_accel=False, detrend=self.detrend_enabled)
                gyro_data_type = "Angle"
                gyro_units = "deg"

        # Handle quaternion mode specially (plot 4 components instead of 3)
        if self.current_imu == "LSM6DSV_SFLP" and self.sflp_view_mode == "QUATERNION":
            title_suffix = f"{sensor_name}"
        elif self.current_imu == "LSM6DSV_BOTH":
            # BOTH mode creates its own title in the plotting section
            title_suffix = f"{sensor_name}"
        else:
            title_suffix = f"{sensor_name} - {data_type} ({units})"

        # Add filter info to title if active
        if self.filter_type != "None":
            title_suffix += f" [Filter: {self.filter_type}]"

        # Downsample data for performance
        if self.current_imu == "LSM6DSV_SFLP" and self.sflp_view_mode == "QUATERNION":
            # For quaternions, downsample all 4 components
            n = len(timestamps)
            if n > self.downsample_threshold:
                factor = int(np.ceil(n / self.max_plot_points))
                timestamps = timestamps[::factor]
                qw = qw[::factor]
                qx = qx[::factor]
                qy = qy[::factor]
                qz = qz[::factor]
        elif self.current_imu != "LSM6DSV_BOTH":
            # Skip downsampling for BOTH mode (does its own downsampling in plotting sections)
            timestamps, x, y, z = self.downsample_data(timestamps, x, y, z)

        # Convert timestamps to seconds for better readability (skip for BOTH mode)
        if self.current_imu != "LSM6DSV_BOTH":
            if len(timestamps) > 0:
                time_seconds = (timestamps - timestamps[0]) / 1e6
            else:
                return

        # Update based on graph type
        if self.current_graph == "XYZ":
            # Handle LSM6DSV_BOTH mode with 3x2 grid (3 rows, 2 columns)
            if self.current_imu == "LSM6DSV_BOTH":
                # Downsample both datasets
                time_seconds_accel = (timestamps_accel - timestamps_accel[0]) / 1e6 if len(timestamps_accel) > 0 else np.array([])
                time_seconds_gyro = (timestamps_gyro - timestamps_gyro[0]) / 1e6 if len(timestamps_gyro) > 0 else np.array([])

                timestamps_accel, accel_x, accel_y, accel_z = self.downsample_data(timestamps_accel, accel_x, accel_y, accel_z)
                timestamps_gyro, gyro_x, gyro_y, gyro_z = self.downsample_data(timestamps_gyro, gyro_x, gyro_y, gyro_z)

                time_seconds_accel = (timestamps_accel - timestamps_accel[0]) / 1e6 if len(timestamps_accel) > 0 else np.array([])
                time_seconds_gyro = (timestamps_gyro - timestamps_gyro[0]) / 1e6 if len(timestamps_gyro) > 0 else np.array([])

                # Check if we need to create 3x2 grid or update existing
                if not hasattr(self, 'ax_both_1') or not hasattr(self, 'ax_both_6'):
                    # Create 3x2 grid
                    self.figure.clear()
                    # Delete orphaned axes from other modes
                    for ax_name in ['ax1', 'ax2', 'ax3', 'ax4', 'ax_mag', 'ax_3d']:
                        if hasattr(self, ax_name):
                            delattr(self, ax_name)
                    self.ax_both_1 = self.figure.add_subplot(3, 2, 1)  # Accel X
                    self.ax_both_2 = self.figure.add_subplot(3, 2, 2, sharex=self.ax_both_1)  # Gyro X
                    self.ax_both_3 = self.figure.add_subplot(3, 2, 3, sharex=self.ax_both_1)  # Accel Y
                    self.ax_both_4 = self.figure.add_subplot(3, 2, 4, sharex=self.ax_both_1)  # Gyro Y
                    self.ax_both_5 = self.figure.add_subplot(3, 2, 5, sharex=self.ax_both_1)  # Accel Z
                    self.ax_both_6 = self.figure.add_subplot(3, 2, 6, sharex=self.ax_both_1)  # Gyro Z
                else:
                    # Save limits only when not streaming (to preserve zoom for static data)
                    if not self.is_streaming:
                        xlim1, ylim1 = self.ax_both_1.get_xlim(), self.ax_both_1.get_ylim()
                        xlim2, ylim2 = self.ax_both_2.get_xlim(), self.ax_both_2.get_ylim()
                        xlim3, ylim3 = self.ax_both_3.get_xlim(), self.ax_both_3.get_ylim()
                        xlim4, ylim4 = self.ax_both_4.get_xlim(), self.ax_both_4.get_ylim()
                        xlim5, ylim5 = self.ax_both_5.get_xlim(), self.ax_both_5.get_ylim()
                        xlim6, ylim6 = self.ax_both_6.get_xlim(), self.ax_both_6.get_ylim()

                    # Clear axes
                    self.ax_both_1.clear()
                    self.ax_both_2.clear()
                    self.ax_both_3.clear()
                    self.ax_both_4.clear()
                    self.ax_both_5.clear()
                    self.ax_both_6.clear()

                # Plot accel and gyro data
                if len(time_seconds_accel) > 0:
                    self.ax_both_1.plot(time_seconds_accel, accel_x, 'r-', linewidth=0.5)
                    self.ax_both_3.plot(time_seconds_accel, accel_y, 'g-', linewidth=0.5)
                    self.ax_both_5.plot(time_seconds_accel, accel_z, 'b-', linewidth=0.5)

                if len(time_seconds_gyro) > 0:
                    self.ax_both_2.plot(time_seconds_gyro, gyro_x, 'r-', linewidth=0.5)
                    self.ax_both_4.plot(time_seconds_gyro, gyro_y, 'g-', linewidth=0.5)
                    self.ax_both_6.plot(time_seconds_gyro, gyro_z, 'b-', linewidth=0.5)

                # Set labels
                self.ax_both_1.set_ylabel(f'Accel X\n({accel_units})', fontsize=9)
                self.ax_both_2.set_ylabel(f'Gyro X\n({gyro_units})', fontsize=9)
                self.ax_both_3.set_ylabel(f'Accel Y\n({accel_units})', fontsize=9)
                self.ax_both_4.set_ylabel(f'Gyro Y\n({gyro_units})', fontsize=9)
                self.ax_both_5.set_ylabel(f'Accel Z\n({accel_units})', fontsize=9)
                self.ax_both_6.set_ylabel(f'Gyro Z\n({gyro_units})', fontsize=9)
                self.ax_both_5.set_xlabel('Time (s)', fontsize=9)
                self.ax_both_6.set_xlabel('Time (s)', fontsize=9)

                # Set titles for each column
                filter_info = f" [Filter: {self.filter_type}]" if self.filter_type != "None" else ""
                self.ax_both_1.set_title(f"ACCELEROMETER\n{accel_data_type} ({accel_units}){filter_info}", fontsize=10)
                self.ax_both_2.set_title(f"GYROSCOPE\n{gyro_data_type} ({gyro_units}){filter_info}", fontsize=10)

                # Add grids
                self.ax_both_1.grid(True, alpha=0.3)
                self.ax_both_2.grid(True, alpha=0.3)
                self.ax_both_3.grid(True, alpha=0.3)
                self.ax_both_4.grid(True, alpha=0.3)
                self.ax_both_5.grid(True, alpha=0.3)
                self.ax_both_6.grid(True, alpha=0.3)

                # Restore zoom limits only when not streaming
                if not self.is_streaming and 'xlim1' in locals() and xlim1 != (0.0, 1.0):
                    self.ax_both_1.set_xlim(xlim1)
                    self.ax_both_1.set_ylim(ylim1)
                    self.ax_both_2.set_ylim(ylim2)
                    self.ax_both_3.set_ylim(ylim3)
                    self.ax_both_4.set_ylim(ylim4)
                    self.ax_both_5.set_ylim(ylim5)
                    self.ax_both_6.set_ylim(ylim6)

            # Handle quaternion mode with 4 subplots
            elif self.current_imu == "LSM6DSV_SFLP" and self.sflp_view_mode == "QUATERNION":
                # Check if we need to reinitialize or just update
                if not hasattr(self, 'ax4'):
                    # Reinitialize plot with 4 subplots
                    self.figure.clear()
                    # Delete orphaned axes from other modes
                    for i in range(1, 7):
                        attr_name = f'ax_both_{i}'
                        if hasattr(self, attr_name):
                            delattr(self, attr_name)
                    for ax_name in ['ax_mag', 'ax_3d']:
                        if hasattr(self, ax_name):
                            delattr(self, ax_name)
                    self.ax1 = self.figure.add_subplot(4, 1, 1)
                    self.ax2 = self.figure.add_subplot(4, 1, 2, sharex=self.ax1)
                    self.ax3 = self.figure.add_subplot(4, 1, 3, sharex=self.ax1)
                    self.ax4 = self.figure.add_subplot(4, 1, 4, sharex=self.ax1)
                else:
                    # Save current axis limits to preserve zoom (only when not streaming)
                    if not self.is_streaming:
                        xlim1, ylim1 = self.ax1.get_xlim(), self.ax1.get_ylim()
                        xlim2, ylim2 = self.ax2.get_xlim(), self.ax2.get_ylim()
                        xlim3, ylim3 = self.ax3.get_xlim(), self.ax3.get_ylim()
                        xlim4, ylim4 = self.ax4.get_xlim(), self.ax4.get_ylim()

                    # Clear axes
                    self.ax1.clear()
                    self.ax2.clear()
                    self.ax3.clear()
                    self.ax4.clear()

                if len(time_seconds) > 0:
                    self.ax1.plot(time_seconds, qw, 'purple', linewidth=0.5)
                    self.ax2.plot(time_seconds, qx, 'r-', linewidth=0.5)
                    self.ax3.plot(time_seconds, qy, 'g-', linewidth=0.5)
                    self.ax4.plot(time_seconds, qz, 'b-', linewidth=0.5)

                self.ax1.set_ylabel('W')
                self.ax2.set_ylabel('X')
                self.ax3.set_ylabel('Y')
                self.ax4.set_ylabel('Z')
                self.ax4.set_xlabel('Time (s)')
                self.ax1.set_title(title_suffix)

                self.ax1.grid(True, alpha=0.3)
                self.ax2.grid(True, alpha=0.3)
                self.ax3.grid(True, alpha=0.3)
                self.ax4.grid(True, alpha=0.3)

                # Restore axis limits to preserve zoom (only when not streaming)
                if not self.is_streaming and hasattr(self, 'ax4') and 'xlim1' in locals() and xlim1 != (0.0, 1.0):
                    self.ax1.set_xlim(xlim1)
                    self.ax1.set_ylim(ylim1)
                    self.ax2.set_ylim(ylim2)
                    self.ax3.set_ylim(ylim3)
                    self.ax4.set_ylim(ylim4)
            else:
                # Normal 3-axis plot
                if not hasattr(self, 'ax1') or hasattr(self, 'ax4') or hasattr(self, 'ax_both_6'):
                    # Need to reinitialize with 3 subplots
                    self.figure.clear()
                    # Delete orphaned axes from other modes
                    if hasattr(self, 'ax4'):
                        del self.ax4
                    for i in range(1, 7):
                        attr_name = f'ax_both_{i}'
                        if hasattr(self, attr_name):
                            delattr(self, attr_name)
                    self.ax1 = self.figure.add_subplot(3, 1, 1)
                    self.ax2 = self.figure.add_subplot(3, 1, 2, sharex=self.ax1)
                    self.ax3 = self.figure.add_subplot(3, 1, 3, sharex=self.ax1)
                else:
                    # Save current axis limits to preserve zoom (only when not streaming)
                    if not self.is_streaming:
                        xlim1, ylim1 = self.ax1.get_xlim(), self.ax1.get_ylim()
                        xlim2, ylim2 = self.ax2.get_xlim(), self.ax2.get_ylim()
                        xlim3, ylim3 = self.ax3.get_xlim(), self.ax3.get_ylim()

                    # Clear only the plot data, not the axes
                    self.ax1.clear()
                    self.ax2.clear()
                    self.ax3.clear()

                if len(time_seconds) > 0:
                    self.ax1.plot(time_seconds, x, 'r-', linewidth=0.5)
                    self.ax2.plot(time_seconds, y, 'g-', linewidth=0.5)
                    self.ax3.plot(time_seconds, z, 'b-', linewidth=0.5)

                self.ax1.set_ylabel(f'X ({units})')
                self.ax2.set_ylabel(f'Y ({units})')
                self.ax3.set_ylabel(f'Z ({units})')
                self.ax3.set_xlabel('Time (s)')
                self.ax1.set_title(title_suffix)

                self.ax1.grid(True, alpha=0.3)
                self.ax2.grid(True, alpha=0.3)
                self.ax3.grid(True, alpha=0.3)

                # Restore axis limits to preserve zoom (only when not streaming)
                if not self.is_streaming and 'xlim1' in locals() and xlim1 != (0.0, 1.0):
                    self.ax1.set_xlim(xlim1)
                    self.ax1.set_ylim(ylim1)
                    self.ax2.set_ylim(ylim2)
                    self.ax3.set_ylim(ylim3)

        elif self.current_graph == "MAGNITUDE":
            # Save current axis limits to preserve zoom (only when not streaming)
            if not self.is_streaming and hasattr(self, 'ax_mag'):
                xlim_mag, ylim_mag = self.ax_mag.get_xlim(), self.ax_mag.get_ylim()
            else:
                xlim_mag, ylim_mag = None, None

            self.ax_mag.clear()

            if self.current_imu == "LSM6DSV_BOTH":
                # Plot both accelerometer and gyroscope magnitudes
                time_seconds_accel = (timestamps_accel - timestamps_accel[0]) / 1e6 if len(timestamps_accel) > 0 else np.array([])
                time_seconds_gyro = (timestamps_gyro - timestamps_gyro[0]) / 1e6 if len(timestamps_gyro) > 0 else np.array([])

                if len(time_seconds_accel) > 0:
                    accel_magnitude = np.sqrt(accel_x**2 + accel_y**2 + accel_z**2)
                    self.ax_mag.plot(time_seconds_accel, accel_magnitude, 'b-', linewidth=0.8, label=f'Accel ({accel_units})')

                if len(time_seconds_gyro) > 0:
                    gyro_magnitude = np.sqrt(gyro_x**2 + gyro_y**2 + gyro_z**2)
                    self.ax_mag.plot(time_seconds_gyro, gyro_magnitude, 'r-', linewidth=0.8, label=f'Gyro ({gyro_units})')

                self.ax_mag.legend(loc='upper right')
                self.ax_mag.set_ylabel('Magnitude')
            elif len(time_seconds) > 0:
                if self.current_imu == "LSM6DSV_SFLP" and self.sflp_view_mode == "QUATERNION":
                    # Quaternion magnitude (should be ~1.0 if normalized)
                    magnitude = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
                else:
                    magnitude = np.sqrt(x**2 + y**2 + z**2)
                self.ax_mag.plot(time_seconds, magnitude, 'purple', linewidth=0.8)

                if self.current_imu == "LSM6DSV_SFLP" and self.sflp_view_mode == "QUATERNION":
                    self.ax_mag.set_ylabel('Quaternion Magnitude')
                else:
                    self.ax_mag.set_ylabel(f'Magnitude ({units})')

            self.ax_mag.set_xlabel('Time (s)')
            self.ax_mag.set_title(title_suffix)
            self.ax_mag.grid(True, alpha=0.3)

            # Restore axis limits to preserve zoom (only when not streaming)
            if not self.is_streaming and xlim_mag is not None and xlim_mag != (0.0, 1.0):
                self.ax_mag.set_xlim(xlim_mag)
                self.ax_mag.set_ylim(ylim_mag)

        elif self.current_graph == "3D":
            # Clear the entire figure to remove old colorbars
            self.figure.clear()
            self.ax_3d = self.figure.add_subplot(1, 1, 1, projection='3d')

            if self.current_imu == "LSM6DSV_BOTH":
                # Plot both accelerometer and gyroscope trajectories
                if len(accel_x) > 0:
                    colors_accel = np.arange(len(accel_x))
                    scatter_accel = self.ax_3d.scatter(accel_x, accel_y, accel_z,
                                                      c=colors_accel, cmap='Blues',
                                                      s=2, alpha=0.6, label='Accelerometer')

                if len(gyro_x) > 0:
                    colors_gyro = np.arange(len(gyro_x))
                    scatter_gyro = self.ax_3d.scatter(gyro_x, gyro_y, gyro_z,
                                                     c=colors_gyro, cmap='Reds',
                                                     s=2, alpha=0.6, label='Gyroscope')

                self.ax_3d.legend(loc='upper right')
                self.ax_3d.set_xlabel(f'X')
                self.ax_3d.set_ylabel(f'Y')
                self.ax_3d.set_zlabel(f'Z')
                self.ax_3d.set_title(f"{title_suffix} - 3D Trajectory")
            else:
                if len(x) > 0:
                    # Use colormap for time progression
                    colors = np.arange(len(x))
                    scatter = self.ax_3d.scatter(x, y, z, c=colors, cmap='viridis',
                                                s=2, alpha=0.6)

                    # Add colorbar
                    if len(x) > 1:
                        self.figure.colorbar(scatter, ax=self.ax_3d, label='Sample Index', shrink=0.8)

                self.ax_3d.set_xlabel(f'X ({units})')
                self.ax_3d.set_ylabel(f'Y ({units})')
                self.ax_3d.set_zlabel(f'Z ({units})')
                self.ax_3d.set_title(title_suffix)

        self.figure.tight_layout()
        self.canvas.draw()

    def update_statistics(self):
        """Update the statistics display."""
        # Update IMU status labels
        self.update_imu_status()

        if self.imu_data.sample_count == 0:
            self.stats_text.setPlainText("No data available")
            return

        # Determine if current sensor is accelerometer or gyroscope
        is_accel = self.current_imu in ["LSM6DSV_ACCEL", "LSM6DSOX_ACCEL", "LIS2DW12", "ADXL362"]

        # Get data based on selected IMU
        if self.current_imu == "LSM6DSV_ACCEL":
            if not self.imu_data.lsm6dsv_available:
                self.stats_text.setPlainText("LSM6DSV not detected or no data available")
                return
            timestamps, x, y, z = self.imu_data.get_lsm_accel_data()
            x, y, z = self.apply_calibration_to_data("LSM6DSV", x, y, z, is_gyro=False)
            imu_name = "LSM6DSV Accelerometer"
        elif self.current_imu == "LSM6DSV_GYRO":
            if not self.imu_data.lsm6dsv_available:
                self.stats_text.setPlainText("LSM6DSV not detected or no data available")
                return
            timestamps, x, y, z = self.imu_data.get_lsm_gyro_data()
            x, y, z = self.apply_calibration_to_data("LSM6DSV", x, y, z, is_gyro=True)
            imu_name = "LSM6DSV Gyroscope"
        elif self.current_imu == "LSM6DSV_BOTH":
            if not self.imu_data.lsm6dsv_available:
                self.stats_text.setPlainText("LSM6DSV not detected or no data available")
                return
            # Get both datasets for statistics
            timestamps_accel, accel_x, accel_y, accel_z = self.imu_data.get_lsm_accel_data()
            timestamps_gyro, gyro_x, gyro_y, gyro_z = self.imu_data.get_lsm_gyro_data()

            # Apply calibration
            accel_x, accel_y, accel_z = self.apply_calibration_to_data("LSM6DSV", accel_x, accel_y, accel_z, is_gyro=False)
            gyro_x, gyro_y, gyro_z = self.apply_calibration_to_data("LSM6DSV", gyro_x, gyro_y, gyro_z, is_gyro=True)

            # Apply filtering
            accel_x, accel_y, accel_z = self.apply_current_filter(timestamps_accel, accel_x, accel_y, accel_z)
            gyro_x, gyro_y, gyro_z = self.apply_current_filter(timestamps_gyro, gyro_x, gyro_y, gyro_z)

            imu_name = "LSM6DSV - Both"
        elif self.current_imu == "LSM6DSOX_ACCEL":
            if not self.imu_data.lsm6dsox_available:
                self.stats_text.setPlainText("LSM6DSOX not detected or no data available")
                return
            timestamps, x, y, z = self.imu_data.get_lsmox_accel_data()
            x, y, z = self.apply_calibration_to_data("LSM6DSOX", x, y, z, is_gyro=False)
            imu_name = "LSM6DSOX Accelerometer"
        elif self.current_imu == "LSM6DSOX_GYRO":
            if not self.imu_data.lsm6dsox_available:
                self.stats_text.setPlainText("LSM6DSOX not detected or no data available")
                return
            timestamps, x, y, z = self.imu_data.get_lsmox_gyro_data()
            x, y, z = self.apply_calibration_to_data("LSM6DSOX", x, y, z, is_gyro=True)
            imu_name = "LSM6DSOX Gyroscope"
        elif self.current_imu == "LIS2DW12":
            if not self.imu_data.lis2dw12_available:
                self.stats_text.setPlainText("LIS2DW12 not detected or no data available")
                return
            timestamps, x, y, z = self.imu_data.get_lis_accel_data()
            x, y, z = self.apply_calibration_to_data("LIS2DW12", x, y, z, is_gyro=False)
            imu_name = "LIS2DW12 Accelerometer"
        elif self.current_imu == "LSM6DSV_SFLP":
            if not self.imu_data.lsm6dsv_sflp_available:
                self.stats_text.setPlainText("LSM6DSV SFLP not detected or no data available")
                return
            timestamps, qw, qx, qy, qz = self.imu_data.get_lsm_sflp_data()

            # Apply filtering to quaternion data
            qw, qx, qy, qz = self.apply_current_filter_to_quaternion(timestamps, qw, qx, qy, qz)

            # Convert to Euler angles if needed
            if self.sflp_view_mode == "EULER":
                x, y, z = IMUData.quaternion_to_euler(qw, qx, qy, qz)
                imu_name = "LSM6DSV SFLP (Euler Angles)"
                data_type = "Orientation"
                units = "deg"
            else:
                # For quaternion statistics, we'll handle specially
                imu_name = "LSM6DSV SFLP (Quaternion)"
                data_type = "Quaternion"
                units = ""

        else:  # ADXL362
            if not self.imu_data.adxl362_available:
                self.stats_text.setPlainText("ADXL362 not detected or no data available")
                return
            timestamps, x, y, z = self.imu_data.get_adxl362_accel_data()
            x, y, z = self.apply_calibration_to_data("ADXL362", x, y, z, is_gyro=False)
            imu_name = "ADXL362 Accelerometer"

        # Apply filtering to 3-axis data (before integration)
        # Skip for LSM6DSV_BOTH (already filtered above) and SFLP quaternion mode
        if self.current_imu != "LSM6DSV_SFLP" and self.current_imu != "LSM6DSV_BOTH":
            x, y, z = self.apply_current_filter(timestamps, x, y, z)
        elif self.current_imu == "LSM6DSV_SFLP" and self.sflp_view_mode == "EULER":
            x, y, z = self.apply_current_filter(timestamps, x, y, z)

        # Apply integration based on integration level (skip for SFLP quaternion and BOTH)
        if self.current_imu != "LSM6DSV_SFLP" and self.current_imu != "LSM6DSV_BOTH":
            if self.integration_level == 0:
                # Raw data
                if self.current_imu == "LSM6DSV_SFLP":
                    # Already set above for Euler
                    pass
                elif is_accel:
                    data_type = "Acceleration"
                    units = "mg"
                else:
                    data_type = "Angular Velocity"
                    units = "mdps"
            elif self.integration_level == 1:
                # First integration
                if len(timestamps) < 2:
                    self.stats_text.setPlainText("Insufficient data for integration")
                    return
                x, y, z = DataIntegrator.integrate_once(timestamps, x, y, z, is_accel=is_accel, detrend=self.detrend_enabled)
                if is_accel:
                    data_type = "Velocity"
                    units = "m/s"
                else:
                    data_type = "Angle"
                    units = "deg"
            else:  # integration_level == 2 (only for accelerometers)
                # Second integration (position)
                if len(timestamps) < 2:
                    self.stats_text.setPlainText("Insufficient data for integration")
                    return
                x, y, z = DataIntegrator.integrate_twice(timestamps, x, y, z, detrend=self.detrend_enabled)
                data_type = "Position"
                units = "m"
        elif self.current_imu == "LSM6DSV_SFLP" and self.sflp_view_mode == "EULER":
            # Integration already applied for SFLP Euler mode
            pass

        # Apply integration for LSM6DSV_BOTH mode (separately for accel and gyro)
        if self.current_imu == "LSM6DSV_BOTH":
            if self.integration_level == 0:
                # Raw data
                accel_data_type = "Acceleration"
                accel_units = "mg"
                gyro_data_type = "Angular Velocity"
                gyro_units = "mdps"
            elif self.integration_level == 1:
                # First integration
                if len(timestamps_accel) >= 2:
                    accel_x, accel_y, accel_z = DataIntegrator.integrate_once(
                        timestamps_accel, accel_x, accel_y, accel_z, is_accel=True, detrend=self.detrend_enabled)
                accel_data_type = "Velocity"
                accel_units = "m/s"

                if len(timestamps_gyro) >= 2:
                    gyro_x, gyro_y, gyro_z = DataIntegrator.integrate_once(
                        timestamps_gyro, gyro_x, gyro_y, gyro_z, is_accel=False, detrend=self.detrend_enabled)
                gyro_data_type = "Angle"
                gyro_units = "deg"
            else:  # integration_level == 2
                # Second integration (only for accel)
                if len(timestamps_accel) >= 2:
                    accel_x, accel_y, accel_z = DataIntegrator.integrate_twice(
                        timestamps_accel, accel_x, accel_y, accel_z, detrend=self.detrend_enabled)
                accel_data_type = "Position"
                accel_units = "m"

                # Gyro can only do first integration
                if len(timestamps_gyro) >= 2:
                    gyro_x, gyro_y, gyro_z = DataIntegrator.integrate_once(
                        timestamps_gyro, gyro_x, gyro_y, gyro_z, is_accel=False, detrend=self.detrend_enabled)
                gyro_data_type = "Angle"
                gyro_units = "deg"

        # Calculate statistics
        if self.current_imu == "LSM6DSV_SFLP" and self.sflp_view_mode == "QUATERNION":
            # For quaternions, calculate stats for all 4 components
            stats_qw = DataStatistics.calculate_stats(qw, qw, qw)  # Use same array 3 times for structure
            stats_qx = DataStatistics.calculate_stats(qx, qx, qx)
            stats_qy = DataStatistics.calculate_stats(qy, qy, qy)
            stats_qz = DataStatistics.calculate_stats(qz, qz, qz)
            magnitude = np.sqrt(qw**2 + qx**2 + qy**2 + qz**2)
            stats_mag = DataStatistics.calculate_stats(magnitude, magnitude, magnitude)
            sample_rate = DataStatistics.calculate_sample_rate(timestamps)
        elif self.current_imu == "LSM6DSV_BOTH":
            # Calculate stats for both accel and gyro
            stats_accel = DataStatistics.calculate_stats(accel_x, accel_y, accel_z)
            stats_gyro = DataStatistics.calculate_stats(gyro_x, gyro_y, gyro_z)
            sample_rate_accel = DataStatistics.calculate_sample_rate(timestamps_accel)
            sample_rate_gyro = DataStatistics.calculate_sample_rate(timestamps_gyro)
        else:
            stats = DataStatistics.calculate_stats(x, y, z)
            sample_rate = DataStatistics.calculate_sample_rate(timestamps)

        # Format statistics text
        stats_text = f"{'='*50}\n"
        if self.current_imu == "LSM6DSV_BOTH":
            stats_text += f"{imu_name}\n"
        else:
            stats_text += f"{imu_name} - {data_type}\n"
        stats_text += f"{'='*50}\n\n"

        if self.current_imu == "LSM6DSV_SFLP":
            stats_text += f"Total Samples: {self.imu_data.lsm_sflp_sample_count}\n"
            stats_text += f"Sample Rate: {sample_rate:.2f} Hz\n"
        elif self.current_imu == "LSM6DSV_BOTH":
            stats_text += f"Total Samples: {self.imu_data.lsm_sample_count}\n"
            stats_text += f"Accel Sample Rate: {sample_rate_accel:.2f} Hz\n"
            stats_text += f"Gyro Sample Rate: {sample_rate_gyro:.2f} Hz\n"
        else:
            stats_text += f"Total Samples: {self.imu_data.sample_count}\n"
            stats_text += f"Sample Rate: {sample_rate:.2f} Hz\n"
        if self.filter_type != "None":
            stats_text += f"Filter: {self.filter_type}\n"
        if self.detrend_enabled and self.integration_level > 0:
            stats_text += f"DC Offset Removed: Yes\n"
        stats_text += "\n"

        # Format quaternion or regular statistics
        if self.current_imu == "LSM6DSV_SFLP" and self.sflp_view_mode == "QUATERNION":
            stats_text += f"{'='*50}\n"
            stats_text += f"QUATERNION COMPONENT STATISTICS\n"
            stats_text += f"{'='*50}\n\n"

            # W component
            stats_text += f"W Component:\n"
            stats_text += f"  Min:    {stats_qw['x']['min']:10.4f}\n"
            stats_text += f"  Max:    {stats_qw['x']['max']:10.4f}\n"
            stats_text += f"  Mean:   {stats_qw['x']['mean']:10.4f}\n"
            stats_text += f"  Std:    {stats_qw['x']['std']:10.4f}\n"
            stats_text += f"  RMS:    {stats_qw['x']['rms']:10.4f}\n\n"

            # X component
            stats_text += f"X Component:\n"
            stats_text += f"  Min:    {stats_qx['x']['min']:10.4f}\n"
            stats_text += f"  Max:    {stats_qx['x']['max']:10.4f}\n"
            stats_text += f"  Mean:   {stats_qx['x']['mean']:10.4f}\n"
            stats_text += f"  Std:    {stats_qx['x']['std']:10.4f}\n"
            stats_text += f"  RMS:    {stats_qx['x']['rms']:10.4f}\n\n"

            # Y component
            stats_text += f"Y Component:\n"
            stats_text += f"  Min:    {stats_qy['x']['min']:10.4f}\n"
            stats_text += f"  Max:    {stats_qy['x']['max']:10.4f}\n"
            stats_text += f"  Mean:   {stats_qy['x']['mean']:10.4f}\n"
            stats_text += f"  Std:    {stats_qy['x']['std']:10.4f}\n"
            stats_text += f"  RMS:    {stats_qy['x']['rms']:10.4f}\n\n"

            # Z component
            stats_text += f"Z Component:\n"
            stats_text += f"  Min:    {stats_qz['x']['min']:10.4f}\n"
            stats_text += f"  Max:    {stats_qz['x']['max']:10.4f}\n"
            stats_text += f"  Mean:   {stats_qz['x']['mean']:10.4f}\n"
            stats_text += f"  Std:    {stats_qz['x']['std']:10.4f}\n"
            stats_text += f"  RMS:    {stats_qz['x']['rms']:10.4f}\n\n"

            # Magnitude statistics
            stats_text += f"{'='*50}\n"
            stats_text += f"QUATERNION MAGNITUDE (should be ~1.0)\n"
            stats_text += f"{'='*50}\n\n"
            stats_text += f"  Min:    {stats_mag['x']['min']:10.4f}\n"
            stats_text += f"  Max:    {stats_mag['x']['max']:10.4f}\n"
            stats_text += f"  Mean:   {stats_mag['x']['mean']:10.4f}\n"
            stats_text += f"  Std:    {stats_mag['x']['std']:10.4f}\n"
            stats_text += f"  RMS:    {stats_mag['x']['rms']:10.4f}\n"
        elif self.current_imu == "LSM6DSV_BOTH":
            # Display both accelerometer and gyroscope statistics side by side
            stats_text += f"{'='*50}\n"
            stats_text += f"ACCELEROMETER - {accel_data_type} ({accel_units})\n"
            stats_text += f"{'='*50}\n\n"

            for axis in ['x', 'y', 'z']:
                stats_text += f"{axis.upper()} Axis:\n"
                stats_text += f"  Min:    {stats_accel[axis]['min']:10.2f}\n"
                stats_text += f"  Max:    {stats_accel[axis]['max']:10.2f}\n"
                stats_text += f"  Mean:   {stats_accel[axis]['mean']:10.2f}\n"
                stats_text += f"  Std:    {stats_accel[axis]['std']:10.2f}\n"
                stats_text += f"  RMS:    {stats_accel[axis]['rms']:10.2f}\n\n"

            stats_text += f"Magnitude:\n"
            stats_text += f"  Min:    {stats_accel['magnitude']['min']:10.2f}\n"
            stats_text += f"  Max:    {stats_accel['magnitude']['max']:10.2f}\n"
            stats_text += f"  Mean:   {stats_accel['magnitude']['mean']:10.2f}\n"
            stats_text += f"  Std:    {stats_accel['magnitude']['std']:10.2f}\n"
            stats_text += f"  RMS:    {stats_accel['magnitude']['rms']:10.2f}\n\n"

            stats_text += f"{'='*50}\n"
            stats_text += f"GYROSCOPE - {gyro_data_type} ({gyro_units})\n"
            stats_text += f"{'='*50}\n\n"

            for axis in ['x', 'y', 'z']:
                stats_text += f"{axis.upper()} Axis:\n"
                stats_text += f"  Min:    {stats_gyro[axis]['min']:10.2f}\n"
                stats_text += f"  Max:    {stats_gyro[axis]['max']:10.2f}\n"
                stats_text += f"  Mean:   {stats_gyro[axis]['mean']:10.2f}\n"
                stats_text += f"  Std:    {stats_gyro[axis]['std']:10.2f}\n"
                stats_text += f"  RMS:    {stats_gyro[axis]['rms']:10.2f}\n\n"

            stats_text += f"Magnitude:\n"
            stats_text += f"  Min:    {stats_gyro['magnitude']['min']:10.2f}\n"
            stats_text += f"  Max:    {stats_gyro['magnitude']['max']:10.2f}\n"
            stats_text += f"  Mean:   {stats_gyro['magnitude']['mean']:10.2f}\n"
            stats_text += f"  Std:    {stats_gyro['magnitude']['std']:10.2f}\n"
            stats_text += f"  RMS:    {stats_gyro['magnitude']['rms']:10.2f}\n"
        else:
            stats_text += f"{'='*50}\n"
            stats_text += f"AXIS STATISTICS ({units})\n"
            stats_text += f"{'='*50}\n\n"

            # Format each axis
            for axis in ['x', 'y', 'z']:
                stats_text += f"{axis.upper()} Axis:\n"
                stats_text += f"  Min:    {stats[axis]['min']:10.2f}\n"
                stats_text += f"  Max:    {stats[axis]['max']:10.2f}\n"
                stats_text += f"  Mean:   {stats[axis]['mean']:10.2f}\n"
                stats_text += f"  Std:    {stats[axis]['std']:10.2f}\n"
                stats_text += f"  RMS:    {stats[axis]['rms']:10.2f}\n\n"

            # Magnitude statistics
            stats_text += f"{'='*50}\n"
            stats_text += f"MAGNITUDE STATISTICS ({units})\n"
            stats_text += f"{'='*50}\n\n"
            stats_text += f"  Min:    {stats['magnitude']['min']:10.2f}\n"
            stats_text += f"  Max:    {stats['magnitude']['max']:10.2f}\n"
            stats_text += f"  Mean:   {stats['magnitude']['mean']:10.2f}\n"
            stats_text += f"  Std:    {stats['magnitude']['std']:10.2f}\n"
            stats_text += f"  RMS:    {stats['magnitude']['rms']:10.2f}\n"

        self.stats_text.setPlainText(stats_text)

    def closeEvent(self, event):
        """Handle application close event."""
        # Stop serial reader if running
        if self.serial_reader.isRunning():
            self.serial_reader.stop()
            self.serial_reader.wait(2000)  # Wait up to 2 seconds

        # Stop timers
        self.plot_timer.stop()
        self.stats_timer.stop()

        event.accept()


def main():
    """Main entry point."""
    app = QApplication(sys.argv)
    window = IMUVisualizer()
    window.show()
    sys.exit(app.exec())


if __name__ == '__main__':
    main()
