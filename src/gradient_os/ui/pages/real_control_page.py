import time
from functools import partial

from PySide6.QtCore import Qt, QTimer
from PySide6.QtWidgets import (
    QWidget, QHBoxLayout, QVBoxLayout, QGridLayout, QGroupBox, QLabel,
    QComboBox, QSlider, QPushButton, QDial, QCheckBox, QLineEdit, QPlainTextEdit,
    QMessageBox
)

from gradient_os.ui.constants import (
    POS_ZERO,
    POS_HOME,
    POS_REST,
    POS_INCREMENT_OPTIONS_MM,
    ORI_INCREMENT_OPTIONS_DEG,
)


class RealControlPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        
        main_layout = QHBoxLayout()

        # Left side: Jog and Trajectory controls
        left_controls_layout = QVBoxLayout()
        
        # Position Jogging
        self.pos_group_box = QGroupBox("Position Jog")
        pos_layout = QGridLayout()
        
        pos_increment_label = QLabel("Increment (mm):")
        self.pos_increment_combo = QComboBox()
        self.pos_increment_combo.addItems(POS_INCREMENT_OPTIONS_MM)
        index_50 = self.pos_increment_combo.findText("50")
        if index_50 != -1:
            self.pos_increment_combo.setCurrentIndex(index_50)
        pos_layout.addWidget(pos_increment_label, 0, 0, 1, 2)
        pos_layout.addWidget(self.pos_increment_combo, 0, 2, 1, 2)

        pos_layout.addWidget(self._create_jog_button("+X", "x", 1), 1, 0)
        pos_layout.addWidget(self._create_jog_button("+Y", "y", 1), 1, 1)
        pos_layout.addWidget(self._create_jog_button("+Z", "z", 1), 1, 2)
        pos_layout.addWidget(self._create_jog_button("-X", "x", -1), 2, 0)
        pos_layout.addWidget(self._create_jog_button("-Y", "y", -1), 2, 1)
        pos_layout.addWidget(self._create_jog_button("-Z", "z", -1), 2, 2)
        
        self.pos_group_box.setLayout(pos_layout)
        left_controls_layout.addWidget(self.pos_group_box)

        # Orientation Jogging
        self.ori_group_box = QGroupBox("Orientation Jog")
        ori_layout = QGridLayout()

        ori_increment_label = QLabel("Increment (deg):")
        self.ori_increment_combo = QComboBox()
        self.ori_increment_combo.addItems(ORI_INCREMENT_OPTIONS_DEG)
        index_15 = self.ori_increment_combo.findText("15")
        if index_15 != -1:
            self.ori_increment_combo.setCurrentIndex(index_15)
        ori_layout.addWidget(ori_increment_label, 0, 0, 1, 2)
        ori_layout.addWidget(self.ori_increment_combo, 0, 2, 1, 2)

        ori_layout.addWidget(self._create_jog_button("+Roll", 0, 1, is_rotation=True), 1, 0)
        ori_layout.addWidget(self._create_jog_button("+Pitch", 1, 1, is_rotation=True), 1, 1)
        ori_layout.addWidget(self._create_jog_button("+Yaw", 2, 1, is_rotation=True), 1, 2)
        ori_layout.addWidget(self._create_jog_button("-Roll", 0, -1, is_rotation=True), 2, 0)
        ori_layout.addWidget(self._create_jog_button("-Pitch", 1, -1, is_rotation=True), 2, 1)
        ori_layout.addWidget(self._create_jog_button("-Yaw", 2, -1, is_rotation=True), 2, 2)

        self.ori_group_box.setLayout(ori_layout)
        left_controls_layout.addWidget(self.ori_group_box)
        
        # Gripper Control
        self.gripper_group_box = QGroupBox("Gripper")
        gripper_layout = QGridLayout()
        gripper_layout.addWidget(QLabel("Angle:"), 0, 0)
        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setRange(0, 180)
        self.gripper_slider.setValue(0)
        self.gripper_slider.valueChanged.connect(self.update_gripper_label)
        gripper_layout.addWidget(self.gripper_slider, 0, 1)
        self.gripper_angle_label = QLabel("0°")
        gripper_layout.addWidget(self.gripper_angle_label, 0, 2)
        open_btn = QPushButton("Open")
        open_btn.clicked.connect(self.open_gripper)
        gripper_layout.addWidget(open_btn, 1, 0)
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(self.close_gripper)
        gripper_layout.addWidget(close_btn, 1, 1)
        set_gripper_btn = QPushButton("Set Angle")
        set_gripper_btn.clicked.connect(self.set_gripper_from_slider)
        gripper_layout.addWidget(set_gripper_btn, 1, 2)
        self.gripper_group_box.setLayout(gripper_layout)
        left_controls_layout.addWidget(self.gripper_group_box)

        # Trajectory Planning
        self.traj_group_box = QGroupBox("Trajectory Planning")
        traj_layout = QVBoxLayout()
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("Trajectory Name:"))
        self.name_input = QLineEdit("my_new_path")
        name_layout.addWidget(self.name_input)
        traj_layout.addLayout(name_layout)
        controls_layout = QHBoxLayout()
        start_btn = QPushButton("Start Planning")
        start_btn.clicked.connect(self.start_planning)
        controls_layout.addWidget(start_btn)
        record_btn = QPushButton("Record Waypoint")
        record_btn.clicked.connect(self.record_waypoint)
        controls_layout.addWidget(record_btn)
        end_btn = QPushButton("End & Save")
        end_btn.clicked.connect(self.end_trajectory)
        controls_layout.addWidget(end_btn)
        traj_layout.addLayout(controls_layout)
        run_layout = QHBoxLayout()
        combo_layout = QVBoxLayout()
        combo_layout.addWidget(QLabel("Run Saved Trajectory:"))
        self.run_traj_combo = QComboBox()
        combo_layout.addStretch()
        combo_layout.addWidget(self.run_traj_combo)
        run_layout.addLayout(combo_layout)
        refresh_traj_btn = QPushButton("Refresh List")
        refresh_traj_btn.clicked.connect(self.refresh_trajectory_list)
        run_layout.addWidget(refresh_traj_btn)
        run_btn = QPushButton("Run")
        run_btn.clicked.connect(self.send_run_trajectory)
        run_layout.addWidget(run_btn)
        self.loop_traj_btn = QPushButton("Loop")
        self.loop_traj_btn.setCheckable(True)
        self.loop_traj_btn.setToolTip("If selected, the trajectory will run continuously until STOP is pressed.")
        run_layout.addWidget(self.loop_traj_btn)
        traj_layout.addLayout(run_layout)
        self.traj_group_box.setLayout(traj_layout)
        left_controls_layout.addWidget(self.traj_group_box)

        # Quick Actions
        self.quick_box = QGroupBox("Quick Actions")
        quick_layout = QHBoxLayout()
        zero_btn = QPushButton("Zero Joints")
        zero_btn.clicked.connect(self.go_zero)
        rest_btn = QPushButton("Rest Pose")
        rest_btn.clicked.connect(self.go_rest)
        quick_layout.addWidget(zero_btn)
        quick_layout.addWidget(rest_btn)
        self.quick_box.setLayout(quick_layout)
        left_controls_layout.addWidget(self.quick_box)

        left_controls_layout.addStretch()
        main_layout.addLayout(left_controls_layout)

        # Right side: State, Logs and direct commands
        right_side_layout = QVBoxLayout()
        self.current_pos = [0.0, 0.0, 0.0]
        self.current_ori = [0.0, 0.0, 0.0]
        self.current_gripper_deg = 0.0
        self.state_group_box = QGroupBox("Robot State")
        state_layout = QGridLayout()
        state_layout.addWidget(QLabel("X:"), 0, 0)
        self.x_val_label = QLabel("0.0")
        state_layout.addWidget(self.x_val_label, 0, 1)
        state_layout.addWidget(QLabel("Y:"), 1, 0)
        self.y_val_label = QLabel("0.0")
        state_layout.addWidget(self.y_val_label, 1, 1)
        state_layout.addWidget(QLabel("Z:"), 2, 0)
        self.z_val_label = QLabel("0.0")
        state_layout.addWidget(self.z_val_label, 2, 1)
        state_layout.addWidget(QLabel("Roll:"), 0, 2)
        self.roll_val_label = QLabel("0.0")
        state_layout.addWidget(self.roll_val_label, 0, 3)
        state_layout.addWidget(QLabel("Pitch:"), 1, 2)
        self.pitch_val_label = QLabel("0.0")
        state_layout.addWidget(self.pitch_val_label, 1, 3)
        state_layout.addWidget(QLabel("Yaw:"), 2, 2)
        self.yaw_val_label = QLabel("0.0")
        state_layout.addWidget(self.yaw_val_label, 2, 3)
        state_layout.addWidget(QLabel("Gripper:"), 3, 0)
        self.gripper_val_label = QLabel("0.0°")
        state_layout.addWidget(self.gripper_val_label, 3, 1)
        self.state_group_box.setLayout(state_layout)
        right_side_layout.addWidget(self.state_group_box)

        self.log = []
        self._max_log_lines = 100  # reduce log history by ~80% for responsiveness
        self.log_text = QPlainTextEdit()
        self.log_text.setReadOnly(True)
        try:
            # Enforce cap at the widget level for efficient trimming
            self.log_text.document().setMaximumBlockCount(self._max_log_lines)
        except Exception:
            pass
        # Toggle to show/hide live logs (hidden by default to reduce UI churn)
        self.show_logs_checkbox = QCheckBox("Show Logs")
        self.show_logs_checkbox.setChecked(False)
        self.show_logs_checkbox.toggled.connect(self._on_show_logs_toggled)
        right_side_layout.addWidget(self.show_logs_checkbox)
        right_side_layout.addWidget(self.log_text, 2)
        self.log_text.setVisible(False)
        # Coalesced log flushing
        self._log_buffer = []
        self._log_flush_timer = QTimer(self)
        self._log_flush_timer.setSingleShot(True)
        self._log_flush_timer.timeout.connect(self._flush_log_buffer)
        self._log_flush_interval_ms = 150

        self.direct_cmd_box = QGroupBox("Direct Commands")
        direct_cmd_layout = QGridLayout()
        self.move_line_input = QLineEdit()
        self.move_line_input.setPlaceholderText("e.g., 0.1,0.2,0.3")
        direct_cmd_layout.addWidget(QLabel("MOVE_LINE:"), 0, 0)
        direct_cmd_layout.addWidget(self.move_line_input, 0, 1)
        self.move_line_rel_input = QLineEdit()
        self.move_line_rel_input.setPlaceholderText("e.g., 0.1,0.2,0.3")
        direct_cmd_layout.addWidget(QLabel("MOVE_LINE_RELATIVE:"), 1, 0)
        direct_cmd_layout.addWidget(self.move_line_rel_input, 1, 1)
        self.set_ori_input = QLineEdit()
        self.set_ori_input.setPlaceholderText("e.g., 90,0,0")
        direct_cmd_layout.addWidget(QLabel("SET_ORIENTATION:"), 2, 0)
        direct_cmd_layout.addWidget(self.set_ori_input, 2, 1)
        self.direct_cmd_box.setLayout(direct_cmd_layout)
        right_side_layout.addWidget(self.direct_cmd_box)
        self.send_btn = QPushButton("Send Direct Command")
        self.send_btn.clicked.connect(self.send_command_from_inputs)
        right_side_layout.addWidget(self.send_btn)
        self.closed_loop_checkbox = QCheckBox("Closed-loop control")
        self.closed_loop_checkbox.setChecked(False)
        right_side_layout.addWidget(self.closed_loop_checkbox)

        self.speed_group = QGroupBox("Speed Multiplier & Diagnostics")
        speed_layout = QHBoxLayout()
        self.speed_dial = QDial()
        self.speed_dial.setNotchesVisible(True)
        self.speed_dial.setWrapping(False)
        self.speed_dial.setMinimum(0)
        self.speed_dial.setMaximum(1000)
        self.speed_dial.setValue(500)
        self.speed_value_label = QLabel("1.00x")
        self.speed_value_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
        self.speed_dial.valueChanged.connect(self._on_speed_dial_changed)
        speed_layout.addWidget(self.speed_dial)
        speed_layout.addWidget(self.speed_value_label)
        self.diagnostics_checkbox = QCheckBox("Diagnostics")
        self.diagnostics_checkbox.setChecked(False)
        self.diagnostics_checkbox.toggled.connect(self._on_diagnostics_toggled)
        speed_layout.addWidget(self.diagnostics_checkbox)
        self.speed_group.setLayout(speed_layout)
        right_side_layout.addWidget(self.speed_group)

        # Focus mode to hide non-essential panels for performance
        self.focus_mode_checkbox = QCheckBox("Focus Mode (hide extra panels)")
        self.focus_mode_checkbox.setChecked(True)
        self.focus_mode_checkbox.toggled.connect(self._on_focus_mode_toggled)
        right_side_layout.addWidget(self.focus_mode_checkbox)

        # Realtime jog now lives exclusively in the web ControlPanel session flow.
        self.jog_group = QGroupBox("Realtime Jog (Moved To Web UI)")
        jog_layout = QGridLayout()
        jog_notice = QLabel(
            "Desktop UDP jog was removed. Use the web ControlPanel at :8000 so jog flows through "
            "the API session endpoints and controller-owned lease/watchdog path."
        )
        jog_notice.setWordWrap(True)
        jog_layout.addWidget(jog_notice, 0, 0, 1, 4)

        # Universal speed entry (mm/s for XYZ, deg/s for RPY) with default 50
        jog_layout.addWidget(QLabel("Linear Speed (mm/s):"), 1, 0)
        self.jog_speed_input = QLineEdit("50")
        self.jog_speed_input.setToolTip("Linear jog speed in mm/s (converted to m/s). Use Speed Dial to scale.")
        jog_layout.addWidget(self.jog_speed_input, 1, 1)
        jog_layout.addWidget(QLabel("Orientation Speed (deg/s):"), 1, 2)
        self.jog_speed_ori_input = QLineEdit("15")
        self.jog_speed_ori_input.setToolTip("Angular jog speed in deg/s. Use Speed Dial to scale.")
        jog_layout.addWidget(self.jog_speed_ori_input, 1, 3)

        # Start/Stop and deadman/debug row
        self.jog_toggle_btn = QPushButton("Start Jog")
        self.jog_toggle_btn.setCheckable(True)
        self.jog_toggle_btn.toggled.connect(self._on_jog_toggled)
        self.jog_toggle_btn.setEnabled(False)
        jog_layout.addWidget(self.jog_toggle_btn, 2, 0)

        # Deadman switch (toggle) default ON
        self.jog_deadman_switch = QCheckBox("Enable Jog (Deadman)")
        self.jog_deadman_switch.setChecked(True)
        self.jog_deadman_switch.toggled.connect(self._on_deadman_switch_toggled)
        self.jog_deadman_switch.setEnabled(False)
        jog_layout.addWidget(self.jog_deadman_switch, 2, 1)

        self.jog_debug_checkbox = QCheckBox("Debug jog logs")
        self.jog_debug_checkbox.setChecked(False)
        self.jog_debug_checkbox.toggled.connect(self._on_jog_debug_toggled)
        self.jog_debug_checkbox.setEnabled(False)
        jog_layout.addWidget(self.jog_debug_checkbox, 2, 2)

        # Linear buttons
        jog_layout.addWidget(self._create_rt_jog_button("+X", axis="x", sign=+1, is_rotation=False), 3, 0)
        jog_layout.addWidget(self._create_rt_jog_button("+Y", axis="y", sign=+1, is_rotation=False), 3, 1)
        jog_layout.addWidget(self._create_rt_jog_button("+Z", axis="z", sign=+1, is_rotation=False), 3, 2)
        jog_layout.addWidget(self._create_rt_jog_button("-X", axis="x", sign=-1, is_rotation=False), 4, 0)
        jog_layout.addWidget(self._create_rt_jog_button("-Y", axis="y", sign=-1, is_rotation=False), 4, 1)
        jog_layout.addWidget(self._create_rt_jog_button("-Z", axis="z", sign=-1, is_rotation=False), 4, 2)

        # Angular buttons
        jog_layout.addWidget(self._create_rt_jog_button("+Roll", axis="roll", sign=+1, is_rotation=True), 5, 0)
        jog_layout.addWidget(self._create_rt_jog_button("+Pitch", axis="pitch", sign=+1, is_rotation=True), 5, 1)
        jog_layout.addWidget(self._create_rt_jog_button("+Yaw", axis="yaw", sign=+1, is_rotation=True), 5, 2)
        jog_layout.addWidget(self._create_rt_jog_button("-Roll", axis="roll", sign=-1, is_rotation=True), 6, 0)
        jog_layout.addWidget(self._create_rt_jog_button("-Pitch", axis="pitch", sign=-1, is_rotation=True), 6, 1)
        jog_layout.addWidget(self._create_rt_jog_button("-Yaw", axis="yaw", sign=-1, is_rotation=True), 6, 2)

        # (Zero button removed; deadman or Stop Jog will zero velocities automatically)

        # Internal state for pressed buttons (counts allow multi-press compose)
        self._jog_linear_counts = {"x": 0, "y": 0, "z": 0}
        self._jog_angular_counts = {"roll": 0, "pitch": 0, "yaw": 0}
        # Current commanded jog vector (vx,vy,vz, r,p,y)
        self._jog_cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.jog_group.setLayout(jog_layout)
        # Place realtime jog at the top of the left column, above Position Jog
        left_controls_layout.insertWidget(0, self.jog_group)

        # Timer to stream jog velocities at 25 Hz
        self.jog_timer = QTimer(self)
        try:
            self.jog_timer.setTimerType(Qt.PreciseTimer)
        except Exception:
            pass
        self.jog_timer.setInterval(20)  # 50 Hz for better responsiveness
        self.jog_timer.timeout.connect(self._send_jog_velocity_tick)

        # Removed periodic state polling; we'll refresh on motion completion instead

        # Reduce UI/network load by only sending on change or periodic keepalive
        self._last_jog_sent = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._last_grip_sent = 0.0
        self._last_jog_send_time_ms = 0
        self._jog_keepalive_interval_ms = 200  # balance responsiveness and load

        # Telemetry Recorder Controls
        self.recorder_group = QGroupBox("Telemetry Recorder")
        rec_layout = QGridLayout()
        rec_layout.addWidget(QLabel("Episodes Dir:"), 0, 0)
        self.rec_dir_input = QLineEdit("recorded_episodes")
        rec_layout.addWidget(self.rec_dir_input, 0, 1, 1, 3)
        rec_layout.addWidget(QLabel("Prompt:"), 1, 0)
        self.rec_prompt_input = QLineEdit("")
        rec_layout.addWidget(self.rec_prompt_input, 1, 1, 1, 3)
        rec_layout.addWidget(QLabel("Base Cam URL (blank=auto):"), 2, 0)
        self.rec_base_cam_input = QLineEdit("")
        rec_layout.addWidget(self.rec_base_cam_input, 2, 1, 1, 3)
        rec_layout.addWidget(QLabel("Wrist Cam URL (blank=auto):"), 3, 0)
        self.rec_wrist_cam_input = QLineEdit("")
        rec_layout.addWidget(self.rec_wrist_cam_input, 3, 1, 1, 3)
        self.rec_cameras_checkbox = QCheckBox("Record Cameras")
        self.rec_cameras_checkbox.setChecked(True)
        self.rec_cameras_checkbox.setToolTip("If unchecked, recording will not start/consume camera streams.")
        rec_layout.addWidget(self.rec_cameras_checkbox, 4, 0, 1, 2)
        rec_layout.addWidget(QLabel("FPS:"), 5, 0)
        self.rec_fps_input = QLineEdit("10")
        rec_layout.addWidget(self.rec_fps_input, 5, 1)
        rec_layout.addWidget(QLabel("Resize (0=native):"), 5, 2)
        self.rec_resize_input = QLineEdit("0")
        rec_layout.addWidget(self.rec_resize_input, 5, 3)
        rec_layout.addWidget(QLabel("State UDP bind:"), 6, 0)
        self.rec_state_udp_input = QLineEdit("0.0.0.0:5555")
        rec_layout.addWidget(self.rec_state_udp_input, 6, 1)
        rec_layout.addWidget(QLabel("Action UDP bind:"), 6, 2)
        self.rec_action_udp_input = QLineEdit("")
        rec_layout.addWidget(self.rec_action_udp_input, 6, 3)
        start_rec_btn = QPushButton("Start Recorder")
        start_rec_btn.clicked.connect(self.start_recorder)
        stop_rec_btn = QPushButton("Stop Recorder")
        stop_rec_btn.clicked.connect(self.stop_recorder)
        rec_layout.addWidget(start_rec_btn, 7, 0, 1, 2)
        rec_layout.addWidget(stop_rec_btn, 7, 2, 1, 2)
        self.recorder_group.setLayout(rec_layout)
        right_side_layout.addWidget(self.recorder_group)
        
        main_layout.addLayout(right_side_layout)
        self.setLayout(main_layout)
        # Apply initial focus mode state
        self._on_focus_mode_toggled(self.focus_mode_checkbox.isChecked())

    def check_gripper_presence(self):
        self.parent.send_command("GET_STATUS")
        self.refresh_state()

    def _create_jog_button(self, text, axis, direction, is_rotation=False):
        button = QPushButton(text)
        button.setFixedSize(100, 50)
        if is_rotation:
            button.clicked.connect(partial(self._send_orientation_jog, axis, direction))
        else:
            button.clicked.connect(partial(self._send_jog_move, axis, direction))
        return button

    def _send_jog_move(self, axis, direction):
        # Pause jog if active to avoid controller thread snapping back
        self._pause_jog_if_active()
        increment_mm = float(self.pos_increment_combo.currentText())
        increment_m = increment_mm / 1000.0
        delta = {
            "x": [increment_m * direction, 0, 0],
            "y": [0, increment_m * direction, 0],
            "z": [0, 0, increment_m * direction]
        }[axis]
        speed_multiplier = self._current_speed_multiplier()
        closed_str = "true" if self.closed_loop_checkbox.isChecked() else "false"
        cmd = f"MOVE_LINE_RELATIVE,{delta[0]},{delta[1]},{delta[2]},{speed_multiplier},{closed_str}"
        self.parent.send_command(cmd)
        self._refresh_on_motion_complete()
        self._resume_jog_if_needed()

    def refresh_trajectory_list(self):
        self.log_message("Requesting trajectory list...")
        self.parent.send_command("GET_TRAJECTORIES")
        # Results will be handled asynchronously by the main window dispatcher

    def _send_orientation_jog(self, axis_index, direction):
        # Pause jog for orientation moves to avoid competing controllers
        self._pause_jog_if_active()
        increment_deg = float(self.ori_increment_combo.currentText())
        self.current_ori[axis_index] += increment_deg * direction
        r, p, y = self.current_ori
        cmd = f"SET_ORIENTATION,{r},{p},{y}"
        self.parent.send_command(cmd)
        self.update_state_display()
        self.refresh_state()
        self._resume_jog_if_needed()

    def update_state_display(self):
        self.x_val_label.setText(f"{self.current_pos[0]:.3f}")
        self.y_val_label.setText(f"{self.current_pos[1]:.3f}")
        self.z_val_label.setText(f"{self.current_pos[2]:.3f}")
        self.roll_val_label.setText(f"{self.current_ori[0]:.2f}")
        self.pitch_val_label.setText(f"{self.current_ori[1]:.2f}")
        self.yaw_val_label.setText(f"{self.current_ori[2]:.2f}")
        self.gripper_val_label.setText(f"{self.current_gripper_deg:.1f}°")

    def refresh_state(self):
        self.parent.send_command("GET_POSITION")
        self.parent.send_command("GET_GRIPPER_STATE")
        self.update_state_display()

    def start_planning(self):
        cmd = "PLAN_TRAJECTORY"
        self.parent.send_command(cmd)
        self.log_message(f"Sent: {cmd}")
        self.log_message("Recording mode started. Move robot and record waypoints.")

    def record_waypoint(self):
        cmd = "REC_POS"
        self.parent.send_command(cmd)
        self.log_message(f"Sent: {cmd}")
        self.log_message("Recorded current position as a waypoint.")

    def end_trajectory(self):
        name = self.name_input.text()
        if not name:
            QMessageBox.warning(self, "Input Error", "Please provide a name for the trajectory.")
            return
        cmd = f"END_TRAJECTORY,{name}"
        self.parent.send_command(cmd)
        self.log_message(f"Sent: {cmd}")
        self.log_message(f"Trajectory saved as '{name}'.")

    def update_gripper_label(self, value):
        self.gripper_angle_label.setText(f"{value}°")
        if not hasattr(self, "_grip_debounce_timer"):
            self._grip_debounce_timer = QTimer(self)
            self._grip_debounce_timer.setSingleShot(True)
            self._grip_debounce_timer.timeout.connect(self.set_gripper_from_slider)
        self._grip_debounce_timer.start(50)

    def set_gripper_from_slider(self):
        angle = self.gripper_slider.value()
        cmd = f"SET_GRIPPER,{angle}"
        self.parent.send_command(cmd)
        self.log_message(f"Sent: {cmd}")

    def open_gripper(self):
        self.gripper_slider.setValue(120)

    def close_gripper(self):
        self.gripper_slider.setValue(0)

    def send_command_from_inputs(self):
        self._pause_jog_if_active()
        cmd_parts = []
        if self.move_line_input.text():
            cmd_parts.append(f"MOVE_LINE,{self.move_line_input.text()}")
        if self.move_line_rel_input.text():
            cmd_parts.append(f"MOVE_LINE_RELATIVE,{self.move_line_rel_input.text()}")
        if self.set_ori_input.text():
            cmd_parts.append(f"SET_ORIENTATION,{self.set_ori_input.text()}")
        if cmd_parts:
            cmd_str = "|".join(cmd_parts)
            self.parent.send_command(cmd_str)
            self.log_message(f"Sent: {cmd_str}")
            # For composite inputs, assume motion may be non-blocking → refresh on completion
            self._refresh_on_motion_complete()
        self._resume_jog_if_needed()

    def log_message(self, msg):
        # Skip logging when logs are hidden and diagnostics are off
        if not self.show_logs_checkbox.isChecked() and not self.diagnostics_checkbox.isChecked():
            return
        # Store in ring buffer and coalesce UI updates
        self.log.append(msg)
        if len(self.log) > self._max_log_lines:
            self.log = self.log[-self._max_log_lines:]
            if self.log_text.isVisible():
                self.log_text.setPlainText("\n".join(self.log))
                self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())
            self._log_buffer.clear()
            return
        self._log_buffer.append(msg)
        if self.log_text.isVisible() and not self._log_flush_timer.isActive():
            self._log_flush_timer.start(self._log_flush_interval_ms)

    def _flush_log_buffer(self):
        if not self._log_buffer or not self.log_text.isVisible():
            self._log_buffer.clear()
            return
        text = "\n".join(self._log_buffer)
        self._log_buffer.clear()
        self.log_text.appendPlainText(text)
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def _on_show_logs_toggled(self, checked: bool):
        self.log_text.setVisible(checked)
        if checked:
            # Rebuild current view from the stored ring buffer
            try:
                self.log_text.setPlainText("\n".join(self.log[-self._max_log_lines:]))
            except Exception:
                pass
            self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())
            if self._log_buffer and not self._log_flush_timer.isActive():
                self._log_flush_timer.start(1)
        else:
            # When hiding logs, clear pending flush to reduce churn
            try:
                self._log_flush_timer.stop()
            except Exception:
                pass

    def _on_focus_mode_toggled(self, checked: bool):
        # Keep only essential panels visible when checked
        self.pos_group_box.setVisible(not checked)
        self.ori_group_box.setVisible(not checked)
        # Keep gripper controls visible even in focus mode
        self.gripper_group_box.setVisible(True if checked else True)
        # Keep trajectory planning visible even in focus mode
        self.traj_group_box.setVisible(True if checked else True)
        # Keep Zero/Rest quick actions visible even in focus mode
        self.quick_box.setVisible(True if checked else True)
        self.direct_cmd_box.setVisible(not checked)
        # Keep episode recorder visible even in focus mode
        self.recorder_group.setVisible(True if checked else True)
        # Keep robot state and speed diagnostics visible in both modes

    def go_home(self):
        self._pause_jog_if_active()
        self.parent.send_command("HOME")
        self.log_message("Sent Home")
        self._refresh_on_motion_complete()
        self._resume_jog_if_needed()

    def go_zero(self):
        self._pause_jog_if_active()
        cmd = ",".join(map(str, POS_ZERO))
        self.parent.send_command(cmd)
        self.log_message(f"Sent Zero: {cmd}")
        self._refresh_on_motion_complete()
        self._resume_jog_if_needed()

    def go_rest(self):
        self._pause_jog_if_active()
        cmd = ",".join(map(str, POS_REST))
        self.parent.send_command(cmd)
        self.log_message(f"Sent Rest: {cmd}")
        self._refresh_on_motion_complete()
        self._resume_jog_if_needed()

    def get_position(self):
        self.parent.send_command("GET_POSITION")
        response = self.parent.receive_data()
        if response:
            self.log_message(f"Received: {response}")
        else:
            self.log_message("No response or timeout.")

    def send_move_line(self):
        self._pause_jog_if_active()
        input_str = self.move_line_input.text()
        parts = [p.strip() for p in input_str.split(',') if p.strip() != '']
        closed_str = "true" if self.closed_loop_checkbox.isChecked() else "false"
        default_v = 0.1 * self._current_speed_multiplier()
        default_a = 0.05 * self._current_speed_multiplier()
        cmd = None
        if len(parts) >= 6:
            cmd = f"MOVE_LINE,{','.join(parts)},{closed_str}"
        elif len(parts) == 5:
            cmd = f"MOVE_LINE,{','.join(parts)},{closed_str}"
        elif len(parts) == 4:
            cmd = f"MOVE_LINE,{parts[0]},{parts[1]},{parts[2]},{parts[3]},{default_a},{closed_str}"
        elif len(parts) == 3:
            cmd = f"MOVE_LINE,{parts[0]},{parts[1]},{parts[2]},{default_v},{default_a},{closed_str}"
        else:
            cmd = f"MOVE_LINE,{input_str}"
        self.parent.send_command(cmd)
        self._refresh_on_motion_complete()
        self._resume_jog_if_needed()

    def send_move_line_rel(self):
        self._pause_jog_if_active()
        input_str = self.move_line_rel_input.text()
        parts = [p.strip() for p in input_str.split(',') if p.strip() != '']
        closed_str = "true" if self.closed_loop_checkbox.isChecked() else "false"
        cmd = None
        if len(parts) >= 5:
            cmd = f"MOVE_LINE_RELATIVE,{','.join(parts)},{closed_str}"
        elif len(parts) == 3:
            cmd = f"MOVE_LINE_RELATIVE,{parts[0]},{parts[1]},{parts[2]},{self._current_speed_multiplier()},{closed_str}"
        else:
            cmd = f"MOVE_LINE_RELATIVE,{input_str}"
        self.parent.send_command(cmd)
        self._refresh_on_motion_complete()
        self._resume_jog_if_needed()

    def send_set_orientation(self):
        self._pause_jog_if_active()
        input_str = self.set_ori_input.text()
        self.parent.send_command(f"SET_ORIENTATION,{input_str}")
        self.refresh_state()
        self._resume_jog_if_needed()

    def send_run_trajectory(self):
        name = self.run_traj_combo.currentText()
        if not name:
            QMessageBox.warning(self, "Selection Error", "Please select a trajectory to run, or refresh the list.")
            return
        is_looping = self.loop_traj_btn.isChecked()
        self.parent.send_command(f"RUN_TRAJECTORY,{name},false,{is_looping}")

    def send_end_trajectory(self):
        name = self.end_traj_input.text()
        self.parent.send_command(f"END_TRAJECTORY,{name}")

    def send_calibrate(self):
        id_str = self.calib_input.text()
        self.parent.send_command(f"CALIBRATE,{id_str}")

    def send_set_zero(self):
        joint = self.set_zero_input.text()
        self.parent.send_command(f"SET_ZERO,{joint}")

    def send_factory_reset(self):
        id_str = self.factory_reset_input.text()
        reply = QMessageBox.question(self, 'Confirm Factory Reset', 'Are you sure? This resets the servo to factory defaults.', QMessageBox.Yes | QMessageBox.No)
        if reply == QMessageBox.Yes:
            self.parent.send_command(f"FACTORY_RESET,{id_str}")

    def send_translate(self):
        self._pause_jog_if_active()
        input_str = self.translate_input.text()
        self.parent.send_command(f"TRANSLATE,{input_str}")
        self.refresh_state()
        self._resume_jog_if_needed()

    def send_rotate(self):
        self._pause_jog_if_active()
        input_str = self.rotate_input.text()
        self.parent.send_command(f"ROTATE,{input_str}")
        self.refresh_state()
        self._resume_jog_if_needed()

    def _on_speed_dial_changed(self, value):
        mult = self._current_speed_multiplier()
        self.speed_value_label.setText(f"{mult:.2f}x")

    def _current_speed_multiplier(self):
        t = self.speed_dial.value() / 1000.0
        exp_val = (t * 2.0) - 1.0
        mult = 10 ** exp_val
        if mult < 0.1:
            mult = 0.1
        if mult > 10.0:
            mult = 10.0
        return mult

    def _on_diagnostics_toggled(self, checked):
        mode = "ON" if checked else "OFF"
        self.parent.send_command(f"DIAGNOSTICS,{mode}")

    def start_recorder(self):
        episodes_dir = self.rec_dir_input.text()
        prompt = self.rec_prompt_input.text()
        base_cam = self.rec_base_cam_input.text().strip()
        wrist_cam = self.rec_wrist_cam_input.text().strip()
        fps = self.rec_fps_input.text().strip()
        resize = self.rec_resize_input.text().strip()
        state_udp = self.rec_state_udp_input.text().strip()
        action_udp = self.rec_action_udp_input.text().strip()
        # Normalize commas in command; empty strings are allowed for cams/action_udp
        parts = ["START_RECORDER", episodes_dir, prompt, base_cam, wrist_cam, fps, resize, state_udp, action_udp]
        cmd = ",".join(parts)
        self.parent.send_command(cmd)

    def stop_recorder(self):
        self.parent.send_command("STOP_RECORDER")

    def _log_realtime_jog_removed(self):
        self.log_message("Realtime jog was removed from the desktop UDP UI. Use the web ControlPanel session jog instead.")

    # -----------------------------
    # Jog helpers
    # -----------------------------
    def _on_jog_toggled(self, checked):
        self.jog_timer.stop()
        if checked:
            self._log_realtime_jog_removed()
            self.jog_toggle_btn.blockSignals(True)
            self.jog_toggle_btn.setChecked(False)
            self.jog_toggle_btn.blockSignals(False)
        self.jog_toggle_btn.setText("Start Jog")

    def _update_jog_labels(self):
        pass

    def _jog_zero_all(self):
        self._jog_linear_counts = {"x": 0, "y": 0, "z": 0}
        self._jog_angular_counts = {"roll": 0, "pitch": 0, "yaw": 0}
        self._jog_cmd = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    def _send_jog_velocity_tick(self):
        return

    def _on_deadman_pressed(self):
        self._log_realtime_jog_removed()

    def _on_deadman_released(self):
        self._jog_zero_all()

    def _on_deadman_switch_toggled(self, checked):
        if not checked:
            self._jog_zero_all()
            self.refresh_state()

    def _on_jog_debug_toggled(self, checked):
        flag = "true" if checked else "false"
        self.parent.send_command(f"SET_JOG_DEBUG,{flag}")

    # Real-time jog button helpers
    def _create_rt_jog_button(self, text, axis: str, sign: int, is_rotation: bool):
        btn = QPushButton(text)
        btn.setCheckable(False)
        btn.setFixedSize(100, 40)
        # Use pressed/released to allow combinations
        if is_rotation:
            btn.pressed.connect(partial(self._rt_jog_change, axis, sign, True))
            btn.released.connect(partial(self._rt_jog_change, axis, -sign, True))
        else:
            btn.pressed.connect(partial(self._rt_jog_change, axis, sign, False))
            btn.released.connect(partial(self._rt_jog_change, axis, -sign, False))
        return btn

    def _rt_jog_change(self, axis: str, delta: int, is_rotation: bool):
        self._log_realtime_jog_removed()

    def _refresh_on_motion_complete(self):
        # Ask the controller to block until current motion completes, then query state
        try:
            self.parent.send_command("WAIT_FOR_IDLE")
        except Exception:
            pass
        self.parent.send_command("GET_POSITION")
        self.parent.send_command("GET_GRIPPER_STATE")

    def _pause_jog_if_active(self):
        self._resume_jog_after_motion = False
        self.jog_timer.stop()
        self._jog_zero_all()

    def _resume_jog_if_needed(self):
        self._resume_jog_after_motion = False


