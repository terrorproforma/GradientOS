# ui_start.py
# 
# This file is the main entry point for the UI.
# It contains the main window and the pages for the different features.
# It also contains the logic for the different features.
# 
# The main window is a QMainWindow that contains a QStackedWidget.
# The QStackedWidget contains the pages for the different features.
# The pages are switched by calling the switch_to_home, switch_to_editor, switch_to_simulation, switch_to_dashboard, switch_to_workflow, switch_to_welding, switch_to_control, and switch_to_real_control functions.

import sys
import argparse
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QStackedWidget, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QTextEdit, QTableWidget, QTableWidgetItem, QListWidget,
    QSlider, QStatusBar, QTextBrowser, QLineEdit, QComboBox, QMessageBox, QGridLayout,
    QGroupBox, QScrollArea, QButtonGroup
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QFont, QIcon

# For 3D simulation
import pyqtgraph.opengl as gl
import numpy as np

# For UDP communication
import socket
import time
from functools import partial

# Predefined poses from the script
POS_ZERO = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
POS_HOME = [0.0, -0.785, 0.785, 0.0, -0.785, 0.0]
POS_REST = [0.0, -1.4, 1.5, 0.0, 0.0, 0.0]

class HomePage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        # Main layout to center all content vertically
        main_layout = QVBoxLayout()
        main_layout.setAlignment(Qt.AlignCenter)
        self.setLayout(main_layout)

        # --- Title Labels ---
        title_layout = QVBoxLayout()
        title_layout.setSpacing(5) # Compact spacing
        
        title_label = QLabel("Industrial Robot Controller")
        title_label.setAlignment(Qt.AlignCenter)
        title_label.setStyleSheet("font-size: 24px; font-weight: bold; color: #FF6600;")
        title_layout.addWidget(title_label)

        robot_label = QLabel("Robot: Gradient Zero")
        robot_label.setAlignment(Qt.AlignCenter)
        robot_label.setStyleSheet("font-size: 14px; color: #CCCCCC;")
        title_layout.addWidget(robot_label)

        main_layout.addLayout(title_layout)

        # Add a fixed spacer
        main_layout.addSpacing(30)

        # --- Button Grid for Navigation ---
        button_grid = QGridLayout()
        button_grid.setSpacing(15)

        # Define buttons
        control_btn = QPushButton("Joint Control")
        control_btn.clicked.connect(self.parent.switch_to_control)

        real_control_btn = QPushButton("Real Robot Control")
        real_control_btn.clicked.connect(self.parent.switch_to_real_control)

        calib_btn = QPushButton("Calibration")
        calib_btn.clicked.connect(self.parent.switch_to_calibration)

        tut_btn = QPushButton("Tutorials & Docs")
        tut_btn.clicked.connect(self.parent.switch_to_tutorials)

        # Add buttons to grid layout
        button_grid.addWidget(control_btn, 0, 0)
        button_grid.addWidget(real_control_btn, 0, 1)
        button_grid.addWidget(calib_btn, 1, 0)
        button_grid.addWidget(tut_btn, 1, 1)
        
        main_layout.addLayout(button_grid)

# class SimulationPage(QWidget):
#     def __init__(self, parent=None):
#         super().__init__(parent)
#         self.parent = parent
#         layout = QVBoxLayout()

#         label = QLabel("Robot Simulation (3D)")
#         label.setAlignment(Qt.AlignCenter)
#         layout.addWidget(label)

#         self.view = gl.GLViewWidget()
#         self.view.setCameraPosition(distance=40)

#         g = gl.GLGridItem()
#         self.view.addItem(g)

#         self.links = []
#         positions = [(0,0,0), (0,0,5), (0,0,10), (0,0,15)]
#         for i in range(3):
#             cyl = gl.GLMeshItem(meshdata=gl.MeshData.cylinder(rows=10, cols=20, radius=[1.0, 1.0], length=5.0))
#             cyl.translate(*positions[i])
#             self.view.addItem(cyl)
#             self.links.append(cyl)

#         layout.addWidget(self.view)

#         play_btn = QPushButton("Play Simulation")
#         layout.addWidget(play_btn)

#         back_btn = QPushButton("Back to Home")
#         back_btn.clicked.connect(self.parent.switch_to_home)
#         layout.addWidget(back_btn)

#         self.setLayout(layout)

class DashboardPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        layout = QVBoxLayout()

        label = QLabel("Mission Dashboard")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)

        self.table = QTableWidget(10, 5)
        self.table.setHorizontalHeaderLabels(["Mission ID", "Brief", "Code", "Priority", "Status"])
        sample_data = [
            ["G-078W", "Infiltrate Target Facility", "NIGHTFALL", "High", "Completed"],
            ["G-079X", "Extract Informant", "MOONSHADOW", "Medium", "Active"],
            ["G-080Y", "Intercept Communication", "WONDERLUST", "Medium", "Reviewing"],
            ["G-081Z", "Deploy Drone Unit", "FIREBRAND", "Low", "Completed"],
            ["G-082A", "Neutralize High-Value Threat", "WINDWHISPER", "High", "Completed"],
            ["G-083B", "Secure Classified Files", "THUND3RS", "Low", "Reviewing"],
            ["G-084C", "Initiate Blackout Protocol", "SHADOWSTEP", "Low", "Completed"],
            ["G-085D", "Monitor Subject Activity", "EV3NFALL", "High", "Active"],
            ["G-086E", "Install Surveillance Kit", "ECHOSONG", "High", "Reviewing"],
            ["G-087F", "Deploy Decoy Assets", "FROSTBITE", "Medium", "Completed"],
        ]
        for row, data in enumerate(sample_data):
            for col, item in enumerate(data):
                self.table.setItem(row, col, QTableWidgetItem(item))

        layout.addWidget(self.table)

        self.setLayout(layout)

class WorkflowPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        layout = QVBoxLayout()

        label = QLabel("Workflow Builder")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)

        node_layout = QHBoxLayout()
        nodes = [
            "Generate E-BOOK Outline",
            "Expand Outline into Full Chapters",
            "Format Content into PDF",
            "Create Product Page",
            "Schedule Social/Email Promotions",
            "Handle Payments & Payouts"
        ]
        for node in nodes:
            btn = QPushButton(node)
            btn.setFixedWidth(150)
            node_layout.addWidget(btn)

        layout.addLayout(node_layout)

        self.setLayout(layout)

class WeldingPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        layout = QVBoxLayout()

        label = QLabel("Welding Menu")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)

        adaptive_btn = QPushButton("Adaptive Path Correct")
        layout.addWidget(adaptive_btn)

        param_btn = QPushButton("Parameter")
        layout.addWidget(param_btn)

        auto_time_btn = QPushButton("Auto Time")
        layout.addWidget(auto_time_btn)

        play_sim_btn = QPushButton("Play Simulation")
        layout.addWidget(play_sim_btn)

        start_prog_btn = QPushButton("Start Program")
        layout.addWidget(start_prog_btn)

        pause_btn = QPushButton("Pause")
        layout.addWidget(pause_btn)

        stop_btn = QPushButton("Stop")
        layout.addWidget(stop_btn)

        cycle_label = QLabel("Estimated Cycle Time: Arc On - Complete Cycle")
        layout.addWidget(cycle_label)

        torch_label = QLabel("Torch Angle: +45° / +15°")
        layout.addWidget(torch_label)

        self.setLayout(layout)

class ControlPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        layout = QVBoxLayout()

        label = QLabel("Joint Control")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)

        self.status_label = QLabel("ROBOT CONTROL IS LIVE")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setStyleSheet("background-color: #000000; color: #FFFFFF; font-weight: bold; font-size: 20px;")
        self.status_label.hide()
        layout.addWidget(self.status_label)

        # --- Live Control Buttons ---
        control_button_layout = QHBoxLayout()
        self.activate_btn = QPushButton("ACTIVATE LIVE CONTROL")
        self.activate_btn.setStyleSheet("background-color: lime; color: black; font-weight: bold; font-size: 16px;")
        self.activate_btn.clicked.connect(self.activate_live_control)
        control_button_layout.addWidget(self.activate_btn)

        self.deactivate_btn = QPushButton("DEACTIVATE")
        self.deactivate_btn.setStyleSheet("background-color: darkred; color: gray; font-weight: bold; font-size: 16px;")
        self.deactivate_btn.clicked.connect(self.deactivate_live_control)
        self.deactivate_btn.setEnabled(False)
        control_button_layout.addWidget(self.deactivate_btn)
        layout.addLayout(control_button_layout)

        joints = ["Base", "Shoulder", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3", "Gripper"]
        self.joints = joints
        self.sliders = {}
        self.value_labels = {}

        # Create a grid to hold all joint controls, two per row
        sliders_grid = QGridLayout()
        sliders_grid.setHorizontalSpacing(15) # Keep space between columns
        sliders_grid.setVerticalSpacing(0)   # Remove space between rows

        for i, joint_name in enumerate(joints):
            # Each joint's controls are in a group box.
            joint_group_box = QGroupBox() 
            joint_layout = QGridLayout()

            # Row 0, Col 1: Joint Label
            j_label = QLabel(f"J{i+1}: {joint_name}")
            j_label.setAlignment(Qt.AlignLeft | Qt.AlignVCenter)
            joint_layout.addWidget(j_label, 0, 1)

            # Row 0, Col 2: Value Label (Bold)
            value_label = QLabel("0°")
            value_label.setStyleSheet("font-weight: bold;")
            value_label.setAlignment(Qt.AlignRight | Qt.AlignVCenter)
            joint_layout.addWidget(value_label, 0, 2)
            self.value_labels[joint_name] = value_label
            
            # Row 1: Slider (spans columns 1 and 2)
            slider = QSlider(Qt.Horizontal)
            if joint_name == "Gripper":
                slider.setRange(0, 180)
            else:
                slider.setRange(-180, 180)
            slider.setValue(0)
            slider.setEnabled(False)
            joint_layout.addWidget(slider, 1, 1, 1, 2)
            self.sliders[joint_name] = slider

            slider.valueChanged.connect(partial(self._handle_slider_change, joint_name=joint_name))
            
            # Jog Buttons (span 2 rows, in cols 0 and 3)
            jog_minus_btn = QPushButton("-")
            jog_minus_btn.setObjectName("JogButton")
            joint_layout.addWidget(jog_minus_btn, 0, 0, 2, 1)
            jog_minus_btn.clicked.connect(partial(self.jog_joint, joint_name, -1))

            jog_plus_btn = QPushButton("+")
            jog_plus_btn.setObjectName("JogButton")
            joint_layout.addWidget(jog_plus_btn, 0, 3, 2, 1)
            jog_plus_btn.clicked.connect(partial(self.jog_joint, joint_name, 1))
            
            joint_group_box.setLayout(joint_layout)
            
            # Add the group box to the main sliders grid
            row = i // 2
            col = i % 2
            sliders_grid.addWidget(joint_group_box, row, col)

        # --- NEW: Global Jog Step as Radio Buttons ---
        # This is placed in the grid cell next to the Gripper control
        jog_step_group_box = QGroupBox("Global Jog Step")
        jog_step_layout = QHBoxLayout()
        self.jog_step_button_group = QButtonGroup()
        
        steps = ["1", "5", "10", "20", "45"]
        for step in steps:
            button = QPushButton(step)
            button.setCheckable(True)
            jog_step_layout.addWidget(button)
            self.jog_step_button_group.addButton(button)
            if step == "5":
                button.setChecked(True)

        jog_step_group_box.setLayout(jog_step_layout)
        
        # Add it to the grid next to the last joint (Gripper)
        sliders_grid.addWidget(jog_step_group_box, 3, 1)

        layout.addLayout(sliders_grid)

        # --- Bottom Controls (Actions & Calibration) in a Grid ---
        bottom_controls_layout = QGridLayout()

        # Column 1: Action Buttons (Apply/Zero)
        action_buttons_layout = QVBoxLayout()
        
        apply_btn = QPushButton("Apply All Positions")
        apply_btn.clicked.connect(self.apply_all_positions)
        action_buttons_layout.addWidget(apply_btn)

        zero_btn = QPushButton("Zero All Sliders")
        zero_btn.clicked.connect(self.reset_sliders)
        action_buttons_layout.addWidget(zero_btn)
        
        action_buttons_layout.addStretch() # Pushes buttons to the top of the cell
        bottom_controls_layout.addLayout(action_buttons_layout, 0, 0, Qt.AlignTop)

        # Column 2: Calibration Tools
        calib_group_box = QGroupBox() # Title removed
        calib_layout = QGridLayout()

        # Set Zero
        calib_layout.addWidget(QLabel("SET_ZERO Joint #:"), 0, 0)
        self.set_zero_input = QLineEdit("1")
        self.set_zero_input.setToolTip("Enter 1-6 for arm joints, 7 for the gripper.")
        calib_layout.addWidget(self.set_zero_input, 0, 1)
        set_zero_btn = QPushButton("Set Current as Zero")
        set_zero_btn.clicked.connect(self.send_set_zero)
        calib_layout.addWidget(set_zero_btn, 0, 2)

        # Factory Reset
        calib_layout.addWidget(QLabel("FACTORY_RESET Servo ID:"), 1, 0)
        self.factory_reset_input = QLineEdit("10")
        calib_layout.addWidget(self.factory_reset_input, 1, 1)
        factory_reset_btn = QPushButton("Factory Reset Servo")
        factory_reset_btn.clicked.connect(self.send_factory_reset)
        calib_layout.addWidget(factory_reset_btn, 1, 2)

        # Refresh Limits button is now created but added later
        refresh_limits_btn = QPushButton("Refresh Servo Limits")
        refresh_limits_btn.clicked.connect(self.send_refresh_limits)
        
        calib_group_box.setLayout(calib_layout)
        bottom_controls_layout.addWidget(calib_group_box, 0, 1)

        # Configure column widths (15% for buttons, 85% for calibration)
        bottom_controls_layout.setColumnStretch(0, 1)
        bottom_controls_layout.setColumnStretch(1, 6)
        
        layout.addLayout(bottom_controls_layout)

        # --- Log Area and Final Buttons ---
        log_area_layout = QGridLayout()
        
        # Column 1: Final Buttons
        final_buttons_layout = QVBoxLayout()
        final_buttons_layout.addWidget(refresh_limits_btn) # Add refresh button here
        final_buttons_layout.addStretch()
        log_area_layout.addLayout(final_buttons_layout, 0, 0, Qt.AlignTop)

        # Column 2: Log Readout
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("background-color: #111111; color: #FF6600; font-family: 'Courier New', monospace;")
        log_area_layout.addWidget(self.log_text, 0, 1)

        # Configure column widths
        log_area_layout.setColumnStretch(0, 1)
        log_area_layout.setColumnStretch(1, 6)

        layout.addLayout(log_area_layout)

        self.setLayout(layout)
        self.live_active = False
        self.log = []
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.toggle_status_visibility)

    def toggle_status_visibility(self):
        if self.status_label.styleSheet() == "background-color: #FF0000; color: #FFFFFF; font-weight: bold; font-size: 20px;":
            self.status_label.setStyleSheet("background-color: #000000; color: #FFFFFF; font-weight: bold; font-size: 20px;")
        else:
            self.status_label.setStyleSheet("background-color: #FF0000; color: #FFFFFF; font-weight: bold; font-size: 20px;")

    def activate_live_control(self):
        self.live_active = True
        self.activate_btn.setStyleSheet("background-color: darkgreen; color: gray; font-weight: bold; font-size: 16px;")
        self.activate_btn.setEnabled(False)
        self.deactivate_btn.setStyleSheet("background-color: red; color: black; font-weight: bold; font-size: 16px;")
        self.deactivate_btn.setEnabled(True)
        for slider in self.sliders.values():
            slider.setEnabled(True)
        self.status_label.setStyleSheet("background-color: #FF0000; color: #FFFFFF; font-weight: bold; font-size: 20px;")
        self.status_label.show()
        self.timer.start(500)
        self.log_message("Live control activated")

        # Sync sliders to current robot positions
        self.parent.send_command("GET_JOINT_ANGLES")
        response = self.parent.receive_data()
        if response and response.startswith("JOINT_ANGLES,"):
            parts = response.split(',')
            for i, joint in enumerate(self.joints):
                try:
                    deg = float(parts[i+1])
                    self.sliders[joint].setValue(int(deg))
                    self.value_labels[joint].setText(f"{int(deg)}°")
                except (IndexError, ValueError):
                    self.log_message(f"Failed to sync {joint}")
        else:
            self.log_message("Failed to sync to current positions")

    def deactivate_live_control(self):
        self.live_active = False
        self.activate_btn.setStyleSheet("background-color: lime; color: black; font-weight: bold; font-size: 16px;")
        self.activate_btn.setEnabled(True)
        self.deactivate_btn.setStyleSheet("background-color: darkred; color: gray; font-weight: bold; font-size: 16px;")
        self.deactivate_btn.setEnabled(False)
        for slider in self.sliders.values():
            slider.setEnabled(False)
        self.timer.stop()
        self.status_label.hide()
        self.log_message("Live control deactivated")

    def _handle_slider_change(self, value, joint_name):
        """
        This slot is connected to the valueChanged signal of all joint sliders.
        It updates the displayed angle and, if live control is active, sends
        the appropriate command to the robot.
        """
        self.value_labels[joint_name].setText(f"{value}°")
        if self.live_active:
            self.apply_all_positions()

    def jog_joint(self, joint_name, direction):
        """Handles the jog button clicks for a specific joint."""
        slider = self.sliders[joint_name]
        
        increment = float(self.jog_step_button_group.checkedButton().text())
        current_value = slider.value()
        new_value = current_value + (increment * direction)
        
        # Clamp to slider limits
        new_value = max(slider.minimum(), min(slider.maximum(), new_value))
        
        slider.setValue(int(new_value))
        
        # If live control is active, send the command immediately
        if self.live_active:
            self.apply_all_positions()

    def reset_sliders(self):
        for joint in self.sliders:
            self.sliders[joint].setValue(0)

    def apply_all_positions(self):
        """Sends the positions of all joints including gripper."""
        values = [self.sliders[joint].value() * (np.pi / 180) for joint in self.joints]  # Convert to radians
        cmd = ",".join(map(str, values))
        self.parent.send_command(cmd)
        self.log_message(f"Sent Joint Command: {cmd}")

    def log_message(self, msg):
        self.log.append(msg)
        self.log_text.append(msg)
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def go_home(self):
        cmd = ",".join(map(str, POS_HOME))
        self.parent.send_command(cmd)
        self.log_message(f"Sent Home: {cmd}")

    def go_zero(self):
        cmd = "0,0,0,0,0,0"
        self.parent.send_command(cmd)
        self.log_message(f"Sent Zero: {cmd}")

    def go_rest(self):
        cmd = ",".join(map(str, POS_REST))
        self.parent.send_command(cmd)
        self.log_message(f"Sent Rest: {cmd}")

    def get_position(self):
        self.parent.send_command("GET_POSITION")
        response = self.parent.receive_data()
        if response:
            self.log_message(f"Received: {response}")
        else:
            self.log_message("No response or timeout.")

    def send_move_line(self):
        input_str = self.move_line_input.text()
        self.parent.send_command(f"MOVE_LINE,{input_str}")

    def send_move_line_rel(self):
        input_str = self.move_line_rel_input.text()
        self.parent.send_command(f"MOVE_LINE_RELATIVE,{input_str}")

    def send_set_orientation(self):
        input_str = self.set_ori_input.text()
        self.parent.send_command(f"SET_ORIENTATION,{input_str}")

    def send_run_trajectory(self):
        name = self.run_traj_combo.currentText()
        if not name:
            QMessageBox.warning(self, "Selection Error", "Please select a trajectory to run, or refresh the list.")
            return

        is_looping = self.loop_traj_btn.isChecked()
        # Send command with loop status. Format: RUN_TRAJECTORY,name,use_cache,loop
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

    def send_refresh_limits(self):
        self.parent.send_command("REFRESH_LIMITS")
        self.log_message("Sent: REFRESH_LIMITS")

    def send_translate(self):
        input_str = self.translate_input.text()
        self.parent.send_command(f"TRANSLATE,{input_str}")

    def send_rotate(self):
        input_str = self.rotate_input.text()
        self.parent.send_command(f"ROTATE,{input_str}")

class TutorialsPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        layout = QVBoxLayout()

        label = QLabel("Tutorials & Docs")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)

        self.doc_list = QListWidget()
        self.load_docs()
        self.doc_list.itemClicked.connect(self.display_doc)
        layout.addWidget(self.doc_list)

        self.doc_viewer = QTextBrowser()
        layout.addWidget(self.doc_viewer)

        self.setLayout(layout)

    def load_docs(self):
        import os
        docs_dir = 'docs'
        if os.path.exists(docs_dir):
            for file in os.listdir(docs_dir):
                if file.endswith('.md'):
                    self.doc_list.addItem(file)

    def display_doc(self, item):
        import os
        doc_path = os.path.join('docs', item.text())
        with open(doc_path, 'r') as f:
            content = f.read()
            self.doc_viewer.setMarkdown(content)

class CalibrationPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        layout = QVBoxLayout()

        label = QLabel("Calibration Tools")
        label.setAlignment(Qt.AlignCenter)
        layout.addWidget(label)

        warning_label = QLabel("Warning: Use with caution! Backup configurations before proceeding.")
        warning_label.setStyleSheet("color: red")
        layout.addWidget(warning_label)

        # Calibration
        calib_hbox = QHBoxLayout()
        calib_label = QLabel("CALIBRATE id")
        self.calib_input = QLineEdit("10")
        calib_btn = QPushButton("Calibrate")
        calib_btn.clicked.connect(self.send_calibrate)
        calib_hbox.addWidget(calib_label)
        calib_hbox.addWidget(self.calib_input)
        calib_hbox.addWidget(calib_btn)
        layout.addLayout(calib_hbox)

        # Set Zero
        set_zero_hbox = QHBoxLayout()
        set_zero_label = QLabel("SET_ZERO joint#")
        self.set_zero_input = QLineEdit("1")
        self.set_zero_input.setToolTip("Enter 1-6 for arm joints, 7 for the gripper.")
        set_zero_btn = QPushButton("Set Zero")
        set_zero_btn.clicked.connect(self.send_set_zero)
        set_zero_hbox.addWidget(set_zero_label)
        set_zero_hbox.addWidget(self.set_zero_input)
        set_zero_hbox.addWidget(set_zero_btn)
        layout.addLayout(set_zero_hbox)

        # Factory Reset
        factory_reset_hbox = QHBoxLayout()
        factory_reset_label = QLabel("FACTORY_RESET id")
        self.factory_reset_input = QLineEdit("10")
        factory_reset_btn = QPushButton("Factory Reset")
        factory_reset_btn.clicked.connect(self.send_factory_reset)
        factory_reset_hbox.addWidget(factory_reset_label)
        factory_reset_hbox.addWidget(self.factory_reset_input)
        factory_reset_hbox.addWidget(factory_reset_btn)
        layout.addLayout(factory_reset_hbox)

        self.setLayout(layout)

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

class RealControlPage(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        
        main_layout = QHBoxLayout()

        # Left side: Jog and Trajectory controls
        left_controls_layout = QVBoxLayout()
        
        # Position Jogging
        pos_group_box = QGroupBox("Position Jog")
        pos_layout = QGridLayout()
        
        pos_increment_label = QLabel("Increment (mm):")
        self.pos_increment_combo = QComboBox()
        self.pos_increment_combo.addItems(["1", "10", "100"])
        pos_layout.addWidget(pos_increment_label, 0, 0, 1, 2)
        pos_layout.addWidget(self.pos_increment_combo, 0, 2, 1, 2)

        # X buttons
        pos_layout.addWidget(self._create_jog_button("-X", "x", -1), 2, 0)
        pos_layout.addWidget(self._create_jog_button("+X", "x", 1), 2, 1)
        # Y buttons
        pos_layout.addWidget(self._create_jog_button("-Y", "y", -1), 1, 0)
        pos_layout.addWidget(self._create_jog_button("+Y", "y", 1), 1, 1)
        # Z buttons
        pos_layout.addWidget(self._create_jog_button("-Z", "z", -1), 3, 0)
        pos_layout.addWidget(self._create_jog_button("+Z", "z", 1), 3, 1)
        
        pos_group_box.setLayout(pos_layout)
        left_controls_layout.addWidget(pos_group_box)

        # Orientation Jogging
        ori_group_box = QGroupBox("Orientation Jog")
        ori_layout = QGridLayout()

        ori_increment_label = QLabel("Increment (deg):")
        self.ori_increment_combo = QComboBox()
        self.ori_increment_combo.addItems(["1", "5", "15", "45"])
        ori_layout.addWidget(ori_increment_label, 0, 0, 1, 2)
        ori_layout.addWidget(self.ori_increment_combo, 0, 2, 1, 2)

        # Roll buttons
        ori_layout.addWidget(self._create_jog_button("-Roll", 0, -1, is_rotation=True), 1, 0)
        ori_layout.addWidget(self._create_jog_button("+Roll", 0, 1, is_rotation=True), 1, 1)
        # Pitch buttons
        ori_layout.addWidget(self._create_jog_button("-Pitch", 1, -1, is_rotation=True), 2, 0)
        ori_layout.addWidget(self._create_jog_button("+Pitch", 1, 1, is_rotation=True), 2, 1)
        # Yaw buttons
        ori_layout.addWidget(self._create_jog_button("-Yaw", 2, -1, is_rotation=True), 3, 0)
        ori_layout.addWidget(self._create_jog_button("+Yaw", 2, 1, is_rotation=True), 3, 1)

        ori_group_box.setLayout(ori_layout)
        left_controls_layout.addWidget(ori_group_box)
        
        # --- NEW: Gripper Control ---
        self.gripper_group_box = QGroupBox("Gripper")
        gripper_layout = QGridLayout()

        # Gripper slider
        gripper_layout.addWidget(QLabel("Angle:"), 0, 0)
        self.gripper_slider = QSlider(Qt.Horizontal)
        self.gripper_slider.setRange(0, 180) # 0° (closed) to 180° (open)
        self.gripper_slider.setValue(0)
        self.gripper_slider.valueChanged.connect(self.update_gripper_label)
        gripper_layout.addWidget(self.gripper_slider, 0, 1)
        self.gripper_angle_label = QLabel("0°")
        gripper_layout.addWidget(self.gripper_angle_label, 0, 2)
        
        # Gripper buttons
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
        # --- End Gripper Control ---

        # Trajectory Planning
        traj_group_box = QGroupBox("Trajectory Planning")
        traj_layout = QVBoxLayout()

        # Name input
        name_layout = QHBoxLayout()
        name_layout.addWidget(QLabel("Trajectory Name:"))
        self.name_input = QLineEdit("my_new_path")
        name_layout.addWidget(self.name_input)
        traj_layout.addLayout(name_layout)

        # Controls
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
        
        # Run Trajectory
        run_layout = QHBoxLayout()
        
        # Column 1: Label and Dropdown (in a VBox)
        combo_layout = QVBoxLayout()
        combo_layout.addWidget(QLabel("Run Saved Trajectory:"))
        self.run_traj_combo = QComboBox()
        combo_layout.addStretch()
        combo_layout.addWidget(self.run_traj_combo)
        run_layout.addLayout(combo_layout)

        # Columns 2 & 3: Buttons
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

        traj_group_box.setLayout(traj_layout)
        left_controls_layout.addWidget(traj_group_box)

        # Zero/Rest quick actions
        quick_box = QGroupBox("Quick Actions")
        quick_layout = QHBoxLayout()
        zero_btn = QPushButton("Zero Joints")
        zero_btn.clicked.connect(self.go_zero)
        rest_btn = QPushButton("Rest Pose")
        rest_btn.clicked.connect(self.go_rest)
        quick_layout.addWidget(zero_btn)
        quick_layout.addWidget(rest_btn)
        quick_box.setLayout(quick_layout)
        left_controls_layout.addWidget(quick_box)

        left_controls_layout.addStretch()
        main_layout.addLayout(left_controls_layout)

        # Right side: State, Logs and direct commands
        right_side_layout = QVBoxLayout()
        
        # Robot State Display
        self.current_pos = [0.0, 0.0, 0.0]  # X, Y, Z
        self.current_ori = [0.0, 0.0, 0.0]  # Roll, Pitch, Yaw
        self.current_gripper_deg = 0.0 # NEW: Gripper angle
        state_group_box = QGroupBox("Robot State")
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

        refresh_btn = QPushButton("Refresh Position")
        refresh_btn.clicked.connect(self.refresh_state)
        state_layout.addWidget(refresh_btn, 4, 0, 1, 4)

        state_group_box.setLayout(state_layout)
        right_side_layout.addWidget(state_group_box)

        # Log Area
        self.log = []
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("background-color: #111111; color: #FF6600; font-family: 'Courier New', monospace;")
        right_side_layout.addWidget(self.log_text, 2) # Give more stretch factor

        # Direct Command Inputs
        direct_cmd_box = QGroupBox("Direct Commands")
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
        
        direct_cmd_box.setLayout(direct_cmd_layout)
        right_side_layout.addWidget(direct_cmd_box)
        
        self.send_btn = QPushButton("Send Direct Command")
        self.send_btn.clicked.connect(self.send_command_from_inputs)
        right_side_layout.addWidget(self.send_btn)
        
        main_layout.addLayout(right_side_layout)
        self.setLayout(main_layout)

    def check_gripper_presence(self):
        """Sends GET_STATUS to the controller and enables/disables the gripper UI."""
        self.parent.send_command("GET_STATUS")
        response = self.parent.receive_data(timeout_seconds=1.0)
        
        gripper_present = False
        if response and response.startswith("STATUS,gripper_present,"):
            try:
                status = response.split(',')[2]
                if status.lower() == 'true':
                    gripper_present = True
            except IndexError:
                pass # Stick with default of False

        self.gripper_group_box.setEnabled(gripper_present)
        if gripper_present:
            self.log_message("Gripper detected and enabled.")
        else:
            self.log_message("Gripper not detected. UI controls disabled.")
        
        # Sync initial positions
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
        increment_mm = float(self.pos_increment_combo.currentText())
        increment_m = increment_mm / 1000.0
        
        delta = {
            "x": [increment_m * direction, 0, 0],
            "y": [0, increment_m * direction, 0],
            "z": [0, 0, increment_m * direction]
        }[axis]
        
        cmd = f"MOVE_LINE_RELATIVE,{delta[0]},{delta[1]},{delta[2]}"
        self.parent.send_command(cmd)
        self.refresh_state() # Refresh position after move

    def refresh_trajectory_list(self):
        """Requests the list of saved trajectories from the robot and updates the dropdown."""
        self.log_message("Requesting trajectory list...")
        self.parent.send_command("GET_TRAJECTORIES")
        
        # We need a slightly longer timeout here as fetching file lists can take time.
        response = self.parent.receive_data(timeout_seconds=3.0)
        
        if response and response.startswith("TRAJECTORIES,"):
            # The controller returns a comma-separated list. Filter out any empty
            # strings that might result from trailing commas or other formatting issues.
            trajectories = [t.strip() for t in response.split(',')[1:] if t.strip()]
            
            self.run_traj_combo.clear()
            if trajectories:
                self.run_traj_combo.addItems(trajectories)
                self.log_message(f"Updated trajectories: {len(trajectories)} found.")
            else:
                self.log_message("No saved trajectories found on robot.")
        elif response:
            self.log_message(f"Failed to get trajectories. Response: {response}")
        else:
            self.log_message("Failed to get trajectories: No response from robot.")

    def _send_orientation_jog(self, axis_index, direction):
        increment_deg = float(self.ori_increment_combo.currentText())
        self.current_ori[axis_index] += increment_deg * direction
        
        r, p, y = self.current_ori
        cmd = f"SET_ORIENTATION,{r},{p},{y}"
        self.parent.send_command(cmd)
        self.update_state_display()

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
        
        # More robust receiver: listen for a specific response prefix,
        # ignoring other messages until a timeout.
        start_time = time.time()
        timeout = 2.0  # 2-second timeout
        response = None
        
        while time.time() - start_time < timeout:
            # Calculate remaining time for the socket to wait
            remaining_time = timeout - (time.time() - start_time)
            if remaining_time <= 0:
                break
                
            # Use the main window's receiver but with a short, non-blocking timeout
            candidate_response = self.parent.receive_data(timeout_seconds=remaining_time)
            
            if candidate_response:
                self.log_message(f"Received: {candidate_response}") # Log everything
                if candidate_response.startswith("CURRENT_POSE,"):
                    response = candidate_response
                    # Don't break yet, we also want to look for gripper state
                elif candidate_response.startswith("GRIPPER_STATE,"):
                    try:
                        parts = candidate_response.split(',')
                        self.current_gripper_deg = float(parts[1])
                        self.log_message(f"Gripper angle updated: {self.current_gripper_deg:.1f}°")
                        # Update the gripper slider to match current position
                        self.gripper_slider.blockSignals(True)  # Prevent triggering valueChanged
                        self.gripper_slider.setValue(int(self.current_gripper_deg))
                        self.gripper_slider.blockSignals(False)
                    except (ValueError, IndexError):
                        self.log_message("Error parsing gripper state response.")

        if response:
            try:
                parts = response.split(',')
                self.current_pos = [float(parts[1]), float(parts[2]), float(parts[3])]
                self.log_message(f"Position updated: {self.current_pos}")
            except (ValueError, IndexError):
                self.log_message("Error parsing position response.")
        else:
            self.log_message("Failed to get position from robot.")

        # Also request gripper state
        self.parent.send_command("GET_GRIPPER_STATE")
        # The logic to parse the response is handled in the same loop above now.
        
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
        # Send command immediately for live update
        self.set_gripper_from_slider()

    def set_gripper_from_slider(self):
        angle = self.gripper_slider.value()
        cmd = f"SET_GRIPPER,{angle}"
        self.parent.send_command(cmd)
        self.log_message(f"Sent: {cmd}")

    def open_gripper(self):
        # Set to maximum angle (180°) to open the gripper
        self.gripper_slider.setValue(120)

    def close_gripper(self):
        # Set to minimum angle (0°) to close the gripper
        self.gripper_slider.setValue(0)

    def send_command_from_inputs(self):
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

    def log_message(self, msg):
        self.log.append(msg)
        self.log_text.append(msg)
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    def go_home(self):
        self.parent.send_command("HOME")
        self.log_message("Sent Home")

    def go_zero(self):
        cmd = ",".join(map(str, POS_ZERO))
        self.parent.send_command(cmd)
        self.log_message(f"Sent Zero: {cmd}")

    def go_rest(self):
        cmd = ",".join(map(str, POS_REST))
        self.parent.send_command(cmd)
        self.log_message(f"Sent Rest: {cmd}")

    def get_position(self):
        self.parent.send_command("GET_POSITION")
        response = self.parent.receive_data()
        if response:
            self.log_message(f"Received: {response}")
        else:
            self.log_message("No response or timeout.")

    def send_move_line(self):
        input_str = self.move_line_input.text()
        self.parent.send_command(f"MOVE_LINE,{input_str}")

    def send_move_line_rel(self):
        input_str = self.move_line_rel_input.text()
        self.parent.send_command(f"MOVE_LINE_RELATIVE,{input_str}")

    def send_set_orientation(self):
        input_str = self.set_ori_input.text()
        self.parent.send_command(f"SET_ORIENTATION,{input_str}")

    def send_run_trajectory(self):
        name = self.run_traj_combo.currentText()
        if not name:
            QMessageBox.warning(self, "Selection Error", "Please select a trajectory to run, or refresh the list.")
            return

        is_looping = self.loop_traj_btn.isChecked()
        # Send command with loop status. Format: RUN_TRAJECTORY,name,use_cache,loop
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
        input_str = self.translate_input.text()
        self.parent.send_command(f"TRANSLATE,{input_str}")

    def send_rotate(self):
        input_str = self.rotate_input.text()
        self.parent.send_command(f"ROTATE,{input_str}")

class MainWindow(QMainWindow):
    def __init__(self, pi_ip="ai-pi.local"):
        super().__init__()
        self.setWindowTitle("Industrial Robot Controller")
        self.setGeometry(50, 50, 1280, 800)

        # --- Setup Footer/Status Bar FIRST ---
        # This is critical because some page constructors trigger actions
        # that try to update the status label upon initialization.
        
        # The home button is on the far left.
        self.home_btn_footer = QPushButton("Back to Home")
        self.home_btn_footer.clicked.connect(self.switch_to_home)
        self.statusBar().addWidget(self.home_btn_footer)

        # The status label is next to the home button.
        self.status_label_footer = QLabel("Robot State: Active")
        self.statusBar().addWidget(self.status_label_footer)

        # The E-Stop button is on the far right.
        estop_btn = QPushButton("EMERGENCY STOP")
        estop_btn.setStyleSheet("background-color: #CC0000; color: #FFFFFF;")
        estop_btn.clicked.connect(self.emergency_stop)
        self.statusBar().addPermanentWidget(estop_btn)

        # --- UDP Configuration ---
        self.PI_IP = pi_ip
        self.UDP_PORT = 3000
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.PI_RESOLVED_IP = socket.gethostbyname(self.PI_IP)
        except socket.gaierror:
            print("Could not resolve PI_IP. Using fallback.")
            self.PI_RESOLVED_IP = self.PI_IP  # Assume it's an IP if hostname fails

        # --- Page Creation and Layout ---
        self.stacked_widget = QStackedWidget()

        # Instantiate pages
        self.home = HomePage(self)
        self.tutorials = TutorialsPage(self)
        self.calibration = CalibrationPage(self)
        self.control = ControlPage(self)
        self.real_control = RealControlPage(self)

        # Now that all pages are created, perform initial setup for pages
        # that require sending commands.
        self.real_control.check_gripper_presence()

        # Wrap control-heavy pages inside scroll areas so that vertical overflow
        # is handled gracefully on 800-pixel-high displays.
        self.control_scroll = QScrollArea()
        self.control_scroll.setWidgetResizable(True)
        self.control_scroll.setWidget(self.control)

        self.real_control_scroll = QScrollArea()
        self.real_control_scroll.setWidgetResizable(True)
        self.real_control_scroll.setWidget(self.real_control)

        self.stacked_widget.addWidget(self.home)
        self.stacked_widget.addWidget(self.tutorials)
        self.stacked_widget.addWidget(self.calibration)
        self.stacked_widget.addWidget(self.control_scroll)
        self.stacked_widget.addWidget(self.real_control_scroll)
        self.setCentralWidget(self.stacked_widget)

        # Connect page changes to footer visibility updates
        self.stacked_widget.currentChanged.connect(self.update_footer_buttons)
        self.update_footer_buttons(self.stacked_widget.currentIndex()) # Set initial state

        print('Starting MainWindow init')
        print('UDP config done')
        print('Pages instantiated')
        print('Stacked widget setup done')
        print('Status bar setup done')
        print('MainWindow init complete')

    def send_command(self, command_str):
        try:
            self.sock.sendto(command_str.encode("utf-8"), (self.PI_IP, self.UDP_PORT))
            self.status_label_footer.setText(f"Sent: {command_str}")
            
            # Determine which logical page is currently visible even if it is
            # wrapped in a QScrollArea.
            current_widget = self.stacked_widget.currentWidget()
            if isinstance(current_widget, QScrollArea):
                current_page = current_widget.widget()
            else:
                current_page = current_widget
            
            if current_page == self.real_control:
                self.real_control.log_message(f"Sent: {command_str}")
        except Exception as e:
            self.status_label_footer.setText(f"Error sending: {e}")

    def receive_data(self, timeout_seconds=2.0):
        self.sock.settimeout(timeout_seconds)
        try:
            data, server_addr = self.sock.recvfrom(1024)
            # The IP check is too strict. In a local network, if we get any
            # response after sending a command, it's highly likely from the
            # intended robot. This is more robust against complex network configs.
            if server_addr[1] == self.UDP_PORT:
                response = data.decode("utf-8").strip()
                self.status_label_footer.setText(f"Received: {response}")
                return response
            else:
                self.status_label_footer.setText(f"Received from unexpected source: {server_addr}")
                return None
        except socket.timeout:
            self.status_label_footer.setText("Timeout waiting for response")
            return None
        except Exception as e:
            self.status_label_footer.setText(f"Receive error: {e}")
            return None
        finally:
            self.sock.settimeout(None)

    def run_test_square_sequence(self):
        try:
            self.send_command("GET_POSITION")
            response = self.receive_data()
            if not response or not response.startswith("CURRENT_POSITION,"):
                raise ValueError("Could not get starting position")

            parts = response.split(',')
            start_pos = np.array([float(parts[1]), float(parts[2]), float(parts[3])])

            p1 = start_pos
            p2 = p1 + np.array([0.0, 0.1, 0.0])
            p3 = p2 + np.array([0.1, 0.0, 0.0])
            p4 = p3 + np.array([0.0, -0.1, 0.0])
            square_path = [p1, p2, p3, p4, p1]

            for point in square_path:
                cmd = f"MOVE_LINE,{point[0]},{point[1]},{point[2]}"
                self.send_command(cmd)
                self.send_command("WAIT_FOR_IDLE")
                time.sleep(0.2)

            self.status_label_footer.setText("Square Test Complete")
            if self.stacked_widget.currentWidget() == self.real_control:
                self.real_control.log_message("Square Test Complete")
        except Exception as e:
            self.status_label_footer.setText(f"Square Test Error: {e}")
            if self.stacked_widget.currentWidget() == self.real_control:
                self.real_control.log_message(f"Square Test Error: {e}")

    def emergency_stop(self):
        self.send_command("STOP")  # Assuming STOP is the command for emergency
        self.status_label_footer.setText("Emergency Stop Activated!")

    def update_footer_buttons(self, index):
        """Shows/hides footer buttons based on the current page."""
        # The home button should not be visible when we are on the home page.
        is_home_page = (self.stacked_widget.widget(index) == self.home)
        self.home_btn_footer.setVisible(not is_home_page)

    def switch_to_home(self):
        """Navigate to the Home page."""
        self.stacked_widget.setCurrentWidget(self.home)

    def switch_to_tutorials(self):
        self.stacked_widget.setCurrentIndex(1)

    def switch_to_calibration(self):
        self.stacked_widget.setCurrentIndex(2)

    def switch_to_control(self):
        """Navigate to the Joint Control page (scroll wrapper)."""
        self.stacked_widget.setCurrentWidget(self.control_scroll)

    def switch_to_real_control(self):
        """Navigate to the Real Robot Control page (scroll wrapper) and refresh trajectories."""
        self.stacked_widget.setCurrentWidget(self.real_control_scroll)
        self.real_control.refresh_trajectory_list()

    def closeEvent(self, event):
        self.sock.close()
        super().closeEvent(event)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Robot Arm UI')
    parser.add_argument('--pi-ip', type=str, default='ai-pi.local',
                        help='The IP address of the Raspberry Pi.')
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setFont(QFont('Courier New', 12))
    app.setStyleSheet("""
    * {
        background-color: #000000;
        color: #FFFFFF;
    }
    QPushButton {
        background-color: #333333;
        border: 2px solid #FF6600;  /* Evangelion orange */
        padding: 5px;
        min-width: 100px;
        min-height: 50px; /* Reduced from 100px */
    }
    QPushButton:hover {
        background-color: #FF6600;
        color: #000000;
    }
    QPushButton:pressed {
        background-color: #CC0000;  /* Red for activation */
    }
    QPushButton:checked {
        background-color: #FF9933; /* A lighter orange for selection */
        color: #000000;
        border: 2px solid #FFFFFF;
    }
    QLineEdit {
        border: 1px solid #FF6600;
        padding: 2px;
    }
    QSlider {
        background-color: #222222;
    }
    QLabel {
        color: #FF6600;
    }
    QStatusBar {
        background-color: #111111;
        color: #FF6600;
    }
    QTableWidget::item {
        color: white;
    }
    QPushButton#JogButton {
        min-width: 40px;
        max-width: 40px;
        min-height: 40px;
        max-height: 40px;
    }
    """)
    window = MainWindow(pi_ip=args.pi_ip)
    window.show()
    print('App created')
    print('Stylesheet set')
    print('Window shown')
    print('Entering app.exec()')
    sys.exit(app.exec())
    print('App exited') # This won't print if hanging in exec