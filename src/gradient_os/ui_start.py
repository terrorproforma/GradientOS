# ui_start.py
# 
# This file is the main entry point for the UI.
# It contains the main window and the pages for the different features.
# It also contains the logic for the different features.
# 
# The main window is a QMainWindow that contains a QStackedWidget.
# The QStackedWidget contains the pages for the different features.
# The pages are switched by calling the switch_to_home, switch_to_tutorials,
# switch_to_calibration, switch_to_control, and switch_to_real_control functions.

import argparse
import socket
import sys
from contextlib import closing
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QStackedWidget, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QTextEdit, QTableWidget, QTableWidgetItem, QListWidget,
    QSlider, QStatusBar, QTextBrowser, QLineEdit, QComboBox, QMessageBox, QGridLayout,
    QGroupBox, QScrollArea, QButtonGroup, QCheckBox, QDial, QPlainTextEdit
)
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QColor, QFont, QIcon
from pathlib import Path

# For numerical operations
import numpy as np

import time
from functools import partial
from src.gradient_os.ui.widgets import set_label_text
from src.gradient_os.ui.network import UdpClient
from src.gradient_os.ui.pages.home_page import HomePage
from src.gradient_os.ui.pages.control_page import ControlPage
from src.gradient_os.ui.pages.real_control_page import RealControlPage
from src.gradient_os.ui.pages.tutorials_page import TutorialsPage
from src.gradient_os.ui.pages.calibration_page import CalibrationPage

from src.gradient_os.ui.constants import POS_ZERO, POS_HOME, POS_REST

# Moved to src/gradient_os/ui/pages/home_page.py
# Moved to src/gradient_os/ui/pages/control_page.py
# Moved to src/gradient_os/ui/pages/tutorials_page.py
# Moved to src/gradient_os/ui/pages/calibration_page.py
# Moved to src/gradient_os/ui/pages/real_control_page.py

class MainWindow(QMainWindow):
    def __init__(self, pi_ip="mini-arm.local"):
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
        # Disable word-wrap to reduce relayout cost
        self.status_label_footer.setWordWrap(False)
        self.statusBar().addWidget(self.status_label_footer)

        # The E-Stop button is on the far right.
        estop_btn = QPushButton("EMERGENCY STOP")
        estop_btn.setProperty("role", "danger")
        estop_btn.clicked.connect(self.emergency_stop)
        self.statusBar().addPermanentWidget(estop_btn)

        # --- UDP Configuration ---
        self.UDP_PORT = 3000
        self.PI_IP = self._select_initial_target(pi_ip)
        print(f"PI_IP is set to: {self.PI_IP}")
        self.client = UdpClient(self.PI_IP, self.UDP_PORT)
        self.client.start()
        # Main thread dispatcher to process messages without blocking UI
        self._dispatch_timer = QTimer(self)
        self._dispatch_timer.setTimerType(Qt.CoarseTimer)
        self._dispatch_timer.timeout.connect(self._process_incoming_messages)
        self._dispatch_timer.start(20)

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
        self.stacked_widget.currentChanged.connect(self._on_page_changed)
        self.update_footer_buttons(self.stacked_widget.currentIndex()) # Set initial state

        print('Starting MainWindow init')
        print('UDP config done')
        print('Pages instantiated')
        print('Stacked widget setup done')
        print('Status bar setup done')
        print('MainWindow init complete')

    def send_command(self, command_str):
        try:
            self.client.send(command_str)
            self._set_status_throttled(f"Sent: {command_str}")
            
            # Determine which logical page is currently visible even if it is
            # wrapped in a QScrollArea.
            current_widget = self.stacked_widget.currentWidget()
            if isinstance(current_widget, QScrollArea):
                current_page = current_widget.widget()
            else:
                current_page = current_widget
            
            if current_page == self.real_control:
                # Avoid logging high-frequency jog commands to keep UI responsive
                if not (
                    command_str.startswith("SET_JOG_VELOCITY")
                    or command_str.startswith("SET_GRIPPER_JOG_VELOCITY")
                    or command_str.startswith("SET_JOG_DEADMAN")
                    or command_str.startswith("SET_JOG_DEBUG")
                    or command_str.startswith("JOG_START")
                    or command_str.startswith("JOG_STOP")
                ):
                    self.real_control.log_message(f"Sent: {command_str}")
        except Exception as e:
            self._set_status_throttled(f"Error sending: {e}")

    def update_pi_ip(self, new_ip):
        """Updates the target IP/hostname used for UDP communication at runtime."""
        self.PI_IP = new_ip
        self.client.set_target(self.PI_IP, self.UDP_PORT)
        # Nudge footer status to reflect change
        set_label_text(self.status_label_footer, f"Target updated: {self.PI_IP}")

    def receive_data(self, timeout_seconds=2.0):
        """Non-blocking receive from the internal queue for backward compatibility."""
        response = self.client.try_receive_text(timeout_seconds)
        if response:
            self._set_status_throttled(f"Received: {response}")
            return response
        else:
            self._set_status_throttled("Timeout waiting for response")
            return None

    def _process_incoming_messages(self):
        for _ in range(50):
            response = self.client.try_receive_text(0.0)
            if not response:
                break
            self._set_status_throttled(f"Received: {response}")
            self._dispatch_robot_message(response)

    def _dispatch_robot_message(self, response: str):
        """Route robot responses to appropriate UI components (main thread)."""
        try:
            if response.startswith("CURRENT_POSE,"):
                parts = response.split(',')
                try:
                    pos = [float(parts[1]), float(parts[2]), float(parts[3])]
                    ori = [float(parts[4]), float(parts[5]), float(parts[6])]
                    self.real_control.current_pos = pos
                    self.real_control.current_ori = ori
                    self.real_control.update_state_display()
                    self.real_control.log_message(f"Pose updated: pos={pos} rpy={ori}")
                except (ValueError, IndexError):
                    pass
            elif response.startswith("GRIPPER_STATE,"):
                try:
                    parts = response.split(',')
                    deg = float(parts[1])
                    self.real_control.current_gripper_deg = deg
                    # Update slider without feedback loop
                    self.real_control.gripper_slider.blockSignals(True)
                    self.real_control.gripper_slider.setValue(int(deg))
                    self.real_control.gripper_slider.blockSignals(False)
                    self.real_control.update_state_display()
                    self.real_control.log_message(f"Gripper angle updated: {deg:.1f}°")
                except (ValueError, IndexError):
                    pass
            elif response.startswith("JOINT_ANGLES,"):
                parts = response.split(',')
                for i, joint in enumerate(self.control.joints):
                    try:
                        deg = float(parts[i+1])
                        slider = self.control.sliders[joint]
                        slider.blockSignals(True)
                        slider.setValue(int(deg))
                        slider.blockSignals(False)
                        set_label_text(self.control.value_labels[joint], f"{int(deg)}°")
                    except (IndexError, ValueError):
                        pass
                self.control.log_message("Joint angles synced.")
            elif response.startswith("STATUS,gripper_present,"):
                try:
                    present = response.split(',')[2].strip().lower() == 'true'
                    self.real_control.gripper_group_box.setEnabled(present)
                    msg = "Gripper detected and enabled." if present else "Gripper not detected. UI controls disabled."
                    self.real_control.log_message(msg)
                except Exception:
                    pass
            elif response.startswith("TRAJECTORIES,"):
                items = [t.strip() for t in response.split(',')[1:] if t.strip()]
                self.real_control.run_traj_combo.clear()
                if items:
                    self.real_control.run_traj_combo.addItems(items)
                    self.real_control.log_message(f"Updated trajectories: {len(items)} found.")
                else:
                    self.real_control.log_message("No saved trajectories found on robot.")
            else:
                # Forward miscellaneous messages to the active page log if applicable
                current_widget = self.stacked_widget.currentWidget()
                if isinstance(current_widget, QScrollArea):
                    current_page = current_widget.widget()
                else:
                    current_page = current_widget
                try:
                    if current_page == self.real_control:
                        # Gate noisy responses when logs are hidden and diagnostics off
                        try:
                            if (self.real_control.show_logs_checkbox.isChecked() or
                                self.real_control.diagnostics_checkbox.isChecked()):
                                self.real_control.log_message(response)
                        except Exception:
                            pass
                    elif current_page == self.control:
                        self.control.log_message(response)
                except Exception:
                    pass
        except Exception:
            pass

    def _on_page_changed(self, index):
        # Maintain footer buttons visibility
        self.update_footer_buttons(index)
        # If the Joint Control page becomes visible, request joint angles to sync sliders
        try:
            widget = self.stacked_widget.widget(index)
            page = widget.widget() if isinstance(widget, QScrollArea) else widget
            if page == self.control:
                self.send_command("GET_JOINT_ANGLES")
        except Exception:
            pass

    def _set_status_throttled(self, text: str):
        # Rate-limit status bar updates to avoid excessive repaints
        if not hasattr(self, "_status_last_text"):
            self._status_last_text = ""
        if not hasattr(self, "_status_next_allowed"):
            self._status_next_allowed = 0.0
        now = time.time()
        # Update immediately if different and interval passed; otherwise coalesce
        if (text != self._status_last_text) and (now >= self._status_next_allowed):
            set_label_text(self.status_label_footer, text)
            self._status_last_text = text
            self._status_next_allowed = now + 0.1  # 10 Hz max
        else:
            # Schedule a delayed update if not already pending
            if not hasattr(self, "_status_timer"):
                self._status_timer = QTimer(self)
                self._status_timer.setSingleShot(True)
                self._status_timer.timeout.connect(lambda: self._set_status_throttled(self._status_last_text))
            self._status_last_text = text
            if not self._status_timer.isActive():
                remaining = max(0, int((self._status_next_allowed - now) * 1000))
                self._status_timer.start(remaining if remaining > 0 else 10)

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

            set_label_text(self.status_label_footer, "Square Test Complete")
            if self.stacked_widget.currentWidget() == self.real_control:
                self.real_control.log_message("Square Test Complete")
        except Exception as e:
            set_label_text(self.status_label_footer, f"Square Test Error: {e}")
            if self.stacked_widget.currentWidget() == self.real_control:
                self.real_control.log_message(f"Square Test Error: {e}")

    def emergency_stop(self):
        self.send_command("STOP")  # Assuming STOP is the command for emergency
        set_label_text(self.status_label_footer, "Emergency Stop Activated!")

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
        try:
            self.client.stop()
        except Exception:
            pass
        super().closeEvent(event)

    def repolish(self, widget):
        try:
            widget.style().unpolish(widget)
            widget.style().polish(widget)
        except Exception:
            pass

    def _select_initial_target(self, configured_host: str) -> str:
        """Prefer localhost/loopback if the controller is running locally."""
        manual_override = configured_host and configured_host != "mini-arm.local"
        if manual_override:
            # Caller explicitly pointed us to a host; honour it but verify reachability.
            return configured_host if self._probe_host(configured_host) else configured_host

        candidates = ["127.0.0.1", "localhost"]
        if configured_host:
            candidates.append(configured_host)

        for host in candidates:
            if self._probe_host(host):
                if host != configured_host:
                    print(f"[UI] Autodetected controller at {host}; overriding default target.")
                return host

        return configured_host

    def _probe_host(self, host: str, timeout: float = 0.25) -> bool:
        """Send a GET_STATUS probe to the controller and confirm a STATUS reply."""
        try:
            addr_info = socket.getaddrinfo(host, self.UDP_PORT, socket.AF_UNSPEC, socket.SOCK_DGRAM)
        except socket.gaierror:
            return False

        for family, socktype, proto, _canon, sockaddr in addr_info:
            try:
                with closing(socket.socket(family, socktype, proto)) as sock:
                    sock.settimeout(timeout)
                    sock.sendto(b"GET_STATUS", sockaddr)
                    data, _ = sock.recvfrom(128)
                    if data.startswith(b"STATUS"):
                        return True
            except (OSError, TimeoutError):
                continue
        return False

def main():
    parser = argparse.ArgumentParser(description='Robot Arm UI')
    parser.add_argument('--pi-ip', type=str, default='mini-arm.local',
                        help='The IP address of the Raspberry Pi.')
    args = parser.parse_args()

    app = QApplication(sys.argv)
    app.setFont(QFont('Courier New', 12))
    # Load global stylesheet from QSS file in the ui folder (relative path)
    try:
        qss_path = Path(__file__).resolve().parent / "ui" / "app.qss"
        with open(qss_path, 'r') as f:
            app.setStyleSheet(f.read())
    except Exception as e:
        print(f"Warning: failed to load stylesheet: {e}")
    window = MainWindow(pi_ip=args.pi_ip)
    window.show()
    print('App created')
    print('Stylesheet set')
    print('Window shown')
    print('Entering app.exec()')
    sys.exit(app.exec())
    print('App exited') # This won't print if hanging in exec

if __name__ == "__main__":
    main()
