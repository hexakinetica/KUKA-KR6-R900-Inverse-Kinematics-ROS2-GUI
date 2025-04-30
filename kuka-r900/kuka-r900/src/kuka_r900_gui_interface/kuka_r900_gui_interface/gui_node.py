#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QSlider, QComboBox, QFrame, QPushButton, QCheckBox
)
from PyQt5.QtCore import Qt, QTimer
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Bool
from math import radians, degrees
from scipy.spatial.transform import Rotation as R

class IKPublisherNode(Node):
    # Changed init to __init__ and added super().__init__
    def __init__(self):
        super().__init__('ik_gui_node')
        self.pose_pub = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.setting_pub = self.create_publisher(String, '/ik_settings', 10)
        self.joint_pub = self.create_publisher(JointTrajectory, '/position_trajectory_controller/joint_trajectory', 10)
        self.mode_pub = self.create_publisher(Bool, '/ik_control_mode', 10)

        self.latest_pose = PoseStamped()
        self.latest_setting = {'elbow': 'up', 'wrist': 'normal'}
        self.current_pose = None
        self.ik_status = "Unknown"
        self.last_joint_state = []
        self.new_fk_received = False
        self.new_joints_received = False

        self.current_pose_sub = self.create_subscription(
            PoseStamped, '/current_pose', self.current_pose_callback, 10)
        self.ik_status_sub = self.create_subscription(
            String, '/ik_status', self.ik_status_callback, 10)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)

    def publish_pose(self):
        self.latest_pose.header.stamp = self.get_clock().now().to_msg()
        self.pose_pub.publish(self.latest_pose)

    def publish_setting(self):
        setting_msg = String()
        setting_msg.data = f"{self.latest_setting['elbow']},{self.latest_setting['wrist']}"
        self.setting_pub.publish(setting_msg)

    def publish_joint_command(self, joint_angles_deg):
        msg = JointTrajectory()
        # Ensure joint names match the robot description
        # This is a common convention, but verify against your robot setup
        msg.joint_names = [f'joint_a{i+1}' for i in range(6)]
        point = JointTrajectoryPoint()
        point.positions = [radians(deg) for deg in joint_angles_deg]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = 100_000_000 # Small delay
        msg.points.append(point)
        self.joint_pub.publish(msg)

    def publish_mode(self, ik_enabled):
        self.mode_pub.publish(Bool(data=ik_enabled))

    def current_pose_callback(self, msg):
        self.current_pose = msg.pose
        self.new_fk_received = True

    def ik_status_callback(self, msg):
        self.ik_status = msg.data

    def joint_state_callback(self, msg):
        # Assuming msg.position contains angles for joints A1-A6 in order
        # You might need to map positions based on msg.name if order isn't guaranteed
        self.last_joint_state = msg.position
        self.new_joints_received = True

class IKGUI(QWidget):
    # Changed init to __init__
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle('KUKA KR6 R900 Control Panel')
        # Adjusted initial size slightly for better fit
        self.resize(500, 800)

        layout = QVBoxLayout()
        title = QLabel("Pose / Joint Control Panel")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 18pt; font-weight: bold; padding: 10px;")
        layout.addWidget(title)

        self.mode_checkbox = QCheckBox("Control Joint Angles (FK Mode)") # Added clearer text
        self.mode_checkbox.stateChanged.connect(self.mode_changed)
        layout.addWidget(self.mode_checkbox)

        self.sliders = {}
        self.labels = {}
        self.dragging = {}
        self.cartesian_axes = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
        self.joint_axes = [f"A{i+1}" for i in range(6)]
        # Default to Cartesian mode (IK enabled)
        self.active_axes = self.cartesian_axes
        self.pending_sync = False

        self.slider_layout = QVBoxLayout()
        layout.addLayout(self.slider_layout)

        self.rebuild_sliders()

        combo_layout = QHBoxLayout()
        self.elbow_combo = QComboBox()
        self.elbow_combo.addItems(['up', 'down'])
        self.elbow_combo.currentIndexChanged.connect(self.setting_changed)
        combo_layout.addWidget(QLabel("Elbow Configuration:")) # Added clearer label
        combo_layout.addWidget(self.elbow_combo)

        self.wrist_combo = QComboBox()
        self.wrist_combo.addItems(['normal', 'flipped'])
        self.wrist_combo.currentIndexChanged.connect(self.setting_changed)
        combo_layout.addWidget(QLabel("Wrist Configuration:")) # Added clearer label
        combo_layout.addWidget(self.wrist_combo)
        layout.addLayout(combo_layout)

        btn_layout = QHBoxLayout()
        home_btn = QPushButton("Set Home Pose / Joints") # Unified button text
        home_btn.clicked.connect(self.set_home_pose)
        btn_layout.addWidget(home_btn)
        layout.addLayout(btn_layout)

        # Use QFrame to visually group status labels
        status_frame = QFrame()
        status_layout = QVBoxLayout()
        status_frame.setLayout(status_layout)
        status_frame.setFrameShape(QFrame.StyledPanel)
        status_frame.setFrameShadow(QFrame.Sunken)

        self.pose_status_label = QLabel("Current Pose: Waiting...")
        self.pose_status_label.setStyleSheet("padding: 5px;")
        status_layout.addWidget(self.pose_status_label)

        self.joint_angle_label = QLabel("Joint Angles: Waiting...")
        self.joint_angle_label.setStyleSheet("padding: 5px;")
        status_layout.addWidget(self.joint_angle_label)

        self.ik_status_label = QLabel("IK Status: Unknown")
        self.ik_status_label.setStyleSheet("padding: 5px; font-weight: bold;")
        status_layout.addWidget(self.ik_status_label)

        layout.addWidget(status_frame)

        # Add stretch to push content to the top
        layout.addStretch()

        self.setLayout(layout)

        # Start the timer immediately to begin updating
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ros)
        self.timer.start(50) # Update GUI more frequently (50ms)

        # Publish initial mode (IK enabled)
        self.ros_node.publish_mode(True)

    def rebuild_sliders(self):
        # Clear existing widgets from the slider layout
        for i in reversed(range(self.slider_layout.count())):
            widget = self.slider_layout.itemAt(i).widget()
            if widget:
                self.slider_layout.removeWidget(widget) # Remove from layout
                widget.deleteLater() # Mark for deletion

        self.sliders.clear()
        self.labels.clear()
        self.dragging.clear()

        for axis in self.active_axes:
            label = QLabel(f"{axis}: 0.000")
            label.setStyleSheet("font-size: 12pt; font-weight: bold;")
            slider = QSlider(Qt.Horizontal)

            if axis in ['X', 'Y', 'Z']:
                # Using integer sliders for mm, adjust range as needed
                slider.setRange(-1200, 1200) # Adjusted range slightly
                # Initial values might need tuning for your robot
                slider.setValue(450 if axis == 'X' else (800 if axis == 'Z' else 0))
            elif axis in ['Roll', 'Pitch', 'Yaw']:
                slider.setRange(-180, 180)
                 # Pitch 180 is common for a downward-facing tool
                slider.setValue(0 if axis != 'Pitch' else 180)
            else: # Joint angles A1-A6
                # Joint limits in degrees. These are general ranges, verify for your robot.
                if axis == 'A1': slider.setRange(-170, 170)
                elif axis == 'A2': slider.setRange(-190, 45) # e.g., KUKA limits
                elif axis == 'A3': slider.setRange(-120, 156)
                elif axis == 'A4': slider.setRange(-185, 185)
                elif axis == 'A5': slider.setRange(-120, 120)
                elif axis == 'A6': slider.setRange(-350, 350) # Typically a wide range
                else: slider.setRange(-180, 180) # Default if not specified

                slider.setValue(0) # Default joint angles to 0

            # Disconnect previous signals before connecting new ones (optional but good practice)
            try: slider.valueChanged.disconnect()
            except TypeError: pass
            try: slider.sliderPressed.disconnect()
            except TypeError: pass
            try: slider.sliderReleased.disconnect()
            except TypeError: pass

            slider.valueChanged.connect(self.slider_changed)
            # Use partial to pass the axis name to the slot
            from functools import partial
            slider.sliderPressed.connect(partial(self.set_dragging, axis, True))
            slider.sliderReleased.connect(partial(self.set_dragging, axis, False))


            self.slider_layout.addWidget(label)
            self.slider_layout.addWidget(slider)

            self.sliders[axis] = slider
            self.labels[axis] = label
            self.dragging[axis] = False # Initialize dragging status

        self.slider_changed() # Update labels with initial values

    def set_dragging(self, axis, value):
        self.dragging[axis] = value

    def slider_changed(self):
        for axis, slider in self.sliders.items():
            val = slider.value()
            if axis in ['X', 'Y', 'Z']:
                self.labels[axis].setText(f"{axis}: {val:.1f} mm")
            elif axis in ['Roll', 'Pitch', 'Yaw']:
                 self.labels[axis].setText(f"{axis}: {val:.1f} °") # Use degree symbol
            else: # Joint Angles
                 self.labels[axis].setText(f"{axis}: {val:.1f} °") # Use degree symbol

    def setting_changed(self):
        self.ros_node.latest_setting['elbow'] = self.elbow_combo.currentText()
        self.ros_node.latest_setting['wrist'] = self.wrist_combo.currentText()
        # Publish settings immediately when changed in IK mode
        if not self.mode_checkbox.isChecked():
             self.ros_node.publish_setting()

    def mode_changed(self):
        ik_enabled = not self.mode_checkbox.isChecked()
        self.ros_node.publish_mode(ik_enabled)
        self.active_axes = self.joint_axes if self.mode_checkbox.isChecked() else self.cartesian_axes
        self.rebuild_sliders()
        self.pending_sync = True
        # Set appropriate initial values or trigger sync
        if self.mode_checkbox.isChecked(): # Switched to Joint mode
            # Attempt to sync sliders with current joint state immediately
             if self.ros_node.last_joint_state:
                 deg_vals = [degrees(j) for j in self.ros_node.last_joint_state[:6]]
                 for i, axis in enumerate(self.joint_axes):
                     if axis in self.sliders:
                         # Ensure value is within slider range before setting
                         limited_val = max(self.sliders[axis].minimum(), min(self.sliders[axis].maximum(), int(deg_vals[i])))
                         self.sliders[axis].setValue(limited_val)
                 self.slider_changed()
                 self.pending_sync = False # Sync done
        else: # Switched to Cartesian mode
            # Attempt to sync sliders with current FK pose immediately
            if self.ros_node.current_pose:
                 p = self.ros_node.current_pose.position
                 o = self.ros_node.current_pose.orientation
                 try:
                     rpy = R.from_quat([o.x, o.y, o.z, o.w]).as_euler('xyz', degrees=True)
                     self.sliders['X'].setValue(int(p.x * 1000))
                     self.sliders['Y'].setValue(int(p.y * 1000))
                     self.sliders['Z'].setValue(int(p.z * 1000))
                     self.sliders['Roll'].setValue(int(rpy[0]))
                     self.sliders['Pitch'].setValue(int(rpy[1]))
                     self.sliders['Yaw'].setValue(int(rpy[2]))
                     self.slider_changed()
                     self.pending_sync = False # Sync done
                 except Exception as e:
                     self.ros_node.get_logger().warn(f"Failed to convert quaternion to Euler for sync: {e}")


    def update_ros(self):
        # sync sliders only once after mode change if data is available
        if self.pending_sync:
            if self.mode_checkbox.isChecked() and self.ros_node.new_joints_received:
                if self.ros_node.last_joint_state and len(self.ros_node.last_joint_state) >= 6:
                    deg_vals = [degrees(j) for j in self.ros_node.last_joint_state[:6]]
                    for i, axis in enumerate(self.joint_axes):
                        if axis in self.sliders:
                            # Ensure value is within slider range before setting
                            limited_val = max(self.sliders[axis].minimum(), min(self.sliders[axis].maximum(), int(deg_vals[i])))
                            self.sliders[axis].setValue(limited_val)
                    self.ros_node.new_joints_received = False
                    self.pending_sync = False
                    self.slider_changed() # Update labels after sync

            elif not self.mode_checkbox.isChecked() and self.ros_node.new_fk_received:
                if self.ros_node.current_pose:
                    p = self.ros_node.current_pose.position
                    o = self.ros_node.current_pose.orientation
                    try:
                        rpy = R.from_quat([o.x, o.y, o.z, o.w]).as_euler('xyz', degrees=True)
                        self.sliders['X'].setValue(int(p.x * 1000))
                        self.sliders['Y'].setValue(int(p.y * 1000))
                        self.sliders['Z'].setValue(int(p.z * 1000))
                        self.sliders['Roll'].setValue(int(rpy[0]))
                        self.sliders['Pitch'].setValue(int(rpy[1]))
                        self.sliders['Yaw'].setValue(int(rpy[2]))
                        self.ros_node.new_fk_received = False
                        self.pending_sync = False
                        self.slider_changed() # Update labels after sync
                    except Exception as e:
                         self.ros_node.get_logger().warn(f"Failed to convert quaternion to Euler during update_ros sync: {e}")


        # Publish commands only if a slider is being dragged in the active mode
        if self.mode_checkbox.isChecked(): # FK (Joint) Mode
            if any(self.dragging[a] for a in self.joint_axes if a in self.dragging):
                joint_angles_deg = [self.sliders[axis].value() for axis in self.joint_axes if axis in self.sliders]
                # Basic check for 6 values before publishing
                if len(joint_angles_deg) == 6:
                     self.ros_node.publish_joint_command(joint_angles_deg)
                else:
                    self.ros_node.get_logger().warn("Mismatch in number of joint sliders vs expected joints.")
        else: # IK (Cartesian) Mode
            if any(self.dragging[a] for a in self.cartesian_axes if a in self.dragging):
                pose = PoseStamped()
                pose.header.frame_id = 'base_link' # Verify your robot's base frame name
                pose.pose.position.x = self.sliders['X'].value() / 1000.0
                pose.pose.position.y = self.sliders['Y'].value() / 1000.0
                pose.pose.position.z = self.sliders['Z'].value() / 1000.0

                rpy = [radians(self.sliders[a].value()) for a in ['Roll', 'Pitch', 'Yaw']]
                try:
                    quat = R.from_euler('xyz', rpy).as_quat()
                    pose.pose.orientation.x = quat[0]
                    pose.pose.orientation.y = quat[1]
                    pose.pose.orientation.z = quat[2]
                    pose.pose.orientation.w = quat[3]
                    self.ros_node.latest_pose = pose
                    self.ros_node.publish_pose()
                    self.ros_node.publish_setting()
                except Exception as e:
                     self.ros_node.get_logger().warn(f"Failed to convert Euler to quaternion for publishing: {e}")


        # Update current pose label
        if self.ros_node.current_pose:
            p = self.ros_node.current_pose.position
            o = self.ros_node.current_pose.orientation
            try:
                # Display using the same Euler convention (xyz)
                rpy = R.from_quat([o.x, o.y, o.z, o.w]).as_euler('xyz')
                rpy_deg = [degrees(v) for v in rpy]
                self.pose_status_label.setText(
                    f"Current Pose: X={p.x*1000.0:.1f} Y={p.y*1000.0:.1f} Z={p.z*1000.0:.1f} mm | "
                    f"Rx={rpy_deg[0]:.1f}° Ry={rpy_deg[1]:.1f}° Rz={rpy_deg[2]:.1f}°")
            except Exception as e:
                 self.ros_node.get_logger().warn(f"Failed to convert quaternion for current pose display: {e}")
                 self.pose_status_label.setText("Current Pose: Error converting orientation")
        else:
             self.pose_status_label.setText("Current Pose: Waiting...")


        # Update joint angle label
        if self.ros_node.last_joint_state and len(self.ros_node.last_joint_state) >= 6:
            # Assuming first 6 positions correspond to A1-A6
            deg_vals = [degrees(j) for j in self.ros_node.last_joint_state[:6]]
            self.joint_angle_label.setText(
                f"Joint Angles: A1={deg_vals[0]:.1f}° A2={deg_vals[1]:.1f}° A3={deg_vals[2]:.1f}° "
                f"A4={deg_vals[3]:.1f}° A5={deg_vals[4]:.1f}° A6={deg_vals[5]:.1f}°")
        else:
             self.joint_angle_label.setText("Joint Angles: Waiting...")


        # Update IK status label
        if self.ros_node.ik_status == "ok":
            self.ik_status_label.setText("IK Status: OK")
            self.ik_status_label.setStyleSheet("color: green; font-weight: bold; padding: 5px;")
        elif self.ros_node.ik_status == "fail":
            self.ik_status_label.setText("IK Status: FAIL")
            self.ik_status_label.setStyleSheet("color: red; font-weight: bold; padding: 5px;")
        else:
            self.ik_status_label.setText(f"IK Status: {self.ros_node.ik_status}") # Display other statuses

    def set_home_pose(self):
        # Stop dragging when home is clicked to prevent publishing intermediate values
        for axis in self.dragging:
            self.dragging[axis] = False

        if not self.mode_checkbox.isChecked(): # IK (Cartesian) Mode
            # Home pose values in mm and degrees
            values = {'X': 450, 'Y': 0, 'Z': 800, 'Roll': 0, 'Pitch': 180, 'Yaw': 0}
            for axis, val in values.items():
                if axis in self.sliders:
                     # Ensure value is within slider range before setting
                     limited_val = max(self.sliders[axis].minimum(), min(self.sliders[axis].maximum(), val))
                     self.sliders[axis].setValue(limited_val)
            self.slider_changed() # Update labels immediately
            # Publish the home pose and settings
            pose = PoseStamped()
            pose.header.frame_id = 'base_link'
            pose.pose.position.x = values['X'] / 1000.0
            pose.pose.position.y = values['Y'] / 1000.0
            pose.pose.position.z = values['Z'] / 1000.0
            rpy = [radians(values[a]) for a in ['Roll', 'Pitch', 'Yaw']]
            try:
                quat = R.from_euler('xyz', rpy).as_quat()
                pose.pose.orientation.x = quat[0]
                pose.pose.orientation.y = quat[1]
                pose.pose.orientation.z = quat[2]
                pose.pose.orientation.w = quat[3]
                self.ros_node.latest_pose = pose
                self.ros_node.publish_pose()
                self.ros_node.publish_setting()
            except Exception as e:
                 self.ros_node.get_logger().warn(f"Failed to convert Euler to quaternion for home pose: {e}")


        else: # FK (Joint) Mode
            # Home joint angles in degrees (typically all zeros)
            values = {f"A{i+1}": 0 for i in range(6)}
            joint_angles_deg = []
            for axis, val in values.items():
                if axis in self.sliders:
                     # Ensure value is within slider range before setting
                     limited_val = max(self.sliders[axis].minimum(), min(self.sliders[axis].maximum(), val))
                     self.sliders[axis].setValue(limited_val)
                     joint_angles_deg.append(limited_val)
            self.slider_changed() # Update labels immediately
            # Publish the home joint command
            if len(joint_angles_deg) == 6:
                 self.ros_node.publish_joint_command(joint_angles_deg)
            else:
                 self.ros_node.get_logger().warn("Mismatch in number of joint sliders for home command.")


def main():
    rclpy.init(args=sys.argv) # Pass sys.argv to rclpy init

    node = IKPublisherNode()

    app = QApplication(sys.argv)
    gui = IKGUI(node)
    gui.show()

    # Timer to spin ROS2 node integrated with PyQt event loop
    spin_timer = QTimer()
    # Use a lambda to call spin_once with the node and a timeout
    spin_timer.timeout.connect(lambda: rclpy.spin_once(node, timeout_sec=0.01)) # Spin faster
    spin_timer.start(10) # Spin very frequently (10ms)

    # Execute the PyQt application
    sys.exit(app.exec_()) # Use sys.exit to ensure clean exit

    # These lines will be reached after app.exec_() exits
    node.destroy_node()
    rclpy.shutdown()

# Fixed the if name == main block
if __name__ == '__main__':
    main()