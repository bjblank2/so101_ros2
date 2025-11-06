#!/usr/bin/env python3
"""
SO101 Unified Node
A single ROS 2 node that operates in either 'leader' or 'follower' mode.

- Leader: Reads SO101 arm positions and publishes JointState messages.
- Follower: Subscribes to JointState messages and mirrors the motion on another SO101 arm.
"""

import math
import json
import yaml
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_srvs.srv import SetBool

# SO101 communication modules
from kiwi_robot.so101 import Motor, MotorCalibration, MotorNormMode, FeetechMotorsBus, OperatingMode


class SO101Node(Node):
    """Unified node for SO101 arm leader/follower operation."""

    def __init__(self):
        super().__init__('so101', automatically_declare_parameters_from_overrides=True)

        # Parameters
        self._declare_parameters({
            'mode': 'leader',  # leader | follower
            'port': '/dev/ttyACM0',
            'publish_rate': 50.0,
            'arm_id': 'so101_arm',
            'peer_arm_id': 'so101_peer',
            'use_degrees': True,
            'max_relative_target': 20.0,
            'calibration_file': '',
        })

        self.mode = str(self.get_parameter('mode').value).lower().strip()
        if self.mode not in ('leader', 'follower'):
            raise ValueError("Parameter 'mode' must be either 'leader' or 'follower'")

        self.port = str(self.get_parameter('port').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.arm_id = str(self.get_parameter('arm_id').value)
        self.peer_arm_id = str(self.get_parameter('peer_arm_id').value)
        self.use_degrees = bool(self.get_parameter('use_degrees').value)
        self.max_relative_target_deg = float(self.get_parameter('max_relative_target').value)
        self.max_relative_target_rad = math.radians(self.max_relative_target_deg)
        self.calibration_file = str(self.get_parameter('calibration_file').value)

        if not self.use_degrees:
            raise ValueError("use_degrees must be True until radian normalization is implemented")

        # Load calibration
        calibration = self.load_calibration_from_parameters('calibration')
        if calibration:
            self.get_logger().info('Loaded calibration from ROS parameters')
        elif self.calibration_file:
            try:
                calibration = self.load_calibration_from_file(self.calibration_file, 'calibration')
                self.get_logger().info(f'Loaded calibration from {self.calibration_file}')
            except Exception as e:
                self.get_logger().warning(f'Failed to load calibration: {e}')

        # Setup motor bus
        norm_mode_body = MotorNormMode.DEGREES
        self.bus = FeetechMotorsBus(
            port=self.port,
            motors={
                "shoulder_pan": Motor(1, "sts3215", norm_mode_body),
                "shoulder_lift": Motor(2, "sts3215", norm_mode_body),
                "elbow_flex": Motor(3, "sts3215", norm_mode_body),
                "wrist_flex": Motor(4, "sts3215", norm_mode_body),
                "wrist_roll": Motor(5, "sts3215", norm_mode_body),
                "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
            },
            calibration=calibration,
        )

        # Shared mapping
        self.joint_name_map = {
            'shoulder_pan': 'shoulder_pan',
            'shoulder_lift': 'shoulder_lift',
            'elbow': 'elbow_flex',
            'wrist_pitch': 'wrist_flex',
            'wrist_roll': 'wrist_roll',
            'wrist_yaw': 'gripper'
        }

        # Connect arm
        self.init_arm()

        if self.mode == 'leader':
            self._setup_leader_mode()
        else:
            self._setup_follower_mode()

    # -------------------------------
    # Common setup utilities
    # -------------------------------
    def _declare_parameters(self, defaults: dict):
        for name, default in defaults.items():
            if not self.has_parameter(name):
                self.declare_parameter(name, default)

    def load_calibration_from_parameters(self, prefix: str):
        params = self.get_parameters_by_prefix(prefix)
        if not params:
            return None
        raw_mapping = {}
        for key, parameter in params.items():
            parts = key.split('.')
            if len(parts) == 2:
                joint, field = parts
                raw_mapping.setdefault(joint, {})[field] = parameter.value
        return self._build_calibration(raw_mapping)

    def load_calibration_from_file(self, path: str, parameter_namespace: str):
        file_path = Path(path)
        if not file_path.exists():
            raise FileNotFoundError(f'Calibration file "{path}" does not exist')
        with file_path.open('r') as handle:
            calib_data = yaml.safe_load(handle) if file_path.suffix.lower() in {'.yaml', '.yml'} else json.load(handle)
        if isinstance(calib_data, dict):
            node_key = self.get_name()
            calib_data = calib_data.get(node_key, calib_data)
            if 'ros__parameters' in calib_data:
                calib_data = calib_data['ros__parameters']
            if parameter_namespace in calib_data:
                calib_data = calib_data[parameter_namespace]
        return self._build_calibration(calib_data)

    def _build_calibration(self, data):
        calibration = {}
        required = {'id', 'homing_offset', 'range_min', 'range_max'}
        for joint, calib in data.items():
            missing = required - calib.keys()
            if missing:
                raise ValueError(f'Calibration entry for {joint} missing fields {missing}')
            calibration[joint] = MotorCalibration(
                id=int(calib['id']),
                drive_mode=int(calib.get('drive_mode', 0)),
                homing_offset=int(calib['homing_offset']),
                range_min=int(calib['range_min']),
                range_max=int(calib['range_max']),
            )
        return calibration

    def init_arm(self):
        """Connect and configure the arm."""
        try:
            self.bus.connect(handshake=True)
            self.bus.disable_torque()
            self.bus.configure_motors()
            for motor in self.bus.motors:
                self.bus.write("Operating_Mode", motor, OperatingMode.POSITION.value)
            self.get_logger().info(f'Connected to SO101 arm on {self.port}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize arm: {e}')
            raise

    def on_shutdown(self):
        if self.bus.is_connected:
            try:
                self.bus.disconnect(disable_torque=True)
                self.get_logger().info(f'{self.mode.capitalize()} arm disconnected')
            except Exception as e:
                self.get_logger().error(f'Error disconnecting: {e}')

    # -------------------------------
    # Leader mode
    # -------------------------------
    def _setup_leader_mode(self):
        self.get_logger().info('Mode: LEADER')
        self.joint_state_pub = self.create_publisher(
            JointState, f'{self.arm_id}/joint_states', 30)
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_joint_states)

    def _publish_joint_states(self):
        if not self.bus.is_connected:
            return
        try:
            positions = self.bus.sync_read("Present_Position", normalize=True)
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = []
            msg.position = []
            for so101_name, val in positions.items():
                our_name = {v: k for k, v in self.joint_name_map.items()}.get(so101_name)
                if our_name:
                    msg.name.append(our_name)
                    msg.position.append(math.radians(val))
            msg.velocity = [0.0] * len(msg.position)
            msg.effort = [0.0] * len(msg.position)
            self.joint_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Leader publish error: {e}')

    # -------------------------------
    # Follower mode
    # -------------------------------
    def _setup_follower_mode(self):
        self.get_logger().info('Mode: FOLLOWER')
        self.teleop_enabled = False
        self.current_joint_states = {}
        self.last_peer_msg = None

        self.joint_state_sub = self.create_subscription(
            JointState, f'{self.peer_arm_id}/joint_states',
            self._peer_joint_callback, 30)

        self.local_pub = self.create_publisher(
            JointState, f'{self.arm_id}/joint_states', 30)

        self.teleop_service = self.create_service(
            SetBool, f'{self.arm_id}/set_teleop', self._teleop_service_cb)

        self.timer = self.create_timer(0.1, self._publish_local_states)

        self.get_logger().info('Follower teleoperation DISABLED by default')

    def _peer_joint_callback(self, msg):
        self.last_peer_msg = msg
        if not self.teleop_enabled or not self.bus.is_connected:
            return
        goal = {}
        for i, joint in enumerate(msg.name):
            so101_name = self.joint_name_map.get(joint)
            if so101_name:
                goal[so101_name] = math.degrees(msg.position[i])
        try:
            self.bus.sync_write("Goal_Position", goal)
        except Exception as e:
            self.get_logger().error(f'Follower command error: {e}')

    def _teleop_service_cb(self, req, res):
        self.teleop_enabled = req.data
        try:
            if req.data:
                self.bus.enable_torque()
                res.success = True
                res.message = 'Teleoperation enabled'
            else:
                self.bus.disable_torque()
                res.success = True
                res.message = 'Teleoperation disabled'
        except Exception as e:
            res.success = False
            res.message = str(e)
        return res

    def _publish_local_states(self):
        if not self.bus.is_connected:
            return
        try:
            positions = self.bus.sync_read("Present_Position", normalize=True)
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = []
            msg.position = []
            for so101_name, val in positions.items():
                our_name = {v: k for k, v in self.joint_name_map.items()}.get(so101_name)
                if our_name:
                    msg.name.append(our_name)
                    msg.position.append(math.radians(val))
            self.local_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Follower publish error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = SO101Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
