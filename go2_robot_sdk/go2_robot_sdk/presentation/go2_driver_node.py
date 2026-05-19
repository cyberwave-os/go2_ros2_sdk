# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import asyncio
import logging
import os
import threading
import time
from typing import Dict, Any

from aiortc import MediaStreamTrack
from cv_bridge import CvBridge

from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos_overriding_options import QoSOverridingOptions
from rcl_interfaces.msg import SetParametersResult
from tf2_ros import TransformBroadcaster

from geometry_msgs.msg import Twist, PoseStamped
from go2_interfaces.msg import Go2State
from go2_interfaces.msg import LowState, VoxelMapCompressed, WebRtcReq
from sensor_msgs.msg import Imu, PointCloud2, JointState, Joy, Image, CameraInfo, BatteryState
from nav_msgs.msg import Odometry

from ..domain.entities import RobotConfig, RobotData, CameraData
from ..application.services import RobotDataService, RobotControlService
from ..infrastructure.ros2 import ROS2Publisher
from ..infrastructure.webrtc import WebRTCAdapter

logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def _configure_video_decoder_logging() -> None:
    """Reduce noisy H264 decoder logs from aiortc/PyAV."""
    logging.getLogger("aiortc.codecs.h264").setLevel(logging.ERROR)
    logging.getLogger("aiortc.codecs").setLevel(logging.ERROR)

    try:
        import av  # type: ignore
        av.logging.set_level(av.logging.ERROR)
    except Exception:
        # PyAV may be unavailable or logging config may fail; ignore safely
        pass


class Go2DriverNode(Node):
    """Main Go2 driver node - entry point to the application"""

    def __init__(self, event_loop=None):
        super().__init__('go2_driver_node')  # Clean architecture main driver
        self.event_loop = event_loop

        _configure_video_decoder_logging()
        
        # Configuration initialization
        self.config = self._setup_configuration()
        
        # Infrastructure initialization
        self.publishers_dict = self._setup_publishers()
        self.broadcaster = TransformBroadcaster(self, qos=QoSProfile(depth=10))
        self.bridge = CvBridge()
        
        # Architecture layers initialization
        self.ros2_publisher = ROS2Publisher(
            node=self,
            config=self.config,
            publishers=self.publishers_dict,
            broadcaster=self.broadcaster
        )
        
        self.robot_data_service = RobotDataService(self.ros2_publisher)
        
        self.webrtc_adapter = WebRTCAdapter(
            config=self.config,
            on_validated_callback=self._on_robot_validated,
            on_video_frame_callback=self._on_video_frame if self.config.enable_video else None,
            event_loop=self.event_loop
        )
        
        self.webrtc_adapter.data_stall_timeout_sec = (
            self.get_parameter('data_stall_timeout_sec').get_parameter_value().double_value
        )
        self.webrtc_adapter.max_reconnect_delay_sec = (
            self.get_parameter('max_reconnect_delay_sec').get_parameter_value().double_value
        )
        self.webrtc_adapter.reconnect_cooldown_sec = (
            self.get_parameter('reconnect_cooldown_sec').get_parameter_value().double_value
        )

        self.robot_control_service = RobotControlService(self.webrtc_adapter)
        
        # Set callback for data
        self.webrtc_adapter.set_data_callback(self._on_robot_data_received)
        
        # Subscribers initialization
        self._setup_subscribers()
        
        # State
        self.joy_state = Joy()
        self._last_cmd_vel_log = 0.0
        self._last_cmd_vel_out_time = 0.0

        # cmd_vel timeout watchdog — sends Move(0,0,0) to the robot when no
        # cmd_vel is received within the timeout window.  Uses a dedicated
        # thread so it fires reliably even when the ROS2 executor is starved
        # on resource-constrained hardware (e.g. Jetson).
        self._cmd_vel_timeout = max(
            0.05,
            self.get_parameter('cmd_vel_timeout_sec').get_parameter_value().double_value,
        )
        self._cmd_vel_lock = threading.Lock()
        self._last_cmd_vel_time: float | None = None
        self._last_cmd_vel_nonzero = False
        self._cmd_vel_watchdog_shutdown = threading.Event()
        self._cmd_vel_watchdog_thread = threading.Thread(
            target=self._cmd_vel_watchdog_loop, daemon=True
        )
        self._cmd_vel_watchdog_thread.start()
        self.get_logger().info(
            f"cmd_vel timeout watchdog started (timeout={self._cmd_vel_timeout:.2f}s)"
        )

    def _setup_configuration(self) -> RobotConfig:
        """Configuration setup"""
        robot_ip = os.getenv('ROBOT_IP', os.getenv('GO2_IP', ''))
        token = os.getenv('ROBOT_TOKEN', os.getenv('GO2_TOKEN', ''))
        conn_type = os.getenv('CONN_TYPE', '')
        aes_128_key = os.getenv('GO2_AES_128_KEY', '')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_ip', robot_ip),
                ('token', token),
                ('conn_type', conn_type),
                ('aes_128_key', aes_128_key),
                ('enable_video', True),
                ('decode_lidar', True),
                ('lite_subscriptions', False),
                ('publish_raw_voxel', False),
                ('obstacle_avoidance', False),
                ('cmd_vel_timeout_sec', 0.25),
                ('data_stall_timeout_sec', 15.0),
                ('max_reconnect_delay_sec', 60.0),
                ('reconnect_cooldown_sec', 45.0),
            ]
        )

        self.add_on_set_parameters_callback(self._on_set_parameters)

        # Get parameter values
        config = RobotConfig.from_params(
            robot_ip=self.get_parameter('robot_ip').get_parameter_value().string_value,
            token=self.get_parameter('token').get_parameter_value().string_value,
            conn_type=self.get_parameter('conn_type').get_parameter_value().string_value,
            enable_video=self.get_parameter('enable_video').get_parameter_value().bool_value,
            decode_lidar=self.get_parameter('decode_lidar').get_parameter_value().bool_value,
            lite_subscriptions=self.get_parameter('lite_subscriptions').get_parameter_value().bool_value,
            publish_raw_voxel=self.get_parameter('publish_raw_voxel').get_parameter_value().bool_value,
            obstacle_avoidance=self.get_parameter('obstacle_avoidance').get_parameter_value().bool_value,
            aes_128_key=self.get_parameter('aes_128_key').get_parameter_value().string_value,
        )

        # Log configuration
        self.get_logger().info(f"Robot IPs: {config.robot_ip_list}")
        self.get_logger().info(f"Connection type: {config.conn_type}")
        self.get_logger().info(f"Connection mode: {config.conn_mode}")
        self.get_logger().info(f"Enable video: {config.enable_video}")
        self.get_logger().info(f"Decode lidar: {config.decode_lidar}")
        self.get_logger().info(f"Lite subscriptions: {config.lite_subscriptions}")
        self.get_logger().info(f"Publish raw voxel: {config.publish_raw_voxel}")
        self.get_logger().info(f"Obstacle avoidance: {config.obstacle_avoidance}")

        return config

    def _setup_publishers(self) -> Dict[str, list]:
        """ROS2 publishers setup"""
        qos_profile = QoSProfile(depth=10)
        best_effort_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        publishers = {
            'joint_state': [],
            'robot_state': [],
            'lidar': [],
            'odometry': [],
            'imu': [],
            'camera': [],
            'camera_info': [],
            'voxel': [],
            'battery': []
        }

        num_robots = len(self.config.robot_ip_list)
        
        for i in range(num_robots):
            # Define topics depending on connection mode
            if self.config.conn_mode == 'single':
                joint_topic = 'joint_states'
                robot_state_topic = 'go2_states'
                lidar_topic = 'point_cloud2'
                odom_topic = 'odom'
                imu_topic = 'imu'
                camera_topic = 'camera/image_raw'
                camera_info_topic = 'camera/camera_info'
                voxel_topic = '/utlidar/voxel_map_compressed'
                battery_topic = 'battery_state'
            else:
                prefix = f'robot{i}'
                joint_topic = f'{prefix}/joint_states'
                robot_state_topic = f'{prefix}/go2_states'
                lidar_topic = f'{prefix}/point_cloud2'
                odom_topic = f'{prefix}/odom'
                imu_topic = f'{prefix}/imu'
                camera_topic = f'{prefix}/camera/image_raw'
                camera_info_topic = f'{prefix}/camera/camera_info'
                voxel_topic = f'{prefix}/utlidar/voxel_map_compressed'
                battery_topic = f'{prefix}/battery_state'

            # Create publishers
            publishers['joint_state'].append(
                self.create_publisher(JointState, joint_topic, qos_profile))
            publishers['robot_state'].append(
                self.create_publisher(Go2State, robot_state_topic, qos_profile))
            publishers['lidar'].append(
                self.create_publisher(
                    PointCloud2, lidar_topic, best_effort_qos,
                    qos_overriding_options=QoSOverridingOptions.with_default_policies()))
            publishers['odometry'].append(
                self.create_publisher(Odometry, odom_topic, qos_profile))
            publishers['imu'].append(
                self.create_publisher(Imu, imu_topic, qos_profile))

            if self.config.enable_video:
                publishers['camera'].append(
                    self.create_publisher(
                        Image, camera_topic, best_effort_qos,
                        qos_overriding_options=QoSOverridingOptions.with_default_policies()))
                publishers['camera_info'].append(
                    self.create_publisher(
                        CameraInfo, camera_info_topic, best_effort_qos,
                        qos_overriding_options=QoSOverridingOptions.with_default_policies()))

            if self.config.publish_raw_voxel:
                publishers['voxel'].append(
                    self.create_publisher(VoxelMapCompressed, voxel_topic, best_effort_qos))

            publishers['battery'].append(
                self.create_publisher(BatteryState, battery_topic, qos_profile))

        return publishers

    def _setup_subscribers(self) -> None:
        """ROS2 subscribers setup"""
        qos_profile = QoSProfile(depth=10)

        # Command subscribers
        num_robots = len(self.config.robot_ip_list)
        
        if self.config.conn_mode == 'single':
            self.create_subscription(
                Twist, 'cmd_vel_out',
                lambda msg: self._on_cmd_vel(msg, "0", source="cmd_vel_out"), qos_profile)
            # Direct Nav2 fallback (bypass twist_mux if cmd_vel_out is idle)
            self.create_subscription(
                Twist, 'cmd_vel_nav',
                lambda msg: self._on_cmd_vel(msg, "0", source="cmd_vel_nav"), qos_profile)
            self.create_subscription(
                WebRtcReq, 'webrtc_req',
                lambda msg: self._on_webrtc_req(msg, "0"), qos_profile)
        else:
            for i in range(num_robots):
                self.create_subscription(
                    Twist, f'robot{i}/cmd_vel_out',
                    lambda msg, robot_id=str(i): self._on_cmd_vel(msg, robot_id, source="cmd_vel_out"), qos_profile)
                # Direct Nav2 fallback (bypass twist_mux if cmd_vel_out is idle)
                self.create_subscription(
                    Twist, f'robot{i}/cmd_vel_nav',
                    lambda msg, robot_id=str(i): self._on_cmd_vel(msg, robot_id, source="cmd_vel_nav"), qos_profile)
                self.create_subscription(
                    WebRtcReq, f'robot{i}/webrtc_req',
                    lambda msg, robot_id=str(i): self._on_webrtc_req(msg, robot_id), qos_profile)

        # Joystick subscriber
        self.create_subscription(Joy, 'joy', self._on_joy, qos_profile)

        # CycloneDDS support
        if self.config.conn_type == 'cyclonedds':
            self.create_subscription(
                LowState, 'lowstate',
                self._on_cyclonedds_low_state, qos_profile)
            self.create_subscription(
                PoseStamped, '/utlidar/robot_pose',
                self._on_cyclonedds_pose, qos_profile)
            self.create_subscription(
                PointCloud2, '/utlidar/cloud',
                self._on_cyclonedds_lidar, qos_profile)

    def _on_set_parameters(self, params) -> SetParametersResult:
        """Callback for parameter changes"""
        result = SetParametersResult(successful=True)

        try:
            for p in params:
                if p.name == 'obstacle_avoidance':
                    self.get_logger().info(f'New obstacle_avoidance value: {p.value}')
                    self.config.obstacle_avoidance = p.value
                    
                    try:
                        self.robot_control_service.set_obstacle_avoidance(p.value, "0")
                    except Exception as e:
                        self.get_logger().error(f"Failed to set obstacle avoidance: {e}")
                        result.successful = False
                        result.reason = str(e)
                        break
                    
                    result.successful = True
                    result.reason = 'Updated obstacle_avoidance'
                    break
        except Exception as e:
            self.get_logger().error(f"Error setting parameters: {e}")
            result.successful = False
            result.reason = str(e)
            
        return result

    def _on_cmd_vel(self, msg: Twist, robot_id: str, source: str = "cmd_vel_out") -> None:
        """Callback for movement commands"""
        now = time.time()
        if source == "cmd_vel_nav" and (now - self._last_cmd_vel_out_time) < 0.5:
            return

        if source == "cmd_vel_out":
            self._last_cmd_vel_out_time = now

        is_nonzero = (
            abs(msg.linear.x) > 1e-6
            or abs(msg.linear.y) > 1e-6
            or abs(msg.angular.z) > 1e-6
        )
        with self._cmd_vel_lock:
            self._last_cmd_vel_time = now
            self._last_cmd_vel_nonzero = is_nonzero

        now = time.time()
        if now - self._last_cmd_vel_log > 1.0:
            self.get_logger().info(
                f"{source} rx (robot {robot_id}): x={msg.linear.x:.3f} "
                f"y={msg.linear.y:.3f} z={msg.angular.z:.3f}"
            )
            self._last_cmd_vel_log = now
        self.robot_control_service.handle_cmd_vel(
            msg.linear.x, msg.linear.y, msg.angular.z, 
            robot_id, self.config.obstacle_avoidance
        )

    def _on_webrtc_req(self, msg: WebRtcReq, robot_id: str) -> None:
        """Callback for WebRTC requests"""
        self.robot_control_service.handle_webrtc_request(
            msg.api_id, msg.parameter, msg.topic, msg.id, robot_id
        )

    def _on_joy(self, msg: Joy) -> None:
        """Callback for joystick"""
        self.joy_state = msg

    def _on_robot_validated(self, robot_id: str) -> None:
        """Callback after robot validation"""
        self.get_logger().info(f"Robot {robot_id} validated and ready")

        if self.config.obstacle_avoidance:
            self.get_logger().info(
                f"Obstacle avoidance enabled — sending SwitchSet + "
                f"UseRemoteCommandFromApi to robot {robot_id}"
            )
            try:
                self.robot_control_service.set_obstacle_avoidance(True, robot_id)
            except Exception as e:
                self.get_logger().error(
                    f"Failed to initialize obstacle avoidance on robot {robot_id}: {e}"
                )

    def _on_robot_data_received(self, msg: Dict[str, Any], robot_id: str) -> None:
        """Callback for receiving data from robot"""
        self.robot_data_service.process_webrtc_message(msg, robot_id)

    async def _on_video_frame(self, track: MediaStreamTrack, robot_id: str) -> None:
        """Callback for processing video frames"""
        logger.info(f"Video frame received for robot {robot_id}")

        while True:
            try:
                frame = await track.recv()
                img = frame.to_ndarray(format="bgr24")

                # Create camera data
                camera_data = CameraData(
                    image=img,
                    height=img.shape[0],
                    width=img.shape[1],
                    encoding="bgr8"
                )

                robot_data = RobotData(
                    robot_id=robot_id,
                    timestamp=0.0,
                    camera_data=camera_data
                )

                # Publish via ROS2Publisher
                self.ros2_publisher.publish_camera_data(robot_data)
                await asyncio.sleep(0)

            except Exception as e:
                logger.error(f"Error processing video frame: {e}")
                break

    def _cmd_vel_watchdog_loop(self) -> None:
        """Background thread: send zero velocity when cmd_vel goes silent."""
        while not self._cmd_vel_watchdog_shutdown.is_set():
            with self._cmd_vel_lock:
                last_t = self._last_cmd_vel_time
                nonzero = self._last_cmd_vel_nonzero

            if last_t is not None and nonzero:
                elapsed = time.time() - last_t
                if elapsed > self._cmd_vel_timeout:
                    self.robot_control_service.handle_cmd_vel(
                        0.0, 0.0, 0.0, "0", self.config.obstacle_avoidance
                    )
                    with self._cmd_vel_lock:
                        self._last_cmd_vel_nonzero = False
                    self.get_logger().info(
                        f"cmd_vel watchdog: no message for {elapsed:.3f}s "
                        "— sent zero velocity to robot"
                    )

            self._cmd_vel_watchdog_shutdown.wait(0.02)

    def stop_cmd_vel_watchdog(self) -> None:
        """Shut down the watchdog and send a final zero velocity."""
        self._cmd_vel_watchdog_shutdown.set()
        try:
            self.robot_control_service.handle_cmd_vel(
                0.0, 0.0, 0.0, "0", self.config.obstacle_avoidance
            )
        except Exception:
            pass

    # CycloneDDS callbacks
    def _on_cyclonedds_low_state(self, msg: LowState) -> None:
        """Processing LowState for CycloneDDS"""
        # You can add processing for CycloneDDS here if needed
        pass

    def _on_cyclonedds_pose(self, msg: PoseStamped) -> None:
        """Processing pose for CycloneDDS"""
        # You can add processing for CycloneDDS here if needed
        pass

    def _on_cyclonedds_lidar(self, msg: PointCloud2) -> None:
        """Processing lidar for CycloneDDS"""
        # You can add processing for CycloneDDS here if needed
        pass

    async def connect_robots(self) -> None:
        """Connect to robots (non-WebRTC paths only).

        For WebRTC, main.py uses webrtc_adapter.connect_with_retry() which
        provides automatic reconnection with exponential backoff.
        """
        if self.config.conn_type == 'webrtc':
            self.get_logger().info(
                "WebRTC connections managed by connect_with_retry loop"
            )
            return
        for i, robot_ip in enumerate(self.config.robot_ip_list):
            try:
                await self.webrtc_adapter.connect(str(i))
            except Exception as e:
                self.get_logger().error(f"Failed to connect to robot {i}: {e}")
                raise

    async def run_robot_control_loop(self, robot_id: str) -> None:
        """Main robot control loop"""
        while True:
            try:
                # Process joystick commands
                if self.joy_state.buttons:
                    self.robot_control_service.handle_joy_command(
                        self.joy_state.buttons, robot_id
                    )

                # Process WebRTC commands
                self.webrtc_adapter.process_webrtc_commands(robot_id)
                
                await asyncio.sleep(0.1)
                
            except Exception as e:
                self.get_logger().error(f"Error in control loop for robot {robot_id}: {e}")
                raise 