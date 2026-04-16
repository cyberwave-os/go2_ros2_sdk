# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import asyncio
import json
import logging
import time
from typing import Callable, Dict, Any, Optional

from ...domain.interfaces import IRobotDataReceiver, IRobotController
from ...domain.entities import RobotData, RobotConfig
from .go2_connection import Go2Connection
from ...application.utils.command_generator import gen_command, gen_mov_command
from ...domain.constants import ROBOT_CMD, RTC_TOPIC

logger = logging.getLogger(__name__)


class WebRTCAdapter(IRobotDataReceiver, IRobotController):
    """WebRTC adapter for robot communication"""

    def __init__(self, config: RobotConfig, on_validated_callback: Callable, on_video_frame_callback: Callable = None, event_loop=None):
        self.config = config
        self.connections: Dict[str, Go2Connection] = {}
        self.data_callback: Callable[[RobotData], None] = None
        self.webrtc_msgs = asyncio.Queue()
        self.on_validated_callback = on_validated_callback
        self.on_video_frame_callback = on_video_frame_callback

        # Reconnection tunables (can be overridden by the driver node)
        self.data_stall_timeout_sec: float = 15.0
        self.max_reconnect_delay_sec: float = 60.0
        self.reconnect_cooldown_sec: float = 45.0

        # Per-robot last-data-received monotonic timestamp
        self._last_data_recv_ts: Dict[str, float] = {}

        # Store the event loop (passed from main thread or detect current)
        if event_loop:
            self.main_loop = event_loop
        else:
            try:
                self.main_loop = asyncio.get_running_loop()
            except RuntimeError:
                self.main_loop = None

    async def connect(self, robot_id: str) -> None:
        """Connect to robot via WebRTC"""
        try:
            robot_idx = int(robot_id)
            robot_ip = self.config.robot_ip_list[robot_idx]
            
            conn = Go2Connection(
                robot_ip=robot_ip,
                robot_num=robot_id,
                token=self.config.token,
                on_validated=self._on_validated,
                on_message=self._on_data_channel_message,
                on_video_frame=self.on_video_frame_callback if self.config.enable_video else None,
                on_disconnected=self._on_connection_lost,
                decode_lidar=self.config.decode_lidar,
            )
            
            self.connections[robot_id] = conn
            await conn.connect()
            need_lidar_stream = self.config.decode_lidar or self.config.publish_raw_voxel
            await conn.disableTrafficSaving(need_lidar_stream)
            
            logger.info(f"Connected to robot {robot_id} at {robot_ip}")
            
        except Exception as e:
            logger.error(f"Failed to connect to robot {robot_id}: {e}")
            raise

    async def disconnect(self, robot_id: str) -> None:
        """Disconnect from robot"""
        if robot_id in self.connections:
            try:
                # Используем правильный метод для закрытия WebRTC соединения
                connection = self.connections[robot_id]
                if hasattr(connection, 'disconnect'):
                    await connection.disconnect()
                elif hasattr(connection, 'pc') and connection.pc:
                    await connection.pc.close()
                del self.connections[robot_id]
                logger.info(f"Disconnected from robot {robot_id}")
            except Exception as e:
                logger.error(f"Error disconnecting from robot {robot_id}: {e}")

    # ------------------------------------------------------------------
    # Reconnection loop
    # ------------------------------------------------------------------

    async def connect_with_retry(self, robot_id: str) -> None:
        """Connect to a robot and automatically reconnect on failure.

        Mirrors the backoff strategy from go2_native_runtime_video_node:
        initial 1s delay, 1.8x multiplier, capped at max_reconnect_delay_sec,
        with a longer cooldown after 5 consecutive quick failures.
        """
        reconnect_delay = 1.0
        consecutive_failures = 0

        while True:
            session_start = time.monotonic()
            should_backoff = False
            try:
                logger.info(f"[reconnect] Connecting to robot {robot_id} ...")
                await self.connect(robot_id)

                conn = self.connections.get(robot_id)
                if conn is None:
                    raise RuntimeError("connection vanished after connect()")

                self._last_data_recv_ts[robot_id] = time.monotonic()

                logger.info(f"[reconnect] Robot {robot_id} connected, monitoring session")

                # Wait until the connection drops (PC/DC event) or data stalls.
                await self._wait_for_session_end(robot_id, conn)

                session_uptime = time.monotonic() - session_start
                if session_uptime >= 20.0:
                    consecutive_failures = 0
                    reconnect_delay = 1.0

            except SystemExit:
                raise
            except Exception as exc:
                logger.warning(f"[reconnect] Robot {robot_id} session error: {exc}")
                should_backoff = True
            finally:
                await self._safe_disconnect(robot_id)

            if should_backoff:
                consecutive_failures += 1
                reconnect_delay = min(
                    self.max_reconnect_delay_sec, reconnect_delay * 1.8
                )
                if consecutive_failures >= 5:
                    logger.error(
                        f"[reconnect] Robot {robot_id}: {consecutive_failures} consecutive "
                        f"failures, cooling down for {self.reconnect_cooldown_sec:.0f}s"
                    )
                    await asyncio.sleep(self.reconnect_cooldown_sec)
                    reconnect_delay = max(reconnect_delay, 10.0)
                    consecutive_failures = 0

            logger.info(
                f"[reconnect] Robot {robot_id}: reconnecting in {reconnect_delay:.1f}s"
            )
            await asyncio.sleep(reconnect_delay)

    async def _wait_for_session_end(
        self, robot_id: str, conn: Go2Connection
    ) -> None:
        """Block until the connection dies or data flow stalls."""
        while True:
            # Check if the PC/DC signaled disconnection
            try:
                await asyncio.wait_for(
                    conn._disconnected_event.wait(), timeout=0.5
                )
                logger.warning(
                    f"[reconnect] Robot {robot_id}: disconnected event fired"
                )
                return
            except asyncio.TimeoutError:
                pass

            # Data-flow stall detection
            if self.data_stall_timeout_sec > 0:
                elapsed = time.monotonic() - self._last_data_recv_ts.get(
                    robot_id, time.monotonic()
                )
                if elapsed > self.data_stall_timeout_sec:
                    raise RuntimeError(
                        f"no data received for >{self.data_stall_timeout_sec:.0f}s"
                    )

    async def _safe_disconnect(self, robot_id: str) -> None:
        """Tear down a connection, swallowing errors."""
        try:
            await self.disconnect(robot_id)
        except Exception as exc:
            logger.debug(f"[reconnect] cleanup error for {robot_id}: {exc}")

    # ------------------------------------------------------------------

    def set_data_callback(self, callback: Callable[[RobotData], None]) -> None:
        """Set callback for data reception"""
        self.data_callback = callback

    def send_command(self, robot_id: str, command: str) -> None:
        """Send command to robot"""
        if robot_id in self.connections:
            try:
                connection = self.connections[robot_id]
                if hasattr(connection, 'data_channel') and connection.data_channel:
                    # Use asyncio.run_coroutine_threadsafe to handle cross-thread calls
                    loop = self._get_or_create_event_loop()
                    if loop and loop.is_running():
                        # Schedule the coroutine in the existing loop
                        asyncio.run_coroutine_threadsafe(
                            self._async_send_command(connection, command),
                            loop
                        )
                    else:
                        # Fallback to synchronous send
                        connection.data_channel.send(command)
                    logger.debug(f"Command sent to robot {robot_id}: {command[:50]}")
                else:
                    logger.warning(f"No data channel available for robot {robot_id}")
            except Exception as e:
                logger.error(f"Error sending command to robot {robot_id}: {e}")

    def _get_or_create_event_loop(self):
        """Get existing event loop or return the main loop"""
        # First try to get the current loop
        try:
            return asyncio.get_running_loop()
        except RuntimeError:
            # If no current loop, return the main loop stored during init
            return self.main_loop

    async def _async_send_command(self, connection, command: str):
        """Async wrapper for sending commands"""
        try:
            if hasattr(connection, 'data_channel') and connection.data_channel:
                connection.data_channel.send(command)
        except Exception as e:
            logger.error(f"Error in async send command: {e}")

    def send_movement_command(self, robot_id: str, x: float, y: float, z: float) -> None:
        """Send movement command to robot"""
        try:
            command = gen_mov_command(
                round(x, 2), 
                round(y, 2), 
                round(z, 2), 
                self.config.obstacle_avoidance
            )
            self.send_command(robot_id, command)
        except Exception as e:
            logger.error(f"Error sending movement command: {e}")

    def send_stand_up_command(self, robot_id: str) -> None:
        """Send stand up command"""
        try:
            stand_up_cmd = gen_command(ROBOT_CMD["StandUp"])
            self.send_command(robot_id, stand_up_cmd)
            
            move_cmd = gen_command(ROBOT_CMD['BalanceStand'])
            self.send_command(robot_id, move_cmd)
        except Exception as e:
            logger.error(f"Error sending stand up command: {e}")

    def send_stand_down_command(self, robot_id: str) -> None:
        """Send stand down command"""
        try:
            stand_down_cmd = gen_command(ROBOT_CMD["StandDown"])
            self.send_command(robot_id, stand_down_cmd)
        except Exception as e:
            logger.error(f"Error sending stand down command: {e}")

    def send_webrtc_request(self, robot_id: str, api_id: int, parameter: Any, topic: str) -> None:
        """Send WebRTC request"""
        try:
            payload = gen_command(api_id, parameter, topic)
            self.webrtc_msgs.put_nowait(payload)
            logger.debug(f"WebRTC request queued for robot {robot_id}")
        except Exception as e:
            logger.error(f"Error sending WebRTC request: {e}")

    def process_webrtc_commands(self, robot_id: str) -> None:
        """Process WebRTC commands from queue"""
        while True:
            try:
                message = self.webrtc_msgs.get_nowait()
                try:
                    self.send_command(robot_id, message)
                finally:
                    self.webrtc_msgs.task_done()
            except asyncio.QueueEmpty:
                break

    def _get_subscription_topics(self):
        """Return subscription topic list based on runtime config."""
        if not self.config.lite_subscriptions:
            return RTC_TOPIC.values()

        topics = [
            RTC_TOPIC["LOW_STATE"],
            RTC_TOPIC["ROBOTODOM"],
            RTC_TOPIC["LF_SPORT_MOD_STATE"],
        ]
        if self.config.decode_lidar or self.config.publish_raw_voxel:
            topics.append(RTC_TOPIC["ULIDAR_ARRAY"])
        return topics

    def _on_validated(self, robot_id: str) -> None:
        """Callback after connection validation"""
        try:
            if robot_id in self.connections:
                for topic in self._get_subscription_topics():
                    self.connections[robot_id].data_channel.send(
                        json.dumps({"type": "subscribe", "topic": topic}))
            
            if self.on_validated_callback:
                self.on_validated_callback(robot_id)
                
        except Exception as e:
            logger.error(f"Error in validated callback: {e}")

    def _on_connection_lost(self, robot_id: str, reason: str) -> None:
        """Called by Go2Connection when the PC or data channel dies."""
        logger.warning(f"Connection to robot {robot_id} lost: {reason}")

    def _on_data_channel_message(self, _, msg: Dict[str, Any], robot_id: str) -> None:
        """Handle incoming data channel messages"""
        try:
            self._last_data_recv_ts[robot_id] = time.monotonic()

            if self.data_callback:
                self.data_callback(msg, robot_id)
                
        except Exception as e:
            logger.error(f"Error processing data channel message: {e}") 