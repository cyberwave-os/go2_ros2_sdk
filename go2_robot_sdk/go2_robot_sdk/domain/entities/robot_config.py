# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

from dataclasses import dataclass
from typing import List


@dataclass
class RobotConfig:
    """Robot configuration parameters"""
    robot_ip_list: List[str]
    token: str
    conn_type: str
    enable_video: bool
    decode_lidar: bool
    lite_subscriptions: bool
    publish_raw_voxel: bool
    obstacle_avoidance: bool
    conn_mode: str  # 'single' or 'multi'
    aes_128_key: str = ""  # per-device key for Go2 >= 1.1.15 / G1 >= 1.5.1

    @classmethod
    def from_params(cls, robot_ip: str, token: str, conn_type: str,
                   enable_video: bool, decode_lidar: bool,
                   lite_subscriptions: bool,
                   publish_raw_voxel: bool, obstacle_avoidance: bool,
                   aes_128_key: str = ""):
        robot_ip_list = robot_ip.replace(" ", "").split(",")
        conn_mode = "single" if (
            len(robot_ip_list) == 1 and conn_type != "cyclonedds") else "multi"
        
        return cls(
            robot_ip_list=robot_ip_list,
            token=token,
            conn_type=conn_type,
            enable_video=enable_video,
            decode_lidar=decode_lidar,
            lite_subscriptions=lite_subscriptions,
            publish_raw_voxel=publish_raw_voxel,
            obstacle_avoidance=obstacle_avoidance,
            conn_mode=conn_mode,
            aes_128_key=aes_128_key,
        ) 