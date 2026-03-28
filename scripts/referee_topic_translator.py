#!/usr/bin/env python3

# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Referee Topic Translator Node

Subscribes to dji_referee_protocol's std_msgs/String (JSON) topics and
republishes them as typed pb_rm_interfaces messages that
pb2025_sentry_behavior can consume.

This node does NOT access the serial port directly, making it resilient
to hardware issues.  It relies on a running dji_referee_protocol node
(local or remote) that handles serial communication.
"""

import json
import math
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import Point, Pose, Quaternion
from pb_rm_interfaces.msg import Buff
from pb_rm_interfaces.msg import EventData
from pb_rm_interfaces.msg import GameRobotHP
from pb_rm_interfaces.msg import GameStatus
from pb_rm_interfaces.msg import GroundRobotPosition
from pb_rm_interfaces.msg import RfidStatus
from pb_rm_interfaces.msg import RobotStatus
from rclpy.node import Node
from std_msgs.msg import String


class RefereeTopicTranslator(Node):
    """Translate dji_referee_protocol String/JSON topics → typed pb_rm_interfaces messages."""

    # Mapping: dji topic suffix → handler method name
    _DJI_TOPICS = [
        "game_status",
        "robot_hp",
        "field_event",
        "robot_performance",
        "robot_heat",
        "robot_position",
        "robot_buff",
        "damage_state",
        "allowed_shoot",
        "rfid_status",
        "ground_robot_position",
    ]

    def __init__(self) -> None:
        super().__init__("referee_topic_translator")

        self.declare_parameter("input_topic_prefix", "/referee")
        self.declare_parameter("output_topic_prefix", "referee")
        self.declare_parameter("publish_rate_hz", 10.0)

        input_prefix = str(self.get_parameter("input_topic_prefix").value)
        self.output_prefix = str(self.get_parameter("output_topic_prefix").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)

        # Accumulated state for composite RobotStatus message
        self.robot_status_msg = RobotStatus()
        self.game_status_msg = GameStatus()
        self.all_robot_hp_msg = GameRobotHP()
        self.event_data_msg = EventData()
        self.rfid_status_msg = RfidStatus()
        self.buff_msg = Buff()
        self.ground_robot_position_msg = GroundRobotPosition()
        self.damage_latched = False

        # Receipt tracking: only publish after first real message received
        self._received_game_status = False
        self._received_robot_hp = False
        self._received_event_data = False
        self._received_robot_status = False
        self._received_rfid_status = False
        self._received_buff = False
        self._received_ground_robot_position = False

        # --- Publishers (typed, for behavior tree) ---
        self.game_status_pub = self.create_publisher(
            GameStatus, f"{self.output_prefix}/game_status", 10
        )
        self.all_robot_hp_pub = self.create_publisher(
            GameRobotHP, f"{self.output_prefix}/all_robot_hp", 10
        )
        self.robot_status_pub = self.create_publisher(
            RobotStatus, f"{self.output_prefix}/robot_status", 10
        )
        self.rfid_status_pub = self.create_publisher(
            RfidStatus, f"{self.output_prefix}/rfid_status", 10
        )
        self.event_data_pub = self.create_publisher(
            EventData, f"{self.output_prefix}/event_data", 10
        )
        self.buff_pub = self.create_publisher(
            Buff, f"{self.output_prefix}/buff", 10
        )
        self.ground_robot_position_pub = self.create_publisher(
            GroundRobotPosition, f"{self.output_prefix}/ground_robot_position", 10
        )

        # --- Subscribers (String, from dji_referee_protocol) ---
        self._subs = []
        for topic_suffix in self._DJI_TOPICS:
            full_topic = f"{input_prefix}/{topic_suffix}"
            handler = getattr(self, f"_on_{topic_suffix}", None)
            if handler is None:
                self.get_logger().warn(f"No handler for {full_topic}, skipping")
                continue
            sub = self.create_subscription(String, full_topic, handler, 10)
            self._subs.append(sub)
            self.get_logger().debug(f"Subscribed to {full_topic}")

        # Periodic publish timer
        period = 1.0 / publish_rate_hz if publish_rate_hz > 0.0 else 0.1
        self.publish_timer = self.create_timer(period, self._publish_all)

        self.get_logger().info(
            f"Referee topic translator started: "
            f"input={input_prefix}/*, output={self.output_prefix}/*"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _parse_json(msg: String) -> Optional[Dict[str, Any]]:
        """Extract the 'data' dict from the dji JSON envelope."""
        try:
            envelope = json.loads(msg.data)
            return envelope.get("data", envelope)
        except (json.JSONDecodeError, AttributeError):
            return None

    @staticmethod
    def _rfid_bit(bits_low: int, bits_high: int, index: int) -> bool:
        if index < 32:
            return bool(bits_low & (1 << index))
        return bool(bits_high & (1 << (index - 32)))

    # ------------------------------------------------------------------
    # DJI topic handlers
    # ------------------------------------------------------------------
    def _on_game_status(self, msg: String) -> None:
        d = self._parse_json(msg)
        if d is None:
            return
        self._received_game_status = True
        self.game_status_msg.game_progress = int(d.get("game_progress", 0))
        self.game_status_msg.stage_remain_time = int(d.get("stage_remain_time", 0))

    def _on_robot_hp(self, msg: String) -> None:
        d = self._parse_json(msg)
        if d is None:
            return
        self._received_robot_hp = True
        for field in (
            "red_1_robot_hp", "red_2_robot_hp", "red_3_robot_hp", "red_4_robot_hp",
            "red_7_robot_hp", "red_outpost_hp", "red_base_hp",
            "blue_1_robot_hp", "blue_2_robot_hp", "blue_3_robot_hp", "blue_4_robot_hp",
            "blue_7_robot_hp", "blue_outpost_hp", "blue_base_hp",
        ):
            setattr(self.all_robot_hp_msg, field, int(d.get(field, 0)))

    def _on_field_event(self, msg: String) -> None:
        d = self._parse_json(msg)
        if d is None:
            return
        self._received_event_data = True
        self.event_data_msg.non_overlapping_supply_zone = (
            EventData.OCCUPIED_FRIEND if d.get("supply_area_1") else EventData.UNOCCUPIED
        )
        self.event_data_msg.overlapping_supply_zone = (
            EventData.OCCUPIED_FRIEND if d.get("supply_area_2") else EventData.UNOCCUPIED
        )
        self.event_data_msg.supply_zone = (
            EventData.OCCUPIED_FRIEND if d.get("rmul_supply_area") else EventData.UNOCCUPIED
        )
        self.event_data_msg.small_energy = int(d.get("small_energy_mech", 0))
        self.event_data_msg.big_energy = int(d.get("big_energy_mech", 0))
        self.event_data_msg.central_highland = int(d.get("central_highland", 0))
        self.event_data_msg.trapezoidal_highland = (
            EventData.OCCUPIED_FRIEND if d.get("trapezoid_highland") else EventData.UNOCCUPIED
        )
        self.event_data_msg.center_gain_zone = int(d.get("center_buff_point", 0))

    def _on_robot_performance(self, msg: String) -> None:
        d = self._parse_json(msg)
        if d is None:
            return
        self._received_robot_status = True
        self.robot_status_msg.robot_id = int(d.get("robot_id", 0))
        self.robot_status_msg.robot_level = int(d.get("robot_level", 0))
        self.robot_status_msg.current_hp = int(d.get("current_hp", 0))
        self.robot_status_msg.maximum_hp = int(d.get("maximum_hp", 0))
        self.robot_status_msg.shooter_barrel_cooling_value = int(
            d.get("shooter_barrel_cooling_value", 0)
        )
        self.robot_status_msg.shooter_barrel_heat_limit = int(
            d.get("shooter_barrel_heat_limit", 0)
        )

    def _on_robot_heat(self, msg: String) -> None:
        d = self._parse_json(msg)
        if d is None:
            return
        self._received_robot_status = True
        self.robot_status_msg.shooter_17mm_1_barrel_heat = int(
            d.get("shooter_17mm_barrel_heat", 0)
        )

    def _on_robot_position(self, msg: String) -> None:
        d = self._parse_json(msg)
        if d is None:
            return
        self._received_robot_status = True
        yaw = math.radians(float(d.get("angle", 0.0)))
        self.robot_status_msg.robot_pos = Pose(
            position=Point(x=float(d.get("x", 0.0)), y=float(d.get("y", 0.0)), z=0.0),
            orientation=Quaternion(
                x=0.0,
                y=0.0,
                z=math.sin(yaw / 2.0),
                w=math.cos(yaw / 2.0),
            ),
        )

    def _on_robot_buff(self, msg: String) -> None:
        d = self._parse_json(msg)
        if d is None:
            return
        self._received_buff = True
        self.buff_msg.recovery_buff = int(d.get("recovery_buff", 0))
        self.buff_msg.cooling_buff = int(d.get("cooling_buff", 0))
        self.buff_msg.defence_buff = int(d.get("defence_buff", 0))
        self.buff_msg.vulnerability_buff = int(d.get("vulnerability_buff", 0))
        self.buff_msg.attack_buff = int(d.get("attack_buff", 0))
        self.buff_msg.remaining_energy = int(d.get("remaining_energy", 0))

    def _on_damage_state(self, msg: String) -> None:
        d = self._parse_json(msg)
        if d is None:
            return
        self._received_robot_status = True
        self.robot_status_msg.armor_id = int(d.get("armor_id", 0))
        self.robot_status_msg.hp_deduction_reason = int(d.get("damage_type", 0))
        self.robot_status_msg.is_hp_deduced = True
        self.damage_latched = True

    def _on_allowed_shoot(self, msg: String) -> None:
        d = self._parse_json(msg)
        if d is None:
            return
        self._received_robot_status = True
        self.robot_status_msg.projectile_allowance_17mm = int(
            d.get("projectile_allowance_17mm", 0)
        )
        self.robot_status_msg.remaining_gold_coin = int(
            d.get("remaining_gold_coin", 0)
        )

    def _on_rfid_status(self, msg: String) -> None:
        d = self._parse_json(msg)
        if d is None:
            return
        self._received_rfid_status = True
        bits_low = int(d.get("detected_rfid_bits_low", 0))
        bits_high = int(d.get("detected_rfid_bits_high", 0))
        b = self._rfid_bit
        self.rfid_status_msg.base_gain_point = b(bits_low, bits_high, 0)
        self.rfid_status_msg.central_highland_gain_point = b(bits_low, bits_high, 1)
        self.rfid_status_msg.enemy_central_highland_gain_point = b(bits_low, bits_high, 2)
        self.rfid_status_msg.friendly_trapezoidal_highland_gain_point = b(bits_low, bits_high, 3)
        self.rfid_status_msg.enemy_trapezoidal_highland_gain_point = b(bits_low, bits_high, 4)
        self.rfid_status_msg.friendly_fly_ramp_front_gain_point = b(bits_low, bits_high, 5)
        self.rfid_status_msg.friendly_fly_ramp_back_gain_point = b(bits_low, bits_high, 6)
        self.rfid_status_msg.enemy_fly_ramp_front_gain_point = b(bits_low, bits_high, 7)
        self.rfid_status_msg.enemy_fly_ramp_back_gain_point = b(bits_low, bits_high, 8)
        self.rfid_status_msg.friendly_central_highland_lower_gain_point = b(bits_low, bits_high, 9)
        self.rfid_status_msg.friendly_central_highland_upper_gain_point = b(bits_low, bits_high, 10)
        self.rfid_status_msg.enemy_central_highland_lower_gain_point = b(bits_low, bits_high, 11)
        self.rfid_status_msg.enemy_central_highland_upper_gain_point = b(bits_low, bits_high, 12)
        self.rfid_status_msg.friendly_highway_lower_gain_point = b(bits_low, bits_high, 13)
        self.rfid_status_msg.friendly_highway_upper_gain_point = b(bits_low, bits_high, 14)
        self.rfid_status_msg.enemy_highway_lower_gain_point = b(bits_low, bits_high, 15)
        self.rfid_status_msg.enemy_highway_upper_gain_point = b(bits_low, bits_high, 16)
        self.rfid_status_msg.friendly_fortress_gain_point = b(bits_low, bits_high, 17)
        self.rfid_status_msg.friendly_outpost_gain_point = b(bits_low, bits_high, 18)
        self.rfid_status_msg.friendly_supply_zone_non_exchange = b(bits_low, bits_high, 19)
        self.rfid_status_msg.friendly_supply_zone_exchange = b(bits_low, bits_high, 20)
        self.rfid_status_msg.friendly_big_resource_island = b(bits_low, bits_high, 21)
        self.rfid_status_msg.enemy_big_resource_island = b(bits_low, bits_high, 22)
        self.rfid_status_msg.center_gain_point = b(bits_low, bits_high, 23)

    def _on_ground_robot_position(self, msg: String) -> None:
        d = self._parse_json(msg)
        if d is None:
            return
        self._received_ground_robot_position = True
        self.ground_robot_position_msg.hero_position = Point(
            x=float(d.get("hero_x", 0.0)), y=float(d.get("hero_y", 0.0)), z=0.0
        )
        self.ground_robot_position_msg.engineer_position = Point(
            x=float(d.get("engineer_x", 0.0)), y=float(d.get("engineer_y", 0.0)), z=0.0
        )
        self.ground_robot_position_msg.standard_3_position = Point(
            x=float(d.get("infantry_3_x", 0.0)), y=float(d.get("infantry_3_y", 0.0)), z=0.0
        )
        self.ground_robot_position_msg.standard_4_position = Point(
            x=float(d.get("infantry_4_x", 0.0)), y=float(d.get("infantry_4_y", 0.0)), z=0.0
        )

    # ------------------------------------------------------------------
    # Periodic publish
    # ------------------------------------------------------------------
    def _publish_all(self) -> None:
        if self._received_game_status:
            self.game_status_pub.publish(self.game_status_msg)
        if self._received_robot_hp:
            self.all_robot_hp_pub.publish(self.all_robot_hp_msg)
        if self._received_robot_status:
            self.robot_status_pub.publish(self.robot_status_msg)
        if self._received_rfid_status:
            self.rfid_status_pub.publish(self.rfid_status_msg)
        if self._received_event_data:
            self.event_data_pub.publish(self.event_data_msg)
        if self._received_buff:
            self.buff_pub.publish(self.buff_msg)
        if self._received_ground_robot_position:
            self.ground_robot_position_pub.publish(self.ground_robot_position_msg)
        if self.damage_latched:
            self.robot_status_msg.is_hp_deduced = False
            self.damage_latched = False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = RefereeTopicTranslator()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as ex:
        logger = rclpy.logging.get_logger("referee_topic_translator_main")
        logger.fatal(f"Fatal error: {ex}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
