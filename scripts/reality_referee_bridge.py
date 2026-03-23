#!/usr/bin/env python3

import importlib.util
import math
import os
import pty
import select
import signal
import subprocess
import sys
import threading
import time
import tty
from pathlib import Path


def _ensure_referee_protocol_importable() -> None:
    if importlib.util.find_spec("dji_referee_protocol") is not None:
        return
    candidates = []
    env_root = os.environ.get("DJI_REFEREE_PROTOCOL_ROOT", "")
    if env_root:
        candidates.append(env_root)
    candidates.append("/home/soyo/sentry_ws/src/dji_referee_protocol")
    for candidate in candidates:
        candidate_path = Path(candidate)
        if candidate_path.exists():
            sys.path.insert(0, str(candidate_path))
            if importlib.util.find_spec("dji_referee_protocol") is not None:
                return
    raise ImportError("Unable to import dji_referee_protocol")


_ensure_referee_protocol_importable()

import rclpy
import serial
from geometry_msgs.msg import Point, Pose, Quaternion
from pb_rm_interfaces.msg import Buff
from pb_rm_interfaces.msg import EventData
from pb_rm_interfaces.msg import GameRobotHP
from pb_rm_interfaces.msg import GameStatus
from pb_rm_interfaces.msg import GroundRobotPosition
from pb_rm_interfaces.msg import RfidStatus
from pb_rm_interfaces.msg import RobotStatus
from rclpy.node import Node

from dji_referee_protocol.protocol_constants import CommandID, SerialConfig
from dji_referee_protocol.protocol_parser import ProtocolParser


class RealityRefereeBridge(Node):
    def __init__(self) -> None:
        super().__init__("reality_referee_bridge")

        self.declare_parameter("serial_port_normal", "/dev/ttyUSB0")
        self.declare_parameter("serial_baud_normal", SerialConfig.NORMAL_BAUDRATE)
        self.declare_parameter("pty_path", "/tmp/referee_pty")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("manage_service", True)
        self.declare_parameter("relaunch_referee", True)
        self.declare_parameter("referee_setup_bash", "/home/soyo/sentry_ws/install/setup.bash")

        self.serial_port_normal = str(self.get_parameter("serial_port_normal").value)
        self.serial_baud_normal = int(self.get_parameter("serial_baud_normal").value)
        self.pty_path = str(self.get_parameter("pty_path").value)
        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.manage_service = bool(self.get_parameter("manage_service").value)
        self.relaunch_referee = bool(self.get_parameter("relaunch_referee").value)
        self.referee_setup_bash = str(self.get_parameter("referee_setup_bash").value)

        self.parser_normal = ProtocolParser()
        self.lock = threading.Lock()
        self.running = True
        self.damage_latched = False
        self.referee_proc = None
        self.service_was_stopped = False

        self.game_status_msg = GameStatus()
        self.robot_status_msg = RobotStatus()
        self.rfid_status_msg = RfidStatus()
        self.all_robot_hp_msg = GameRobotHP()
        self.event_data_msg = EventData()
        self.buff_msg = Buff()
        self.ground_robot_position_msg = GroundRobotPosition()

        self._takeover_serial_port()
        self._setup_pty()
        self._open_serial()
        self._relaunch_referee_node()
        self._create_publishers()

        period = 1.0 / self.publish_rate_hz if self.publish_rate_hz > 0.0 else 0.1
        self.publish_timer = self.create_timer(period, self.publish_all)
        self.serial_read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self.pty_read_thread = threading.Thread(target=self._pty_read_loop, daemon=True)
        self.serial_read_thread.start()
        self.pty_read_thread.start()

        self.get_logger().info(
            f"Reality referee bridge started: serial={self.serial_port_normal}, pty={self.pty_symlink_target}"
        )

    def _takeover_serial_port(self) -> None:
        if not self.manage_service:
            return
        result = subprocess.run(
            ["sudo", "systemctl", "stop", "ros2-dji-referee.service"],
            capture_output=True, text=True, timeout=15,
        )
        if result.returncode == 0:
            self.service_was_stopped = True
            self.get_logger().info("Stopped ros2-dji-referee.service to take over serial port")
        else:
            self.get_logger().warn(
                f"Could not stop ros2-dji-referee.service: {result.stderr.strip()}. "
                "If the serial port is busy, run: sudo systemctl stop ros2-dji-referee.service"
            )
        time.sleep(0.5)

    def _relaunch_referee_node(self) -> None:
        if not self.relaunch_referee:
            return
        setup_bash = self.referee_setup_bash
        if not Path(setup_bash).exists():
            self.get_logger().warn(f"Cannot relaunch referee node: {setup_bash} not found")
            return
        cmd = (
            f'source "{setup_bash}" && '
            f'exec ros2 run dji_referee_protocol referee_serial_node '
            f'--ros-args -p serial_port_normal:={self.pty_path}'
        )
        self.referee_proc = subprocess.Popen(
            ["bash", "-c", cmd],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=os.setsid,
        )
        self.get_logger().info(
            f"Relaunched referee_serial_node (pid={self.referee_proc.pid}) on PTY {self.pty_path}"
        )

    def _setup_pty(self) -> None:
        self.pty_master_fd, self.pty_slave_fd = pty.openpty()
        tty.setraw(self.pty_slave_fd)
        self.pty_slave_name = os.ttyname(self.pty_slave_fd)
        self.pty_symlink_target = self.pty_path
        pty_path = Path(self.pty_path)
        if pty_path.exists() or pty_path.is_symlink():
            pty_path.unlink()
        os.symlink(self.pty_slave_name, self.pty_path)
        os.set_blocking(self.pty_master_fd, False)

    def _open_serial(self) -> None:
        self.serial_normal = serial.Serial(
            port=self.serial_port_normal,
            baudrate=self.serial_baud_normal,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=SerialConfig.TIMEOUT,
        )

    def _create_publishers(self) -> None:
        self.game_status_pub = self.create_publisher(GameStatus, "referee/game_status", 10)
        self.robot_status_pub = self.create_publisher(RobotStatus, "referee/robot_status", 10)
        self.rfid_status_pub = self.create_publisher(RfidStatus, "referee/rfid_status", 10)
        self.all_robot_hp_pub = self.create_publisher(GameRobotHP, "referee/all_robot_hp", 10)
        self.event_data_pub = self.create_publisher(EventData, "referee/event_data", 10)
        self.buff_pub = self.create_publisher(Buff, "referee/buff", 10)
        self.ground_robot_position_pub = self.create_publisher(
            GroundRobotPosition, "referee/ground_robot_position", 10
        )

    def _serial_read_loop(self) -> None:
        while self.running:
            try:
                if not self.serial_normal.is_open:
                    time.sleep(0.05)
                    continue
                waiting = self.serial_normal.in_waiting
                if waiting <= 0:
                    time.sleep(0.001)
                    continue
                data = self.serial_normal.read(waiting)
                if not data:
                    continue
                os.write(self.pty_master_fd, data)
                self.parser_normal.feed_data(data)
                while True:
                    result = self.parser_normal.unpack()
                    if result is None:
                        break
                    cmd_id, parsed_data = result
                    self._handle_packet(cmd_id, parsed_data)
            except serial.SerialException as ex:
                self.get_logger().error(f"Serial read failed: {ex}")
                time.sleep(0.1)
            except OSError as ex:
                self.get_logger().error(f"PTY write failed: {ex}")
                time.sleep(0.1)
            except Exception as ex:
                self.get_logger().error(f"Bridge parse failed: {ex}")
                time.sleep(0.1)

    def _pty_read_loop(self) -> None:
        while self.running:
            try:
                readable, _, _ = select.select([self.pty_master_fd], [], [], 0.05)
                if not readable:
                    continue
                data = os.read(self.pty_master_fd, 4096)
                if not data:
                    continue
                self.serial_normal.write(data)
            except serial.SerialException as ex:
                self.get_logger().error(f"Serial write failed: {ex}")
                time.sleep(0.1)
            except OSError:
                time.sleep(0.05)
            except Exception as ex:
                self.get_logger().error(f"PTY read failed: {ex}")
                time.sleep(0.1)

    def _handle_packet(self, cmd_id: int, data) -> None:
        with self.lock:
            if cmd_id == CommandID.GAME_STATUS:
                self.game_status_msg.game_progress = int(data.game_progress)
                self.game_status_msg.stage_remain_time = int(data.stage_remain_time)
            elif cmd_id == CommandID.ROBOT_HP:
                self.all_robot_hp_msg.red_1_robot_hp = int(data.red_1_robot_hp)
                self.all_robot_hp_msg.red_2_robot_hp = int(data.red_2_robot_hp)
                self.all_robot_hp_msg.red_3_robot_hp = int(data.red_3_robot_hp)
                self.all_robot_hp_msg.red_4_robot_hp = int(data.red_4_robot_hp)
                self.all_robot_hp_msg.red_7_robot_hp = int(data.red_7_robot_hp)
                self.all_robot_hp_msg.red_outpost_hp = int(data.red_outpost_hp)
                self.all_robot_hp_msg.red_base_hp = int(data.red_base_hp)
                self.all_robot_hp_msg.blue_1_robot_hp = int(data.blue_1_robot_hp)
                self.all_robot_hp_msg.blue_2_robot_hp = int(data.blue_2_robot_hp)
                self.all_robot_hp_msg.blue_3_robot_hp = int(data.blue_3_robot_hp)
                self.all_robot_hp_msg.blue_4_robot_hp = int(data.blue_4_robot_hp)
                self.all_robot_hp_msg.blue_7_robot_hp = int(data.blue_7_robot_hp)
                self.all_robot_hp_msg.blue_outpost_hp = int(data.blue_outpost_hp)
                self.all_robot_hp_msg.blue_base_hp = int(data.blue_base_hp)
            elif cmd_id == CommandID.FIELD_EVENT:
                self.event_data_msg.non_overlapping_supply_zone = (
                    EventData.OCCUPIED_FRIEND if data.supply_area_1 else EventData.UNOCCUPIED
                )
                self.event_data_msg.overlapping_supply_zone = (
                    EventData.OCCUPIED_FRIEND if data.supply_area_2 else EventData.UNOCCUPIED
                )
                self.event_data_msg.supply_zone = (
                    EventData.OCCUPIED_FRIEND if data.rmul_supply_area else EventData.UNOCCUPIED
                )
                self.event_data_msg.small_energy = int(data.small_energy_mech)
                self.event_data_msg.big_energy = int(data.big_energy_mech)
                self.event_data_msg.central_highland = int(data.central_highland)
                self.event_data_msg.trapezoidal_highland = (
                    EventData.OCCUPIED_FRIEND if data.trapezoid_highland else EventData.UNOCCUPIED
                )
                self.event_data_msg.center_gain_zone = int(data.center_buff_point)
            elif cmd_id == CommandID.ROBOT_PERFORMANCE:
                self.robot_status_msg.robot_id = int(data.robot_id)
                self.robot_status_msg.robot_level = int(data.robot_level)
                self.robot_status_msg.current_hp = int(data.current_hp)
                self.robot_status_msg.maximum_hp = int(data.maximum_hp)
                self.robot_status_msg.shooter_barrel_cooling_value = int(data.shooter_barrel_cooling_value)
                self.robot_status_msg.shooter_barrel_heat_limit = int(data.shooter_barrel_heat_limit)
            elif cmd_id == CommandID.ROBOT_HEAT:
                self.robot_status_msg.shooter_17mm_1_barrel_heat = int(data.shooter_17mm_barrel_heat)
            elif cmd_id == CommandID.ROBOT_POSITION:
                yaw = math.radians(float(data.angle))
                self.robot_status_msg.robot_pos = Pose(
                    position=Point(x=float(data.x), y=float(data.y), z=0.0),
                    orientation=Quaternion(
                        x=0.0,
                        y=0.0,
                        z=math.sin(yaw / 2.0),
                        w=math.cos(yaw / 2.0),
                    ),
                )
            elif cmd_id == CommandID.ROBOT_BUFF:
                self.buff_msg.recovery_buff = int(data.recovery_buff)
                self.buff_msg.cooling_buff = int(data.cooling_buff)
                self.buff_msg.defence_buff = int(data.defence_buff)
                self.buff_msg.vulnerability_buff = int(data.vulnerability_buff)
                self.buff_msg.attack_buff = int(data.attack_buff)
                self.buff_msg.remaining_energy = int(data.remaining_energy)
            elif cmd_id == CommandID.DAMAGE_STATE:
                self.robot_status_msg.armor_id = int(data.armor_id)
                self.robot_status_msg.hp_deduction_reason = int(data.damage_type)
                self.robot_status_msg.is_hp_deduced = True
                self.damage_latched = True
            elif cmd_id == CommandID.ALLOWED_SHOOT:
                self.robot_status_msg.projectile_allowance_17mm = int(data.projectile_allowance_17mm)
                self.robot_status_msg.remaining_gold_coin = int(data.remaining_gold_coin)
            elif cmd_id == CommandID.RFID_STATUS:
                self._update_rfid_status(int(data.detected_rfid_bits_low), int(data.detected_rfid_bits_high))
            elif cmd_id == CommandID.GROUND_ROBOT_POSITION:
                self.ground_robot_position_msg.hero_position = Point(
                    x=float(data.hero_x), y=float(data.hero_y), z=0.0
                )
                self.ground_robot_position_msg.engineer_position = Point(
                    x=float(data.engineer_x), y=float(data.engineer_y), z=0.0
                )
                self.ground_robot_position_msg.standard_3_position = Point(
                    x=float(data.infantry_3_x), y=float(data.infantry_3_y), z=0.0
                )
                self.ground_robot_position_msg.standard_4_position = Point(
                    x=float(data.infantry_4_x), y=float(data.infantry_4_y), z=0.0
                )

    def _rfid_bit(self, bits_low: int, bits_high: int, index: int) -> bool:
        if index < 32:
            return bool(bits_low & (1 << index))
        return bool(bits_high & (1 << (index - 32)))

    def _update_rfid_status(self, bits_low: int, bits_high: int) -> None:
        self.rfid_status_msg.base_gain_point = self._rfid_bit(bits_low, bits_high, 0)
        self.rfid_status_msg.central_highland_gain_point = self._rfid_bit(bits_low, bits_high, 1)
        self.rfid_status_msg.enemy_central_highland_gain_point = self._rfid_bit(bits_low, bits_high, 2)
        self.rfid_status_msg.friendly_trapezoidal_highland_gain_point = self._rfid_bit(bits_low, bits_high, 3)
        self.rfid_status_msg.enemy_trapezoidal_highland_gain_point = self._rfid_bit(bits_low, bits_high, 4)
        self.rfid_status_msg.friendly_fly_ramp_front_gain_point = self._rfid_bit(bits_low, bits_high, 5)
        self.rfid_status_msg.friendly_fly_ramp_back_gain_point = self._rfid_bit(bits_low, bits_high, 6)
        self.rfid_status_msg.enemy_fly_ramp_front_gain_point = self._rfid_bit(bits_low, bits_high, 7)
        self.rfid_status_msg.enemy_fly_ramp_back_gain_point = self._rfid_bit(bits_low, bits_high, 8)
        self.rfid_status_msg.friendly_central_highland_lower_gain_point = self._rfid_bit(bits_low, bits_high, 9)
        self.rfid_status_msg.friendly_central_highland_upper_gain_point = self._rfid_bit(bits_low, bits_high, 10)
        self.rfid_status_msg.enemy_central_highland_lower_gain_point = self._rfid_bit(bits_low, bits_high, 11)
        self.rfid_status_msg.enemy_central_highland_upper_gain_point = self._rfid_bit(bits_low, bits_high, 12)
        self.rfid_status_msg.friendly_highway_lower_gain_point = self._rfid_bit(bits_low, bits_high, 13)
        self.rfid_status_msg.friendly_highway_upper_gain_point = self._rfid_bit(bits_low, bits_high, 14)
        self.rfid_status_msg.enemy_highway_lower_gain_point = self._rfid_bit(bits_low, bits_high, 15)
        self.rfid_status_msg.enemy_highway_upper_gain_point = self._rfid_bit(bits_low, bits_high, 16)
        self.rfid_status_msg.friendly_fortress_gain_point = self._rfid_bit(bits_low, bits_high, 17)
        self.rfid_status_msg.friendly_outpost_gain_point = self._rfid_bit(bits_low, bits_high, 18)
        self.rfid_status_msg.friendly_supply_zone_non_exchange = self._rfid_bit(bits_low, bits_high, 19)
        self.rfid_status_msg.friendly_supply_zone_exchange = self._rfid_bit(bits_low, bits_high, 20)
        self.rfid_status_msg.friendly_big_resource_island = self._rfid_bit(bits_low, bits_high, 21)
        self.rfid_status_msg.enemy_big_resource_island = self._rfid_bit(bits_low, bits_high, 22)
        self.rfid_status_msg.center_gain_point = self._rfid_bit(bits_low, bits_high, 23)

    def publish_all(self) -> None:
        with self.lock:
            self.game_status_pub.publish(self.game_status_msg)
            self.robot_status_pub.publish(self.robot_status_msg)
            self.rfid_status_pub.publish(self.rfid_status_msg)
            self.all_robot_hp_pub.publish(self.all_robot_hp_msg)
            self.event_data_pub.publish(self.event_data_msg)
            self.buff_pub.publish(self.buff_msg)
            self.ground_robot_position_pub.publish(self.ground_robot_position_msg)
            if self.damage_latched:
                self.robot_status_msg.is_hp_deduced = False
                self.damage_latched = False

    def destroy_node(self) -> bool:
        self.running = False
        if self.referee_proc is not None:
            try:
                os.killpg(os.getpgid(self.referee_proc.pid), signal.SIGTERM)
                self.referee_proc.wait(timeout=5)
            except Exception:
                try:
                    os.killpg(os.getpgid(self.referee_proc.pid), signal.SIGKILL)
                except Exception:
                    pass
            self.get_logger().info("Stopped relaunched referee_serial_node")
        if hasattr(self, "serial_read_thread") and self.serial_read_thread.is_alive():
            self.serial_read_thread.join(timeout=1.0)
        if hasattr(self, "pty_read_thread") and self.pty_read_thread.is_alive():
            self.pty_read_thread.join(timeout=1.0)
        if hasattr(self, "serial_normal") and self.serial_normal.is_open:
            self.serial_normal.close()
        if hasattr(self, "pty_master_fd"):
            os.close(self.pty_master_fd)
        if hasattr(self, "pty_slave_fd"):
            os.close(self.pty_slave_fd)
        pty_path = Path(self.pty_path)
        if pty_path.exists() or pty_path.is_symlink():
            pty_path.unlink()
        if self.service_was_stopped and self.manage_service:
            subprocess.run(
                ["sudo", "systemctl", "start", "ros2-dji-referee.service"],
                capture_output=True, text=True, timeout=10,
            )
            self.get_logger().info("Restarted ros2-dji-referee.service")
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RealityRefereeBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
