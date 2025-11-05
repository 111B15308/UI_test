from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
from model import formation_setting
from geopy.distance import geodesic

class Drone:
    """
    以 pymavlink 直接控制的無人機類別
    可直接連線至 SITL 或真機 MAVLink 端口
    """

    def __init__(self, connection_string):
        print(f"🔗 嘗試連線至無人機: {connection_string}")
        self.connection = None
        self.connected = False
        try:
            self.connection = mavutil.mavlink_connection(connection_string)
            print("⌛ 等待 HEARTBEAT ...")
            hb = self.connection.recv_match(type='HEARTBEAT', blocking=True, timeout=30)
            if not hb:
                raise TimeoutError("Heartbeat timeout")
            print("✅ 連線成功，接收到 HEARTBEAT")
            self.connected = True
        except Exception as e:
            print(f"❌ 無法連線至無人機: {e}")

    # ----------------------------------------------------------
    # 模式控制
    # ----------------------------------------------------------
    def set_mode(self, mode_name="GUIDED"):
        if not self.link:
            print("⚠️ 尚未連線無人機")
            return
        mode_map = {
            "GUIDED": 4,
            "LOITER": 5,
            "RTL": 6,
            "LAND": 9,
            "BRAKE": 17,
        }
        mode_id = mode_map.get(mode_name.upper(), 4)
        self.link.mav.set_mode_send(
            self.link.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"🧭 切換模式為: {mode_name}")

    def arm_and_takeoff(self, alt=10):
        if not self.connection:
            return
        print("🌀 解鎖馬達...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(2)
        print(f"🚁 起飛至 {alt} 公尺")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt
        )

    def disarm(self):
        """上鎖馬達"""
        if not self.link:
            return
        self.link.mav.command_long_send(
            self.link.target_system,
            self.link.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        print("🔒 已上鎖馬達")

    # ----------------------------------------------------------
    # 飛行控制
    # ----------------------------------------------------------
    def takeoff(self, altitude):
        """起飛到指定高度（公尺）"""
        if not self.link:
            return
        print(f"🚁 起飛至 {altitude} 公尺...")
        self.link.mav.command_long_send(
            self.link.target_system,
            self.link.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )

    def rtl(self):
        if not self.connection:
            return
        print("🔙 返航中...")
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0, 0, 0, 0, 0, 0, 0, 0
        )

    def land(self):
        """降落"""
        if not self.link:
            return
        self.link.mav.command_long_send(
            self.link.target_system,
            self.link.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        print("🪂 正在降落...")

    # ----------------------------------------------------------
    # 狀態讀取
    # ----------------------------------------------------------
    def get_state(self):
        """取得即時狀態"""
        if not self.connection:
            return None
        try:
            msg = self.connection.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            hb = self.connection.recv_match(type='HEARTBEAT', blocking=False)
            if not msg:
                return None
            return {
                "lat": msg.lat / 1e7,
                "lon": msg.lon / 1e7,
                "alt": msg.relative_alt / 1000.0,
                "yaw": getattr(msg, "hdg", 0) / 100.0,
                "mode": mavutil.mode_string_v10(hb) if hb else "UNKNOWN",
                "armed": bool(getattr(hb, "base_mode", 0) & 0b10000000)
            }
        except Exception as e:
            print(f"⚠️ 無法取得狀態: {e}")
            return None

    # ----------------------------------------------------------
    # 其他控制
    # ----------------------------------------------------------
    def condition_yaw(self, heading, relative=False):
        """設定朝向角"""
        if not self.link:
            return
        is_relative = 1 if relative else 0
        self.link.mav.command_long_send(
            self.link.target_system,
            self.link.target_component,
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,
            0,
            heading, 10, 1, is_relative, 0, 0, 0
        )

    def send_global_velocity(self, vx, vy, vz):
        """設定全域速度 (m/s)"""
        if not self.link:
            return
        self.link.mav.set_position_target_global_int_send(
            0,
            self.link.target_system,
            self.link.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111000111,  # 僅啟用速度控制
            0, 0, 0,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )

    # ----------------------------------------------------------
    # 關閉連線
    # ----------------------------------------------------------
    def close_conn(self):
        if self.link:
            self.link.close()
            print("❎ 已關閉無人機連線")
