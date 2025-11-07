import dronekit
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
from PyQt5 import QtWidgets
from model import formation_setting
from geopy.distance import geodesic


class Drone:
    """
    以 pymavlink 直接控制的無人機類別
    可直接連線至 SITL 或真機 MAVLink 端口
    """

    def __init__(self, drone_id,  connection_string, alt, speed):
        self.id = drone_id
        self.connection_string = connection_string
        self.alt = alt
        self.speed = speed
        self.vehicle = None
        self.connected = False

        print(f"🔗 嘗試連線至無人機 vehicle{drone_id}: {connection_string}")

        try:
            self.vehicle = connect(connection_string, wait_ready=True, timeout=20)
            self.connected = True
            print(f"✅ vehicle{drone_id} 連線成功！")
        except Exception as e:
            self.connected = False
            self.vehicle = None
            print(f"❌ vehicle{drone_id} 連線失敗：{e}")

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

    def set_guided_and_arm(self):
        """
        設定為 GUIDED 模式並解鎖
        """
        if not self.connected or self.vehicle is None:
            raise RuntimeError(f"Drone {self.id}: vehicle 尚未初始化")
        v = self.vehicle

        while not v.is_armable:
            print(f"Drone {self.id}: 等待初始化中...")
            time.sleep(1)

        # 切換為 GUIDED 模式
        while v.mode.name != "GUIDED":
            v.mode = VehicleMode("GUIDED")
            time.sleep(1)
        print(f"Drone {self.id}: 模式切換為 GUIDED")

        # 解鎖
        v.armed = True
        while not v.armed:
            print(f"Drone {self.id}: 等待解鎖...")
            time.sleep(1)
        print(f"✅ Drone {self.id}: 已解鎖完成")

        time.sleep(2)  # 讓槳轉穩定一點

    def takeoff(self, target_alt):
        """
        起飛到設定高度 (阻塞直到達到目標高度)
        """
        if not self.connected or self.vehicle is None:
            raise RuntimeError(f"Drone {self.id}: vehicle 尚未初始化")

        print(f"Drone {self.id}: 起飛至 {target_alt} 公尺...")

        # 送出起飛命令
        self.vehicle.simple_takeoff(target_alt)

        # 進入阻塞式等待
        while True:
            current_alt = self.vehicle.location.global_relative_frame.alt

            print(f"Drone {self.id}: 正在上升，目前高度 = {current_alt:.2f} m")
            
            # ✅ 讓 PyQt 的事件能繼續處理（避免 UI 卡死）
            QtWidgets.QApplication.processEvents()

            # 若達到目標高度的 95%，就視為起飛完成
            if current_alt >= target_alt * 0.95:
                print(f"✅ Drone {self.id}: 已達目標高度 {current_alt:.2f} m！")
                break

            # 每 1 秒檢查一次高度
            time.sleep(1)
        print(f"Drone {self.id}: 起飛完成")


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
        """
        取得 Drone 狀態，回傳字典
        """
        if not hasattr(self, "vehicle") or self.vehicle is None:
            return None

        v = self.vehicle
        state = {
            "lat": v.location.global_relative_frame.lat,
            "lon": v.location.global_relative_frame.lon,
            "alt": v.location.global_relative_frame.alt,
            "speed": v.airspeed if v.airspeed else 0,
            "yaw": v.heading if hasattr(v, "heading") else 0,
            "mode": v.mode.name if v.mode else "UNKNOWN",
            "armed": v.armed if hasattr(v, "armed") else False
        }

        return state

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
