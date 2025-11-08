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

    def set_parameter(self, param_name, value):
        """設定無人機參數"""
        if self.connected and self.vehicle:
            print(f"Drone {self.id}: 設定參數 {param_name} -> {value}")
            self.vehicle.parameters[param_name] = value
        else:
            print(f"Drone {self.id}: 未連線，無法設定參數")

    # ----------------------------------------------------------
    # 模式控制
    # ----------------------------------------------------------
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

    def set_loiter_mode(self):
        """切換到 LOITER (懸停) 模式"""
        if not self.connected or self.vehicle is None:
            print(f"⚠️ Drone {self.id}: 尚未連線，無法切換模式")
            return
        if self.vehicle.mode.name != "LOITER":
            self.vehicle.mode = VehicleMode("LOITER")
            print(f"🚁 Drone {self.id}: 已切換至 LOITER 模式 (原地懸停)")

    def hold_position(self):
        """
        發送速度為 0 的指令，讓無人機在原地懸停。
        這是比切換到 LOITER 更直接的懸停方式。
        """
        if not self.connected or self.vehicle is None:
            print(f"⚠️ Drone {self.id}: 尚未連線，無法執行懸停")
            return

        # ✅ 使用 dronekit 的 message_factory 創建 MAVLink 訊息
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,       # time_boot_ms (not used)
            0, 0,    # target_system, target_component (not used)
            mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
            0b0000111111000111, # type_mask (only speeds enabled)
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0) # x, y, z, vx, vy, vz, afx, afy, afz, yaw, yaw_rate
        self.vehicle.send_mavlink(msg)
        print(f"🛑 Drone {self.id}: 已發送原地懸停指令 (速度歸零)。")

    def disarm(self):
        """上鎖馬達"""
        if not self.connected or self.vehicle is None:
            return
        self.vehicle.armed = False
        print("🔒 已上鎖馬達")

    # ----------------------------------------------------------
    def rtl(self):
        """切換到 RTL (返航) 模式"""
        if not self.connected or self.vehicle is None:
            print(f"⚠️ Drone {self.id}: 尚未連線，無法返航")
            return
        self.vehicle.mode = VehicleMode("RTL")
        print(f"🔙 Drone {self.id}: 已切換至 RTL 模式 (返航中...)")

    def land(self):
        """降落"""
        if not self.connected or self.vehicle is None:
            return
        self.vehicle.mode = VehicleMode("LAND")
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
    # 關閉連線
    # ----------------------------------------------------------
    def close_conn(self):
        if self.vehicle:
            self.vehicle.close()
            print("❎ 已關閉無人機連線")
