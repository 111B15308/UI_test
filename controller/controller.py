import json
import dronekit
import threading
from PyQt5.QtCore import QObject
from PyQt5 import QtWidgets, QtWebEngineWidgets, QtCore
from view.settings_dialog import SettingsDialog
from view.drone_config_dialog import DroneConfigDialog
from view.view import MapView
from controller.mission_api import mission_api
from dronekit import connect, VehicleMode, LocationGlobalRelative
from drone.drone import Drone
from geopy.distance import geodesic
import time

class MapController(QObject):
    def __init__(self, model, view, drone_count):
        super().__init__()
        self.model = model
        self.view = view
        
        self.map_loaded = False  # 新增 flag，預設 False，等 WebView 載入完成後觸發
        self.drones = []
        self.current_wp_index = 0
        self.sequence_flying = False # ✅ 新增一個旗標，用於防止重複啟動循序飛行任務
        self.stop_sequence_flag = False # ✅ 用於從外部停止循序飛行執行緒
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.refresh_status)
        self.timer.timeout.connect(self.update_all_states)
        self.timer.start(1000)  # 每 1000ms 更新一次

        # connect UI buttons (top_bar 隱藏但按鈕物件存在)

        #self.view.add_btn.clicked.connect(self.on_add_marker_clicked)
        #self.view.center_btn.clicked.connect(self.on_center_clicked)
        self.view.arm_takeoff_btn.clicked.connect(self._on_arm_and_takeoff_all)
        self.view.fly_btn.clicked.connect(self._on_fly_to_next_wp)
        self.view.seq_btn.clicked.connect(self._on_fly_sequence)
        self.view.clear_btn.clicked.connect(self.on_clear_markers)
        self.view.stop_btn.clicked.connect(self.on_emergency_stop)
        self.view.rtl_btn.clicked.connect(self.on_rtl)

        # connect from JS (右鍵航點) -> Bridge slot emits waypointAdded signal
        self.view.bridge.waypointAdded.connect(self.on_waypoint_added)

        # IMPORTANT: 等 WebView 載入完成後再 sync（避免 setCenter 等函式尚未定義）
        self.view.webview.page().loadFinished.connect(self.on_map_loaded)
        self.view.connect_btn.clicked.connect(lambda: self.on_connect_clicked(drone_count))
        mission_api.start_position_watcher(self.on_drone_states_update)

    def sync_model_to_view(self, reset_center=False):
        c = self.model.center
        z = self.model.zoom
        if reset_center:
            self.view.run_js(f"setCenter({c['lat']}, {c['lng']}, {z});")
        self.view.run_js("clearMarkers();")
        coords = []
        for m in self.model.markers:
            label = (m.get("label") or "").replace("'", "\\'")
            self.view.run_js(f"addMarker('{m['id']}', {m['lat']}, {m['lng']}, '{label}');")
            coords.append([m['lat'], m['lng']])
        if len(coords) > 1:
            self.view.run_js(f"drawPath({json.dumps(coords)});")
    
    def on_map_loaded(self, ok):
        if ok:
            print("✅ 地圖載入完成")
            self.map_loaded = True
            # 地圖載入完成後，再同步 model 航點到 view
            self.sync_model_to_view(reset_center=True)
        else:
            print("❌ 地圖載入失敗")

    def on_connect_clicked(self, drone_count):
        """使用者點擊「連線」後的邏輯"""
        # 一一設定每台無人機
        if not hasattr(self, "drones"):
            self.drones = []

        for i in range(drone_count):
            # 若已連線成功，跳過不重設
            if i < len(self.drones) and self.drones[i].connected:
                print(f"🟢 Vehicle{i+1} 已連線，跳過設定。")
                continue
            config_dialog = DroneConfigDialog(i + 1)
            if config_dialog.exec_() != QtWidgets.QDialog.Accepted:
                print(" 使用者取消設定")
                return
            address = config_dialog.addr_input.text().strip()
            port = int(config_dialog.port_input.text())
            alt = config_dialog.alt_input.value()
            speed = config_dialog.speed_input.value()
            connection_str = f"udp:{address}:{port}"

            # 若該 index 已存在 Drone 物件 → 更新；否則新增
            if i < len(self.drones):
                drone = self.drones[i]
                drone.connection_string = connection_str
                drone.alt = alt
                drone.speed = speed
                if not drone.connected:
                    print(f"🔁 重新嘗試連線 vehicle{i+1}...")
                    try:
                        drone = Drone(i + 1, connection_str, alt, speed)
                        drone.connected = True
                    except Exception as e:
                        drone.connected = False
                        print(f"❌ vehicle{i+1} 重新連線失敗：{e}")
            else:
                drone = Drone(i + 1, connection_str, alt, speed)
                self.drones.append(drone)
            # ✅ 設定 RTL 時機頭朝向航點
            if drone.connected:
                drone.set_parameter('WP_YAW_BEHAVIOR', 1)
        self.drones.sort(key=lambda drone: drone.id)
        success_count = sum(1 for d in self.drones if d.connected)
        self.model.drones = self.drones
        print(f"📡 成功連線 {success_count}/{drone_count} 台無人機")

        # ✅ 將連線後的 drones 列表傳給 mission_api
        mission_api.drones = self.drones
        

    def refresh_status(self):
        self.view.update_status_panels(self.model.drones)    

    def update_all_states(self):
        """定期更新所有無人機狀態到地圖"""
        if not getattr(self, "map_loaded", False):
            return
        states = {}
        for drone in self.model.drones:
            s = drone.get_state()
            if s:
                states[drone.id] = {
                    "lat": s["lat"],
                    "lon": s["lon"],
                    "alt": s["alt"],
                    "speed": s["speed"],
                    "yaw": s["yaw"],
                    "mode": s["mode"]
                }
        if states:
            self.view.update_drone_positions(states)
                
    def on_add_marker_clicked(self):
        try:
            lat = float(self.view.lat_input.text())
            lng = float(self.view.lng_input.text())
        except Exception:
            return
        marker = {
            "id": f"m{len(self.model.markers)+1}",
            "lat": lat, "lng": lng,
            "label": f"第{len(self.model.markers)+1}航點"
        }
        self.model.add_marker(marker)
        self.sync_model_to_view()

    def on_center_clicked(self):
        try:
            lat = float(self.view.lat_input.text())
            lng = float(self.view.lng_input.text())
        except Exception:
            return
        self.model.center = {"lat": lat, "lng": lng}

    def on_clear_markers(self):
        """清除模型中的航點，並呼叫 JS 清除地圖上的圖示和線條"""
        self.model.clear_markers()
        self.view.run_js("clearMarkers();")  # ✅ 呼叫 JS 清除圖示和紅線
        self.current_wp_index = 0  # ✅ 將航點索引重設為 0

    def on_waypoint_added(self, lat, lng):
        if lat is None or lng is None:
            print("❌ 無效航點")
            return
        marker = {
            "id": f"m{len(self.model.markers)+1}",
            "lat": float(lat),
            "lng": float(lng),
            "label": f"第{len(self.model.markers)+1}航點"
        }
        self.model.add_marker(marker)
        self.sync_model_to_view()

    def _on_fly_to_next_wp(self):
        """讓 Drone1 飛往下一個航點"""
        if not hasattr(self, "current_wp_index"):
            self.current_wp_index = 0  # ✅ 初始化航點索引

        if not self.model.markers:
            print("❌ 尚未設定航點")
            return
        if not self.drones:
            print("❌ 尚未連線任何無人機")
            return

        if self.current_wp_index >= len(self.model.markers):
            print("✅ 已抵達最後一個航點")
            return

        wp = self.model.markers[self.current_wp_index]
        lat, lon = wp["lat"], wp["lng"]

        try:
            drone = self.drones[0]
            # ✅ 1. 在飛行前檢查是否已解鎖
            if not drone.get_state().get("armed"):
                self.view.show_warning("無人機還未解鎖!")
                return

            vehicle = drone.vehicle
            alt = vehicle.location.global_relative_frame.alt
            print(f"🛫 Drone1 飛往第 {self.current_wp_index + 1} 個航點: ({lat}, {lon}, {alt})")

            # 若不是 GUIDED 模式就切換
            if vehicle.mode.name != "GUIDED":
                vehicle.mode = VehicleMode("GUIDED")
                time.sleep(1)

            # ✅ 2. 記錄飛行前的位置
            start_pos = vehicle.location.global_relative_frame

            # ✅ 設定機頭朝向目標點
            # WP_YAW_BEHAVIOR=1: FACE NEXT WAYPOINT
            if vehicle.parameters['WP_YAW_BEHAVIOR'] != 1:
                vehicle.parameters['WP_YAW_BEHAVIOR'] = 1

            # ✅ 送出飛行指令
            vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt))

            # ✅ 3. 延遲一小段時間後，檢查無人機是否移動
            time.sleep(2) # 等待 2 秒讓無人機有時間反應
            current_pos = vehicle.location.global_relative_frame
            distance_moved = geodesic(
                (start_pos.lat, start_pos.lon),
                (current_pos.lat, current_pos.lon)
            ).meters

            # ✅ 4. 判斷是否成功飛行
            if distance_moved < 0.5: # 如果移動距離小於 0.5 公尺，視為未成功飛行
                print("⚠️ 未成功飛行，目標航點將保持不變。")
                # 因為沒有成功飛行，所以不增加 current_wp_index
            else:
                print("✅ 飛行指令已成功執行，無人機移動中。")
                self.current_wp_index += 1  # 成功飛行，移到下一個航點

        except Exception as e:
            print(f"⚠️ 飛行指令失敗: {e}")
    
    def _on_arm_and_takeoff_all(self):
        """所有已連線的無人機進入GUIDED並起飛到設定高度"""
        if not self.drones:
            print("❌ 尚未連線任何無人機")
            return
 
        print("🟡 所有無人機準備解鎖並起飛...")

        for drone in self.drones:       
            if not drone.connected:
                print(f"⛔ Drone {drone.id} 未連線，略過起飛。")
                continue
            try:
                print(f"🚁 Drone {drone.id} 起飛中...")
                drone.set_guided_and_arm()
                # ✅ 使用背景執行緒執行起飛（避免 UI 卡死）
                thread = threading.Thread(target=drone.takeoff, args=(drone.alt,))
                thread.start()
            except Exception as e:
                print(f"⚠️ Drone {drone.id} 起飛失敗: {e}")

        print("✅ 所有無人機已起飛完成")

    # === 新增：控制按鈕事件 ===
    def on_emergency_stop(self):
        print("⚠️ 按下緊急停止")
        self.stop_sequence_flag = True # ✅ 設定停止旗標，通知背景執行緒終止
        # 這裡不重設 self.current_wp_index，以便下次可以從同一個航點繼續
        mission_api.emergency_stop()

    def on_rtl(self):
        print("🟢 按下返回Home")
        mission_api.return_to_launch()
    
    def on_apply_settings_from_settings_dialog(self):
        """被 SettingsController 呼叫或在 UI 中按下確認後觸發"""
        try:
            count = getattr(self.model, "drone_count", 3)
            form = getattr(self.model, "formation", "Line")
            mission_api.set_params(count, form)
            print(f"Applied settings -> drones: {count}, formation: {form}")
        except Exception as e:
            print("on_apply_settings error:", e)

    def _on_fly_sequence(self):
        """Drone1 依序飛往剩下的航點（背景執行緒）"""
        if not self.drones or not self.model.markers:
            print("❌ 尚未連線或未設定航點")
            return
        
        # ✅ 檢查是否已有循序飛行任務在執行
        if self.sequence_flying:
            print("⚠️ 循序飛行任務已在執行中，請勿重複點擊。")
            return
        self.sequence_flying = True # ✅ 上鎖
        self.stop_sequence_flag = False # ✅ 重設停止旗標

        # ✅ 增加未解鎖警告
        drone = self.drones[0]
        if not drone.get_state().get("armed"):
            self.view.show_warning("無人機還未解鎖!")
            return

        def sequence_thread():
            drone = self.drones[0]  # 目前只控制 Drone1
            vehicle = drone.vehicle

            try:
                while self.current_wp_index < len(self.model.markers):
                    # ✅ 在每個迴圈開始時檢查停止旗標
                    if self.stop_sequence_flag:
                        print("🛑 循序飛行任務被手動終止。")
                        break

                    wp = self.model.markers[self.current_wp_index]
                    lat, lon = wp["lat"], wp["lng"]
                    alt = vehicle.location.global_relative_frame.alt
                    print(f"🛫 Drone1 飛往第 {self.current_wp_index + 1} 個航點: ({lat}, {lon}, {alt})")

                    # 確保 GUIDED 模式
                    if vehicle.mode.name != "GUIDED":
                        vehicle.mode = VehicleMode("GUIDED")
                        time.sleep(1)

                    # 增加飛行確認機制
                    start_pos = vehicle.location.global_relative_frame
                    vehicle.simple_goto(LocationGlobalRelative(lat, lon, alt))
                    time.sleep(2) # 等待反應
                    current_pos = vehicle.location.global_relative_frame
                    distance_moved = geodesic((start_pos.lat, start_pos.lon), (current_pos.lat, current_pos.lon)).meters

                    if distance_moved < 0.5:
                        print(f"⚠️ Drone1 未能成功飛往第 {self.current_wp_index + 1} 個航點，任務終止。")
                        break

                    # 等待到達航點
                    while True:
                        # ✅ 在等待時也檢查停止旗標
                        if self.stop_sequence_flag:
                            break
                        current = vehicle.location.global_relative_frame
                        dist_to_target = geodesic((current.lat, current.lon), (lat, lon)).meters
                        if dist_to_target < 1.0: # 到達半徑 1 公尺內
                            break
                        time.sleep(1)

                    if self.stop_sequence_flag: continue # 如果是手動停止，直接跳到 while 迴圈的開頭進行最終檢查

                    print(f"✅ Drone1 抵達第 {self.current_wp_index + 1} 個航點")
                    self.current_wp_index += 1

                print("✅ Drone1 已完成或終止循序飛行任務")
            finally:
                # ✅ 使用 finally 確保任務無論如何結束，都會解鎖
                self.sequence_flying = False

        # 啟動背景執行緒
        threading.Thread(target=sequence_thread, daemon=True).start()

    def on_drone_states_update(self, states):
        """接收 mission_api 回報的無人機狀態，轉給 view 更新地圖"""
        if not getattr(self, "map_loaded", False):
            return  # JS 還沒準備好
        self.view.update_drone_positions(states)
    
class SettingsController:
    def __init__(self, model):
        self.model = model
        self.view = SettingsView()

        # 綁定 "確認" 按鈕
        self.view.confirm_btn.clicked.connect(self.apply_settings)

    def apply_settings(self):
        # 讀取選項
        index_count = self.view.combo_drone_count.currentIndex()
        if index_count == 0:
            self.model.drone_count = 3
        elif index_count == 1:
            self.model.drone_count = 5

        formations = ["Line", "Wedge", "Square"]
        self.model.formation = formations[self.view.combo_formation.currentIndex()]
        self.view.close()
