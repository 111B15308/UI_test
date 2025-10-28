import json
from PyQt5.QtCore import QObject
from PyQt5 import QtWidgets, QtWebEngineWidgets, QtCore
from view.settings_view import SettingsView
from view.settings_dialog import SettingsDialog, DroneConfigDialog
from controller.mission_api import mission_api

class MapController(QObject):
    def __init__(self, model, view):
        super().__init__()
        self.model = model
        self.view = view

        # connect UI buttons (top_bar 隱藏但按鈕物件存在)
        try:
            self.view.add_btn.clicked.connect(self.on_add_marker_clicked)
            self.view.center_btn.clicked.connect(self.on_center_clicked)
            self.view.clear_btn.clicked.connect(self.on_clear_markers)

            # 新增：緊急停止 / 返回Home
            self.view.stop_btn.clicked.connect(self.on_emergency_stop)
            self.view.rtl_btn.clicked.connect(self.on_rtl)
            self.view.start_btn.clicked.connect(self.on_start_mission)
        except Exception:
            # 如果 top_bar 被完全移除，這裡會發生例外，我們安全忽略
            pass

        # connect from JS (右鍵航點) -> Bridge slot emits waypointAdded signal
        self.view.bridge.waypointAdded.connect(self.on_waypoint_added)

        # IMPORTANT: 等 WebView 載入完成後再 sync（避免 setCenter 等函式尚未定義）
        self.view.webview.page().loadFinished.connect(self.sync_model_to_view)
        mission_api.initialize_formation()
        mission_api.start_position_watcher(self.on_drone_states_update)
        self.view.connect_btn.clicked.connect(self.on_connect_clicked)

    def sync_model_to_view(self, reset_center=False):
        """把 model 同步到 view (JS)"""
        c = self.model.center
        z = self.model.zoom

        # set center and clear markers
        if reset_center:
            self.view.run_js(f"setCenter({c['lat']}, {c['lng']}, {z});")
        self.view.run_js("clearMarkers();")

        coords = []
        for m in self.model.markers:
            label = (m.get("label") or "").replace("'", "\\'")
            js = f"addMarker('{m['id']}', {m['lat']}, {m['lng']}, '{label}');"
            self.view.run_js(js)
            coords.append([m['lat'], m['lng']])

        if len(coords) > 1:
            coord_js = json.dumps(coords)
            self.view.run_js(f"drawPath({coord_js});")
    
    def on_connect_clicked(self):
          """使用者點擊「連線」後的邏輯"""
          settings_dialog = SettingsDialog()
          if settings_dialog.exec_() != QtWidgets.QDialog.Accepted:
              return  # 使用者取消
    
          settings = settings_dialog.get_settings()
          drone_count = settings["drone_count"]
          formation = settings["formation"]
    
          print(f" 選擇 {drone_count} 架無人機，隊形：{formation}")
    
          # 一一設定每台無人機
          drone_configs = []
          for i in range(drone_count):
              config_dialog = DroneConfigDialog(i + 1)
              if config_dialog.exec_() != QtWidgets.QDialog.Accepted:
                  print(" 使用者取消設定")
                  return
    
              drone_configs.append({
                  "port": int(config_dialog.port_input.text()),
                  "alt": config_dialog.alt_input.value(),
                  "speed": config_dialog.speed_input.value()
              })
    
          print(f"無人機設定完成：{drone_configs}")
    
          # 呼叫 mission_api 初始化 DroneKit（背景執行）
          mission_api.initialize_formation(drone_configs)

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
        self.model.clear_markers()

    def on_waypoint_added(self, lat, lng):
        """由地圖右鍵新增航點時呼叫（Bridge.emit -> 這裡接收）"""
        marker = {
            "id": f"m{len(self.model.markers)+1}",
            "lat": lat, "lng": lng,
            "label": f"第{len(self.model.markers)+1}航點"
        }
        self.model.add_marker(marker)
        self.sync_model_to_view()

    # === 新增：控制按鈕事件 ===
    def on_emergency_stop(self):
        print("⚠️ 按下緊急停止")
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

    def on_start_mission(self):
        """由 UI 的 '開始任務' 按鈕觸發"""
        # collect waypoints from model.markers (順序為加入順序)
        coords = []
        for m in self.model.markers:
            # marker 可能只有 lat/lng
            lat = m.get("lat")
            lng = m.get("lng")
            # 你可以提供 alt 若 UI 有該欄位；這裡不強制 alt
            coords.append((lat, lng))
        if not coords:
            print("尚未設定航點 (markers 為空)。")
            return
        # apply settings from model to mission_api
        mission_api.set_params(self.model.drone_count, self.model.formation)
        try:
            started = mission_api.start_mission(coords)
            if started:
                print("任務已啟動 (background thread)。")
        except Exception as e:
            print("啟動任務失敗：", e)
    
    def on_drone_states_update(self, states):
        """每秒更新地圖上的無人機位置"""
        for drone_id, s in states.items():
            lat = s.get("GlobalLat")
            lon = s.get("GlobalLon")
            if lat is None or lon is None:
                continue
            js = f"updateDroneMarker({drone_id}, {lat}, {lon});"
            self.view.run_js(js)
    
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
