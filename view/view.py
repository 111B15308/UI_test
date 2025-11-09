import os
import json
from PyQt5 import QtWidgets, QtWebEngineWidgets, QtCore
from PyQt5.QtCore import Qt, QUrl, QObject, pyqtSignal
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QPushButton
)
from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel
from PyQt5.QtWidgets import QComboBox, QLabel, QMessageBox
from PIL import Image, ImageDraw


class Bridge(QObject):
    waypointAdded = pyqtSignal(float, float)  # 傳航點座標到 Python

    @pyqtSlot(float, float)
    def addWaypoint(self, lat, lng):
        # 觸發 signal
        self.waypointAdded.emit(lat, lng)


class MapView(QMainWindow):
    def __init__(self, model, drone_count):
        super().__init__()
        self.model = model
        self.status_panels = []
        self.setWindowTitle("Simulator Map (MVC)")
        self.setGeometry(100, 100, 1280, 800)
        
        # 當 model 狀態改變時，更新地圖
        self.model.state_changed.connect(self.update_formation)

        central = QWidget()
        self.setCentralWidget(central)
        # ✅ 改用水平佈局，左邊是地圖，右邊是按鈕
        self.main_layout = QtWidgets.QHBoxLayout()
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        central.setLayout(self.main_layout)

        # Leaflet map webview
        self.webview = QWebEngineView()
        # ✅ 讓地圖佔用大部分空間
        self.main_layout.addWidget(self.webview, 1)

        # 建立 QWebChannel 橋接
        self.channel = QWebChannel()
        self.bridge = Bridge()
        self.channel.registerObject("qtbridge", self.bridge)
        self.webview.page().setWebChannel(self.channel)

        # ✅ 創建控制面板並加入主佈局
        self._create_overlay_controls()
        # ✅ 將控制面板的寬度固定
        self.top_bar.setFixedWidth(170)

        self._load_map_html(drone_count)

        # 綁定按鈕
        self.connect_bar = QtWidgets.QHBoxLayout()
        self.connect_btn = QtWidgets.QPushButton("連線")
        self.connect_bar.addWidget(self.connect_btn)

        # ✅ 將連線按鈕設為浮動視窗，並在 resizeEvent 中定位
        self.connect_widget = QWidget(self)
        self.connect_widget.setLayout(self.connect_bar)
        self.connect_widget.setFixedSize(150, 50)

    def update_drone_positions(self, states):
        """更新地圖上無人機圖標位置"""
        if not hasattr(self, "webview"):
            return

        js_code = f"updateAllDrones({json.dumps(states)});"
        self.webview.page().runJavaScript(js_code)

    def update_formation(self):
        """根據 model 的 drone_count 和 formation 在地圖上畫出無人機"""
        count = self.model.drone_count
        formation = self.model.formation

        js_clear = "clearMarkers();"
        self.run_js(js_clear)

        lat, lng = self.model.center["lat"], self.model.center["lng"]

        if formation == "Line":
            for i in range(count):
                self.run_js(f"addMarker('d{i}', {lat}, {lng + i*0.0001}, 'Drone {i+1}');")
        elif formation == "Wedge":
            for i in range(count):
                offset = (i - count//2) * 0.0001
                self.run_js(f"addMarker('d{i}', {lat + abs(offset)}, {lng + offset}, 'Drone {i+1}');")
        elif formation == "Square":
            size = int(count**0.5)
            k = 0
            for i in range(size):
                for j in range(size):
                    if k < count:
                        self.run_js(f"addMarker('d{k}', {lat + i*0.0001}, {lng + j*0.0001}, 'Drone {k+1}');")
                        k += 1

    def _create_overlay_controls(self):
        """右上角控制按鈕"""
        self.top_bar = QWidget(self)
        self.top_bar.setAttribute(Qt.WidgetAttribute.WA_StyledBackground, True)
        self.top_bar.setStyleSheet("background: rgba(0,0,0,0.6); color: white; border-radius: 6px;")
        v = QVBoxLayout()
        v.setContentsMargins(8, 8, 8, 8)
        v.setSpacing(10)
        self.top_bar.setLayout(v)
    
        # --- 原有按鈕 ---
        deploy_btn = QPushButton("展開隊型")
        deploy_btn.setToolTip("讓所有無人機飛到第一個航點就位並對齊姿態")
        deploy_btn.setFixedSize(150, 40)
        arm_takeoff_btn = QPushButton("解鎖並起飛")
        arm_takeoff_btn.setFixedSize(150, 40)
        fly_btn = QPushButton("飛向下一個航點")
        fly_btn.setFixedSize(150, 40)
        seq_btn = QPushButton("依序飛往航點")
        seq_btn.setToolTip("單機時依序飛行，多機時執行群飛任務") # ✅ 增加提示文字
        seq_btn.setFixedSize(150, 40)
        # formation_btn = QPushButton("執行群飛任務") # ❌ 不再需要獨立按鈕
        # formation_btn.setFixedSize(150, 40)
        clear_btn = QPushButton("清除航點")
        clear_btn.setFixedSize(150, 40)
        stop_btn = QPushButton("緊急停止")
        stop_btn.setFixedSize(150, 40)
        rtl_btn = QPushButton("RTL")
        rtl_btn.setFixedSize(150, 40)
 
        v.addWidget(arm_takeoff_btn)
        v.addWidget(deploy_btn)
        v.addWidget(fly_btn)
        v.addWidget(seq_btn)
        # v.addWidget(formation_btn) # ❌ 不再需要獨立按鈕
        v.addWidget(clear_btn)
        v.addWidget(stop_btn)
        v.addWidget(rtl_btn)


        # --- 保存引用以便在 controller 綁定 ---
        self.deploy_btn = deploy_btn
        self.arm_takeoff_btn = arm_takeoff_btn
        self.fly_btn = fly_btn
        self.seq_btn = seq_btn
        self.formation_btn = seq_btn # ✅ 讓 formation_btn 指向 seq_btn，確保 controller 不出錯
        self.clear_btn = clear_btn
        self.stop_btn = stop_btn
        self.rtl_btn = rtl_btn

        # ✅ 手動調整 top_bar 的大小以適應其內容
        # 讓佈局計算其最小需要的高度，然後設定給 top_bar
        self.top_bar.setFixedSize(self.top_bar.sizeHint())

    def show_warning(self, message):
        """顯示一個通用的警告訊息視窗"""
        msg = QMessageBox(self)
        msg.setWindowTitle("警告")
        msg.setText(message)
        msg.setIcon(QMessageBox.Warning)
        msg.setStandardButtons(QMessageBox.Ok)
        msg.exec_()

    def resizeEvent(self, event):
        """覆寫視窗大小改變事件，以重新定位左下角的按鈕"""
        super().resizeEvent(event)
        # ✅ 將 connect_widget 定位在左下角
        self.connect_widget.move(20, self.height() - self.connect_widget.height() - 20)

        # ✅ 將 top_bar (右側按鈕) 定位在右上角
        self.top_bar.move(self.width() - self.top_bar.width() - 20, 20)

        # ✅ 重新定位左上角的狀態面板
        y_offset = 20
        for panel_info in self.status_panels:
            panel_info["panel"].move(20, y_offset)
            y_offset += panel_info["panel"].height() + 10


    def create_status_panels(self, drone_count):
        """根據 drone_count 動態生成左側無人機狀態欄"""
        for panel in self.status_panels:
            panel.setParent(None)
        self.status_panels.clear()

        y_offset = 20  # 初始 y 座標
        for i in range(drone_count):
            panel = QtWidgets.QWidget(self)
            panel.setStyleSheet("background: rgba(0,0,0,0.6); color: white; border-radius: 6px;")
            panel.setFixedSize(150, 120)

            layout = QtWidgets.QVBoxLayout(panel)
            layout.setContentsMargins(8, 8, 8, 8)
            layout.setSpacing(5)
            
            label_name = QtWidgets.QLabel(f" Drone {i+1}")
            label_speed = QtWidgets.QLabel(" 速度: 0 m/s")
            label_alt = QtWidgets.QLabel(" 高度: 0 m")
            label_mode = QtWidgets.QLabel(" 模式: N/A")
            label_yaw = QtWidgets.QLabel(" 頭朝向: 0°")

            layout.addWidget(label_name)
            layout.addWidget(label_speed)
            layout.addWidget(label_alt)
            layout.addWidget(label_mode)
            layout.addWidget(label_yaw)

            panel.move(20, y_offset) # 初始定位
            panel.show()

            # 存 panel 及 label 以便更新
            self.status_panels.append({
                "panel": panel,
                "speed": label_speed,
                "alt": label_alt,
                "mode": label_mode,
                "yaw": label_yaw
            })

            y_offset += panel.height() + 10  # panel 高度 + 間距

    def update_status_panels(self, drones):
        """
        根據 drones 列表更新左側狀態面板
        drones: list of Drone 物件
        """
        for i, drone in enumerate(drones):
            if i >= len(self.status_panels):
                continue
            panel_info = self.status_panels[i]
            state = drone.get_state()
            if state:
                panel_info["speed"].setText(f"速度: {state.get('speed', 0):.1f} m/s")
                panel_info["alt"].setText(f"高度: {state.get('alt', 0):.1f} m")
                panel_info["mode"].setText(f"模式: {state.get('mode', 'N/A')}")
                panel_info["yaw"].setText(f"頭朝向: {state.get('yaw', 0):.1f}°")
                
    def _load_map_html(self, drone_count):
        # ✅ 呼叫新方法來產生帶有指針的圖示
        new_drone_icon_path = self._create_drone_icon_with_pointer()
        drone_url = QUrl.fromLocalFile(new_drone_icon_path).toString()

        html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8">
            <script src="qrc:///qtwebchannel/qwebchannel.js"></script>
            <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css"/>
            <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
            <script src="https://rawcdn.githack.com/bbecquet/Leaflet.RotatedMarker/master/leaflet.rotatedMarker.js"></script>
            <style>
                html, body, #map {{ height: 100%; margin: 0; }}
                #popup {{
                    display: none;
                    position: fixed;
                    top: 50%;
                    left: 50%;
                    transform: translate(-50%, -50%);
                    background: white;
                    border: 2px solid black;
                    border-radius: 8px;
                    padding: 20px;
                    text-align: center;
                    z-index: 1000;
                }}
                #popup button {{
                    margin-top: 10px;
                    padding: 6px 20px;
                }}
            </style>
        </head>
        <body>
            <div id="map"></div>
            <div id="popup">
                <p>目前沒有航點喔!</p>
                <button onclick="closePopup()">確認</button>
            </div>
            <script>
                var map = L.map('map').setView([22.9048880, 120.2719823], 20);
                L.tileLayer('https://{{s}}.tile.openstreetmap.org/{{z}}/{{x}}/{{y}}.png', {{
                    attribution: '© OpenStreetMap contributors'
                }}).addTo(map);

                var markers = [];
                var polylines = [];

                // 無人機圖示
                var droneIcon = L.icon({{
                    iconUrl: '{drone_url}',
                    iconSize: [48, 48],
                    iconAnchor: [24, 24],
                    popupAnchor: [0, -24]
                }});
                var droneMarkers = {{}};
                // var yawLines = {{}}; // 已經不需要 yawLines 了
                // ... (addMarker 和其他函數保持不變)

                function setCenter(lat, lng, zoom) {{
                    map.setView([lat, lng], zoom);
                }}

                function drawPath(coords) {{
                    if (!coords || coords.length < 2) return;
                    var latlngs = coords.map(c => [c[0], c[1]]);
                    var line = L.polyline(latlngs, {{color: 'red'}}).addTo(map);
                    // ✅ 將新建立的線條加入 polylines 陣列，以便後續清除
                    polylines.push(line);
                }}

                function clearMarkers() {{
                    for(var i=0;i<markers.length;i++) {{
                        map.removeLayer(markers[i]);
                    }}
                    markers = [];
                    for(var i=0;i<polylines.length;i++) {{
                        map.removeLayer(polylines[i]);
                    }}
                    polylines = [];
                }}

                function addMarker(id, lat, lng, label) {{
                    if (lat === undefined || lng === undefined) {{
                        console.error("Invalid LatLng for marker", id, lat, lng);
                        return;
                    }}
                    var m = L.marker([lat, lng]).addTo(map);
                    m.bindPopup(label || ("航點 " + id));
                    markers.push(m);

                    if(markers.length > 1){{
                        var prev = markers[markers.length - 2];
                        var line = L.polyline([prev.getLatLng(), m.getLatLng()], {{color: 'red'}}).addTo(map);
                        polylines.push(line);
                    }}
                }}


                function showPopup() {{
                    document.getElementById("popup").style.display = "block";
                }}
                function closePopup() {{
                    document.getElementById("popup").style.display = "none";
                }}

                function flyToFirstWaypoint() {{
                    if(markers.length === 0) {{
                        showPopup();
                        return;
                    }}
                    flyToTarget(markers[0].getLatLng());
                }}

                function flyToTarget(wp, callback) {{
                    var current = droneMarker.getLatLng();
                    var dx = wp.lng - current.lng;
                    var dy = wp.lat - current.lat;
                    var angle = Math.atan2(dy, dx) * 180 / Math.PI;
                    droneMarker.setRotationAngle(angle);

                    var steps = 100;
                    var i = 0;
                    var interval = setInterval(function() {{
                        i++;
                        var lat = current.lat + (wp.lat - current.lat) * (i/steps);
                        var lng = current.lng + (wp.lng - current.lng) * (i/steps);
                        droneMarker.setLatLng([lat, lng]);

                        if(i >= steps) {{
                            clearInterval(interval);
                            if(callback) callback();
                        }}
                    }}, 50);
                }}

                function flySequentialWaypoints() {{
                    if(markers.length === 0) {{
                        showPopup();
                        return;
                    }}
                    var idx = 0;
                    function next() {{
                        if(idx >= markers.length) return;
                        var wp = markers[idx].getLatLng();
                        idx++;
                        flyToTarget(wp, next);
                    }}
                    next();
                }}

                new QWebChannel(qt.webChannelTransport, function(channel) {{
                    window.qtbridge = channel.objects.qtbridge;

                    map.on('contextmenu', function(e){{
                        if(e.latlng && e.latlng.lat && e.latlng.lng){{
                            addMarker('m' + (markers.length+1), e.latlng.lat, e.latlng.lng, "航點 " + (markers.length+1));
                            window.qtbridge.addWaypoint(e.latlng.lat, e.latlng.lng);
                        }} else {{
                            console.error("Invalid right-click location", e.latlng);
                        }}
                    }});
                }});

                function updateAllDrones(states) {{
                    // ✅ 這裡不再解析 JSON，而是直接接收 dict (controller 已經轉好)
                    for (var id in states) {{
                        const s = states[id];
                        updateDroneMarker(id, s.lat, s.lon, s.yaw, droneIcon);
                    }}
                }}
                function updateDroneMarker(id, lat, lon, yaw, icon) {{
                    if (!window.droneMarkers) window.droneMarkers = {{}};
                    
                    // ✅ 因為指針已經在圖示上，不再需要計算線條
                    // 如果圖示不存在，則創建一個新的
                    if (!window.droneMarkers[id]) {{
                        window.droneMarkers[id] = L.marker([lat, lon], {{
                            icon: icon, // 使用傳入的 icon 變數
                            rotationAngle: yaw || 0,
                            rotationOrigin: "center center"
                        }}).addTo(map);
                        window.droneMarkers[id].bindPopup("Drone " + id);

                    // 如果圖示已存在，則更新其位置和方向
                    }} else {{
                        const marker = window.droneMarkers[id];
                        marker.setLatLng([lat, lon]);
                        if (typeof marker.setRotationAngle === 'function') {{
                            marker.setRotationAngle(yaw || 0);
                        }}

                    }}
                }}

            </script>
        </body>
        </html>


        """
        self.webview.setHtml(html, baseUrl=QUrl.fromLocalFile(os.path.dirname(__file__) + "/"))

    def _create_drone_icon_with_pointer(self):
        """
        使用 Pillow 在原始 drone.png 上繪製一條綠色指針，並儲存為新檔案。
        """
        base_path = os.path.dirname(__file__)
        original_icon_path = os.path.join(base_path, "picture", "drone.png")
        new_icon_path = os.path.join(base_path, "picture", "drone_with_pointer.png")

        # 打開原始圖示
        img = Image.open(original_icon_path).convert("RGBA")
        draw = ImageDraw.Draw(img)
        width, height = img.size
        center_x, center_y = width // 2, height // 2

        # 在圖示正上方（機頭方向）畫一條鮮綠色的粗線
        # 線條從中心點向上延伸到頂部邊緣
        draw.line([(center_x, center_y), (center_x, 0)], fill="#00FF00", width=4)

        # 儲存新圖示
        img.save(new_icon_path)
        return new_icon_path

    def run_js(self, js_code, callback=None):
        if callback:
            self.webview.page().runJavaScript(js_code, callback)
        else:
            self.webview.page().runJavaScript(js_code)

    def _on_arm_and_takeoff_all(self):
        """按鈕：解鎖並起飛"""
        pass
