import os
from PyQt5 import QtWidgets, QtWebEngineWidgets, QtCore
from PyQt5.QtCore import Qt, QUrl, QObject, pyqtSignal
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QPushButton
)
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel
from PyQt5.QtWidgets import QComboBox, QLabel


class Bridge(QObject):
    waypointAdded = pyqtSignal(float, float)  # 傳航點座標到 Python


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
        self.main_layout = QVBoxLayout()
        self.main_layout.setContentsMargins(0, 0, 0, 0)
        central.setLayout(self.main_layout)

        # Leaflet map webview
        self.webview = QWebEngineView()
        self.main_layout.addWidget(self.webview)

        # 建立 QWebChannel 橋接
        self.channel = QWebChannel()
        self.bridge = Bridge()
        self.channel.registerObject("qtbridge", self.bridge)
        self.webview.page().setWebChannel(self.channel)

        # overlay control bar
        self._create_overlay_controls()
        self._load_map_html(drone_count)

        # 綁定按鈕
        self.clear_btn.clicked.connect(self._on_clear_markers)
        self.fly_btn.clicked.connect(self._on_fly_to_wp1)
        self.seq_btn.clicked.connect(self._on_fly_sequential)
        
        self.connect_bar = QtWidgets.QHBoxLayout()
        self.connect_btn = QtWidgets.QPushButton("連線")
        self.connect_bar.addWidget(self.connect_btn)
        self.connect_bar.addStretch()
        self.main_layout.addLayout(self.connect_bar)

    def update_drone_positions(self, states):
        """更新地圖上無人機圖標位置"""
        if not hasattr(self, "webview"):
            return

        js_code = f"updateAllDrones({{json.dumps(states)}});"
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
        clear_btn = QPushButton("清除航點")
        clear_btn.setFixedSize(150, 40)
        fly_btn = QPushButton("飛向第1航點")
        fly_btn.setFixedSize(150, 40)
        seq_btn = QPushButton("依序飛到所有航點")
        seq_btn.setFixedSize(150, 40)
        stop_btn = QPushButton("緊急停止")
        stop_btn.setFixedSize(150, 40)
        rtl_btn = QPushButton("返回Home")
        rtl_btn.setFixedSize(150, 40)

        v.addWidget(clear_btn)
        v.addWidget(fly_btn)
        v.addWidget(seq_btn)
        v.addWidget(stop_btn)
        v.addWidget(rtl_btn)


        # --- 保存引用以便在 controller 綁定 ---
        self.clear_btn = clear_btn
        self.fly_btn = fly_btn
        self.seq_btn = seq_btn
        self.stop_btn = stop_btn
        self.rtl_btn = rtl_btn

        # --- 固定位置與大小 ---
        self.top_bar.setFixedWidth(170)
        self.top_bar.setFixedHeight(500)
        self.top_bar.move(self.width() - 180, 20)
        self.top_bar.show()

    
    def create_status_panels(self, drone_count):
        """根據 drone_count 動態生成左側無人機狀態欄"""
        for panel in self.status_panels:
            panel.setParent(None)
        self.status_panels.clear()

        y_offset = 0
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

            panel.move(20, y_offset)
            panel.show()

            # 存 panel 及 label 以便更新
            self.status_panels.append({
                "panel": panel,
                "speed": label_speed,
                "alt": label_alt,
                "mode": label_mode,
                "yaw": label_yaw
            })

            y_offset += 130  # panel 高度 + 間距

    def update_status(self, drone_states):
        """
        動態更新每台無人機的狀態
        drone_states: list of dict
        例如:
        [
            {"speed": 5, "alt": 15, "mode": "GUIDED", "yaw": 90},
            {"speed": 4, "alt": 16, "mode": "GUIDED", "yaw": 45},
        ]
        """
        for i, state in enumerate(drone_states):
            if i >= len(self.status_panels):
                break
            panel_info = self.status_panels[i]
            panel_info["speed"].setText(f"速度: {state.get('speed',0)} m/s")
            panel_info["alt"].setText(f"高度: {state.get('alt',0)} m")
            panel_info["mode"].setText(f"模式: {state.get('mode','N/A')}")
            panel_info["yaw"].setText(f"頭朝向: {state.get('yaw',0)}°")  

    def _load_map_html(self, drone_count):
        drone_path = os.path.abspath(os.path.join(os.path.dirname(__file__), "picture", "drone.png"))
        drone_url = QUrl.fromLocalFile(drone_path).toString()
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
                var droneCount = {drone_count};
                var droneIcon = L.icon({{
                    iconUrl: '{drone_url}',
                    iconSize: [48, 48],
                    iconAnchor: [24, 24],
                    popupAnchor: [0, -24]
                }});
                
                var droneMarkers = {{}}; 
                var droneMarker = L.marker([22.9048880, 120.2719823], {{
                    icon: droneIcon,
                    rotationAngle: 0,
                    rotationOrigin: "center center"
                }}).addTo(map);
                droneMarker.bindPopup("無人機位置");
                // 初始化多台無人機
                for (var i = 0; i < droneCount; i++) {{
                    var lat = 22.9048880 + i*0.0001;  // 位置偏移避免疊在一起
                    var lng = 120.2719823;
                    droneMarkers[i] = L.marker([lat, lng], {{
                        icon: droneIcon,
                        rotationAngle: 0,
                        rotationOrigin: 'center center'
                    }}).addTo(map);
                    droneMarkers[i].bindPopup("Drone " + (i+1));
                }}

                function addDroneMarker(id, lat, lng) {{
                    var m = L.marker([lat, lng], {{icon: droneIcon}}).addTo(map);
                    m.bindPopup("Drone " + id);
                    droneMarkers[id] = m;
                }}
                
                function updateDroneMarker(id, lat, lng) {{
                    if (!window.droneMarkers) window.droneMarkers = {{}};
                    if (!window.droneMarkers[id]) {{
                        window.droneMarkers[id] = L.marker([lat, lng], {{icon: droneIcon}}).addTo(map);
                    }} else {{
                        window.droneMarkers[id].setLatLng([lat, lng]);
                    }}
                }}
                function addMarker(lat, lng) {{
                    var n = markers.length + 1;
                    var m = L.marker([lat, lng]).addTo(map);
                    m.bindPopup("第 " + n + " 航點<br>Lat: " + lat.toFixed(6) + "<br>Lng: " + lng.toFixed(6));
                    markers.push(m);

                    if(markers.length > 1){{
                        var prev = markers[markers.length - 2];
                        var line = L.polyline([prev.getLatLng(), m.getLatLng()], {{color: 'red'}}).addTo(map);
                        polylines.push(line);
                    }}
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

                function showPopup() {{
                    document.getElementById("popup").style.display = "block";
                }}
                function closePopup() {{
                    document.getElementById("popup").style.display = "none";
                }}

                // 飛向第1航點
                function flyToFirstWaypoint() {{
                    if(markers.length === 0) {{
                        showPopup();
                        return;
                    }}
                    flyToTarget(markers[0].getLatLng());
                }}

                // 通用飛行函數
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

                // 依序飛到所有航點
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

                // 右鍵點擊新增航點
                new QWebChannel(qt.webChannelTransport, function(channel) {{
                    window.qtbridge = channel.objects.qtbridge;
                }});

                map.on('contextmenu', function(e) {{
                    addMarker(e.latlng.lat, e.latlng.lng);
                    if(window.qtbridge) {{
                        window.qtbridge.waypointAdded(e.latlng.lat, e.latlng.lng);
                    }}
                }});

                function updateAllDrones(states_json) {{
                    try {{
                        var states = JSON.parse(states_json);
                        for (var id in states) {{
                            var s = states[id];
                            var lat = s.lat;
                            var lon = s.lon;
                            var yaw = s.yaw || 0;

                            // 若地圖上沒這台無人機，就新增一個 marker
                            if (!droneMarkers[id]) {{
                                droneMarkers[id] = L.marker([lat, lon], {{
                                    icon: droneIcon,
                                    rotationAngle: yaw,
                                    rotationOrigin: "center center"
                                }}).addTo(map);
                                droneMarkers[id].bindPopup("Drone " + id);
                            }} else {{
                                // 更新位置與角度
                                droneMarkers[id].setLatLng([lat, lon]);
                                droneMarkers[id].setRotationAngle(yaw);
                            }}
                        }}
                    }} catch (e) {{
                        console.error("⚠️ 更新無人機圖標失敗:", e);
                    }}
                }}                                 
            </script>
        </body>
        </html>
        """
        self.webview.setHtml(html, baseUrl=QUrl.fromLocalFile(os.path.dirname(drone_path) + "/"))

    def run_js(self, js_code, callback=None):
        if callback:
            self.webview.page().runJavaScript(js_code, callback)
        else:
            self.webview.page().runJavaScript(js_code)

    def _on_clear_markers(self):
        self.model.clear_markers()
        self.run_js("clearMarkers();")

    def _on_fly_to_wp1(self):
        self.run_js("flyToFirstWaypoint();")

    def _on_fly_sequential(self):
        self.run_js("flySequentialWaypoints();")
