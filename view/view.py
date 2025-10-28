import os
from PyQt5.QtCore import Qt, QUrl, QObject, pyqtSignal
from PyQt5.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QPushButton
)
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWebChannel import QWebChannel
from PyQt5.QtWidgets import QComboBox


class Bridge(QObject):
    waypointAdded = pyqtSignal(float, float)  # 傳航點座標到 Python


class MapView(QMainWindow):
    def __init__(self, model):
        super().__init__()
        self.model = model
        self.setWindowTitle("Simulator Map (MVC)")
        self.setGeometry(100, 100, 1280, 800)
        
        # 當 model 狀態改變時，更新地圖
        self.model.state_changed.connect(self.update_formation)

        central = QWidget()
        self.setCentralWidget(central)
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        central.setLayout(layout)

        # Leaflet map webview
        self.webview = QWebEngineView()
        layout.addWidget(self.webview)

        # 建立 QWebChannel 橋接
        self.channel = QWebChannel()
        self.bridge = Bridge()
        self.channel.registerObject("qtbridge", self.bridge)
        self.webview.page().setWebChannel(self.channel)

        # overlay control bar
        self._create_overlay_controls()
        self._load_map_html()

        # 綁定按鈕
        self.clear_btn.clicked.connect(self._on_clear_markers)
        self.fly_btn.clicked.connect(self._on_fly_to_wp1)
        self.seq_btn.clicked.connect(self._on_fly_sequential)
        
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
    
        v.addWidget(clear_btn)
        v.addWidget(fly_btn)
        v.addWidget(seq_btn)
    
        # --- 新增兩個按鈕 ---
        stop_btn = QPushButton("緊急停止")
        stop_btn.setFixedSize(150, 40)
        rtl_btn = QPushButton("返回Home")
        rtl_btn.setFixedSize(150, 40)
    
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


    def resizeEvent(self, event):
        self.top_bar.move(self.width() - 180, 20)
        super().resizeEvent(event)

    def _load_map_html(self):
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

                function addDroneMarker(id, lat, lng) {{
                    var m = L.marker([lat, lng], {{icon: droneIcon}}).addTo(map);
                    m.bindPopup("Drone " + id);
                    droneMarkers[id] = m;
                }}
                
                function updateDroneMarker(id, lat, lng) {{
                    if (droneMarkers[id]) {{
                        droneMarkers[id].setLatLng([lat, lng]);
                    }} else {{
                        addDroneMarker(id, lat, lng);
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
