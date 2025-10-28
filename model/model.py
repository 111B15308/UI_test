from PyQt5.QtCore import QObject, pyqtSignal

class MapModel(QObject):
    # === Signals ===
    state_changed = pyqtSignal()
    emergency_stop_signal = pyqtSignal()
    rtl_signal = pyqtSignal()

    def __init__(self):
        super().__init__()
        self._center = {"lat": 25.033964, "lng": 121.564468}  # 預設中心（台北101）
        self._zoom = 14
        self._markers = []

        # 模擬無人機設定
        self.drone_count = 0
        self.formation = "Line"
        self.drone_configs = []

    # === Map 狀態 ===
    @property
    def center(self):
        return self._center

    @center.setter
    def center(self, val):
        self._center = val
        self.state_changed.emit()

    @property
    def zoom(self):
        return self._zoom

    @zoom.setter
    def zoom(self, val):
        self._zoom = val
        self.state_changed.emit()

    @property
    def markers(self):
        return list(self._markers)

    def add_marker(self, marker):
        self._markers.append(marker)
        self.state_changed.emit()

    def clear_markers(self):
        self._markers = []
        self.state_changed.emit()

    # === 無人機控制 ===
    def emergency_stop(self):
        """UI 呼叫的緊急停止 -> 轉到 mission_api"""
        print("緊急停止!")
        try:
            mission_api.emergency_stop()
        except Exception as e:
            print("emergency_stop error:", e)

    def return_to_launch(self):
        """UI 呼叫的 RTL -> 轉到 mission_api.rtl_all()"""
        print("切換成RTL模式")
        try:
            mission_api.rtl_all()
        except Exception as e:
            print("return_to_launch error:", e)