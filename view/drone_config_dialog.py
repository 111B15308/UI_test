from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtWidgets import QLineEdit

class DroneConfigDialog(QtWidgets.QDialog):
    def __init__(self, index, drone_id=1):
        super().__init__()
        self.setWindowTitle(f"第 {index} 台無人機設定")
        self.drone_id = drone_id
        self.resize(300, 120)  # 300 120

        layout = QtWidgets.QFormLayout(self)

        # === 連線位置 (IP 與 Port) ===
        h_ip = QtWidgets.QHBoxLayout()
        label_ip = QtWidgets.QLabel("連線位置 (IP 與 Port):udp:")

        # 前面一格：協定/IP (長)
        self.addr_input = QtWidgets.QLineEdit("172.29.192.1")
        self.addr_input.setFixedWidth(200)  # 調寬一點比較清楚

        # 後面一格：Port (短)
        self.port_input = QtWidgets.QLineEdit(str(14550 + (drone_id - 1) * 10))
        self.port_input.setFixedWidth(80)

        # ✅ 正確加到 h_ip 內
        h_ip.addWidget(label_ip)
        h_ip.addWidget(self.addr_input)
        h_ip.addWidget(self.port_input)
        h_ip.addStretch()  

        # ✅ 加入主 layout
        layout.addRow(h_ip)

        # === 高度 (1~30 公尺) ===
        self.alt_input = QtWidgets.QSpinBox()
        self.alt_input.setRange(1, 30)
        self.alt_input.setValue(15)
        layout.addRow("起飛高度 (m):", self.alt_input)

        # === 速度 (1~10 m/s) ===
        self.speed_input = QtWidgets.QDoubleSpinBox()
        self.speed_input.setRange(1, 10)
        self.speed_input.setDecimals(1)
        self.speed_input.setValue(5.0)
        layout.addRow("飛行速度 (m/s):", self.speed_input)

        # === 確認按鈕 ===
        self.confirm_btn = QtWidgets.QPushButton("確認")
        self.confirm_btn.clicked.connect(self.accept)
        layout.addRow(self.confirm_btn)


    def validate_and_accept(self):
        """檢查輸入值是否在範圍內"""
        try:
            port = int(self.port_input.text())
        except ValueError:
            self.show_error("Port 必須是數字!")
            return

        altitude = self.alt_input.value()
        speed = self.speed_input.value()

        if not (1000 <= port <= 10000):
            self.show_error("Port 超出輸入範圍!! (1000 ~ 10000)")
            return
        if not (1 <= altitude <= 30):
            self.show_error("高度超出輸入範圍!! (1 ~ 30)")
            return
        if not (1 <= speed <= 10):
            self.show_error("速度超出輸入範圍!! (1 ~ 10)")
            return

        self.accept()

    def show_error(self, message):
        msg = QtWidgets.QMessageBox(self)
        msg.setWindowTitle("錯誤")
        msg.setText(message)
        msg.setIcon(QtWidgets.QMessageBox.Warning)
        msg.setStandardButtons(QtWidgets.QMessageBox.Ok)
        msg.exec_()
