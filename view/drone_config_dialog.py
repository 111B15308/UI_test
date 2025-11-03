from PyQt5 import QtWidgets, QtGui

class DroneConfigDialog(QtWidgets.QDialog):
    def __init__(self, index):
        super().__init__()
        self.setWindowTitle(f"第 {index} 台無人機設定")
        self.resize(300, 120)

        layout = QtWidgets.QFormLayout(self)

        # 高度 (1~30 公尺)
        self.alt_input = QtWidgets.QSpinBox()
        self.alt_input.setRange(1, 30)
        self.alt_input.setValue(15)

        # 速度 (1~10 m/s)
        self.speed_input = QtWidgets.QDoubleSpinBox()
        self.speed_input.setRange(1, 10)
        self.speed_input.setDecimals(1)
        self.speed_input.setValue(5.0)

        layout.addRow("起飛高度 (m):", self.alt_input)
        layout.addRow("飛行速度 (m/s):", self.speed_input)

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
