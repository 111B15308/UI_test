
from PyQt5 import QtWidgets, QtCore

class SettingsView(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Drone Formation Control")
        self.resize(400, 180)

        central = QtWidgets.QWidget(self)
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)

        # 無人機數量
        h1 = QtWidgets.QHBoxLayout()
        self.label_drone_count = QtWidgets.QLabel("無人機數量:")
        self.combo_drone_count = QtWidgets.QComboBox()
        self.combo_drone_count.addItems(["3 台", "5 台"])
        h1.addWidget(self.label_drone_count)
        h1.addWidget(self.combo_drone_count)
        layout.addLayout(h1)

        # 隊形
        h2 = QtWidgets.QHBoxLayout()
        self.label_formation = QtWidgets.QLabel("群飛隊形:")
        self.combo_formation = QtWidgets.QComboBox()
        self.combo_formation.addItems(["Line", "Wedge", "Square"])
        h2.addWidget(self.label_formation)
        h2.addWidget(self.combo_formation)
        layout.addLayout(h2)
 
        # 確認按鈕
        self.confirm_btn = QtWidgets.QPushButton("確認")
        self.confirm_btn.setFixedHeight(35)
        layout.addWidget(self.confirm_btn, alignment=QtCore.Qt.AlignCenter)
