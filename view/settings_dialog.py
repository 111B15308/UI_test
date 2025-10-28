# view/settings_dialog.py
from PyQt5 import QtWidgets, QtCore

class SettingsDialog(QtWidgets.QDialog):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("無人機隊型設定")
        self.resize(300, 100)

        layout = QtWidgets.QVBoxLayout(self)

        # --- 無人機數量 ---
        h1 = QtWidgets.QHBoxLayout()
        label_drone_count = QtWidgets.QLabel("無人機數量:")
        self.combo_drone_count = QtWidgets.QComboBox()
        self.combo_drone_count.addItems(["1台", "3 台", "5 台"])
        h1.addWidget(label_drone_count)
        h1.addWidget(self.combo_drone_count)
        layout.addLayout(h1)

        # --- 隊形 ---
        h2 = QtWidgets.QHBoxLayout()
        label_formation = QtWidgets.QLabel("群飛隊形:")
        self.combo_formation = QtWidgets.QComboBox()
        self.combo_formation.addItems(["Line", "Wedge", "Square"])
        h2.addWidget(label_formation)
        h2.addWidget(self.combo_formation)
        layout.addLayout(h2)

        # --- 確認按鈕 ---
        btn = QtWidgets.QPushButton("確認")
        btn.clicked.connect(self.accept)  # 按下去就會結束 dialog 並回傳 Accepted
        layout.addWidget(btn, alignment=QtCore.Qt.AlignCenter)

    def get_settings(self):
        """回傳使用者選擇的設定"""
        count_map = {0: 1, 1: 3, 2: 5}  # 對應 ComboBox 的索引
        return {
            "drone_count": count_map.get(self.combo_drone_count.currentIndex(), 1),
            "formation": self.combo_formation.currentText()
        }
