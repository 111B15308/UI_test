import sys
import os

from PyQt5 import QtWidgets
from view.settings_dialog import SettingsDialog  # import 類別
from controller.drone_launcher import launch_sitl

app = QtWidgets.QApplication(sys.argv)

# 建立對話框
dialog = SettingsDialog()
dialog.show()  # 顯示對話框，讓你可以選擇


# 等待使用者按下確認
if dialog.exec_() == QtWidgets.QDialog.Accepted:
    settings = dialog.get_settings()  # 呼叫 get_settings
    print("回傳的設定:", settings)
    launch_sitl(settings["drone_count"])  # 使用選擇的無人機數量啟動 SITL

#測試wsl啟動sitl(已成功)
