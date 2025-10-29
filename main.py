# main.py
import sys
from PyQt5.QtWidgets import QApplication
from model.model import MapModel
from view.view import MapView
from controller.controller import MapController
#from controller.settings_controller import SettingsController
from controller.mission_api import mission_api


def main():
    """
    專案主程式入口：
    - 初始化 MVC 架構
    - 啟動無人機設定視窗
    - 顯示地圖主畫面
    """
    app = QApplication(sys.argv)

    # === Model ===
    model = MapModel()

    # === View ===
    view = MapView(model)

    # === Controller ===
    controller = MapController(model, view)
    # === 開啟設定視窗（選擇無人機數量與隊形） ===
    #settings_ctrl = SettingsController(model)
    #settings_ctrl.view.show()

    # === 顯示主畫面 ===
    view.show()

    print("✅ 系統啟動完成，等待使用者操作 UI...")
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
