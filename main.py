import sys
from PyQt5.QtWidgets import QApplication
from model.model import MapModel
from view.view import MapView
from controller.controller import MapController
from view.settings_dialog import SettingsDialog  # 導入設定視窗
from controller.mission_api import mission_api
from controller.drone_launcher import launch_sitl

def main():
    app = QApplication(sys.argv)

    # --- 開啟設定視窗，先取得使用者設定 ---
    settings_dialog = SettingsDialog()
    if settings_dialog.exec_() == settings_dialog.Accepted:
        settings = settings_dialog.get_settings()
        drone_count = settings["drone_count"]
        formation = settings["formation"]
    else:
        print("❌ 使用者取消設定，程式結束。")
        sys.exit(0)

    sitl_proc = launch_sitl(drone_count)

    # === Model ===
    model = MapModel()
    model.drone_count = drone_count
    model.formation = formation

    # === View ===
    view = MapView(model, drone_count)  # 一次建立完成
    view.create_status_panels(drone_count)  # 生成左側無人機狀態面板

    # === Controller ===
    controller = MapController(model, view)

    # === 顯示主畫面 ===
    view.show()

    print("✅ 系統啟動完成，等待使用者操作 UI...")
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()