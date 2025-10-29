import subprocess
import threading
import time
import os
import sys

from drone.formation_flying import FormationFlying


class MissionAPI:
    def __init__(self):
        self._formation = None
        self._sitl_processes = []
        self._sitl_base_port = 5760  # 預設第一台埠號

    # -------------------------------
    # 自動啟動多個 SITL 實例
    # -------------------------------
    def _launch_sitl_instances(self, drone_configs):
        """根據無人機設定啟動對應數量的 SITL 實例"""
        print(" 啟動 SITL 模擬器...")

        base_home = [22.90494, 120.27240, 27.48, 0]  # 長榮大學 圖書館前，機頭朝北

        # 嘗試找出 dronekit-sitl 執行方式（Windows 可能要用 python -m）
        use_python_module = False
        try:
            subprocess.run(["dronekit-sitl", "--version"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        except FileNotFoundError:
            use_python_module = True
            print("⚠️ 找不到 dronekit-sitl 可執行檔，改用 'python -m dronekit_sitl' 啟動")

        # 啟動 SITL 實例
        for i, cfg in enumerate(drone_configs):
            port = cfg["port"]
            instance = i
            lat = base_home[0] + i * 0.0002
            lon = base_home[1] + i * 0.0002
            home = f"--home={lat},{lon},{base_home[2]},{base_home[3]}"

            if use_python_module:
                cmd = [sys.executable, "-m", "dronekit_sitl", "copter", "--instance", str(instance), home, f"--udp:{port}"]
            else:
                cmd = ["dronekit-sitl", "copter", "--instance", str(instance), home, f"--udp:{port}"]

            print(f"啟動第 {i+1} 架 SITL: {' '.join(cmd)}")

            # 啟動子程序 (獨立執行 SITL)
            try:
                process = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                self._sitl_processes.append(process)
                time.sleep(3)  # 稍等避免同時啟動造成衝突
            except Exception as e:
                print(f"無法啟動第 {i+1} 架 SITL ({port}):", e)

        # 等待所有 SITL 啟動穩定
        print("⌛ 等待 SITL 初始化...")
        time.sleep(8)
        print("✅ 所有 SITL 模擬器已啟動完成。")

    # -------------------------------
    # 初始化群飛
    # -------------------------------
    def initialize_formation(self, drone_configs):
        """初始化群飛 FormationFlying"""
        try:
            # 啟動 SITL
            self._launch_sitl_instances(drone_configs)
            print("🧩 正在初始化 FormationFlying...")

            # 啟動 FormationFlying 並嘗試連線
            self._formation = FormationFlying(drone_configs)
            print("✅ FormationFlying 初始化完成")
        except Exception as e:
            print("FormationFlying 初始化失敗:", e)

    # -------------------------------
    # 開始任務
    # -------------------------------
    def start_mission(self):
        """開始任務"""
        if not self._formation:
            print("⚠️ FormationFlying 尚未初始化")
            return

        print("開始執行群飛任務")
        # 在背景執行任務（避免阻塞 UI）
        threading.Thread(target=self._run_mission, daemon=True).start()

    def _run_mission(self):
        """模擬任務流程"""
        try:
            print("所有無人機起飛中...")
            self._formation.set_rtl_alt_all()
            # TODO: 可在這裡根據 helpers 載入航點後執行自動導航
            time.sleep(3)
            print("✅ 群飛任務完成")
        except Exception as e:
            print("❌ 群飛任務錯誤:", e)

    # -------------------------------
    # 緊急停止 / 返航
    # -------------------------------
    def emergency_stop(self):
        """緊急停止所有無人機"""
        if not self._formation:
            print("⚠️ 尚未初始化 FormationFlying")
            return
        print(" 緊急停止所有無人機")
        self._formation.set_brake_mode_all()

    def return_to_launch(self):
        """所有無人機返航"""
        if not self._formation:
            print("⚠️ 尚未初始化 FormationFlying")
            return
        print("無人機返航中...")
        self._formation.rtl_all()

    # -------------------------------
    # 狀態監控（每秒回報）
    # -------------------------------
    def start_position_watcher(self, callback):
        """每秒更新一次無人機狀態"""
        if not self._formation:
            print("⚠️ 尚未初始化 FormationFlying")
            return

        def _watch():
            while True:
                try:
                    states = {}
                    for i, drone in self._formation.drones.items():
                        states[i] = drone.get_state()
                    callback(states)
                    time.sleep(1)
                except Exception as e:
                    print("❌ 位置更新錯誤:", e)
                    break

        threading.Thread(target=_watch, daemon=True).start()

    # -------------------------------
    # 關閉 SITL
    # -------------------------------
    def shutdown_sitl(self):
        """關閉所有 SITL 子程序"""
        for p in self._sitl_processes:
            try:
                p.terminate()
            except Exception:
                pass
        self._sitl_processes.clear()
        print("已關閉所有 SITL 模擬器。")


# 建立全域實例供其他 controller 使用
mission_api = MissionAPI()
