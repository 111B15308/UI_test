import subprocess
import threading
import time
import os
import sys
import inspect
from drone.formation_flying import FormationFlying 
from drone.drone import Drone 
from controller.drone_launcher import launch_sitl


class MissionAPI:
    def __init__(self):
        self._formation = None
        self._sitl_processes = []
        self._mavproxy_processes = []
    # -------------------------------
    # 初始化群飛
    # -------------------------------
    def initialize_formation(self, drone_configs):
        """初始化群飛 FormationFlying"""
        try:
            drone_count = len(drone_configs)
            self._sitl_processes = launch_sitl(drone_count)
            print("🧩 正在初始化 FormationFlying...")
            from drone.formation_flying import FormationFlying
            self._formation = FormationFlying(drone_configs)
            print("✅ FormationFlying 初始化完成")
            def print_positions(states):
                for i, s in states.items():
                    print(f"🛰️ Drone {i}: lat={s['lat']:.6f}, lon={s['lon']:.6f}, "
                        f"alt={s['alt']:.1f}, mode={s['mode']}")
            self.start_position_watcher(print_positions)
        except Exception as e:
            print("❌ FormationFlying 初始化失敗:", e)


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
        if not self.drones:
            print("⚠️ 尚未連線任何無人機")
            return
        print("🚁 緊急停止所有無人機 (原地懸停)")
        for drone in self.drones:
            drone.hold_position()

    def return_to_launch(self):
        """所有無人機返航"""
        if not self.drones:
            print("⚠️ 尚未連線任何無人機")
            return
        print("🔙 所有無人機返航中 (切換至 RTL 模式)...")
        for drone in self.drones:
            drone.rtl()


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
