import subprocess
import threading
import time
import os
import sys
import inspect
from drone.formation_flying import FormationFlying 
from controller.drone_launcher import launch_drones, shutdown_all


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
            self._sitl_processes, self._mavproxy_processes = launch_drones(drone_count)
            print("🧩 正在初始化 FormationFlying...")
            from drone.formation_flying import FormationFlying
            self._formation = FormationFlying(drone_configs)
            print("✅ FormationFlying 初始化完成")
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
