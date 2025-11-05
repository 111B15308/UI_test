import json
import subprocess
import time
import os
import sys

def launch_sitl(drone_count):
    """
    根據使用者選擇的數量啟動對應的批次檔。
    例如:
        1台 -> start_sitl_1.bat
        3台 -> start_sitl_3.bat
        5台 -> start_sitl_5.bat
    """

    # 批次檔名稱對應表
    bat_map = {
        1: "start_sitl_1.bat",
        3: "start_sitl_3.bat",
        5: "start_sitl_5.bat"
    }
    
    bat_file = bat_map.get(drone_count)
    bat_file_path = os.path.join(os.path.dirname(__file__), bat_file)
    # 確認批次檔存在
    if not os.path.exists(bat_file_path):
        print(f"❌ 找不到批次檔：{bat_file}")
        return None

    print(f"🚁 啟動 {drone_count} 架 SITL（執行 {bat_file}）...")
    
    # 啟動批次檔
    proc = subprocess.Popen(["cmd.exe", "/c", bat_file_path], creationflags=subprocess.CREATE_NEW_CONSOLE)

    print("⌛ 正在啟動中，請稍候...")
    time.sleep(5)
    print(f"✅ 已成功啟動 {drone_count} 架無人機模擬器！")

    return proc
