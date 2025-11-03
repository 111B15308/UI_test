import json
import subprocess
import time
import os


def launch_drones(drone_count, config_path="configs/drones.json"):
    """
    根據使用者設定的無人機數量啟動對應的 SITL + MAVProxy。
    drone_count: 使用者選擇的數量 (1, 3, 5)
    config_path: drones.json 檔案路徑
    """
    sitl_processes = []
    mavproxy_processes = []

    # 讀取 drones.json
    with open(config_path, "r", encoding="utf-8") as f:
        drones = json.load(f)

    # 取前 N 架無人機
    selected_drones = drones[:drone_count]
    print(f"🛫 將啟動 {drone_count} 架模擬無人機...")

    for i, d in enumerate(selected_drones):
        home = ",".join(map(str, d["home"]))
        sitl_port = d["sitl_port"]
        out_port = d["out_port"]
        instance = d["id"]

        # === 啟動 SITL ===
        sitl_cmd = [
            "dronekit-sitl", "copter",
            "--instance", str(instance),
            "--home", home,
            "--model", d["model"],
            "--tcp:127.0.0.1:{}".format(sitl_port)
        ]
        print(f"🚁 啟動第 {i+1} 架 SITL：{sitl_cmd}")
        sitl_proc = subprocess.Popen(sitl_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        sitl_processes.append(sitl_proc)
        time.sleep(3)

        # === 啟動 MAVProxy ===
        mavproxy_cmd = [
            "python", "-m", "MAVProxy.mavproxy",
            "--master", f"tcp:127.0.0.1:{sitl_port}",
            "--out", f"udp:127.0.0.1:{out_port}",
            "--cmd", "set shownoise 0"
        ]
        print(f"🔗 啟動第 {i+1} 架 MAVProxy：{mavproxy_cmd}")
        mavproxy_proc = subprocess.Popen(mavproxy_cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        mavproxy_processes.append(mavproxy_proc)
        time.sleep(2)

    print("⌛ 等待 SITL & MAVProxy 穩定啟動中...")
    time.sleep(5)
    print(f"✅ 已成功啟動 {drone_count} 架無人機模擬器！")

    return sitl_processes, mavproxy_processes


def shutdown_all(processes):
    """關閉所有子程序"""
    for p in processes:
        p.terminate()
    print("🧹 已關閉所有模擬程序。")
