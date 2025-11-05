from dronekit import connect, VehicleMode
from pymavlink import mavutil, mavwp
from dronekit import LocationGlobalRelative
import time

""" """ # 連線到 SITL
print("Connecting to vehicle...")
vehicle = connect('udp:172.29.192.1:14550', wait_ready=True, timeout=100)

# 1️⃣ 切換 GUIDED 模式
vehicle.mode = VehicleMode("GUIDED")
while vehicle.mode.name != "GUIDED":
    print("等待切換到 GUIDED 模式...")
    time.sleep(1)
print("已切換到 GUIDED 模式")

# 2️⃣ 解鎖無人機
vehicle.armed = True
while not vehicle.armed:
    print("等待無人機解鎖...")
    vehicle.armed = True
    time.sleep(1)
print("無人機已解鎖")

# 3️⃣ 起飛到目標高度
target_alt = 10  # 起飛到 10 米
print(f"起飛至 {target_alt} m...")
vehicle.simple_takeoff(target_alt)

# 等待達到高度
while True:
    current_alt = vehicle.location.global_relative_frame.alt
    print(f"目前高度: {current_alt:.1f} m")
    if current_alt >= target_alt * 0.95:
        print("已達到目標高度")
        break
    time.sleep(1)

# 4️⃣ 設定目標 GPS 座標
target_location = LocationGlobalRelative(22.9049399147239,120.272397994995,27.48)  # 長榮大學)
vehicle.simple_goto(target_location)
vehicle.airspeed = 5  # 設定飛行速度 5 m/s
print("飛向目標座標...")
while True:
        # 取得目前座標
        location = vehicle.location.global_frame  # GPS 原始座標 (lat, lon, alt)
        print(f"Latitude: {location.lat}, Longitude: {location.lon}, Altitude: {location.alt:.1f} m")
        time.sleep(5) 

