from operator import index
from dronekit import LocationGlobalRelative
from geopy import distance
from geopy.distance import geodesic
import math
import numpy as np
import os
from pymavlink import mavutil  # Import mavutil to use MAV_FRAME and MAV_CMD constants
import time
from model import formation_setting
import matplotlib.pyplot as plt

def calculate_distance_lla(pos1:LocationGlobalRelative, pos2:LocationGlobalRelative) ->float:
    # Calculate the distance between two points in meters
    return distance.distance((pos1.lat, pos1.lon), (pos2.lat, pos2.lon)).m

def calculate_desired_positions_global(formation_center:LocationGlobalRelative, offsets:np.array) -> list:
    """
    formation_center is a LocationGlobalRelative(lat, lon , 0) 
    return a list of LocationGlobalRelative(lat, lon , 0)
    """
    desired_positions = []
    for offset in offsets:
        # Calculate the new latitude and longitude based on the offset
        new_position = geodesic(meters=offset[0]).destination((formation_center.lat, formation_center.lon), 0)  # Offset in the north direction
        new_lat = new_position.latitude
        new_position = geodesic(meters=offset[1]).destination((new_lat, formation_center.lon), 90)  # Offset in the east direction
        new_lon = new_position.longitude
        desired_positions.append(LocationGlobalRelative(new_lat, new_lon, 0))
    return desired_positions

def interpolate_waypoints(waypoints: list, num_points: int)->list: 
    '''
    Interpolate between the waypoints given in the list to generate a new waypoint_list.
    waypoints: List of waypoints(LocationGlobalRelative(lat, lon , 0)) to interpolate between.
    num_points: Number of points to interpolate between two consecutive waypoints.
    '''
    interpolated_waypoints = []
    for i in range(len(waypoints) - 1):
        lat_diff = (waypoints[i + 1].lat - waypoints[i].lat) / num_points
        lon_diff = (waypoints[i + 1].lon - waypoints[i].lon) / num_points
        for j in range(num_points):
            interpolated_waypoints.append(LocationGlobalRelative(waypoints[i].lat + j * lat_diff, waypoints[i].lon + j * lon_diff, 0))
   
    interpolated_waypoints.append(waypoints[-1])
    return interpolated_waypoints

def calculate_yaw_angle(current_pos: LocationGlobalRelative, center_pos: LocationGlobalRelative) -> float:
    delta_lat = center_pos.lat - current_pos.lat
    delta_lon = center_pos.lon - current_pos.lon
    yaw_rad = math.atan2(delta_lon, delta_lat)  # 注意經度放在前面
    yaw_deg = math.degrees(yaw_rad)
    return (yaw_deg + 360) % 360  # 確保角度範圍是 [0, 360)

def get_bearing(loc1: LocationGlobalRelative, loc2: LocationGlobalRelative) -> float:
    """
    計算從 loc1 到 loc2 的偏航角 (bearing)。
    Args:
        loc1 (LocationGlobalRelative): 起始位置
        loc2 (LocationGlobalRelative): 結束位置
    Returns:
        float: 偏航角 (0-360 度，北為 0，順時針增加)
    """
    lat1 = math.radians(loc1.lat)
    lon1 = math.radians(loc1.lon)
    lat2 = math.radians(loc2.lat)
    lon2 = math.radians(loc2.lon)

    delta_lon = lon2 - lon1

    x = math.sin(delta_lon) * math.cos(lat2)
    y = math.cos(lat1) * math.sin(lat2) - (math.sin(lat1) * math.cos(lat2) * math.cos(delta_lon))

    bearing_rad = math.atan2(x, y)
    bearing_deg = math.degrees(bearing_rad)
    
    # 確保角度在 0 到 360 之間
    bearing_deg = (bearing_deg + 360) % 360
    
    return bearing_deg

# 從 .waypoints 檔案讀取航點
def load_waypoints_from_file(filepath: str) -> list[LocationGlobalRelative]:
    """
    從 Mission Planner .waypoints 檔案讀取航點座標。
    檔案格式預期為：QGC WPL 110 等開頭，然後每行包含航點數據。
    主要提取緯度、經度和高度。
    """
    waypoints = []
    # 檢查檔案是否存在
    if not os.path.exists(filepath):
        print(f"錯誤：航點檔案 '{filepath}' 不存在。請檢查路徑或確保檔案在程式碼相同目錄下。")
        return [] 

    try:
        with open(filepath, 'r') as f:
            for i, line in enumerate(f):
                if i == 0: # 跳過檔案頭 (QGC WPL 110)
                    if not line.strip().startswith("QGC WPL 110"):
                        print("警告：這可能不是一個標準的 Mission Planner .waypoints 檔案。")
                    continue # 繼續處理下一行
                
                parts = line.strip().split('\t') # 假設是 Tab 分隔，如果不是請改為 ' '
                
                # 至少要有 11 個欄位 (索引 0-10) 來獲取 Lat, Lon, Alt
                if len(parts) < 11:
                    print(f"警告：航點檔案行 {i+1} 格式不正確，跳過：{line.strip()}")
                    continue

                try:
                    # 預期格式為: Index Current WP Frame Command Param1 Param2 Param3 Param4 X Y Z Autocontinue
                    # 我們需要 X (緯度), Y (經度), Z (高度)
                    index = int(parts[0])
                    command_id = int(parts[3])
                    # 忽略 HOME (Index 0) 與 TAKEOFF 航點
                    if index == 0 or command_id == mavutil.mavlink.MAV_CMD_NAV_TAKEOFF:
                        continue
                                        
                    lat = float(parts[8])
                    lon = float(parts[9])
                    alt = float(parts[10])

                    waypoints.append(LocationGlobalRelative(lat, lon, alt))

                except (ValueError, IndexError) as e:
                    print(f"解析航點檔案行 {i+1} 時的數據轉換錯誤：{line.strip()} - {e}")
                    continue
    except IOError as e:
        print(f"無法打開或讀取檔案 '{filepath}': {e}")
        return []

    print(f"成功從 '{filepath}' 載入 {len(waypoints)} 個航點。")
    return waypoints

def get_location_metres(original_location:LocationGlobalRelative, dNorth, dEast, dDown=0):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth`, `dEast`
    of the `original_location`. The `dDown` is optional and defaults to 0.
    This method is accurate for short distances (up to a few hundred meters).
    """
    earth_radius = 6378137.0 # WGS84 earth radius in metres

    # Coordinate offsets in radians
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.radians(original_location.lat)))

    # New position in decimal degrees
    newlat = original_location.lat + math.degrees(dLat)
    newlon = original_location.lon + math.degrees(dLon)
    
    newalt = original_location.alt - dDown 

    return LocationGlobalRelative(newlat, newlon, newalt)

def save_all_drone_missions():
    """
    儲存所有無人機的航線規劃，每個非起始虛擬航點i要加入2個實際航點， 
    第1個其waypoint_bearing為第i-1個虛擬航點到第i個虛擬航點的方向。
    第2個其waypoint_bearing為第i個虛擬航點到第i+1個虛擬航點的方向。
    """
    # 創立儲存所有無人機的航線規劃，字典的key對應無人機編號，內容為虛擬航點的列表
    all_drone_missions = {i: [] for i in range(1, formation_setting.formation_params["num_drones"] + 1)}

    # 將waypoint檔案讀到的航點存入virtual_center_waypoints_objs列表
    virtual_center_waypoints_objs = load_waypoints_from_file(formation_setting.waypoint_file)
    if not virtual_center_waypoints_objs:
        print("警告：未能讀取到任何虛擬航點")
        return

    # 計算每個虛擬航點到下一航點的航向
    waypoint_bearings = []
    for i in range(len(virtual_center_waypoints_objs)):
        current_wp = virtual_center_waypoints_objs[i]
        next_wp = virtual_center_waypoints_objs[(i + 1) % len(virtual_center_waypoints_objs)]
        
        if len(virtual_center_waypoints_objs) == 1:
            waypoint_bearings.append(None)  # 自動偏航，如果只有一個航點
        else:
            bearing = get_bearing(current_wp, next_wp)
            waypoint_bearings.append(bearing)

    # 遍歷虛擬中心航線的每個航點
    for i, virtual_waypoint in enumerate(virtual_center_waypoints_objs):#virtual_center_waypoints_objs 由waypoint 檔案中載入的航點
        # 獲取這個航點應該設定的偏航角 (隊形的目標方向)
        # 對於非起始的虛擬航點（即第2到第n個航點），加入一個實際航點
        if i > 0 :  # 非起始航點
            #i-1虛擬航點的bearing，由虛擬航點i-1指向虛擬航點i
            target_yaw_degrees = waypoint_bearings[i - 1]
            # 計算實際航點，偏移會根據前後航點的方向來計算
            target_yaw_radians = math.radians(target_yaw_degrees)

            # 對於每台無人機，計算該點的偏移
            for drone_id in range(1, formation_setting.formation_params["num_drones"] + 1):
                dx_body, dy_body, dz = formation_setting.drone_offsets_body_frame[drone_id] #
                
                # 根據偏航角旋轉隊形局部偏移量
                dNorth_rotated = dx_body * math.cos(target_yaw_radians) - dy_body * math.sin(target_yaw_radians)
                dEast_rotated = dx_body * math.sin(target_yaw_radians) + dy_body * math.cos(target_yaw_radians)

                # 計算實際航點位置
                # actual_waypoint_location: LocationGlobalRelative(newlat, newlon, newalt)
                actual_waypoint_location = get_location_metres(
                    virtual_waypoint, 
                    dNorth_rotated,  
                    dEast_rotated,   
                    dz               
                ) 
                
                all_drone_missions[drone_id].append(actual_waypoint_location)

        target_yaw_degrees = waypoint_bearings[i]
        target_yaw_radians = math.radians(target_yaw_degrees) if target_yaw_degrees is not None else None

        # 對於每台無人機
        for drone_id in range(1, formation_setting.formation_params["num_drones"] + 1):
            dx_body, dy_body, dz = formation_setting.drone_offsets_body_frame[drone_id]

            # 根據目標偏航角旋轉隊形局部偏移量，得到地球座標系下的偏移量 (dEast, dNorth)
            if target_yaw_radians is not None:
                dNorth_rotated = dx_body * math.cos(target_yaw_radians) - dy_body * math.sin(target_yaw_radians)
                dEast_rotated = dx_body * math.sin(target_yaw_radians) + dy_body * math.cos(target_yaw_radians)
            else:
                dEast_rotated = dx_body
                dNorth_rotated = dy_body

            # 計算實際航點位置
            actual_waypoint_location = get_location_metres(
                virtual_waypoint, 
                dNorth_rotated,  # dNorth 偏移量
                dEast_rotated,   # dEast 偏移量
                dz               # dDown 偏移量
            )
           
            all_drone_missions[drone_id].append(actual_waypoint_location)
 
    return all_drone_missions

def plot_mission_waypoints(all_drone_missions):
    """
    繪製所有無人機的實際航點軌跡
    """
    plt.figure(figsize=(8, 6))
    for drone_id, wp_list in all_drone_missions.items():
        lats = [wp.lat for wp in wp_list]
        lons = [wp.lon for wp in wp_list]
        plt.plot(lons, lats, marker='o', label=f"Drone {drone_id}")
    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    plt.title("Drone Formation Missions")
    plt.legend()
    plt.grid()
    plt.axis("equal")
    plt.show()

def transpose_to_location_relative(
    all_drone_waypoints: dict[int, list[LocationGlobalRelative]]) -> dict[int, list[LocationGlobalRelative]]:
    max_len = max(len(wps) for wps in all_drone_waypoints.values())
    transposed = {i: [] for i in range(1,max_len+1)}

    for drone_id, waypoints in all_drone_waypoints.items(): #drone_id 從1開始
        for i, loc in enumerate(waypoints): # i從0開始
            transposed[i+1].append(loc)  #各台無人機航點從1開始
    
    return transposed
# 示例用法
if __name__ == "__main__":
    all_drone_missions = save_all_drone_missions()
    transposed_all_drone_missions=transpose_to_location_relative(all_drone_missions)
    #plot_mission_waypoints(all_drone_missions)
    """
    all_drone_missions 
    {
        drone_id1: [(lat1, lon1, alt1), (lat2, lon2, alt2), ...],
        drone_id2: [(lat1, lon1, alt1), (lat2, lon2, alt2), ...],
        ...
    }
    transposed_all_drone_missions
    {
        waypoint1: [drone1的第1點, drone2的第1點, ...],
        waypoint2: [drone1的第2點, drone2的第2點, ...],
        ...
    }

           
    for drone_id, waypoints in all_drone_missions.items():
        print(f"無人機 {drone_id} 的航點：")
        for wp in waypoints:
            print(f" 緯度: {wp.lat:.8f}, 經度: {wp.lon:.8f}, 高度: {wp.alt:.2f}m")
    
    for waypoint_id, waypoints in transposed_all_drone_missions.items():
        print(f"航點{waypoint_id}：")
        for wp in waypoints:
            print(f"航點:{waypoint_id} 緯度: {wp.lat:.8f}, 經度: {wp.lon:.8f}, 高度: {wp.alt:.2f}m")
    
    """
    # 輸出所有無人機的航點