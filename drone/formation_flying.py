from dronekit import LocationGlobalRelative, LocationGlobal
from drone.drone import Drone
import numpy as np
from model import formation_setting, helpers
#from helpers import calculate_desired_positions_global, calculate_yaw_angle, interpolate_waypoints, save_all_drone_missions
import threading
import time
from geopy.distance import geodesic
from pymavlink import mavutil
from dronekit import connect


class FormationFlying(object):
    def __init__(self, drones: list[Drone]):
        """
        直接接收從 Controller 傳入的、已經連線好的 Drone 物件列表。
        drones: list of Drone objects
        """
        self.drones = {drone.id: drone for drone in drones}
        self.num_uavs = len(self.drones)
        print(f"🧩 FormationFlying 已接管 {self.num_uavs} 架已連線的無人機。")

        # 從傳入的 drone 物件中獲取設定
        self.takeoff_alt = {id: d.alt for id, d in self.drones.items()}
        self.speed = {id: d.speed for id, d in self.drones.items()}
        # RTL 高度可以基於起飛高度動態計算
        self.rtl_alt = {id: int(d.alt * 100) for id, d in self.drones.items()}
        

    def set_rtl_alt_all(self): ##設定RTL高度，依照起飛高度，也就是飛行高度
       for i, drone in self.drones.items():               
            if (drone.set_rtl_alt(self.rtl_alt[i])==True):
                print(f"set the UAV {i} RTL_ALT {(self.rtl_alt[i])/100} m successful")
    
    def set_guided_mode_all(self):
        for i, drone in self.drones.items():               
            if (drone.set_guided_mode()==True):
                print(f"set the UAV {i} GUIDED mode successful")
    
    def set_loiter_mode_all(self):
        for i, drone in self.drones.items():               
            if (drone.set_loiter_mode()==True):
                print(f"set the UAV {i} loiter mode successful")
    
    def set_brake_mode_all(self):
        for i, drone in self.drones.items():               
            if (drone.set_brake_mode()==True):
                print(f"set the UAV {i} brake mode successful")

    def initialize_formation(self, waypoints: list[LocationGlobalRelative]): # 紀錄home點、設定guided 模式、解鎖、起飛，飛到第1個航點排列隊形 
        print("Starting Mission!")
        self.home=[]
        """
        for i in range(1, self.num_uavs+1):
            home=self.drones[i].get_home_location()
            print(f"UAV {i} home location set: {home.lat}, {home.lon}, {home.alt}")
            self.home.append(home) 
        """
               
 
        while(input("\033[93m {}\033[00m" .format("Change UAVs to GUIDED mode and takeoff? y/n\n")) != "y"):
            pass

        for i in range(1,self.num_uavs+1): # change drone to GUIDED mode and arm
            self.drones[i].set_guided_and_arm()
            print(f"UAV {i} changed mode to GUIDED and armed successfully!")
            self.drones[i].takeoff(self.takeoff_alt[i])
            print(f"UAV {i} took off successfully!") 
        
        while(input("\033[93m {}\033[00m" .format("Initializing Formation ? y/n\n")) != "y"):
            pass
        print("Initializing Formation!")
        
        for i in range(1,self.num_uavs+1):
            desired_pos=waypoints[i-1]#LocationGlobalRelative(lat, lon , virtual_waypoint中的高度)
            self.drones[i].fly_to_point_non_blocking(desired_pos,self.speed)
        time.sleep(1)
        
        has_moved=[False]*self.num_uavs #檢查每台UAV是否已開始移動
        still_forming = True       
        while still_forming:
            still_forming = False
            for i in range(1,self.num_uavs+1):
                desired_pos=waypoints[i-1] #
                if not has_moved[i-1]:
                    current_vel=self.drones[i].get_ground_speed()
                    time.sleep(1)
                    if current_vel>0.5:
                        has_moved[i-1]=True
                        print(f"drone {i} has moved")
                    else:
                        self.drones[i].fly_to_point_non_blocking(desired_pos, self.speed) #再發送一次航點

                current_pos=self.drones[i].read_global_position() #drone: 編號從1開始 global_relative_frame
                distance_to_formation = geodesic((current_pos.lat, current_pos.lon), (desired_pos.lat, desired_pos.lon)).meters
                print(f"Drone {i} Distance to Formation: {distance_to_formation}")
                if distance_to_formation > formation_setting.wp_radius:
                    still_forming = True
                time.sleep(1) 
        
        print("Initial Formation Achieved! Proceeding to Waypoints")
        #time.sleep(2y)
        while(input("\033[93m {}\033[00m" .format("continue ? y/n\n")) != "y"):
            pass

    def waypoint_following(self, waypoints: list[LocationGlobalRelative], stop_flag: threading.Event = None):
        """
        讓所有無人機同時飛往各自的目標點，並等待所有無人機都到達。
        waypoints: 一個 LocationGlobalRelative 列表，索引對應無人機ID-1。
        stop_flag: 一個 threading.Event 物件，用於從外部中斷等待。
        """
        # 1. 同時向所有無人機發送指令
        for i, drone in self.drones.items():
            # waypoints 列表的索引是 0-based，而 drone.id 是 1-based
            if (i - 1) < len(waypoints):
                desired_pos = waypoints[i-1]
                drone.fly_to_point_non_blocking(desired_pos, drone.speed)
        
        # 2. 循環檢查，直到所有無人機都到達目標
        while True:
            # ✅ 在每次循環開始時檢查停止旗標
            if stop_flag and stop_flag.is_set():
                print("🛑 waypoint_following 等待被外部中斷。")
                break

            all_arrived = True
            for i, drone in self.drones.items():
                if (i - 1) < len(waypoints):
                    desired_pos = waypoints[i-1]
                    # ✅ 使用 get_state() 方法獲取位置
                    state = drone.get_state()
                    if state:
                        current_pos = LocationGlobalRelative(state['lat'], state['lon'], state['alt'])
                        distance_to_target = geodesic((current_pos.lat, current_pos.lon), (desired_pos.lat, desired_pos.lon)).meters
                        # 如果任何一台無人機距離目標點還很遠，則標記為尚未全部到達
                        if distance_to_target > formation_setting.wp_radius:
                            all_arrived = False
                            # print(f"Drone {i} 距離目標: {distance_to_target:.1f}m") # 可選：顯示除錯訊息
                    else:
                        all_arrived = False # 如果讀不到位置，也當作未到達
            
            if all_arrived:
                print("✅ 所有無人機已抵達當前航點。")
                break # 所有無人機都已到達，跳出循環

            time.sleep(0.5) # 短暫等待後再次檢查

    def deploy_and_orient_formation(self, target_positions: list[LocationGlobalRelative], target_bearing: float, stop_flag: threading.Event = None):
        """
        執行兩階段部署：先飛到位置，然後調整姿態。
        Args:
            target_positions: 初始編隊的目標位置列表。
            target_bearing: 編隊最終需要朝向的角度 (0-360)。
            stop_flag: 用於中斷任務的事件旗標。
        """
        # --- 第一階段：位置就位 ---
        print("🚁 [階段 1/2] 所有無人機飛往初始編隊點...")
        self.waypoint_following(target_positions, stop_flag)

        if stop_flag and stop_flag.is_set():
            print("🛑 部署任務在位置就位階段被中斷。")
            return

        print("✅ [階段 1/2] 所有無人機位置已就位！")
        time.sleep(1) # 短暫停頓

        # --- 第二階段：姿態就位 ---
        print(f"🧭 [階段 2/2] 所有無人機開始調整姿態，朝向 {target_bearing:.1f}°...")
        for drone in self.drones.values():
            drone.condition_yaw(target_bearing)

        # 循環檢查，直到所有無人機都朝向正確方向
        while True:
            if stop_flag and stop_flag.is_set():
                print("🛑 部署任務在姿態就位階段被中斷。")
                break

            all_oriented = True
            for drone in self.drones.values():
                current_yaw = drone.get_state().get("yaw", 0)
                # 檢查角度差是否在容許範圍內 (例如 ±5度)
                angle_diff = abs((current_yaw - target_bearing + 180) % 360 - 180)
                if angle_diff > 5.0:
                    all_oriented = False
            
            if all_oriented:
                print("✅ [階段 2/2] 所有無人機姿態已就位！編隊部署完成！")
                break
            time.sleep(0.5)

    def leader_step(self, target_pos: LocationGlobalRelative, next_bearing: float, stop_flag: threading.Event = None):
        """
        【領頭機專用】執行一個完整的飛行步驟：飛到目標點 -> 轉向。
        """
        leader_drone = self.drones.get(1)
        if not leader_drone:
            print("❌ 找不到領頭機 (Drone 1)！")
            return False

        # --- 1. 飛到目標點 ---
        print(f"🚁 Drone 1 (Leader) 飛往目標點...")
        leader_drone.fly_to_point_non_blocking(target_pos, leader_drone.speed)
        
        # 等待領頭機到達
        while True:
            if stop_flag and stop_flag.is_set(): return False
            state = leader_drone.get_state()
            if not state: 
                time.sleep(1)
                continue
            dist = geodesic((state['lat'], state['lon']), (target_pos.lat, target_pos.lon)).meters
            if dist <= formation_setting.wp_radius:
                print("✅ Drone 1 已到達目標點。")
                break
            time.sleep(0.5)

        # --- 2. 調整姿態 ---
        print(f"🧭 Drone 1 (Leader) 調整姿態朝向 {next_bearing:.1f}°...")
        leader_drone.condition_yaw(next_bearing)

        # 等待領頭機轉向完成
        while True:
            if stop_flag and stop_flag.is_set(): return False
            state = leader_drone.get_state()
            if not state: 
                time.sleep(1)
                continue
            angle_diff = abs((state.get("yaw", 0) - next_bearing + 180) % 360 - 180)
            if angle_diff <= 5.0:
                print("✅ Drone 1 姿態已對齊。")
                return True # 領頭機步驟完成
            time.sleep(0.5)

    def followers_sync(self, leader_target_pos: LocationGlobalRelative, leader_bearing: float, stop_flag: threading.Event = None):
        """
        【追隨者專用】根據領頭機的目標狀態，計算各自的目標點並飛過去。
        """
        follower_drones = {i: d for i, d in self.drones.items() if i != 1}
        if not follower_drones:
            print("沒有追隨者，跳過同步。")
            return

        # 根據領頭機的目標狀態計算所有追隨者的目標點
        target_positions = helpers.calculate_formation_positions_at_waypoint(leader_target_pos, leader_bearing)
        
        # 讓所有追隨者飛到自己的位置
        print("🚁 Followers 開始同步位置...")
        self.waypoint_following(list(target_positions.values()), stop_flag)
        print("✅ Followers 位置同步完成！")

    def verify_formation_and_orientation(self, target_positions: dict[int, LocationGlobalRelative], target_bearing: float, stop_flag: threading.Event = None):
        """
        【前置檢查】驗證並修正編隊，直到所有無人機都到達其隊形點並朝向正確。
        Args:
            target_positions: 一個字典 {drone_id: LocationGlobalRelative}，定義了每台無人機應在的位置。
            target_bearing: 所有無人機應朝向的絕對角度 (0-360)。
            stop_flag: 用於中斷任務的事件旗標。
        """
        print("🔎 [前置檢查階段 1/2] 開始驗證隊形位置...")
        while True:
            if stop_flag and stop_flag.is_set():
                print("🛑 前置檢查任務被中斷。")
                return False # 表示未完成

            all_in_position = True
            for drone_id, drone in self.drones.items():
                state = drone.get_state()
                if not state: continue

                # 檢查位置
                target_pos = target_positions.get(drone_id)
                if not target_pos: continue

                current_pos = LocationGlobalRelative(state['lat'], state['lon'], state['alt'])
                distance_to_target = geodesic((current_pos.lat, current_pos.lon), (target_pos.lat, target_pos.lon)).meters
                if distance_to_target > formation_setting.wp_radius:
                    all_in_position = False
                    drone.fly_to_point_non_blocking(target_pos, drone.speed) # 位置不對？重新發送 simple_goto
            
            if all_in_position:
                print("✅ [前置檢查階段 1/2] 所有無人機位置已就位。")
                break # 進入姿態調整階段
            
            time.sleep(1)

        print("🔎 [前置檢查階段 2/2] 開始驗證隊形姿態...")
        # 先發送一次轉向指令給所有無人機
        for drone in self.drones.values():
            drone.condition_yaw(target_bearing)

        while True:
            if stop_flag and stop_flag.is_set():
                print("🛑 前置檢查任務被中斷。")
                return False

            all_oriented = True
            for drone in self.drones.values():
                state = drone.get_state()
                if not state: continue
                angle_diff = abs((state.get("yaw", 0) - target_bearing + 180) % 360 - 180)
                if angle_diff > 5.0:
                    all_oriented = False
                    # 可以選擇在這裡重新發送指令，但通常一次就夠了
            
            if all_oriented:
                print("✅ [前置檢查階段 2/2] 所有無人機姿態已對齊！準備就緒！")
                return True

            time.sleep(1) # 等待1秒後再次檢查


    def rtl_all(self): #每台依其RTL高度返航
        for i in range(1,self.num_uavs+1):
            self.drones[i].rtl() #drones是一個dict，key 由1開始
            time.sleep(1)
   
if __name__ == "__main__":
    try:
        all_drone_missions = helpers.save_all_drone_missions() #取得航線dict, {id_1:[(lat, lon, alt),()], id_2:[(lat, lon, alt),()]...} 
        transposed_all_drone_missions=helpers.transpose_to_location_relative(all_drone_missions)
        formation_flying = FormationFlying()
        formation_flying.set_rtl_alt_all()
        #紀錄home點、設定guided 模式、解鎖、起飛，飛到第1個航點排列隊形 
        formation_flying.initialize_formation(transposed_all_drone_missions[1]) # 1是waypoint id 
        #執行任務
        for waypoint_id, waypoints in transposed_all_drone_missions.items():
            if waypoint_id==1 :
                continue # 跳過第一個航點
            if waypoint_id == max(transposed_all_drone_missions.keys()):
                continue  # 跳過最後一個航點，最後一個航點只是將隊形朝向第一個航點
            formation_flying.waypoint_following(waypoints)
            #TODO brake 後的處理
        #return to home locations
        formation_flying.rtl_all()
        print("Mission Completed!")
    except KeyboardInterrupt:
        print("\nMission interrupted by user!")
        # 在這裡可以加入任何需要在中斷時執行的清理工作
        formation_flying.set_brake_mode_all()  # brake
        while(input("\033[93m {}\033[00m" .format("Change UAVs to RTL? y/n\n")) != "y"):
            pass
        formation_flying.rtl_all()
        #print("Loitering...")
        #continue
        
    finally:
        # 這裡可以加入任何程式結束前的清理工作
        pass

    