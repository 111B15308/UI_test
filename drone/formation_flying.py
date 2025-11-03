from dronekit import LocationGlobalRelative, LocationGlobal
from drone.drone import Drone
import numpy as np
from model import formation_setting, helpers
#from helpers import calculate_desired_positions_global, calculate_yaw_angle, interpolate_waypoints, save_all_drone_missions
import time
from geopy.distance import geodesic


class FormationFlying(object):
    def __init__(self, drone_configs):
        """
        drone_configs: 使用者設定，例如：
        [
            {"alt": 15, "speed": 5.0},
            {"alt": 17, "speed": 5.0},
            {"alt": 13, "speed": 5.0},
        ]
        """

        self.num_uavs = len(drone_configs)
        self.drones = {}
        self.base_port = 14550  # 第一台 SITL 的主 TCP port
        print(f"🧩 初始化 FormationFlying，共 {self.num_uavs} 架無人機")

        # ==============================
        # ✅ 每台 DroneKit 自動連接對應埠
        # ==============================
        for i, cfg in enumerate(drone_configs, start=1):
            # 每個 SITL instance 會自動啟在 5760, 5770, 5780... (由 drone_launcher 控制)
            port = self.base_port + (i - 1) * 10
            conn_str = f"tcp:127.0.0.1:{port}"
            print(f"🔗 嘗試連線第 {i} 架無人機 ({conn_str}) ...")
            try:
                self.drones[i] = Drone(conn_str)
                print(f"✅ 第 {i} 架無人機已連線成功")
            except Exception as e:
                print(f"❌ 第 {i} 架無人機連線失敗 ({port}): {e}")

        print(f"📡 成功連線 {len(self.drones)} 架無人機")

        # ==============================
        # ✅ 將高度、速度、RTL 高度整合
        # ==============================
        self.takeoff_alt = {i: cfg["alt"] for i, cfg in enumerate(drone_configs, start=1)}
        self.speed = {i: cfg["speed"] for i, cfg in enumerate(drone_configs, start=1)}

        if hasattr(formation_setting, "rtl_alt"):
            self.rtl_alt = formation_setting.rtl_alt
        else:
            self.rtl_alt = {i: int(self.takeoff_alt[i] * 100 * 0.8) for i in range(1, self.num_uavs + 1)}

        print("🪶 起飛高度:", self.takeoff_alt)
        print("⚡ 飛行速度:", self.speed)
        print("🔁 RTL 高度:", self.rtl_alt)

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

    def waypoint_following(self, waypoints: list[LocationGlobalRelative]):
        for i in range(1,self.num_uavs+1):
            desired_pos=waypoints[i-1] #LocationGlobalRelative(lat, lon , alt)
            self.drones[i].fly_to_point_non_blocking(desired_pos,self.speed) #drones是一個dict，key 由1開始
        time.sleep(1)

        has_moved=[False]*self.num_uavs #檢查每台UAV是否已開始移動
        still_forming = True       
        while still_forming:
            still_forming = False
            for i in range(1,self.num_uavs+1):
                desired_pos=waypoints[i-1] 

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
        print("Formation Achieved! Proceeding to Waypoints")
        time.sleep(2)
        #while(input("\033[93m {}\033[00m" .format("continue ? y/n\n")) != "y"):
        #y    pass

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

    