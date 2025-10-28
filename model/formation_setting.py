
uav_speed=5 #m/sec
#connection_port=5762
connection_port=14551

wp_radius=1.2 #meter
#waypoint_file = 'n1.waypoints' #n1.waypoints 實飛測試用
waypoint_file = 'n2.waypoints'


"""
# --- 1. 定義隊形參數 ---
#       u3
#   u2      u4
# u1            u5
# 設定隊形相關參數

formation_params = {
    "num_drones": 5,
    "lead_drone_id": 3,  # 隊形中心點對應的無人機，通常在倒V字頂點 (U3)
    "spacing_x": 5,   # 無人機在隊形橫向（左右）的間距 (米)
    "spacing_y": 5,   # 無人機在隊形縱向（前後）的間距 (米)
    "altitude": 15     # 編隊的預設飛行高度 (米) 和.waypoint 檔案內一致
}
"""
"""
# --- 1. 定義隊形參數 ---
#       u2
#   u1      
# 設定隊形相關參數
formation_params = {
    "num_drones": 2,
    "lead_drone_id": 2,  # 隊形中心點對應的無人機，通常在倒V字頂點 (U2)
    "spacing_x": 5,   # 無人機在隊形橫向（左右）的間距 (米)
    "spacing_y": 5,   # 無人機在隊形縱向（前後）的間距 (米)
    "altitude": 15     # 編隊的預設飛行高度 (米) 和.waypoint 檔案內一致
}
"""

# --- 1. 定義隊形參數 ---
#       u2
#   u1      u3
# 設定隊形相關參數

formation_params = {
    "num_drones": 3,
    "lead_drone_id": 2,  # 隊形中心點對應的無人機，通常在倒V字頂點 (U2)
    "spacing_x": 5,   # 無人機在隊形橫向（左右）的間距 (米)
    "spacing_y": 5,   # 無人機在隊形縱向（前後）的間距 (米)
    "altitude": 15     # 編隊的預設飛行高度 (米) 和.waypoint 檔案內一致
}


# 定義每台無人機相對於隊形中心點的偏移量 (dx_body, dy_body, dz) (FRD: xyz)
# 這裡的 dx_body 和 dy_body 是在隊形自身的局部座標系(FRD)中定義的。
# dx_body: 沿著隊形縱向軸的偏移 (向前為正，向後為負)
# dy_body: 沿著隊形橫向軸的偏移 (向右為正)
# dz: 方向向下
nav_alt_spacing=2 #meter
"""
drone_offsets_body_frame = {
    1: (-2 * formation_params["spacing_x"], -2 * formation_params["spacing_y"], 2*nav_alt_spacing), # U1: 最左後 高度往下2*nav_alt_spacing
    2: (-1 * formation_params["spacing_x"], -1 * formation_params["spacing_y"], 1*nav_alt_spacing), # U2: 左後
    3: (0, 0, 0*nav_alt_spacing),                                                                  # U3: 中心 (領頭機)
    4: (-1 * formation_params["spacing_x"], 1 * formation_params["spacing_y"], -1*nav_alt_spacing),  # U4: 右後
    5: (-2 * formation_params["spacing_x"], 2 * formation_params["spacing_y"], -2*nav_alt_spacing)   # U5: 最右後
}
"""
"""
drone_offsets_body_frame = {
    1: (-1 * formation_params["spacing_x"], -1 * formation_params["spacing_y"], 1*nav_alt_spacing), # U1: 左後
    2: (0, 0, 0*nav_alt_spacing)                                                              # U2: 中心 (領頭機)
}
"""

drone_offsets_body_frame = {
    1: (-1 * formation_params["spacing_x"], -1 * formation_params["spacing_y"], 1*nav_alt_spacing), # U1: 左後
    2: (0, 0, 0*nav_alt_spacing),                                                                  # U2: 中心 (領頭機)
    3: (-1 * formation_params["spacing_x"], 1 * formation_params["spacing_y"], -1*nav_alt_spacing)  # U3: 右後
}


#takeoff_altitude=20 # meter
#takeoff_alt_diff=2 #meter
"""
takeoff_alt={
    1:formation_params["altitude"]+(-1)*2*nav_alt_spacing,
    2:formation_params["altitude"]+(-1)*1*nav_alt_spacing,
    3:formation_params["altitude"],
    4:formation_params["altitude"]+(1)*1*nav_alt_spacing,
    5:formation_params["altitude"]+(1)*2*nav_alt_spacing,
}
"""
"""
takeoff_alt={
    1:formation_params["altitude"]+(-1)*1*nav_alt_spacing,
    2:formation_params["altitude"]
}
"""
takeoff_alt={
    1:formation_params["altitude"]+(-1)*1*nav_alt_spacing,
    2:formation_params["altitude"],
    3:formation_params["altitude"]+(1)*1*nav_alt_spacing
}


rtl_alt= {key: value *100 for key, value in takeoff_alt.items()} #rtl_alt(cm), takeoff_alt(m)
rtl_speed=500 #cm/s

"""
#rtl_alt_apacing=3 #meter
rtl_alt={
    1:formation_params["altitude"]+2*rtl_alt_apacing,
    2:formation_params["altitude"]+1*rtl_alt_apacing,
    3:formation_params["altitude"],
    4:formation_params["altitude"]+3*rtl_alt_apacing,
    5:formation_params["altitude"]+4*rtl_alt_apacing,
}
"""




