uav_speed=10 #m/sec
#connection_port=5762
connection_port=14551
 
wp_radius=1.2 #meter
#waypoint_file = 'n1.waypoints' #n1.waypoints 實飛測試用
waypoint_file = 'n2.waypoints'
 
# --- 1. 定義隊形參數 ---
#       u2
#   u1      u3
# 設定隊形相關參數
# 預設為 3機 V 形 (Wedge)
formation_params = {
    "num_drones": 3,
    "lead_drone_id": 2,  # 隊形中心點對應的無人機，通常在倒V字頂點 (U2)
    "spacing_x": 15,   # 無人機在隊形橫向（左右）的間距 (米)
    "spacing_y": 15,   # 無人機在隊形縱向（前後）的間距 (米)
    "altitude": 15     # 編隊的預設飛行高度 (米) 和.waypoint 檔案內一致
}
 
# 定義每台無人機相對於隊形中心點的偏移量 (dx_body, dy_body, dz) (FRD: xyz)
# 這裡的 dx_body 和 dy_body 是在隊形自身的局部座標系(FRD)中定義的。
# dx_body: 沿著隊形縱向軸的偏移 (向前為正，向後為負)
# dy_body: 沿著隊形橫向軸的偏移 (向右為正)
# dz: 方向向下
nav_alt_spacing=2 #meter
# 預設為 3機 V 形 (Wedge)
drone_offsets_body_frame = {
    1: (-1 * formation_params["spacing_x"], -1 * formation_params["spacing_y"], 1*nav_alt_spacing), # U1: 左後
    2: (0, 0, 0*nav_alt_spacing),                                                                  # U2: 中心 (領頭機)
    3: (-1 * formation_params["spacing_x"], 1 * formation_params["spacing_y"], -1*nav_alt_spacing)  # U3: 右後
}
# 預設為 3機 V 形 (Wedge)
takeoff_alt={
    1:formation_params["altitude"]+(-1)*1*nav_alt_spacing,
    2:formation_params["altitude"],
    3:formation_params["altitude"]+(1)*1*nav_alt_spacing
}
 
 
rtl_alt= {key: value *100 for key, value in takeoff_alt.items()} #rtl_alt(cm), takeoff_alt(m)
rtl_speed=500 #cm/s
 
def set_formation(formation_name: str, num_drones: int, spacing_x: int = 15, spacing_y: int = 15, base_alt: int = 15, leader_id: int = 1):
    """
    根據隊形名稱和無人機數量，動態設定 formation_params 和 drone_offsets_body_frame
    """
    global formation_params, drone_offsets_body_frame, takeoff_alt
 
    formation_params = {
        "num_drones": num_drones,
        "lead_drone_id": leader_id,
        "spacing_x": spacing_x,
        "spacing_y": spacing_y,
        "altitude": base_alt
    }
 
    new_offsets = {}
    if formation_name.lower() == "line":
        # 一字形：以第一台為中心，其餘向後排列
        # U1 -> U2 -> U3 ...
        for i in range(1, num_drones + 1):
            new_offsets[i] = (-(i - 1) * spacing_x, 0, 0)
 
    elif formation_name.lower() == "wedge":
        # V字形
        # Drone 1 是頂點 (0,0,0)
        # Drone 2, 3 在第一排斜後方
        # Drone 4, 5 在第二排斜後方, etc.
        for i in range(1, num_drones + 1):
            if i == 1:
                new_offsets[1] = (0, 0, 0)
            else:
                # ✅ 修正：使用正確的公式計算排數
                row = i // 2 
                # 偶數ID在左(-y), 奇數ID在右(+y)
                side = -1 if i % 2 == 0 else 1
                # 計算x, y偏移
                offset_x = -row * spacing_x
                offset_y = side * row * spacing_y
                new_offsets[i] = (offset_x, offset_y, 0)
 
    elif formation_name.lower() == "square":
        # 方形 (假設為 4 或 9 架)
        side = int(num_drones**0.5)
        if side * side != num_drones:
            print(f"警告：方形隊伍的無人機數量 ({num_drones}) 不是完全平方數，隊形可能不正確。")
            return # 或者 fallback 到 line
        
        k = 1
        for i in range(side):
            for j in range(side):
                # 以左上角為 (0,0)
                new_offsets[k] = (-i * spacing_x, j * spacing_y, 0)
                k += 1
 
    else:
        print(f"警告：未知的隊形 '{formation_name}'。將使用預設隊形。")
        return
 
    # --- ✅ 將 Drone 1 (或指定的 leader_id) 設定為絕對領頭機 (偏移量歸零) ---
    if leader_id in new_offsets:
        # 1. 獲取指定領頭機的初始偏移量
        leader_initial_offset = new_offsets[leader_id]
        
        # 2. 計算需要平移的向量 (目標是讓領頭機的偏移變為0,0,0)
        translation_vector = (-leader_initial_offset[0], -leader_initial_offset[1], -leader_initial_offset[2])

        # 3. 將所有無人機的偏移量都應用這個平移向量
        final_offsets = {}
        for i, offset in new_offsets.items():
            final_offsets[i] = (offset[0] + translation_vector[0], offset[1] + translation_vector[1], offset[2] + translation_vector[2])
        new_offsets = final_offsets

    drone_offsets_body_frame = new_offsets
    # 根據新的偏移量，簡單地設定起飛高度 (這裡僅為示例，您可以自訂更複雜的邏輯)
    new_takeoff_alt = {}
    for i, offset in drone_offsets_body_frame.items():
        # 讓 Z 軸偏移量影響起飛高度
        new_takeoff_alt[i] = base_alt - offset[2]
    takeoff_alt = new_takeoff_alt
 
    print(f"✅ 已設定隊形為 '{formation_name}'，共 {num_drones} 架無人機。")
    print("隊形偏移量:", drone_offsets_body_frame)

