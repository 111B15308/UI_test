from dronekit import connect
import time

# --- 定義訊息監聽器 ---
# 這是用來處理 RAW_IMU 訊息的回調函式
count=0
def handle_raw_imu(self, name, msg):
    """
    處理 RAW_IMU 訊息的回調函式。
    """
    global count
    count=count+1
    print(f"接收到 RAW_IMU 訊息 -{count} XAcc: {msg.xacc}, YAcc: {msg.yacc}, ZAcc: {msg.zacc}")

def handle_global_position_int(self, name, msg):
    """
    處理GLOBAL_POSITION_INT 訊息的回調函式。
    """
    global count
    count=count+1
    print(f"接收到 LOCAL_POSITION_NED 訊息 {count} LAT: {msg.lat}, LON: {msg.lon}, ALT: {msg.alt}")

# --- 連接無人機 ---
print("正在連接無人機...")
try:
    vehicle = connect('tcp:127.0.0.1:5760', wait_ready=['mode'], rate=1, timeout=60)
    print("無人機已連接！")
except Exception as e:
    print(f"連接無人機失敗: {e}")
    exit()

vehicle.add_message_listener('GLOBAL_POSITION_INT', handle_global_position_int)
#LOCAL_POSITION_NED (32) 收不到

time.sleep(5) # 觀察一段時間，確認是否有收到訊息

vehicle.remove_message_listener('GLOBAL_POSITION_INT', handle_local_position_ned)

vehicle.close()
print("無人機連接已關閉。")