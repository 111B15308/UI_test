from dronekit import connect
import time

# 設定要輸出的檔案名稱
output_filename = "ardupilot_parameters.txt"

print("正在連接無人機...")
try:
    vehicle = connect('tcp:127.0.0.1:5762', wait_ready=True, timeout=60)
    print("無人機已連接！")
except Exception as e:
    print(f"連接無人機失敗: {e}")
    exit()

vehicle.parameters['SR0_EXTRA1'] = 0
while vehicle.parameters['SR0_EXTRA1']!=0:
    vehicle.parameters['SR0_EXTRA1'] = 0
    print("SR1_RAW_SENS 參數設定中")
    time.sleep(1)
'''    
print(f"\n正在將所有參數寫入檔案: {output_filename}")

# 打開檔案以寫入模式 (w)，如果檔案不存在則創建，如果存在則覆蓋
try:
    with open(output_filename, 'w') as f:
        f.write("--- ArduPilot Parameters ---\n")
        f.write(f"取得時間: {time.ctime()}\n\n")

        # 遍歷 vehicle.parameters 並寫入檔案
        for key, value in vehicle.parameters.items():
            f.write(f"Key:{key} Value:{value}\n")
    
    print(f"所有參數已成功寫入 {output_filename} 檔案中。")

except Exception as e:
    print(f"寫入檔案時發生錯誤: {e}")

# 關閉無人機連接
vehicle.close()
print("\n無人機連接已關閉。")
''' 