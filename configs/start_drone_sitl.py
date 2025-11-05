import json
import subprocess
import time
import os

python_path = os.path.join(os.environ["CONDA_PREFIX"], "python.exe")
with open("drones.json") as f:
    drones = json.load(f)

for d in drones:
    home = ",".join(map(str, d["home"][:3]))
    sys_id = d["id"] + 1

    # cmd 命令，注意沒有空格在 -I 與 --out 之間
    cmd = [
        "cmd", "/k",
        python_path, "-m", "dronekit-sitl", "copter",
        f"--home={home}",
        f"--instance={d['id']}",
        f"--out=127.0.0.1:{d['out_port']}",
        f"--model={d['model']}",
        "-I", str(sys_id),
        "--console"
    ]

    print(f"🚁 啟動 {d['name']} (out port={d['out_port']}, sysID={sys_id})")
    subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    time.sleep(1)
