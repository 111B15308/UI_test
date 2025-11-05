
@echo off

set SITL_PATH=/home/sam/ardupilot/ArduCopter

start "drone1" wsl -d Ubuntu -- bash -c "cd %SITL_PATH% && source /home/sam/ardupilot/env/bin/activate && ../Tools/autotest/sim_vehicle.py -v ArduCopter --model + --speedup 1 --instance 0 --sysid 1 --custom-location=22.904888,120.271982,30,0"

timeout /t 10 >nul

start "drone2" wsl -d Ubuntu -- bash -c "cd %SITL_PATH% && source /home/sam/ardupilot/env/bin/activate && ../Tools/autotest/sim_vehicle.py -v ArduCopter --model + --speedup 1 --instance 1 --sysid 2 --custom-location=22.904889,120.271983,30,0"

timeout /t 10 >nul

start "drone3" wsl -d Ubuntu -- bash -c "cd %SITL_PATH% && source /home/sam/ardupilot/env/bin/activate && ../Tools/autotest/sim_vehicle.py -v ArduCopter --model + --speedup 1 --instance 2 --sysid 3 --custom-location=22.904890,120.271984,30,0"

timeout /t 10 >nul

start "drone4" wsl -d Ubuntu -- bash -c "cd %SITL_PATH% && source /home/sam/ardupilot/env/bin/activate && ../Tools/autotest/sim_vehicle.py -v ArduCopter --model + --speedup 1 --instance 3 --sysid 4 --custom-location=22.904891,120.271985,30,0"

timeout /t 10 >nul

start "drone5" wsl -d Ubuntu -- bash -c "cd %SITL_PATH% && source /home/sam/ardupilot/env/bin/activate && ../Tools/autotest/sim_vehicle.py -v ArduCopter --model + --speedup 1 --instance 4 --sysid 5 --custom-location=22.904892,120.271986,30,0"

pause
