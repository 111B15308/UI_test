
@echo off

set SITL_PATH=/home/sam/ardupilot/ArduCopter

start "drone1" wsl -d Ubuntu -- bash -c "cd %SITL_PATH% && source /home/sam/ardupilot/env/bin/activate && ../Tools/autotest/sim_vehicle.py -v ArduCopter --model + --speedup 1 --instance 0 --sysid 1 --custom-location=22.904888,120.271982,30,0"

pause
