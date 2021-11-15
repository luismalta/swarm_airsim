import os
import subprocess
from time import sleep

# os.system('export AIRSIM_PATH=/home/luismalta/Documentos/Blocks/LinuxNoEditor/Blocks.sh')
os.environ["PX4_PATH"] = "/home/luismalta/softwares/PX4-Autopilot"
os.environ["AIRSIM_PATH"] = "/home/luismalta/Documentos/Blocks/LinuxNoEditor/Blocks.sh"

def start_process():

    PX4_PATH = os.getenv('PX4_PATH')
    args = ('make -C ' + PX4_PATH + ' px4_sitl_default none_iris',)
    subprocess.Popen(args)

    AIRSIM_PATH = os.getenv('AIRSIM_PATH')
    args = (AIRSIM_PATH,)
    subprocess.Popen(args)

    sleep(15)

    os.system('roslaunch mavros px4.launch fcu_url:="udp://127.0.0.1:14555@14555"')


if __name__ == "__main__":
  os.setpgrp()
  try:
    start_process()
  finally:
    os.killpg(0, signal.SIGKILL) # kill all processes in my group