import math
import time
from interbotix_xs_modules.hexapod import InterbotixHexapodXS

# This script makes the hexapod body move up and down for approximately 20 seconds
#
# To get started, open a terminal and type 'roslaunch interbotix_xshexapod_control xshexapod_control.launch robot_model:=pxmark4'
# Then change to this directory and type 'python push_ups.py'

def main():
    bot = InterbotixHexapodXS('pxmark4')
    bot.hex.move_in_place(z=0.08)
    for step in range(500):
        z = 0.08 + 0.05 * math.sin(math.pi * step/25.0)
        bot.hex.move_in_place(z=z, moving_time=0.15, blocking=False)
        time.sleep(0.04)
    bot.hex.reset_hexapod('sleep')

if __name__=='__main__':
    main()
