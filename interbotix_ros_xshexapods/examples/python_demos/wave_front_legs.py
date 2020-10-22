import math
import time
from interbotix_xs_modules.hexapod import InterbotixHexapodXS

# This script makes the hexapod wave its front legs for approximately 20 seconds
#
# To get started, open a terminal and type 'roslaunch interbotix_xshexapod_control xshexapod_control.launch robot_model:=mark4'
# Then change to this directory and type 'python wave_front_legs.py'

def main():
    bot = InterbotixHexapodXS('pxmark4')
    bot.hex.move_in_place(x=-0.05, z=0.08, pitch=-0.1)
    bot.hex.move_leg("right_front", [0, 0.05, 0.08])
    bot.hex.move_leg("left_front", [0, -0.05, 0.08])
    z_prev = 0
    for step in range(500):
        z = 0.03 * math.sin(math.pi * step/25.0)
        bot.hex.move_leg("right_front", [0, 0, z - z_prev], moving_time=0.150, accel_time=0.075, blocking=False)
        bot.hex.move_leg("left_front", [0, 0, z_prev - z], moving_time=0.150, accel_time=0.075, blocking=False)
        z_prev = z
        time.sleep(0.04)
    bot.hex.reset_hexapod('sleep')

if __name__=='__main__':
    main()
