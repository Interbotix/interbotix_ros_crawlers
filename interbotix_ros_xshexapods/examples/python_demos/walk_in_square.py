from interbotix_xs_modules.hexapod import InterbotixHexapodXS

# This script makes the hexapod follow a 'square-like' trajectory
#
# To get started, open a terminal and type 'roslaunch interbotix_xshexapod_control xshexapod_control.launch robot_model:=mark4'
# Then change to this directory and type 'python walk_in_square.py'

def main():
    bot = InterbotixHexapodXS('pxmark4')
    bot.hex.move_in_place(z=0.1)
    bot.hex.modify_stance(-0.02)
    bot.hex.move_in_world(x_stride=0.06, num_cycles=2, gait_type="tripod")
    bot.hex.move_in_world(y_stride=0.06, num_cycles=2, gait_type="ripple")
    bot.hex.move_in_world(x_stride=-0.06, num_cycles=2, gait_type="wave")
    bot.hex.move_in_world(y_stride=-0.06, num_cycles=2, gait_type="wave")
    bot.hex.reset_hexapod('sleep')

if __name__=='__main__':
    main()
