from interbotix_xs_modules.hexapod import InterbotixHexapodXS

# This script makes the hexapod move forward/backwards using a version of the tripod gait geared for uneven terrain
#
# To get started, open a terminal and type 'roslaunch interbotix_xshexapod_control xshexapod_control.launch robot_model:=wxmark4'
# Then change to this directory and type 'python uneven_terrain.py'

def main():
    bot = InterbotixHexapodXS('wxmark4')
    # move hexapod legs up/down to get rid of any stress the motors might be experiencing from picking the hexapod body up
    bot.hex.move_in_world()
    # move the hexapod body up to allow for more foot clearance
    bot.hex.move_in_place(z=0.12)
    # begin uneven gait - moving forward
    # reset foot points to defaults every two cycles - this prevents drift building up
    for cntr in range(10):
	    bot.hex.move_in_world_rough(x_stride=0.06, reset_foot_points=True, reset_height=0.12, max_foot_height=0.05, num_cycles=2)
    # begin uneven gait - moving backward
    # reset foot points to defaults every two cycles - this prevents drift building up
    for cntr in range(10):
	    bot.hex.move_in_world_rough(x_stride=-0.06, reset_foot_points=True, reset_height=0.12, max_foot_height=0.05, num_cycles=2)
    bot.hex.reset_hexapod(pose_type="sleep")

if __name__=='__main__':
    main()
