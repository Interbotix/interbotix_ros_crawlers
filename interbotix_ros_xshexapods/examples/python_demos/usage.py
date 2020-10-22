from interbotix_xs_modules.hexapod import InterbotixHexapodXS

# This script is a walkthrough on using the hexapod
#
# To get started, open a terminal and type 'roslaunch interbotix_xshexapod_control xshexapod_control.launch robot_model:=pxmark4'
# Then change to this directory and type 'python usage.py'

GOLD = 0xD4AF37

def main():
    # Instantiate an instance of the hexapod (goes to the default Home Pose when done)
    bot = InterbotixHexapodXS('pxmark4')

    # The stance the robot is currently in is the default Home Pose. Whenever you call
    # the 'reset_hexapod("home") function or the code gets stuck (perhaps due to a joint limt
    # being reached or failing to find an IK solution), this is the pose it will go to.
    # However, you can change the default Home Pose to be something else that's more convenient.
    # Do it by first setting a new height (physically, this is the distance from the 'base_link'
    # frame to the ground)...
    bot.hex.move_in_place(z=0.1)

    # ...assuming the new height looks good, set it to be the new Home Pose height
    bot.hex.set_home_height(0.1)

    # ...then modify the stance if you want. By default, the hexapod's stance is a bit wide which can
    # cause slipping. So make it two centimeters tighter by typing the following command. This essentially
    # takes each foot point and brings it two centimeters closer to its coxa joint
    bot.hex.modify_stance(-0.02)

    # Now get the new foot points...
    foot_points = bot.hex.get_foot_points()

    # Set them to be the new 'Home Pose' foot points...
    bot.hex.set_home_foot_points(foot_points)

    # And confirm that when you execute the following command, the hexapod doesn't move...
    bot.hex.reset_hexapod("home")

    # Now make the hexapod walk forward using the ripple gait. Every gait cycle, it should
    # walk 8 cm for a total of 40 cm.
    bot.hex.move_in_world(x_stride=0.08, gait_type="ripple", num_cycles=5)

    # Now make the hexapod walk backwards using the wave gait the same distance.
    bot.hex.move_in_world(x_stride=-0.08, gait_type="wave", num_cyles=5)

    # Set the LEDs to both be gold for no good reason, just cuz...
    bot.pixels.set_color(color=GOLD, set_all_leds=True)

    # Go to sleep :)
    bot.hex.reset_hexapod("sleep")

if __name__=='__main__':
    main()
