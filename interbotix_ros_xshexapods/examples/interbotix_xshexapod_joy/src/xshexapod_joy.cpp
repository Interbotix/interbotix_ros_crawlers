#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include "interbotix_xshexapod_joy/HexJoy.h"

// PS3 Controller button mappings
static const std::map<std::string, int> ps3 = {{"PREVIOUS", 0},             // buttons start here
                                               {"WIDEN_STANCE", 1},
                                               {"NEXT", 2},
                                               {"REBOOT_NARROW_STANCE", 3},
                                               {"PLACE_Z_DEC", 4},
                                               {"PLACE_Z_INC", 5},
                                               {"WORLD_YAW_CCW", 6},
                                               {"WORLD_YAW_CW", 7},
                                               {"SLEEP_POSE", 8},
                                               {"HOME_POSE", 9},
                                               {"MOVE_TYPE", 10},
                                               {"SET_HOME_POSE", 11},
                                               {"FLIP_IN_PLACE_MODE", 12},
                                               {"SPEED_INC", 13},
                                               {"SPEED_DEC", 14},
                                               {"SPEED_COURSE", 15},
                                               {"SPEED_FINE", 16},
                                               {"WORLD_Y", 0},                  // axes start here
                                               {"WORLD_X", 1},
                                               {"PLACE_Y_ROLL", 3},
                                               {"PLACE_X_PITCH", 4}};

// PS4 Controller button mappings
static const std::map<std::string, int> ps4 = {{"PREVIOUS", 0},             // buttons start here
                                               {"WIDEN_STANCE", 1},
                                               {"NEXT", 2},
                                               {"REBOOT_NARROW_STANCE", 3},
                                               {"PLACE_Z_DEC", 4},
                                               {"PLACE_Z_INC", 5},
                                               {"WORLD_YAW_CCW", 6},
                                               {"WORLD_YAW_CW", 7},
                                               {"SLEEP_POSE", 8},
                                               {"HOME_POSE", 9},
                                               {"MOVE_TYPE", 10},
                                               {"SET_HOME_POSE", 11},
                                               {"FLIP_IN_PLACE_MODE", 12},
                                               {"WORLD_Y", 0},                  // axes start here
                                               {"WORLD_X", 1},
                                               {"PLACE_Y_ROLL", 3},
                                               {"PLACE_X_PITCH", 4},
                                               {"SPEED_TYPE", 6},
                                               {"SPEED", 7}};

ros::Publisher pub_joy_cmd;                                 // ROS Publisher to publish HexJoy messages
double threshold;                                           // Joystick sensitivity threshold
std::string controller_type;                                // Holds the name of the controller received from the ROS Parameter server
std::map<std::string, int> cntlr;                           // Holds the controller button mappings
interbotix_xshexapod_joy::HexJoy prev_joy_cmd;              // Keep track of the previously commanded HexJoy message so that only unique messages are published

/// @brief Joystick callback to create custom HexJoy messages to control the Hexapod
/// @param msg - raw sensor_msgs::Joy data
void joy_state_cb(const sensor_msgs::Joy &msg)
{
  static bool flip_move_type_cmd = false;
  static bool flip_move_type_cmd_last_state = false;
  static bool flip_in_place_cmd = false;
  static bool flip_in_place_cmd_last_state = false;
  interbotix_xshexapod_joy::HexJoy joy_cmd;

  // Check if the move_type_cmd should be flipped
  if (msg.buttons.at(cntlr["MOVE_TYPE"]) == 1 && flip_move_type_cmd_last_state == false)
  {
    flip_move_type_cmd = true;
    joy_cmd.move_type_cmd = interbotix_xshexapod_joy::HexJoy::MOVE_LEG;
  }
  else if (msg.buttons.at(cntlr["MOVE_TYPE"]) == 1 && flip_move_type_cmd_last_state == true)
  {
    flip_move_type_cmd = false;
    joy_cmd.move_type_cmd = interbotix_xshexapod_joy::HexJoy::MOVE_HEXAPOD;
  }
  else if (msg.buttons.at(cntlr["MOVE_TYPE"]) == 0)
    flip_move_type_cmd_last_state = flip_move_type_cmd;

  // Check if the flip_in_place_cmd should be flipped
  if (msg.buttons.at(cntlr["FLIP_IN_PLACE_MODE"]) == 1 && flip_in_place_cmd_last_state == false)
    flip_in_place_cmd = true;
  else if (msg.buttons.at(cntlr["FLIP_IN_PLACE_MODE"]) == 1 && flip_in_place_cmd_last_state == true)
    flip_in_place_cmd = false;
  else if (msg.buttons.at(cntlr["FLIP_IN_PLACE_MODE"]) == 0)
    flip_in_place_cmd_last_state = flip_in_place_cmd;

  // Check the place_y_cmd / place_roll_cmd
  if (msg.axes.at(cntlr["PLACE_Y_ROLL"]) >= threshold && flip_in_place_cmd == false)
    joy_cmd.place_y_cmd = interbotix_xshexapod_joy::HexJoy::PLACE_Y_INC;
  else if (msg.axes.at(cntlr["PLACE_Y_ROLL"]) <= -threshold && flip_in_place_cmd == false)
    joy_cmd.place_y_cmd = interbotix_xshexapod_joy::HexJoy::PLACE_Y_DEC;
  else if (msg.axes.at(cntlr["PLACE_Y_ROLL"]) >= threshold && flip_in_place_cmd == true)
    joy_cmd.place_roll_cmd = interbotix_xshexapod_joy::HexJoy::PLACE_ROLL_CW;
  else if (msg.axes.at(cntlr["PLACE_Y_ROLL"]) <= -threshold && flip_in_place_cmd == true)
    joy_cmd.place_roll_cmd = interbotix_xshexapod_joy::HexJoy::PLACE_ROLL_CCW;

  // Check the place_x_cmd / place_pitch_cmd
  if (msg.axes.at(cntlr["PLACE_X_PITCH"]) >= threshold && flip_in_place_cmd == false)
    joy_cmd.place_x_cmd = interbotix_xshexapod_joy::HexJoy::PLACE_X_INC;
  else if (msg.axes.at(cntlr["PLACE_X_PITCH"]) <= -threshold && flip_in_place_cmd == false)
    joy_cmd.place_x_cmd = interbotix_xshexapod_joy::HexJoy::PLACE_X_DEC;
  else if (msg.axes.at(cntlr["PLACE_X_PITCH"]) >= threshold && flip_in_place_cmd == true)
    joy_cmd.place_pitch_cmd = interbotix_xshexapod_joy::HexJoy::PLACE_PITCH_DOWN;
  else if (msg.axes.at(cntlr["PLACE_X_PITCH"]) <= -threshold && flip_in_place_cmd == true)
    joy_cmd.place_pitch_cmd = interbotix_xshexapod_joy::HexJoy::PLACE_PITCH_UP;

  // Check the world_x_cmd
  if (msg.axes.at(cntlr["WORLD_X"]) >= threshold)
    joy_cmd.world_x_cmd = interbotix_xshexapod_joy::HexJoy::WORLD_X_INC;
  else if (msg.axes.at(cntlr["WORLD_X"]) <= -threshold)
    joy_cmd.world_x_cmd = interbotix_xshexapod_joy::HexJoy::WORLD_X_DEC;

  // Check the world_y_cmd
  if (msg.axes.at(cntlr["WORLD_Y"]) >= threshold)
    joy_cmd.world_y_cmd = interbotix_xshexapod_joy::HexJoy::WORLD_Y_INC;
  else if (msg.axes.at(cntlr["WORLD_Y"]) <= -threshold)
    joy_cmd.world_y_cmd = interbotix_xshexapod_joy::HexJoy::WORLD_Y_DEC;

  // Check the place_z_cmd
  if (msg.buttons.at(cntlr["PLACE_Z_INC"]) == 1)
   joy_cmd.place_z_cmd = interbotix_xshexapod_joy::HexJoy::PLACE_Z_INC;
  else if (msg.buttons.at(cntlr["PLACE_Z_DEC"]) == 1)
   joy_cmd.place_z_cmd = interbotix_xshexapod_joy::HexJoy::PLACE_Z_DEC;

  // Check the world_yaw_cmd
  if (msg.buttons.at(cntlr["WORLD_YAW_CCW"]) == 1)
   joy_cmd.world_yaw_cmd = interbotix_xshexapod_joy::HexJoy::WORLD_YAW_CCW;
  else if (msg.buttons.at(cntlr["WORLD_YAW_CW"]) == 1)
   joy_cmd.world_yaw_cmd = interbotix_xshexapod_joy::HexJoy::WORLD_YAW_CW;

  // Check the pose_cmd
  if (msg.buttons.at(cntlr["HOME_POSE"]) == 1)
   joy_cmd.pose_cmd = interbotix_xshexapod_joy::HexJoy::HOME_POSE;
  else if (msg.buttons.at(cntlr["SLEEP_POSE"]) == 1)
   joy_cmd.pose_cmd = interbotix_xshexapod_joy::HexJoy::SLEEP_POSE;

  // Check the gait_toggle_cmd
  if (msg.buttons.at(cntlr["NEXT"]) == 1 && flip_move_type_cmd == false)
   joy_cmd.gait_toggle_cmd = interbotix_xshexapod_joy::HexJoy::GAIT_NEXT;
  else if (msg.buttons.at(cntlr["PREVIOUS"]) == 1 && flip_move_type_cmd == false)
   joy_cmd.gait_toggle_cmd = interbotix_xshexapod_joy::HexJoy::GAIT_PREVIOUS;

  // Check the leg_toggle_cmd
  if (msg.buttons.at(cntlr["NEXT"]) == 1 && flip_move_type_cmd == true)
   joy_cmd.leg_toggle_cmd = interbotix_xshexapod_joy::HexJoy::LEG_NEXT;
  else if (msg.buttons.at(cntlr["PREVIOUS"]) == 1 && flip_move_type_cmd == true)
   joy_cmd.leg_toggle_cmd = interbotix_xshexapod_joy::HexJoy::LEG_PREVIOUS;

  // Check the stance_cmd
  if (msg.buttons.at(cntlr["WIDEN_STANCE"]) == 1 && flip_move_type_cmd == false)
    joy_cmd.stance_cmd = interbotix_xshexapod_joy::HexJoy::WIDEN_STANCE;
  else if (msg.buttons.at(cntlr["REBOOT_NARROW_STANCE"]) == 1 && flip_move_type_cmd == false)
    joy_cmd.stance_cmd = interbotix_xshexapod_joy::HexJoy::NARROW_STANCE;

  // Check the reboot_cmd
  if (msg.buttons.at(cntlr["REBOOT_NARROW_STANCE"]) == 1 && flip_move_type_cmd == true)
    joy_cmd.reboot_cmd = interbotix_xshexapod_joy::HexJoy::REBOOT_MOTORS;

  // Check the set_home_pose_cmd
  if (msg.buttons.at(cntlr["SET_HOME_POSE"]) == 1)
    joy_cmd.set_home_pose_cmd = interbotix_xshexapod_joy::HexJoy::SET_HOME_POSE;

  if (controller_type == "ps3")
  {
   // Check the speed_cmd
   if (msg.buttons.at(cntlr["SPEED_INC"]) == 1)
     joy_cmd.speed_cmd = interbotix_xshexapod_joy::HexJoy::SPEED_INC;
   else if (msg.buttons.at(cntlr["SPEED_DEC"]) == 1)
     joy_cmd.speed_cmd = interbotix_xshexapod_joy::HexJoy::SPEED_DEC;

   // Check the speed_toggle_cmd
   if (msg.buttons.at(cntlr["SPEED_COURSE"]) == 1)
     joy_cmd.speed_toggle_cmd = interbotix_xshexapod_joy::HexJoy::SPEED_COURSE;
   else if (msg.buttons.at(cntlr["SPEED_FINE"]) == 1)
     joy_cmd.speed_toggle_cmd = interbotix_xshexapod_joy::HexJoy::SPEED_FINE;
  }
  else if (controller_type == "ps4")
  {
   // Check the speed_cmd
   if (msg.axes.at(cntlr["SPEED"]) == 1)
     joy_cmd.speed_cmd = interbotix_xshexapod_joy::HexJoy::SPEED_INC;
   else if (msg.axes.at(cntlr["SPEED"]) == -1)
     joy_cmd.speed_cmd = interbotix_xshexapod_joy::HexJoy::SPEED_DEC;

   // Check the toggle_speed_cmd
   if (msg.axes.at(cntlr["SPEED_TYPE"]) == 1)
     joy_cmd.speed_toggle_cmd = interbotix_xshexapod_joy::HexJoy::SPEED_COURSE;
   else if (msg.axes.at(cntlr["SPEED_TYPE"]) == -1)
     joy_cmd.speed_toggle_cmd = interbotix_xshexapod_joy::HexJoy::SPEED_FINE;
  }

  // Only publish a HexJoy message if any of the following fields have changed.
  if (!(prev_joy_cmd.world_x_cmd == joy_cmd.world_x_cmd &&
     prev_joy_cmd.world_y_cmd == joy_cmd.world_y_cmd &&
     prev_joy_cmd.world_yaw_cmd == joy_cmd.world_yaw_cmd &&
     prev_joy_cmd.place_x_cmd == joy_cmd.place_x_cmd &&
     prev_joy_cmd.place_y_cmd == joy_cmd.place_y_cmd &&
     prev_joy_cmd.place_z_cmd == joy_cmd.place_z_cmd &&
     prev_joy_cmd.place_roll_cmd == joy_cmd.place_roll_cmd &&
     prev_joy_cmd.place_pitch_cmd == joy_cmd.place_pitch_cmd &&
     prev_joy_cmd.pose_cmd == joy_cmd.pose_cmd &&
     prev_joy_cmd.move_type_cmd == joy_cmd.move_type_cmd &&
     prev_joy_cmd.gait_toggle_cmd == joy_cmd.gait_toggle_cmd &&
     prev_joy_cmd.leg_toggle_cmd == joy_cmd.leg_toggle_cmd &&
     prev_joy_cmd.stance_cmd == joy_cmd.stance_cmd &&
     prev_joy_cmd.reboot_cmd == joy_cmd.reboot_cmd &&
     prev_joy_cmd.set_home_pose_cmd == joy_cmd.set_home_pose_cmd &&
     prev_joy_cmd.speed_cmd == joy_cmd.speed_cmd &&
     prev_joy_cmd.speed_toggle_cmd == joy_cmd.speed_toggle_cmd))
     pub_joy_cmd.publish(joy_cmd);
  prev_joy_cmd = joy_cmd;
}

int main(int argc, char **argv)
{
 ros::init(argc, argv, "xshexapod_joy");
 ros::NodeHandle n;
 ros::param::get("~threshold", threshold);
 ros::param::get("~controller", controller_type);
 if (controller_type == "ps3")
   cntlr = ps3;
 else
   cntlr = ps4;
 ros::Subscriber sub_joy_raw = n.subscribe("commands/joy_raw", 10, joy_state_cb);
 pub_joy_cmd = n.advertise<interbotix_xshexapod_joy::HexJoy>("commands/joy_processed", 10);
 ros::spin();
 return 0;
}
