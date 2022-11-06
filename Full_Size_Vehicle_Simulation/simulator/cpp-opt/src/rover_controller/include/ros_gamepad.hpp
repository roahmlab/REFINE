#ifndef ROAHM_ROS_GAMEPAD_HPP_
#define ROAHM_ROS_GAMEPAD_HPP_
#include <sensor_msgs/Joy.h>
/// @file ros_gamepad.hpp Contains classes to handle gamepad inputs for the
/// rover control
namespace roahm {
class GamepadButton {
 private:
  /// True iff at least one previous button update has been received
  bool has_initialized_;
  /// True iff the button was being held as of the last update
  bool held_;
  /// True iff the button has just been pressed
  bool just_pressed_;
  /// True iff the button has just been released
  bool just_released_;

 public:
  /// Constructor that takes an initial value
  /// \param held whether the button is currently held
  GamepadButton(bool held)
      : has_initialized_{false},
        held_{held},
        just_pressed_{false},
        just_released_{false} {}

  /// Default constructor, defaults the button state to not being held
  GamepadButton() : GamepadButton(false) {}

  /// Checks whether the button is currently held down
  /// \return true if the button is currently held
  bool Held() const { return held_; }

  /// Checks whether the button has just been pressed
  /// \return true if the button has just been pressed, false if only one value
  /// has been received or if the button has not just been pressed
  bool Pressed() const { return just_pressed_; }

  /// Checks whether the button has just been released
  /// \return true if the button has just been released, false if only one value
  /// has been received or if the button has not just been released
  bool Released() const { return just_released_; }

  /// Updates the button state given a new value
  /// \param held whether the button is currently held
  void Update(bool held) {
    if (has_initialized_) {
      just_pressed_ = !held_ && held;
      just_released_ = held_ && !held;
    } else {
      just_pressed_ = false;
      just_released_ = false;
    }
    held_ = held;
    has_initialized_ = true;
  }
};
/// Contains gamepad state information
struct GamepadInfo {
  double left_stick_x_;
  double left_stick_y_;
  double right_stick_x_;
  double right_stick_y_;
  GamepadButton dpad_up_;
  GamepadButton dpad_down_;
  GamepadButton dpad_right_;
  GamepadButton dpad_left_;
  GamepadButton x_;
  GamepadButton a_;
  GamepadButton b_;
  GamepadButton y_;
  GamepadButton l_bump_;
  GamepadButton r_bump_;
  GamepadButton l_trig_;
  GamepadButton r_trig_;
  GamepadButton back_;
  GamepadButton start_;
  GamepadButton l_stick_;
  GamepadButton r_stick_;
  GamepadInfo() = delete;

  /// Update the internal gamepad state using the latest ROS message.
  /// \param msg the latest ROS message with new gamepad states
  void Update(const sensor_msgs::Joy& msg) {
    left_stick_x_ = -msg.axes.at(0);
    left_stick_y_ = msg.axes.at(1);
    right_stick_x_ = -msg.axes.at(2);
    right_stick_y_ = msg.axes.at(3);
    const int dpad_x = static_cast<int>(-msg.axes.at(4));
    const int dpad_y = static_cast<int>(msg.axes.at(5));
    dpad_up_.Update(dpad_y == 1);
    dpad_down_.Update(dpad_y == -1);
    dpad_right_.Update(dpad_x == 1);
    dpad_left_.Update(dpad_x == -1);
    x_.Update(static_cast<bool>(msg.buttons.at(0)));
    a_.Update(static_cast<bool>(msg.buttons.at(1)));
    b_.Update(static_cast<bool>(msg.buttons.at(2)));
    y_.Update(static_cast<bool>(msg.buttons.at(3)));
    l_bump_.Update(static_cast<bool>(msg.buttons.at(4)));
    r_bump_.Update(static_cast<bool>(msg.buttons.at(5)));
    l_trig_.Update(static_cast<bool>(msg.buttons.at(6)));
    r_trig_.Update(static_cast<bool>(msg.buttons.at(7)));
    back_.Update(static_cast<bool>(msg.buttons.at(8)));
    start_.Update(static_cast<bool>(msg.buttons.at(9)));
    l_stick_.Update(static_cast<bool>(msg.buttons.at(10)));
    r_stick_.Update(static_cast<bool>(msg.buttons.at(11)));
  }
};
}  // namespace roahm
#endif
