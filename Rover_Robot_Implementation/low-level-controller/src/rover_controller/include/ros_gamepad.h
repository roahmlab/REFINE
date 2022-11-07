#ifndef ROAHM_ROS_GAMEPAD_H_
#define ROAHM_ROS_GAMEPAD_H_
#include <sensor_msgs/Joy.h>
namespace roahm {
class GamepadButton {
 private:
  bool held_;
  bool just_pressed_;
  bool just_released_;

 public:
  GamepadButton() = default;
  GamepadButton(bool held)
      : held_(held), just_pressed_(false), just_released_(false) {}
  bool Held() const { return held_; }
  bool Pressed() const { return just_pressed_; }
  bool Released() const { return just_released_; }
  void Update(bool held) {
    just_pressed_ = !held_ && held;
    just_released_ = held_ && !held;
    held_ = held;
  }
};
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
  GamepadInfo() = default;
  GamepadInfo(const sensor_msgs::Joy& msg)
      : left_stick_x_{-msg.axes[0]},
        left_stick_y_{msg.axes[1]},
        right_stick_x_{-msg.axes[2]},
        right_stick_y_{msg.axes[3]},
        dpad_up_{static_cast<int>(msg.axes[5]) == 1},
        dpad_down_{static_cast<int>(msg.axes[5]) == -1},
        dpad_right_{static_cast<int>(-msg.axes[4]) == 1},
        dpad_left_{static_cast<int>(-msg.axes[4]) == -1},
        x_{static_cast<bool>(msg.buttons[0])},
        a_{static_cast<bool>(msg.buttons[1])},
        b_{static_cast<bool>(msg.buttons[2])},
        y_{static_cast<bool>(msg.buttons[3])},
        l_bump_{static_cast<bool>(msg.buttons[4])},
        r_bump_{static_cast<bool>(msg.buttons[5])},
        l_trig_{static_cast<bool>(msg.buttons[6])},
        r_trig_{static_cast<bool>(msg.buttons[7])},
        back_{static_cast<bool>(msg.buttons[8])},
        start_{static_cast<bool>(msg.buttons[9])},
        l_stick_{static_cast<bool>(msg.buttons[10])},
        r_stick_{static_cast<bool>(msg.buttons[11])} {}

  void Update(const sensor_msgs::Joy& msg) {
    left_stick_x_ = -msg.axes[0];
    left_stick_y_ = msg.axes[1];
    right_stick_x_ = -msg.axes[2];
    right_stick_y_ = msg.axes[3];
    // dpad_x_ = static_cast<int>(-msg.axes[4]);
    // dpad_y_ = static_cast<int>(msg.axes[5]);
    const int dpad_x = static_cast<int>(-msg.axes[4]);
    const int dpad_y = static_cast<int>(msg.axes[5]);
    dpad_up_.Update(dpad_y == 1);
    dpad_down_.Update(dpad_y == -1);
    dpad_right_.Update(dpad_x == 1);
    dpad_left_.Update(dpad_x == -1);
    x_.Update(static_cast<bool>(msg.buttons[0]));
    a_.Update(static_cast<bool>(msg.buttons[1]));
    b_.Update(static_cast<bool>(msg.buttons[2]));
    y_.Update(static_cast<bool>(msg.buttons[3]));
    l_bump_.Update(static_cast<bool>(msg.buttons[4]));
    r_bump_.Update(static_cast<bool>(msg.buttons[5]));
    l_trig_.Update(static_cast<bool>(msg.buttons[6]));
    r_trig_.Update(static_cast<bool>(msg.buttons[7]));
    back_.Update(static_cast<bool>(msg.buttons[8]));
    start_.Update(static_cast<bool>(msg.buttons[9]));
    l_stick_.Update(static_cast<bool>(msg.buttons[10]));
    r_stick_.Update(static_cast<bool>(msg.buttons[11]));
  }
};
}  // namespace roahm
#endif
