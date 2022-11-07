#ifndef ROAHM_RMANU_TYPE_H_
#define ROAHM_RMANU_TYPE_H_
#include <ros/ros.h>

namespace roahm {
enum class RManuType { kDirChange, kLanChange, kSpdChange, kNone };
RManuType FromString(const std::string& manu_type_str) {
  if (manu_type_str == "DIR") {
    return RManuType::kDirChange;
  } else if (manu_type_str == "LAN") {
    return RManuType::kLanChange;
  } else if (manu_type_str == "SPD") {
    return RManuType::kSpdChange;
  } else if (manu_type_str == "NON") {
    return RManuType::kNone;
  } else {
    ROS_WARN_STREAM("RManuType " << manu_type_str << " does not exist");
    return RManuType::kSpdChange;
  }
}
std::string ToString(const RManuType& manu_type) {
  switch (manu_type) {
    case RManuType::kDirChange:
      return "DIR";
    case RManuType::kLanChange:
      return "LAN";
    case RManuType::kSpdChange:
      return "SPD";
    case RManuType::kNone:
      return "NON";
  }
  ROS_WARN("Manu Type Error in ToString");
  assert(false);
  return "UKN";
}

double GetGenParamAu(const RManuType& manu_type, double u0, double k_param) {
  // Au = K for speed change, Au = u0 otherwise
  if (manu_type == RManuType::kSpdChange) {
    return k_param;
  }
  return u0;
}

double GetGenParamAy(const RManuType& manu_type, double u0, double k_param) {
  if (manu_type == RManuType::kSpdChange) {
    return u0;
  }
  return k_param;
}

bool IsSpd(RManuType rmanu_type) { return rmanu_type == RManuType::kSpdChange; }
bool IsDir(RManuType rmanu_type) { return rmanu_type == RManuType::kDirChange; }
bool IsLan(RManuType rmanu_type) { return rmanu_type == RManuType::kLanChange; }
bool IsNone(RManuType rmanu_type) { return rmanu_type == RManuType::kNone; }
bool IsDirLan(RManuType rmanu_type) {
  return IsDir(rmanu_type) or IsLan(rmanu_type);
}
bool IsSpdDir(RManuType rmanu_type) {
  return IsSpd(rmanu_type) or IsDir(rmanu_type);
}
bool IsValid(RManuType rmanu_type) {
  return IsSpd(rmanu_type) or IsDirLan(rmanu_type);
}
double GetPreBrakeTime(RManuType rmanu_type) {
  // TODO from consts
  if (IsSpdDir(rmanu_type)) {
    return 1.5;
  } else if (IsLan(rmanu_type)) {
    return 3.0;
  } else if (IsNone(rmanu_type)) {
    return 0.0;
  }
  // TODO warn
  return 0.0;
}
std::uint8_t ManuToUint8(RManuType rmanu_type) {
  if (IsSpd(rmanu_type)) {
    return '0';
  } else if (IsDir(rmanu_type)) {
    return '1';
  } else if (IsLan(rmanu_type)) {
    return '2';
  } else if (IsNone(rmanu_type)) {
    return 'N';
  }
  ROS_WARN("Unknown manu type in conversion to uint8");
  assert(false);
  return 'U';
}
RManuType ManuFromUint8(std::uint8_t char_code) {
  if (char_code == '0') {
    return RManuType::kSpdChange;
  } else if (char_code == '1') {
    return RManuType::kDirChange;
  } else if (char_code == '2') {
    return RManuType::kLanChange;
  } else if (char_code == 'N') {
    return RManuType::kNone;
  }
  ROS_WARN_STREAM("Manu Type Character '" << char_code << "' not recognized");
  assert(false);
  return RManuType::kNone;
}
}  // namespace roahm

#endif  // ROAHM_RMANU_TYPE_H_
