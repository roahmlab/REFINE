#ifndef ROAHM_WARNING_GUARDS_HPP_
#define ROAHM_WARNING_GUARDS_HPP_
/**
 * Macros to disable warnings (e.g. for when including ROS headers, etc.)
 */

/// @file warning_guards.hpp

/// Disables clang warnings after this statement, until
#define WARN_DISABLE_BEGIN                                 \
  _Pragma("clang diagnostic push")                         \
      _Pragma("clang diagnostic ignored \"-Weverything\"") \
          _Pragma("clang diagnostic push")                 \
              _Pragma("clang diagnostic ignored \"-Wreserved-identifier\"")

/// Re-enables clang's warnings
#define WARN_DISABLE_END \
  _Pragma("clang diagnostic pop") _Pragma("clang diagnostic pop")

/// Disables warnings for argument, then re-enable them afterwards.
#define WARN_DISABLE(x) \
  WARN_DISABLE_BEGIN    \
  x WARN_DISABLE_END

#endif  // ROVER_RTD_NEW_WARNING_GUARDS_HPP
