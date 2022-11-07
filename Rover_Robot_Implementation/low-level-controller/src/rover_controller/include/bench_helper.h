#ifndef BENCH_HELPER_H_
#define BENCH_HELPER_H_

#include <chrono>

namespace roahm {
inline auto Tick() { return std::chrono::high_resolution_clock::now(); }

template <typename T, typename S>
double GetDeltaS(const T& t1, const S& t0) {
  return static_cast<double>(
             std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
                 .count()) /
         1.0e9;
}
template <typename T, typename S>
double GetDeltaMillis(const T& t1, const S& t0) {
  return static_cast<double>(
             std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0)
                 .count()) /
         1.0e6;
}
}  // namespace roahm

#endif
