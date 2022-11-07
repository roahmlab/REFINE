#ifndef ROAHM_POINT_XY_H_
#define ROAHM_POINT_XY_H_
namespace roahm {
struct PointXY {
  double x_, y_;
  PointXY() : x_{}, y_{} {}
  PointXY(double x, double y) : x_(x), y_(y) {}
};
}  // namespace roahm
#endif  // ROAHM_POINT_XY_H_
