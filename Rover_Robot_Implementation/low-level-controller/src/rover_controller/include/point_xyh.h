#ifndef ROAHM_POINT_XYH_H_
#define ROAHM_POINT_XYH_H_
namespace roahm {
struct PointXYH {
  double x_, y_, h_;
  PointXYH() : x_{}, y_{}, h_{} {}
  PointXYH(double x, double y, double h) : x_(x), y_(y), h_(h) {}
};
}  // namespace roahm
#endif  // ROAHM_POINT_XYH_H_
