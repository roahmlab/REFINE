#ifndef ROAHM_GENCON_HPP_
#define ROAHM_GENCON_HPP_

#include <iosfwd>  // for ostream
#include <memory>  // for unique_ptr, shared_ptr
#include <string>  // for string
#include <vector>  // for vector

#include "common.hpp"      // for IndexT
#include "frs_loader.hpp"  // for Vehrs
#include "point_xyh.hpp"   // for PointXYH

/// @file gencon.hpp Contains constraint generation code and related utilities

namespace roahm {

class DynObs {
 private:
  double c_x0_;
  double c_y0_;
  double heading_rad_;
  double velocity_;
  double length_;
  double width_;
 public:
  double CenterX0() const { return c_x0_; }
  double CenterY0() const { return c_y0_; }
  double HeadingRad() const { return heading_rad_; }
  double Velocity() const { return velocity_; }
  double Length() const { return length_; }
  double Width() const { return width_; }
  DynObs(double x, double y, double h, double v, double l, double w)
      : c_x0_{x},
        c_y0_{y},
        heading_rad_{h},
        velocity_{v},
        length_{l},
        width_{w} {}
  DynObs RelativeTo(PointXYH pt, bool mirror) const {
    const double mirror_mult = mirror ? -1.0 : 1.0;
    const double dx = c_x0_ - pt.x_;
    const double dy = c_y0_ - pt.y_;
    const double dh = ::roahm::ToAngle(heading_rad_ - pt.h_); // TODO mirror
    const double cos_h = std::cos(pt.h_);
    const double sin_h = std::sin(pt.h_);
    const double new_x = cos_h * dx + sin_h * dy;
    const double new_y = -sin_h * dx + cos_h * dy;
    DynObs ret{new_x, 
               mirror_mult * new_y, 
               mirror_mult * dh,
               velocity_,
               length_,
               width_};
    //std::cout << "Tranforming DynObs relative to " << pt << "[mirror: " << mirror << "]\n   " << *this << "\n-> " << ret << std::endl;
    return ret;
  }

  friend std::ostream& operator<<(std::ostream& out, const DynObs& obs) {
    return std::cout << "[c_x0: " << obs.c_x0_ << "] [c_y0: " << obs.c_y0_
              << "] [h: " << obs.heading_rad_ << "] [vel: " << obs.velocity_
              << "] [length: " << obs.length_ << "] [width: " << obs.width_
              << "]";
  }
};

/// Contains zonotope representations of a set of obstacles, which have two
/// generators per obstacle
struct ObsInfo {
private:
  std::vector<DynObs> dyn_obs_;

public:
  ObsInfo RelativeTo(PointXYH pt, bool mirror) const {
    ObsInfo ret;
    for (const auto& obs : dyn_obs_) {
      ret.PushObs(obs.RelativeTo(pt, mirror));
    }
    return ret;
  }
  /// Default constructor
  ObsInfo() : dyn_obs_{} {}
  
  struct CenterXY {
    double x_;
    double y_;
  };

  struct GenXY {
    double x1_;
    double y1_;
    double x2_;
    double y2_;
  };

  void ClearAll() {
    dyn_obs_.clear();
  }

  std::size_t GetNumObs() const {
    return dyn_obs_.size();
  }

  void PushObs(DynObs obs) {
    dyn_obs_.push_back(obs);
  }

  void PrintObsConv(const int obs_idx, const ::roahm::Interval t_interval) const {
    const auto obs = dyn_obs_.at(obs_idx);
    const auto center = ComputeCenter(obs_idx, t_interval);
    const auto gens = GetGenerators(obs_idx, t_interval);
    std::cout << "Obs [" << obs_idx << "] at t = [" 
      << t_interval.Min() << ", " << t_interval.Max() << "]" << std::endl;
    std::cout << "| Obs Params: " << obs << std::endl;
    std::cout << "| Center: " << center.x_ << ", " << center.y_ << std::endl;
    std::cout << "| Gen 1: " << gens.x1_ << ", " << gens.y1_ << std::endl;
    std::cout << "| Gen 2: " << gens.x2_ << ", " << gens.y2_ << std::endl;
    
  }
  CenterXY ComputeCenter(const int obs_idx, const ::roahm::Interval t_interval) const {
    // TODO move this into GetCenter
    const auto dyn_obs = dyn_obs_.at(obs_idx);
    const double t_center = t_interval.Midpoint();
    const double velocity = dyn_obs.Velocity(); 
    const double heading_rad = dyn_obs.HeadingRad();
    
    const double dist_from_start = velocity * t_center;
    const double cos_h = std::cos(heading_rad);
    const double sin_h = std::sin(heading_rad);
    const double x0 = dyn_obs.CenterX0();
    const double y0 = dyn_obs.CenterY0();
    return {x0 + dist_from_start * cos_h, y0 + dist_from_start * sin_h};
  }

  CenterXY GetCenter(const int obs_idx, const ::roahm::Interval t_interval) const {
    //PrintObsConv(obs_idx, t_interval);
    return ComputeCenter(obs_idx, t_interval);
  }

  GenXY GetGenerators(int obs_idx, ::roahm::Interval t_interval) const {
    const auto dyn_obs = dyn_obs_.at(obs_idx);
    const double length = dyn_obs.Length();
    const double velocity = dyn_obs.Velocity();
    const double width = dyn_obs.Width();
    const double heading_rad = dyn_obs.HeadingRad();
    
    const double delta_t = t_interval.Width();
    
    const double longitudinal_gen_mag = (length + (velocity * delta_t)) / 2.0;
    const double lateral_gen_mag = width / 2.0;
    const double cos_h = std::cos(heading_rad);
    const double sin_h = std::sin(heading_rad);
    return {longitudinal_gen_mag * cos_h, longitudinal_gen_mag * sin_h,
            lateral_gen_mag * -sin_h, lateral_gen_mag * cos_h};
  }
  
  CenterXY GetSingleGenerator(int obs_idx, int gen_idx, ::roahm::Interval t_interval) const {
    assert(gen_idx >= 0);
    assert(gen_idx < 2);
    assert(obs_idx < static_cast<int>(GetNumObs()));
    auto gens = GetGenerators(obs_idx, t_interval);
    if (gen_idx == 0) {
      return {gens.x1_, gens.y1_};
    }
    return {gens.x2_, gens.y2_};
  }
  

  void PushObs(const double c_x, double c_y, double g1_x, double g1_y, double g2_x, double g2_y) {
    // TODO TODO TODO TODO TODO TODO TODO TODO
    // FIXME FIXME FIXME FIXME FIXME FIXME 
    //DEL
    //DEL
    std::cerr << "why is this called" << std::endl;
    assert(false && "this shouldn't be called");
    dyn_obs_.emplace_back(0.0, 0.0, 0.0, 0.0, 1.0, 1.0);
  }
};

/// TODO
struct ZonoInfo {
  /// The number of zonotopes, \f$ n \f$
  IndexT num_zonos_;
  /// TODO
  IndexT cumsum_num_out_zono_gens_;
  /// TODO
  std::unique_ptr<IndexT[]> num_out_zono_gens_arr_;
  /// TODO
  std::unique_ptr<IndexT[]> cum_size_arr_;
  /// TODO
  std::unique_ptr<double[]> zono_arr_;

  /// Default constructor
  ZonoInfo()
      : num_zonos_{0},
        cumsum_num_out_zono_gens_{0},
        num_out_zono_gens_arr_{},
        cum_size_arr_{},
        zono_arr_{} {}
};

/// TODO
struct Constraints {
  /// Number of elements in the a matrix
  IndexT num_a_elts_;
  /// Number of elements in the b vector
  IndexT num_b_elts_;
  /// TODO
  std::shared_ptr<double[]> a_con_arr_;
  /// TODO
  std::shared_ptr<double[]> b_con_arr_;
  /// TODO
  std::vector<IndexT> zono_startpoints_;
  /// TODO
  std::vector<IndexT> zono_obs_sizes_;
  /// Default constructor
  Constraints()
      : num_a_elts_{0},
        num_b_elts_{0},
        a_con_arr_{},
        b_con_arr_{},
        zono_startpoints_{},
        zono_obs_sizes_{} {}
};

/// Generates constraints given a reachable set, predicted \f$ (u, v, r) \f$
/// which the reachable set can be sliced at, and obstacles
/// \param vehrs the reachable set, sliced with initial conditions
/// \param obs_info the obstacles in the local frame relative to the predicted
/// pose
/// \return constraints to be used in evaluating the optimization problem
Constraints GenerateConstraints(const Vehrs& vehrs, const ObsInfo& obs_info);

/// Writes constraints in a form that is MATLAB-parseable to a std::ostream
/// \param constraints the constraints to write
/// \param o_stream the std::ostream to write to
void PrintConstraints(const Constraints& constraints, std::ostream& o_stream);

/// Writes obstacle information to a file that is MATALB-parseable
/// \param obs_info the obstacle information to write
/// \param fname the file name to write the output to
void WriteObsTestInfo(const ObsInfo& obs_info, std::string fname);

/// Prints constraints to std::cout in a MATLAB readable form
/// \param constraints the constraints to write to the file
void WriteConstraints(const Constraints& constraints);

/// Writes constraints to file that is readable by MATLAB
/// \param constraints the constraints to write to the file
void PrintConstraints(const Constraints& constraints);
}  // namespace roahm

#endif  // ROAHM_GENCON_HPP_
