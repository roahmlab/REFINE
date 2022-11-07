#ifndef ROAHM_FRS_LOADER_HPP_
#define ROAHM_FRS_LOADER_HPP_

#include <string>  // for string
#include <vector>  // for vector

#include "common.hpp"       // for IndexT
#include "manu_type.hpp"    // for ManuType
#include "point_xy.hpp"     // for PointXY
#include "simple_util.hpp"  // for Interval

/// @file frs_loader.hpp Contains data structures to hold the Forward Reachable
/// Sets (FRS) and load them from files.

namespace roahm {
/// Contains information about one dimension of slicing
struct Sliceable {
  /// the dimension that the slice value is along
  int dim_;
  /// the center of the parameter's slicing range
  double center_val_;
  /// the "radius" of the slicing operation
  double slc_val_;
  /// the corresponding value in the x dimension
  double x_;
  /// the corresponding value in the y dimension
  double y_;
  /// the corresponding value in the h dimension
  double h_;
  /// Constructor
  /// \param dim The dimension to slice along
  /// \param center_val the center of the parameter's slicing range
  /// \param slc_val the "radius" of the slicing operation
  /// \param x the corresponding value in the x dimension
  /// \param y the corresponding value in the y dimension
  /// \param h the corresponding value in the h dimension
  Sliceable(int dim, double center_val, double slc_val, double x, double y,
            double h)
      : dim_(dim),
        center_val_(center_val),
        slc_val_(slc_val),
        x_(x),
        y_(y),
        h_(h) {}
};

struct SlicedInfo {
  /// TODO
  double slc_x_;
  /// TODO
  double slc_y_;
  /// TODO
  double slc_h_;
  /// TODO
  double slc_val_;
  /// TODO
  double center_slc_val_;
  /// the sum along the x dimension from slicing
  double x_sliced_sum_;
  /// the sum along the y dimension from slicing
  double y_sliced_sum_;
  /// the sum along the h dimension from slicing
  double h_sliced_sum_;
  /// true iff the lambda values are all in \f$ [-1, 1] \f$
  bool lambda_ok_;
  /// whether the x and y values were sliced
  bool slc_xy_set_;
  /// TODO
  SlicedInfo()
      : slc_x_{0},
        slc_y_{0},
        slc_h_{0},
        slc_val_{0},
        center_slc_val_{0},
        x_sliced_sum_{0},
        y_sliced_sum_{0},
        h_sliced_sum_{0},
        lambda_ok_{true},
        slc_xy_set_{false} {}
};
/// TODO
struct ZonoSliceInfo {
  /// Output information after a slicing operation
  /// TODO
  std::vector<Sliceable> slc_vals_;
  /// Computes the xyh values from a slicing operation
  /// \param u the longitudinal vehicle speed [m/s] to slice at
  /// \param v the lateral vehicle speed [m/s] to slice at
  /// \param r the vehicle yaw rate [rad/s] to slice at
  /// \return the computed values from the slicing operation
  SlicedInfo Slice(double u, double v, double r) const;
};

/// A single reachable set
struct Vehrs {
  /// The xy center points of each zonotope
  std::vector<double> xy_centers_;
  /// The heading center point of each zonotope
  std::vector<double> h_centers_;
  /// The xy portion of each zonotope's generators
  std::vector<double> zono_xy_;
  /// The h portion of each zonotope's generators
  std::vector<double> zono_h_;
  /// The number of generators corresponding to each zonotope
  std::vector<IndexT> zono_sizes_;
  /// The slice values associated with each zonotope
  std::vector<ZonoSliceInfo> slc_infos_;
  /// Which time segment this FRS corresponds to (e.g. first or second half
  /// of a lane change)
  int t_eval_idx_;
  /// The trajectory parameter "radius" about the central value
  double k_rng_;

  std::vector<double> slc_x_;
  std::vector<double> slc_y_;
  std::vector<bool> slc_xy_set_;
  /// Gets the number of zonotopes in the FRS
  /// \return the number of zonotopes
  inline IndexT GetNumZonos() const { return zono_sizes_.size(); }

  inline Vehrs SliceAt(double u, double v, double r) const {
    Vehrs ret{};
    ret.xy_centers_.resize(xy_centers_.size());
    ret.h_centers_.resize(h_centers_.size());
    ret.slc_x_.resize(h_centers_.size());
    ret.slc_y_.resize(h_centers_.size());
    ret.slc_xy_set_.resize(h_centers_.size());
    for (std::size_t i = 0; i < zono_sizes_.size(); ++i) {
      const auto sliced_info = slc_infos_.at(i).Slice(u, v, r);
      const double zono_center_x =
          sliced_info.x_sliced_sum_ + xy_centers_.at(i * 2);
      const double zono_center_y =
          sliced_info.y_sliced_sum_ + xy_centers_.at(i * 2 + 1);
      const double zono_center_h = sliced_info.h_sliced_sum_ + h_centers_.at(i);
      ret.xy_centers_.at(i * 2) = zono_center_x;
      ret.xy_centers_.at(i * 2 + 1) = zono_center_y;
      ret.h_centers_.at(i) = zono_center_h;

      for (const auto& sl : slc_infos_.at(i).slc_vals_) {
        if (sl.dim_ == 11 || sl.dim_ == 12) {
          ret.slc_x_.at(i) = sl.x_;
          ret.slc_y_.at(i) = sl.y_;
          ret.slc_xy_set_.at(i) = true;
        }
      }
      assert(ret.slc_xy_set_.at(i));
    }
    ret.zono_sizes_ = zono_sizes_;
    ret.zono_h_ = zono_h_;
    ret.zono_xy_ = zono_xy_;
    ret.zono_xy_.at(0) += 1.0;
    ret.zono_xy_.at(1) += 2.0;
    ret.zono_xy_.at(3) += 3.0;
    ret.zono_xy_.at(4) += 4.0;
    ret.slc_infos_ = slc_infos_;
    return ret;
  }
};

/// TODO
struct FrsSelectInfo {
  /// the maneuver type
  ManuType manu_type_;
  /// the u0 index number
  int idxu0_;
  /// idx0 the first index of the FRS
  int idx0_;
  /// the second index of the FRS, ignored if it is a speed change
  int idx1_;
  /// true iff the frs is mirrored
  bool mirror_;
  /// Constructor
  /// \param manu_type the maneuver type
  /// \param idxu0 the u0 index number
  /// \param idx0 the first index of the FRS
  /// \param idx1 the second index of the FRS, ignored if it is a speed change
  /// \param mirror true if the FRS is mirrored
  FrsSelectInfo(ManuType manu_type, int idxu0, int idx0, int idx1, bool mirror)
      : manu_type_{manu_type},
        idxu0_{idxu0},
        idx0_{idx0},
        idx1_{idx1},
        mirror_{mirror} {}
};

/// TODO
struct FrsMega {
  // The rows/dimensions containing v0 (min/max) and r0 (min/max) in the FRS
  /// The row containing the v0 minimum in the FRS data
  constexpr static int kV0MinRow = 5 - 1;
  /// The row containing the v0 maximum in the FRS data
  constexpr static int kV0MaxRow = 6 - 1;
  /// The row containing the r0 minimum in the FRS data
  constexpr static int kR0MinRow = 7 - 1;
  /// The row containing the r0 maximum in the FRS data
  constexpr static int kR0MaxRow = 8 - 1;

  /// The set of speed change maneuvers
  std::vector<Vehrs> au_;
  /// Table containing information about speed change maneuvers
  std::vector<std::vector<double>> autb_;

  /// The set of dir change maneuvers
  std::vector<std::vector<Vehrs>> dir_;
  /// Table containing information about dir changes maneuvers
  std::vector<std::vector<std::vector<double>>> dirtb_;

  /// The set of lane change maneuvers
  std::vector<std::vector<Vehrs>> lan_;
  /// Table containing information about lane change maneuvers
  std::vector<std::vector<std::vector<double>>> lantb_;

  /// Struct containing the minimum and maximum v0 and r0 values
  struct VRMinMax {
    /// The minimum initial lateral velocity [m/s]
    double v0_min_;
    /// The maximum initial lateral velocity [m/s]
    double v0_max_;
    /// The minimum initial yaw rate[rad/s]
    double r0_min_;
    /// The maximum initial yaw rate[rad/s]
    double r0_max_;
  };

  /// Returns either the set of direction or lane changes
  /// \param manu_type the maneuver type to get, either direction or lane
  /// changes
  /// \return either the set of direction or lane changes
  const std::vector<std::vector<Vehrs>>& GetDirLanSet(ManuType manu_type) const;

  /// Checks whether a given table is populated with values or not
  /// \param manu_type the maneuver type to check
  /// \return true iff a given table is populated with values
  bool IsTablePopulated(ManuType manu_type) const;

  /// Returns the min and max v/r values for a given maneuver
  /// \param manu_type the maneuver type
  /// \param i the first index of the maneuver in its table
  /// \param j the second index of the dir change maneuver in its table
  /// (ignored for speed change)
  /// \return the min and max v/r values for a given maneuver
  VRMinMax GetMinMax(ManuType manu_type, int idx1, int idx2) const;

 private:
  /// Returns the min and max v/r values for a speed change
  /// \param i the index of the speed change maneuver in the table
  /// \return the min and max v/r values for a speed change
  VRMinMax SpdVRMinMax(int i) const;

  /// Returns the min and max v/r values for a dir change
  /// \param i the first index of the dir change maneuver in the table
  /// \param j the second index of the dir change maneuver in the table
  /// \return the min and max v/r values for a dir change
  VRMinMax DirVRMinMax(int i, int j) const;

  /// Returns the min and max v/r values for a lane change
  /// \param i the first index of the lane change maneuver in the table
  /// \param j the second index of the lane change maneuver in the table
  /// \return the min and max v/r values for a lane change
  VRMinMax LanVRMinMax(int i, int j) const;
};
/// TODO
struct FrsTotal {
  /// The initial speed ranges that each set of FRSes are compatible with
  std::vector<Interval> u0_intervals_;
  /// Sets of FRSes
  std::vector<FrsMega> megas_;
  /// The minimum initial speed
  double u0_min_;
  /// The maximum initial speed
  double u0_max_;
  /// Whether the set of FRSes was successfully loaded or not
  bool successful_;
  /// Whether only every other bin has au values
  bool alternating_au_;
  /// Returns the minimum initial speed that can be used with the FRSes
  /// \return the minimum initial speed that can be used with the FRSes
  double MinU0() const;

  /// Returns the maximum initial speed that can be used with the FRSes
  /// \return the maximum initial speed that can be used with the FRSes
  double MaxU0() const;

  /// Returns the index of the FRSes that are compatible with the given u0
  /// \param u0 the initial speed to find a set of FRSes that are compatible
  /// with
  /// \param can_clamp_u whether an out of range u0 should be clamped to the
  /// nearest applicable range. Can be useful if noisy measurements exist
  /// \return the index of the set of FRSes that are compatible with the
  /// provided u0
  long SelectU0Idx(double& u0, bool can_clamp_u) const;

  /// Checks if only every other bin has Au values
  /// \return true iff only every other bin has Au values
  bool AlternatingAu() const;

  /// Return a reachable set corresponding with the provided information
  /// \param info the indices corresponding to a given FRS
  /// \return a reachable set corresponding with the provided information
  const Vehrs& GetVehrs(const FrsSelectInfo& info) const;
};

/// Loads FRSes from a file
/// \param f_name the file name to load
/// \return the loaded FRS information, which contains a flag for whether the
/// operation was successful or not
FrsTotal LoadFrs(const std::string& f_name);

}  // namespace roahm
#endif  // ROAHM_FRS_LOADER_HPP_
