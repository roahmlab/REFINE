#ifndef ROAHM_GENCON_HPP_
#define ROAHM_GENCON_HPP_

#include <jsk_recognition_msgs/PolygonArray.h>  // for PolygonArray

#include <iosfwd>  // for ostream
#include <memory>  // for unique_ptr, shared_ptr
#include <string>  // for string
#include <vector>  // for vector

#include "common.hpp"      // for IndexT
#include "frs_loader.hpp"  // for Vehrs
#include "point_xyh.hpp"   // for PointXYH

/// @file gencon.hpp Contains constraint generation code and related utilities

namespace roahm {

/// Contains zonotope representations of a set of obstacles, which have two
/// generators per obstacle
struct ObsInfo {
  /// The number of obstacles, \f$ n \f$
  IndexT num_obs_;

  /// The generators for each zonotope, only 2 generators per obstacle
  /// \f$ \begin{bmatrix}
  /// x_{00} & y_{00} & x_{01} & y_{01} & \cdots &
  /// x_{n0} & y_{n0} & x_{n1} & y_{n1} &
  /// \end{bmatrix}^\top \f$
  /// length is \f$ 4 \cdot \text{num obs} \f$
  std::vector<double> obs_arr_;

  /// The centers for each zonotope,
  /// \f$ \begin{bmatrix}
  /// x_0 & y_0 & \cdots & x_n & y_n & \vdots
  /// \end{bmatrix}^\top
  /// \f$
  std::vector<double> obs_centers_;

  /// Default constructor
  ObsInfo() : num_obs_{0}, obs_arr_{}, obs_centers_{} {}
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

/// Convert obstacles in the world frame into their zonotope representations in
/// the local frame of reference.
/// \param latest_read_obs the latest set of obstacles
/// \param local_frame the local frame pose w.r.t. the world frame
/// \param mirror true if the y-axis should be mirrored, false otherwise
/// \return the obstacles converted into their zonotope representations in the
/// local frame of reference
ObsInfo ConvertObsToZonoWithHeading(
    const jsk_recognition_msgs::PolygonArray& latest_read_obs,
    const PointXYH& local_frame, bool mirror);

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
