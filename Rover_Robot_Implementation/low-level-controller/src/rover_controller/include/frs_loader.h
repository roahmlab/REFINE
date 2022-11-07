#ifndef ROAHM_FRS_LOADER_H_
#define ROAHM_FRS_LOADER_H_
#include <cassert>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "bench_helper.h"
#include "common.h"
#include "point_xy.h"
#include "rmanu_type.h"
#include "simple_util.h"

namespace roahm {
struct Sliceable {
  int dim_;
  double center_val_;
  double slc_val_;
  double x_;
  double y_;
  double h_;
  Sliceable(int dim, double center_val, double slc_val, double x, double y,
            double h)
      : dim_(dim),
        center_val_(center_val),
        slc_val_(slc_val),
        x_(x),
        y_(y),
        h_(h) {}
};

struct ZonoSliceInfo {
  struct SlicedInfo {
    double slc_x_;
    double slc_y_;
    double slc_h_;
    double slc_val_;
    double center_slc_val_;
    double x_sliced_sum_;
    double y_sliced_sum_;
    double h_sliced_sum_;
    bool lambda_ok_;
    bool slc_xy_set_;
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
  std::vector<Sliceable> slc_vals_;
  SlicedInfo Slice(double u, double v, double r) const {
    SlicedInfo ret;
    ret.lambda_ok_ = true;
    ret.slc_xy_set_ = false;
    for (const auto& sliceable : slc_vals_) {
      const auto sl_dim = sliceable.dim_;
      const auto sl_val = sliceable.slc_val_;
      if (sl_dim == 11 || sl_dim == 12) {
        ret.slc_x_ = sliceable.x_;
        ret.slc_y_ = sliceable.y_;
        ret.slc_h_ = sliceable.h_;
        ret.slc_val_ = sl_val;
        ret.center_slc_val_ = sliceable.center_val_;
        ret.slc_xy_set_ = true;
      } else {
        double slice_pt = 0.0;
        if (sl_dim == 7) {
          slice_pt = u;
        } else if (sl_dim == 8) {
          slice_pt = v;
        } else if (sl_dim == 9) {
          slice_pt = r;
        } else {
          assert(false);
        }
        constexpr double kLambdaEps = 1.0e-9;
        double slice_lambda = (slice_pt - sliceable.center_val_) / sl_val;
        double abs_lambda = std::abs(slice_lambda);
        if (abs_lambda > 1.0 && abs_lambda <= 1.0 + kLambdaEps) {
          if (slice_lambda < 0) {
            slice_lambda = -1.0;
          } else {
            slice_lambda = 1.0;
          }
        }
        abs_lambda = std::abs(slice_lambda);
        if (abs_lambda > 1.0) {
          std::cout << "Slice Point Out of bounds\n";
          std::cout << "slice_pt:              " << slice_pt << std::endl;
          std::cout << "sliceable.center_val_: " << sliceable.center_val_
                    << std::endl;
          std::cout << "sl_dim: " << sl_dim << std::endl;
          std::cout << "sl_val:    " << sl_val << std::endl;
          std::cout << "sl_lambda: " << slice_lambda << std::endl;
        }
        ret.lambda_ok_ &= (std::abs(slice_lambda) <= 1.0);
        ret.x_sliced_sum_ += slice_lambda * sliceable.x_;
        ret.y_sliced_sum_ += slice_lambda * sliceable.y_;
        ret.h_sliced_sum_ += slice_lambda * sliceable.h_;
      }
    }
    return ret;
  }
};

struct Vehrs {
  std::vector<double> xy_centers_;
  std::vector<double> h_centers_;
  std::vector<double> zono_xy_;
  std::vector<double> zono_h_;
  std::vector<IndexT> zono_sizes_;
  std::vector<ZonoSliceInfo> slc_vals_;
  std::vector<PointXY> hull_;
  int t_eval_idx_;
  double k_rng_;
  IndexT GetNumZonos() const { return zono_sizes_.size(); }
};
struct FrsSelectInfo {
  RManuType manu_type_;
  int idxu0_;
  int idx0_;
  int idx1_;
  bool mirror_;
  FrsSelectInfo(RManuType manu_type, int idxu0, int idx0, int idx1, bool mirror)
      : manu_type_{manu_type},
        idxu0_{idxu0},
        idx0_{idx0},
        idx1_{idx1},
        mirror_{mirror} {}
};

struct FrsMega {
  // The rows/dimensions containing v0 (min/max) and r0 (min/max) in the FRS
  constexpr static int kV0MinRow = 5 - 1;
  constexpr static int kV0MaxRow = 6 - 1;
  constexpr static int kR0MinRow = 7 - 1;
  constexpr static int kR0MaxRow = 8 - 1;
  std::vector<Vehrs> au_;
  std::vector<std::vector<double>> autb_;
  std::vector<std::vector<Vehrs>> dir_;
  std::vector<std::vector<std::vector<double>>> dirtb_;
  std::vector<std::vector<Vehrs>> lan_;
  std::vector<std::vector<std::vector<double>>> lantb_;
  struct VRMinMax {
    double v0_min_;
    double v0_max_;
    double r0_min_;
    double r0_max_;
  };
  const std::vector<std::vector<Vehrs>>& GetDirLanSet(
      const RManuType manu_type) const {
    return manu_type == RManuType::kDirChange ? dir_ : lan_;
  }
  bool IsTablePopulated(RManuType manu_type) const {
    switch (manu_type) {
      case RManuType::kSpdChange:
        return IsPopulatedMultidim(autb_);
      case RManuType::kDirChange:
        return IsPopulatedMultidim(dirtb_);
      case RManuType::kLanChange:
        return IsPopulatedMultidim(lantb_);
      case RManuType::kNone:
      default:
        return false;
    }
  }
  VRMinMax GetMinMax(RManuType manu_type, int idx1, int idx2) const {
    switch (manu_type) {
      case RManuType::kSpdChange:
        return SpdVRMinMax(idx1);
      case RManuType::kDirChange:
        return DirVRMinMax(idx1, idx2);
      case RManuType::kLanChange:
        return LanVRMinMax(idx1, idx2);
      case RManuType::kNone:
      default:
        return SpdVRMinMax(0);
    }
  }
  VRMinMax SpdVRMinMax(const int i) const {
    return {autb_.at(kV0MinRow).at(i), autb_.at(kV0MaxRow).at(i),
            autb_.at(kR0MinRow).at(i), autb_.at(kR0MaxRow).at(i)};
  }
  VRMinMax DirVRMinMax(const int i, const int j) const {
    return {dirtb_.at(kV0MinRow).at(i).at(j), dirtb_.at(kV0MaxRow).at(i).at(j),
            dirtb_.at(kR0MinRow).at(i).at(j), dirtb_.at(kR0MaxRow).at(i).at(j)};
  }
  VRMinMax LanVRMinMax(const int i, const int j) const {
    return {lantb_.at(kV0MinRow).at(i).at(j), lantb_.at(kV0MaxRow).at(i).at(j),
            lantb_.at(kR0MinRow).at(i).at(j), lantb_.at(kR0MaxRow).at(i).at(j)};
  }
};
struct FrsTotal {
  std::vector<Interval> u0_intervals_;
  std::vector<FrsMega> megas_;
  double u0_min_;
  double u0_max_;
  bool successful_;
  bool alternating_au_;
  double MinU0() const { return u0_min_; }
  double MaxU0() const { return u0_max_; }
  long SelectU0Idx(double& u0, bool can_clamp_u) const {
    if (can_clamp_u) {
      u0 = ClampWithWarn(u0, MinU0(), MaxU0(), "U0");
    }
    const auto comp_dist_to_u0 = [u0](const Interval& a, const Interval& b) {
      return a.DistanceTo(u0) < b.DistanceTo(u0);
    };
    return static_cast<long>(MinElementIdx(u0_intervals_, comp_dist_to_u0));
  }
  long SelectU0Idx(double u0) const { return SelectU0Idx(u0, false); }
  bool AlternatingAu() const { return alternating_au_; }
  const Vehrs& GetVehrs(const FrsSelectInfo& info) const {
    const auto& curr_mega = megas_.at(info.idxu0_);
    return (info.manu_type_ == RManuType::kSpdChange)
               ? (curr_mega.au_.at(info.idx0_))
               : ((info.manu_type_ == RManuType::kDirChange)
                      ? curr_mega.dir_.at(info.idx0_).at(info.idx1_)
                      : curr_mega.lan_.at(info.idx0_).at(info.idx1_));
  }
};

bool LoadZono(std::istream& f_in, Vehrs& vehrs) {
  std::string in_str;
  IndexT num_dims = 0;
  IndexT num_gens = 0;
  f_in >> in_str >> num_dims >> num_gens;
  if (num_dims != 3) {  // XYH
    std::cerr << "num_dims != 3\n";
    return false;
  }
  vehrs.zono_sizes_.push_back(num_gens);
  for (IndexT gen_idx = 0; gen_idx < num_gens; ++gen_idx) {
    double x, y, h;
    f_in >> x >> y >> h;
    vehrs.zono_xy_.push_back(x);
    vehrs.zono_xy_.push_back(y);
    vehrs.zono_h_.push_back(h);
  }

  double c_x, c_y, c_h;
  f_in >> in_str >> c_x >> c_y >> c_h;
  if (in_str != "CENTER") {
    std::cerr << "in_str != \"CENTER\" (\"" << in_str << "\" != CENTER)\n";
    return false;
  }
  vehrs.xy_centers_.push_back(c_x);
  vehrs.xy_centers_.push_back(c_y);
  vehrs.h_centers_.push_back(c_h);
  IndexT num_slc_vals;
  f_in >> in_str >> num_slc_vals;
  if (in_str != "SLICE_VALS:") {
    std::cerr << "in_str != \"SLICE_VALS:\"\n";
    return false;
  }
  vehrs.slc_vals_.emplace_back();
  for (IndexT i = 0; i < num_slc_vals; ++i) {
    int dim;
    double center_val, slc_val, x, y, h;
    f_in >> in_str >> dim >> center_val >> slc_val >> x >> y >> h;
    if (in_str != "SLC") {
      std::cerr << "in_str != \"SLC\"\n";
      return false;
    }
    vehrs.slc_vals_.back().slc_vals_.emplace_back(dim, center_val, slc_val, x,
                                                  y, h);
  }
  return true;
}

bool LoadVehrs(std::istream& f_in, Vehrs& vehrs) {
  std::string in_str;
  IndexT num_zonos;
  f_in >> in_str;  // ({MANU_TYPE}_{IDX})
  f_in >> in_str;  // vehRS_save
  if (in_str != "vehRS_save") {
    std::cerr << "in_str != \"vehRS_save\"\n";
    return false;
  }
  int t_eval_idx = -1;
  f_in >> num_zonos >> t_eval_idx;
  if (t_eval_idx < 0) {
    std::cerr << "t_eval_idx <= 0\n";
    return false;
  }

  f_in >> in_str;
  if (in_str != "K_RNG") {
    std::cout << "in_str != \"K_RNG\"\n";
    return false;
  }
  double k_rng = 0.0;
  f_in >> k_rng;
  if (k_rng < 1.0e-2) {
    std::cout << "k_rng < 1.0e-2\n";
    return false;
  }
  std::cout << "k_rng: " << k_rng << std::endl;
  vehrs.k_rng_ = k_rng;

  f_in >> in_str;
  if (in_str != "VEHRS_CONV_HULL") {
    std::cout << "in_str != \"VEHRS_CONV_HULL\"\n";
    return false;
  }
  int hull_dim0 = 0;
  int hull_dim1 = 0;
  f_in >> hull_dim0 >> hull_dim1;
  if (hull_dim0 != 2) {
    std::cout << "hull_dim0 != 2\n";
    return false;
  }
  if (hull_dim1 < 4) {
    std::cout << "hull_dim1 < 4\n";
    return false;
  }

  {
    double tmp_val = 0.0;
    vehrs.hull_.resize(hull_dim1);
    for (int c = 0; c < hull_dim1; ++c) {
      f_in >> tmp_val;
      vehrs.hull_.at(c).x_ = tmp_val;
    }
    for (int c = 0; c < hull_dim1; ++c) {
      f_in >> tmp_val;
      vehrs.hull_.at(c).y_ = tmp_val;
    }
  }
  vehrs.t_eval_idx_ = t_eval_idx;
  for (IndexT i = 0; i < num_zonos; ++i) {
    if (!LoadZono(f_in, vehrs)) {
      std::cerr << "LoadZono failed\n";
      return false;
    }
  }
  return true;
}
template <typename T>
void CheckEqMsg(std::string a_name, std::string b_name, const T& a,
                const T& b) {
  std::cerr << a_name << " != " << b_name << " (" << a << " != " << b << ")\n";
}
bool LoadMega(std::istream& f_in, FrsMega& mega) {
  std::string in_str;
  f_in >> in_str;
  if (in_str != "M_MEGA") {
    std::cerr << "in_str != \"M_MEGA\"\n";
    return false;
  }
  f_in >> in_str;  // mega idx
  f_in >> in_str;  // (MEGA_{N})

  // Load AU
  std::string manu_type;
  IndexT au_num = 0;
  f_in >> manu_type >> au_num;
  if (manu_type != "AU") {
    std::cerr << "manu_type != \"AU\"\n";
    return false;
  }
  mega.au_.resize(au_num);
  for (IndexT i = 0; i < au_num; ++i) {
    if (!LoadVehrs(f_in, mega.au_.at(i))) {
      std::cerr << "AU LoadVehrs failed\n";
      return false;
    }
  }

  // Load AUTB
  IndexT autb_rows, autb_cols;
  f_in >> in_str >> in_str >> autb_rows >> autb_cols;
  if (in_str != "AUTB") {
    std::cerr << "in_str != \"AUTB\"\n";
    return false;
  }
  // TODO if (autb_rows != 14) {
  // TODO   std::cerr << "autb_rows != 14\n";
  // TODO   return false;
  // TODO }
  if (autb_cols != au_num) {
    std::cerr << "autb_cols != au_num\n";
    return false;
  }
  mega.autb_.resize(autb_rows);
  for (IndexT autb_r = 0; autb_r < autb_rows; ++autb_r) {
    for (IndexT autb_c = 0; autb_c < autb_cols; ++autb_c) {
      double tmp_val;
      if (!(f_in >> tmp_val)) {
        std::cerr << "!(f_in >> tmp_val)\n";
        return false;
      }
      mega.autb_.at(autb_r).push_back(tmp_val);
    }
  }
  if (mega.autb_.size() != autb_rows) {
    CheckEqMsg("autb size", "autb rows", mega.autb_.size(), autb_rows);
    // std::cerr << "mega.autb_.size() != autb_rows * autb_cols\n";
    return false;
  }
  for (auto& autb_row : mega.autb_) {
    if (autb_row.size() != autb_cols) {
      CheckEqMsg("autb row size", "autb cols", autb_row.size(), autb_cols);
      return false;
    }
  }

  // Load DIR
  f_in >> in_str >> in_str;
  if (in_str != "DIR") {
    std::cerr << "in_str != \"DIR\"\n";
    return false;
  }
  IndexT dir_rows, dir_cols;
  f_in >> dir_rows >> dir_cols;
  if (not(dir_rows > 0)) {
    std::cerr << "check (dir_rows > 0) failed\n";
    return false;
  }

  mega.dir_.resize(dir_rows);
  for (auto& dir_column : mega.dir_) {
    dir_column.resize(dir_cols);
  }
  for (IndexT dir_r = 0; dir_r < dir_rows; ++dir_r) {
    for (IndexT dir_c = 0; dir_c < dir_cols; ++dir_c) {
      if (!LoadVehrs(f_in, mega.dir_.at(dir_r).at(dir_c))) {
        std::cerr << "DIR LoadVehrs failed\n";
        return false;
      }
    }
  }

  // Load DIRTB
  IndexT dirtb_dim0, dirtb_dim1, dirtb_dim2;
  f_in >> in_str >> in_str >> dirtb_dim0 >> dirtb_dim1 >> dirtb_dim2;

  // Verify things
  if (in_str != "DIRTB") {
    std::cerr << "in_str != \"DIRTB\"\n";
    return false;
  }
  if (dirtb_dim2 > 0) {
    if (dirtb_dim0 != 14) {
      std::cerr << "dirtb_dim1 != 14\n";
      return false;
    }
    if (dirtb_dim1 != dir_rows) {
      std::cerr << "dirtb_dim1 != dir_rows\n";
      return false;
    }
  }
  if (dirtb_dim2 != dir_cols) {
    std::cerr << "dirtb_dim2 != dir_cols\n";
    return false;
  }

  mega.dirtb_.resize(dirtb_dim0);
  for (IndexT dirtb_i = 0; dirtb_i < dirtb_dim0; ++dirtb_i) {
    mega.dirtb_.at(dirtb_i).resize(dirtb_dim1);
    for (IndexT dirtb_j = 0; dirtb_j < dirtb_dim1; ++dirtb_j) {
      for (IndexT dirtb_k = 0; dirtb_k < dirtb_dim2; ++dirtb_k) {
        double tmp_val;
        f_in >> tmp_val;
        mega.dirtb_.at(dirtb_i).at(dirtb_j).push_back(tmp_val);
      }
    }
  }
  if (mega.dirtb_.size() != dirtb_dim0) {
    std::cerr << "mega.dirtb_.size() != dirtb_dim0\n";
    return false;
  }
  for (const auto& dirtb_col : mega.dirtb_) {
    if (dirtb_col.size() != dirtb_dim1) {
      std::cerr << "mega.dirtb_.at(i).size() != dirtb_dim1\n";
      return false;
    }
    for (const auto& dirtb_t0 : dirtb_col) {
      if (dirtb_t0.size() != dirtb_dim2) {
        std::cerr << "mega.dirtb_.at(i).at(j).size() != dirtb_dim2\n";
        return false;
      }
    }
  }

  // Load LAN
  f_in >> in_str >> in_str;
  if (in_str != "LAN") {
    std::cerr << "in_str != \"LAN\"\n";
    return false;
  }
  IndexT lan_rows, lan_cols;
  f_in >> lan_rows >> lan_cols;

  if (not(lan_rows > 0)) {
    std::cerr << "check (lan_rows > 0) failed\n";
    return false;
  }

  mega.lan_.resize(lan_rows);
  for (auto& lan_column : mega.lan_) {
    lan_column.resize(lan_cols);
  }
  for (IndexT lan_r = 0; lan_r < lan_rows; ++lan_r) {
    for (IndexT lan_c = 0; lan_c < lan_cols; ++lan_c) {
      if (!LoadVehrs(f_in, mega.lan_.at(lan_r).at(lan_c))) {
        std::cerr << "LAN LoadVehrs failed\n";
        return false;
      }
    }
  }

  // Load LANTB
  IndexT lantb_dim0, lantb_dim1, lantb_dim2;
  f_in >> in_str >> in_str >> lantb_dim0 >> lantb_dim1 >> lantb_dim2;
  if (in_str != "LANTB") {
    std::cerr << "in_str != \"LANTB\"\n";
    return false;
  }
  if (lantb_dim2 > 0) {
    if (lantb_dim0 != 14) {
      std::cerr << "lantb_dim0 != 14\n";
      return false;
    }
    if (lantb_dim1 != lan_rows) {
      std::cerr << "lantb_dim1 != lan_rows\n";
      return false;
    }
  }
  if (lantb_dim2 != lan_cols) {
    std::cerr << "lantb_dim2 != lan_cols\n";
    return false;
  }
  mega.lantb_.resize(lantb_dim0);
  for (IndexT lantb_i = 0; lantb_i < lantb_dim0; ++lantb_i) {
    mega.lantb_.at(lantb_i).resize(lantb_dim1);
    for (IndexT lantb_j = 0; lantb_j < lantb_dim1; ++lantb_j) {
      for (IndexT lantb_k = 0; lantb_k < lantb_dim2; ++lantb_k) {
        double tmp_val;
        f_in >> tmp_val;
        mega.lantb_.at(lantb_i).at(lantb_j).push_back(tmp_val);
      }
    }
  }
  if (mega.lantb_.size() != lantb_dim0) {
    std::cerr << "mega.lantb_.size() != lantb_dim0 * lantb_dim1 * lantb_dim2\n";
    return false;
  }
  for (auto& lantb_row : mega.lantb_) {
    if (lantb_row.size() != lantb_dim1) {
      std::cerr << "lantb_row.size() != lantb_dim1\n";
      return false;
    }
    for (auto& lantb_t0 : lantb_row) {
      if (lantb_t0.size() != lantb_dim2) {
        std::cerr << "lantb_t0.size() != lantb_dim2\n";
        return false;
      }
    }
  }
  return true;
}

bool LoadU0Intervals(std::istream& f_in, std::vector<Interval>& u0_intervals) {
  std::string in_str = "";
  int u0_rows = 0;
  int u0_cols = 0;
  f_in >> in_str >> u0_rows >> u0_cols;
  if (in_str != "u0_intervals") {
    std::cerr << "in_str != \"u0_intervals\"\n";
    return false;
  }
  if (u0_rows != 2) {
    std::cerr << "u0_rows != 2\n";
    return false;
  }
  u0_intervals.resize(u0_cols);
  for (int i = 0; i < u0_cols; ++i) {
    double tmp_val_min;
    f_in >> tmp_val_min;
    u0_intervals.at(i).min_ = tmp_val_min;
  }
  for (int i = 0; i < u0_cols; ++i) {
    double tmp_val_max;
    f_in >> tmp_val_max;
    u0_intervals.at(i).max_ = tmp_val_max;
  }
  return true;
}

void FrsFailureMsg(std::string f_name) {
  std::cerr << "Failed to load FRS '" << f_name << "'\n";
}

FrsTotal LoadFrs(std::string f_name) {
  std::cout << "Loading FRS '" << f_name << "'...\n";
  std::ifstream f_in(f_name);
  std::string in_str;
  IndexT num_megas = 0;
  FrsTotal frs_total;
  frs_total.successful_ = true;
  f_in >> in_str >> frs_total.u0_min_ >> frs_total.u0_max_;
  if (in_str != "MINMAX_U0") {
    std::cerr << "in_str != \"MINMAX_U0\"\n";
    frs_total.successful_ = false;
    return frs_total;
  }
  f_in >> in_str >> frs_total.alternating_au_;
  if (in_str != "ALTERNATING_AU") {
    std::cerr << "in_str != \"ALTERNATING_AU\"\n";
    frs_total.successful_ = false;
    return frs_total;
  }
  // Set u0's
  f_in >> in_str >> num_megas;
  if (in_str != "NUM_MEGAS") {
    std::cerr << "in_str != \"NUM_MEGAS\"\n";
    frs_total.successful_ = false;
    return frs_total;
  }
  std::cout << "NUM MEGAS: " << num_megas << std::endl;
  if (num_megas <= 0) {
    std::cerr << "num_megas <= 0" << std::endl;
    frs_total.successful_ = false;
    FrsFailureMsg(f_name);
    return frs_total;
  }
  frs_total.megas_.resize(num_megas);
  for (IndexT i = 0; i < num_megas; ++i) {
    if (!LoadMega(f_in, frs_total.megas_.at(i))) {
      std::cerr << "LoadMega failed\n";
      frs_total.successful_ = false;
      FrsFailureMsg(f_name);
      return frs_total;
    }
  }
  if (!LoadU0Intervals(f_in, frs_total.u0_intervals_)) {
    std::cerr << "Load u0 intervals failed.\n";
    frs_total.successful_ = false;
    return frs_total;
  }
  f_in.close();
  if (frs_total.u0_intervals_.size() != frs_total.megas_.size()) {
    std::cerr << "frs_total.u0_intervals_.size() != frs_total.megas_.size()\n";
    FrsFailureMsg(f_name);
    frs_total.successful_ = false;
    return frs_total;
  }
  if (frs_total.successful_) {
    std::cout << "Successfully loaded '" << f_name << "'\n";
    std::cout << "FRS MEGAS: " << frs_total.megas_.size() << std::endl;
  } else {
    FrsFailureMsg(f_name);
  }
  return frs_total;
}
}  // namespace roahm
#endif  // ROAHM_FRS_LOADER_H_
