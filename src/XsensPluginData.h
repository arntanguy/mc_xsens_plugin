#pragma once
#include <SpaceVecAlg/SpaceVecAlg>
#include <eigen3/Eigen/Eigen>
#include <map>
#include <string>

namespace mc_xsens_plugin
{
struct XsensData
{
  std::map<std::string, sva::PTransformd> segment_poses_;
  std::map<std::string, sva::MotionVecd> segment_vels_;
  std::map<std::string, sva::MotionVecd> segment_accs_;
  std::map<std::string, Eigen::Vector3d> CoMdata_;
};
}  // namespace mc_xsens_plugin
