#pragma once
#include <SpaceVecAlg/SpaceVecAlg>
#include <map>
#include <string>

namespace mc_xsens_plugin
{
struct XsensData
{
  std::map<std::string, sva::PTransformd> segment_poses_;
  std::map<std::string, sva::MotionVecd> segment_vels_;
  std::map<std::string, sva::MotionVecd> segment_accs_;
  Eigen::Vector3d comPosition_;
  Eigen::Vector3d comVelocity_;
  Eigen::Vector3d comAcceleration_;
};
}  // namespace mc_xsens_plugin
