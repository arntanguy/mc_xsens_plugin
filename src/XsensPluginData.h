#pragma once
#include <SpaceVecAlg/SpaceVecAlg>
#include <map>
#include <string>
#include <eigen3/Eigen/Eigen>

namespace mc_xsens_plugin
{
struct XsensData
{
  std::map<std::string, sva::PTransformd> segment_poses_;
  std::map<std::string, Eigen::Vector3d> CoMdata_;
};
} // namespace mc_xsens_plugin
