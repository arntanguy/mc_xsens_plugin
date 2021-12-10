#pragma once
#include <SpaceVecAlg/SpaceVecAlg>
#include <map>
#include <string>

namespace mc_xsens_plugin
{
struct XsensData
{
  std::map<std::string, sva::PTransformd> segment_poses_;
};
} // namespace mc_xsens_plugin
