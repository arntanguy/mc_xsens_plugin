#pragma once
#include <mc_rtc/Configuration.h>

namespace mc_xsens_plugin
{
struct XsensBodyConfiguration
{
  std::string segmentName{};
  std::string bodyName{};
  sva::PTransformd offset = sva::PTransformd::Identity();
};
}  // namespace mc_xsens_plugin
