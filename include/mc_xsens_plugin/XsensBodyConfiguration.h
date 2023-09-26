#pragma once
#include <mc_rtc/Configuration.h>

namespace mc_xsens_plugin
{
struct XsensBodyConfiguration
{
  std::string segmentName{};
  std::string bodyName{};
  sva::PTransformd offset = sva::PTransformd::Identity();

  void load(const mc_rtc::Configuration& config)
  {
    config("segmentName", segmentName);
    config("bodyName", bodyName);
    config("offset", offset);
  }
};
}  // namespace mc_xsens_plugin
