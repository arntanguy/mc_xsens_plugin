#pragma once
#include <mc_rtc/Configuration.h>

namespace mc_xsens_plugin
{

struct XsensSegments
{
  inline const std::string &segmentName(size_t id)
  {
    return segmentIdToName_.at(id);
  }

  inline size_t segmentId(const std::string &name) const
  {
    return segmentNameToId_.at(name);
  }

  inline bool hasSegment(const std::string &segmentName) const noexcept
  {
    return segmentNameToId_.count(segmentName) > 0;
  }

  inline const auto &segmentNameToId() const noexcept
  {
    return segmentNameToId_;
  }

  void load(const mc_rtc::Configuration &config)
  {
    segmentNameToId_ = config;
    for (const auto &seg : segmentNameToId_)
    {
      segmentIdToName_[seg.second] = seg.first;
    }
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration config;
    config.push(segmentNameToId_);
    return config;
  }

 protected:
  std::map<std::string, size_t> segmentNameToId_;
  std::map<size_t, std::string> segmentIdToName_;
};
}  // namespace mc_xsens_plugin

namespace mc_rtc
{
template <>
struct ConfigurationLoader<mc_xsens_plugin::XsensSegments>
{
  static mc_xsens_plugin::XsensSegments load(const mc_rtc::Configuration &config)
  {
    mc_xsens_plugin::XsensSegments mappings;
    mappings.load(config);
    return mappings;
  }

  static mc_rtc::Configuration save(const mc_xsens_plugin::XsensSegments &object)
  {
    return object.save();
  }
};
}  // namespace mc_rtc
