#pragma once
#include <mc_rtc/Configuration.h>
#include <mc_xsens_plugin/XsensBodyConfiguration.h>

namespace mc_xsens_plugin
{
struct XsensBodyMappings
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

  void load(const mc_rtc::Configuration &config)
  {
    std::map<std::string, mc_rtc::Configuration> bodies = config("bodies");
    for (const auto &[bodyName, bodyConf] : bodies)
    {
      XsensBodyConfiguration bc;
      bc.bodyName = bodyName;
      bc.segmentName = static_cast<std::string>(bodyConf("segment"));
      bodyConf("offset", bc.offset);
      bodyConfigurations_[bodyName] = bc;
    }

    segmentNameToId_ = config("segments");
    for (const auto &seg : segmentNameToId_)
    {
      segmentIdToName_[seg.second] = seg.first;
    }
  }

  mc_rtc::Configuration save() const
  {
    // TODO unimplemented
    return mc_rtc::Configuration{};
  }

 public:
  std::map<std::string, XsensBodyConfiguration> bodyConfigurations_;  ///< Default mapping between bodies and segments
  std::map<std::string, size_t> segmentNameToId_;
  std::map<size_t, std::string> segmentIdToName_;
};
}  // namespace mc_xsens_plugin

namespace mc_rtc
{
template <>
struct ConfigurationLoader<mc_xsens_plugin::XsensBodyMappings>
{
  static mc_xsens_plugin::XsensBodyMappings load(const mc_rtc::Configuration &config)
  {
    mc_xsens_plugin::XsensBodyMappings mappings;
    mappings.load(config);
    return mappings;
  }

  static mc_rtc::Configuration save(const mc_xsens_plugin::XsensBodyMappings &object)
  {
    return object.save();
  }
};
}  // namespace mc_rtc
