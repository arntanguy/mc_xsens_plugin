#pragma once
#include <mc_rtc/Configuration.h>
#include <mc_xsens_plugin/XsensBodyConfiguration.h>

namespace mc_xsens_plugin {
struct XsensBodyMappings {
  void load(const mc_rtc::Configuration &config) {
    std::map<std::string, mc_rtc::Configuration> bodies = config("bodies");
    for (const auto &[bodyName, bodyConf] : bodies) {
      XsensBodyConfiguration bc;
      bc.segmentName = static_cast<std::string>(bodyConf("segment"));
      bodyConf("offset", bc.offset);
      bodyConfigurations_[bodyName] = bc;
    }
  }

  mc_rtc::Configuration save() const {
    // TODO unimplemented
    return mc_rtc::Configuration{};
  }

  const auto &bodyConfigurations() const noexcept {
    return bodyConfigurations_;
  }

protected:
  std::map<std::string, XsensBodyConfiguration>
      bodyConfigurations_; ///< Default mapping between bodies and segments
};
} // namespace mc_xsens_plugin

namespace mc_rtc {
template <> struct ConfigurationLoader<mc_xsens_plugin::XsensBodyMappings> {
  static mc_xsens_plugin::XsensBodyMappings
  load(const mc_rtc::Configuration &config) {
    mc_xsens_plugin::XsensBodyMappings mappings;
    mappings.load(config);
    return mappings;
  }

  static mc_rtc::Configuration
  save(const mc_xsens_plugin::XsensBodyMappings &object) {
    return object.save();
  }
};
} // namespace mc_rtc
