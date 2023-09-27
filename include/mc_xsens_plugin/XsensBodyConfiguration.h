#pragma once
#include <mc_rtc/Configuration.h>

#include <iostream>

namespace mc_xsens_plugin
{
struct XsensBodyConfiguration
{
  std::string segmentName{};
  sva::PTransformd offset = sva::PTransformd::Identity();
  double weight = 1000;
  double stiffness = 100;
  bool flatBody = false;  ///< When true, make sure that the body is flat w.r.t ground

  void load(const mc_rtc::Configuration &config)
  {
    config("segment", segmentName);
    config("offset", offset);
    config("weight", weight);
    config("stiffness", stiffness);
    config("flatBody", flatBody);
  }

  mc_rtc::Configuration save() const
  {
    mc_rtc::Configuration c;
    c.add("segment", segmentName);
    c.add("offset", offset);
    c.add("weight", weight);
    c.add("stiffness", stiffness);
    c.add("flatBody", flatBody);
    return c;
  }
};
}  // namespace mc_xsens_plugin

namespace mc_rtc
{
template <>
struct ConfigurationLoader<mc_xsens_plugin::XsensBodyConfiguration>
{
  static mc_xsens_plugin::XsensBodyConfiguration load(const mc_rtc::Configuration &config)
  {
    mc_xsens_plugin::XsensBodyConfiguration mappings;
    mappings.load(config);
    return mappings;
  }

  static mc_rtc::Configuration save(const mc_xsens_plugin::XsensBodyConfiguration &object)
  {
    return object.save();
  }
};
}  // namespace mc_rtc
