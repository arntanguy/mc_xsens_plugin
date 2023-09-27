#pragma once
#include <mc_rtc/Configuration.h>
#include <mc_rtc/DataStore.h>
#include <mc_xsens_plugin/XsensDataInput.h>
#include <mc_xsens_plugin/XsensSegments.h>

namespace mc_xsens_plugin
{
struct XsensDataInputDatastore : public XsensDataInput
{
  XsensDataInputDatastore(const XsensSegments& segments, const mc_rtc::DataStore& ds, const mc_rtc::Configuration& config = {});
  bool update() override;

 protected:
  const mc_rtc::DataStore& datastore_;
};
}  // namespace mc_xsens_plugin
