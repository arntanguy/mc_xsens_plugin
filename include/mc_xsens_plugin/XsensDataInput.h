#pragma once
#include <mc_xsens_plugin/XsensPluginData.h>
#include <mc_xsens_plugin/XsensSegments.h>

namespace mc_xsens_plugin {

struct XsensDataInput {
  XsensDataInput(const XsensSegments &segments) : segments_(segments) {}

  virtual bool update() = 0;

  const XsensData &data() const noexcept { return data_; }

protected:
  XsensSegments segments_;
  XsensData data_;
};

} // namespace mc_xsens_plugin
