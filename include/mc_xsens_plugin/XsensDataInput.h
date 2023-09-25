#pragma once
#include <mc_xsens_plugin/XsensBodyMappings.h>
#include <mc_xsens_plugin/XsensPluginData.h>

namespace mc_xsens_plugin
{

struct XsensDataInput
{
  XsensDataInput(const XsensBodyMappings& bodyConfigs)
      : bodyMappings_(bodyConfigs)
  {
  }

  virtual bool update() = 0;

  const XsensData& data() const noexcept
  {
    return data_;
  }

 protected:
  XsensBodyMappings bodyMappings_;
  XsensData data_;
};

}  // namespace mc_xsens_plugin
