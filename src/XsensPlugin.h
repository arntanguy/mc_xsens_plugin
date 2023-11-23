/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>
#include <mc_xsens_plugin/XsensBodyMappings.h>
#include <mc_xsens_plugin/XsensDataInput.h>

namespace mc_xsens_plugin
{

struct XsensPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~XsensPlugin() override;

  XsensData & data() { return *data_; }

  const XsensData & data() const { return *data_; }

private:
  std::shared_ptr<XsensDataInput> input_;
  std::shared_ptr<XsensData> rawInputData_;
  std::shared_ptr<XsensData> data_;
  bool verbose_ = false;
  bool liveMode_ = true; // by default true, live xsens reading
  bool logData_ = true;

  bool debugmode_ = false;
};

} // namespace mc_xsens_plugin
