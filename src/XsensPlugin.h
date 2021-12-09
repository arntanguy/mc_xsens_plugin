/*
 * Copyright 2021 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#pragma once

#include <mc_control/GlobalPlugin.h>

struct UdpServer;

namespace mc_plugin
{

struct XsensPlugin : public mc_control::GlobalPlugin
{
  void init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config) override;

  void reset(mc_control::MCGlobalController & controller) override;

  void before(mc_control::MCGlobalController &) override;

  void after(mc_control::MCGlobalController & controller) override;

  mc_control::GlobalPlugin::GlobalPluginConfiguration configuration() override;

  ~XsensPlugin() override;
  
  inline const std::string & segmentName(size_t id)
  {
    return segmentIdToName_.at(id);
  }

  inline size_t segmentId(const std::string & name) const
  {
    return segmentNameToId_.at(name);
  }

private:
  std::shared_ptr<UdpServer> server_;
  std::map<std::string, size_t> segmentNameToId_;
  std::map<size_t, std::string> segmentIdToName_;
};

} // namespace mc_plugin
