#pragma once
#include <mc_rtc/Configuration.h>
#include <mc_xsens_plugin/XsensDataInput.h>

#include "mc_xsens_plugin/XsensSegments.h"

struct UdpServer;

namespace mc_xsens_plugin {
struct XsensDataInputLive : public XsensDataInput {
  XsensDataInputLive(const XsensSegments &segments,
                     const mc_rtc::Configuration &config);

  bool update() override;

protected:
  std::string host_{"localhost"};
  uint16_t port_{9763};
  std::shared_ptr<UdpServer> server_;
};
} // namespace mc_xsens_plugin
