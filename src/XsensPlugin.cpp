#include "XsensPlugin.h"

#include <mc_control/GlobalPluginMacros.h>

namespace mc_plugin
{

XsensPlugin::~XsensPlugin() = default;

void XsensPlugin::init(mc_control::MCGlobalController & controller, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("XsensPlugin::init called with configuration:\n{}", config.dump(true, true));
}

void XsensPlugin::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("XsensPlugin::reset called");
}

void XsensPlugin::before(mc_control::MCGlobalController &)
{
  mc_rtc::log::info("XsensPlugin::before");
}

void XsensPlugin::after(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("XsensPlugin::after");
}

mc_control::GlobalPlugin::GlobalPluginConfiguration XsensPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = true;
  out.should_always_run = true;
  return out;
}

} // namespace mc_plugin

EXPORT_MC_RTC_PLUGIN("XsensPlugin", mc_plugin::XsensPlugin)
