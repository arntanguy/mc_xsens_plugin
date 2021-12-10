#include "XsensPlugin.h"

#include <mc_control/GlobalPluginMacros.h>
#include "XsensPluginData.h"
#include <xsens_streaming/udpserver.h>

namespace mc_xsens_plugin
{

XsensPlugin::~XsensPlugin() = default;

void XsensPlugin::init(mc_control::MCGlobalController & gc, const mc_rtc::Configuration & config)
{
  mc_rtc::log::info("XsensPlugin::init called with configuration:\n{}", config.dump(true, true));
  config("verbose", verbose_);
  auto host = config("host", std::string{"localhost"});
  auto port = config("port", 9763);
  segmentNameToId_ = config("segments");
  for(const auto & seg : segmentNameToId_)
  {
    segmentIdToName_[seg.second] = seg.first;
  }
  server_.reset(new UdpServer(host, port));
  auto & ctl = gc.controller();
  ctl.datastore().make<bool>("XsensPlugin", true);
  auto & data = ctl.datastore().make<XsensData>("XsensPlugin::Data");
  ctl.datastore().make_call("XsensPlugin::GetSegmentPose",
                            [&data](const std::string & segmentName) { return data.segment_poses_.at(segmentName); });
}

void XsensPlugin::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("XsensPlugin::reset called");
}

void XsensPlugin::before(mc_control::MCGlobalController & gc)
{
  auto & ctl = gc.controller();
  auto & data = ctl.datastore().get<XsensData>("XsensPlugin::Data");
  auto quaternions = server_->quaternions();
  for(const auto & quat : quaternions)
  {
    Eigen::Vector3d pos{quat.position[0], quat.position[1], quat.position[2]};
    Eigen::Quaterniond q{quat.orientation[0], quat.orientation[1], quat.orientation[2], quat.orientation[3]};
    const auto & name = segmentName(quat.segmentId);
    if(verbose_)
    {
      mc_rtc::log::info("Received quaternion message:\n\tType: Quaternion\n\tSegment ID: {}\n\tSegment Name: "
                        "{}\n\tPosition: {}\n\tOrientation: {:.3f} {:.3f} {:.3f} {:.3f}",
                        quat.segmentId, name, pos.transpose(), q.w(), q.x(), q.y(), q.z());
    }
    data.segment_poses_[name] = sva::PTransformd{q.inverse(), pos};
  }
}

void XsensPlugin::after(mc_control::MCGlobalController & controller) {}

mc_control::GlobalPlugin::GlobalPluginConfiguration XsensPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

} // namespace mc_xsens_plugin

EXPORT_MC_RTC_PLUGIN("XsensPlugin", mc_xsens_plugin::XsensPlugin)
