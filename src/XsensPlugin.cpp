#include "XsensPlugin.h"

/** Work-around for C++17 while OpenRTM is not updated to remove the throw(...) specification */
#  if __cplusplus >= 201703L
/** Include C++ headers that won't work well without the specification */
#    include <iostream>
#    include <optional>
#    include <string>
#    define throw(...)
#  endif
#include <xsens_streaming/udpserver.h>
#  if __cplusplus >= 201703L
#    undef throw
#  endif
#include <mc_control/GlobalPluginMacros.h>
#include "XsensPluginData.h"


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
  ctl.datastore().make_call("XsensPlugin::GetCoMpos",
                            [&data]() { return data.CoMdata_.at("pose"); });
  ctl.datastore().make_call("XsensPlugin::GetCoMvel",
                            [&data]() { return data.CoMdata_.at("velocity"); });
  ctl.datastore().make_call("XsensPlugin::GetCoMacc",
                            [&data]() { return data.CoMdata_.at("acceleration"); });


  // TODO: make it modular (robot name as argument etc. See XsensHuman state)
  auto robotConfig = static_cast<std::map<std::string, mc_rtc::Configuration>>(ctl.config()("Xsens")("human"));
  for(const auto & bodyConfig : robotConfig)
  {
    const auto & bodyName = bodyConfig.first;
    const auto & bodyConf = bodyConfig.second;
    bodyConfigurations_[bodyName] = XsensBodyConfiguration{};
    auto & bodyC = bodyConfigurations_[bodyName];
    bodyC.bodyName = bodyName;
    bodyC.segmentName = static_cast<std::string>(bodyConf("segment"));
    bodyConf("offset", bodyC.offset);
    
  }

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
    sva::PTransformd segoffset = sva::PTransformd::Identity();
    for(const auto & body: bodyConfigurations_)
    {
      if (body.second.segmentName == name)
      {
        segoffset = body.second.offset;
      }
    }
    auto segpose = sva::PTransformd{q.inverse(), pos};
    data.segment_poses_[name] = segoffset * segpose;
  }

  auto CoMdata = server_->comData();
  data.CoMdata_["pose"] = Eigen::Vector3d{CoMdata.pos[0], CoMdata.pos[1], CoMdata.pos[2]};
  data.CoMdata_["velocity"] = Eigen::Vector3d{CoMdata.vel[0], CoMdata.vel[1], CoMdata.vel[2]};
  data.CoMdata_["acceleration"] = Eigen::Vector3d{CoMdata.acc[0], CoMdata.acc[1], CoMdata.acc[2]};
  
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
