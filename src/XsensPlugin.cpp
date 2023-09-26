#include "XsensPlugin.h"

#include <mc_control/GlobalPluginMacros.h>
#include <mc_rtc/logging.h>
#include <mc_xsens_plugin/XsensDataInput.h>
#include <mc_xsens_plugin/XsensDataInputDatastore.h>

#include <SpaceVecAlg/SpaceVecAlg>
#ifdef WITH_XSENS_STREAMING
#include <mc_xsens_plugin/XsensDataInputLive.h>
#endif

namespace mc_xsens_plugin
{

XsensPlugin::~XsensPlugin() = default;

void XsensPlugin::init(mc_control::MCGlobalController& gc, const mc_rtc::Configuration& config)
{
  auto& ctl = gc.controller();
  data_ = std::make_shared<XsensData>();

  auto fullConfig = config;
  if (ctl.config().has("Xsens") && ctl.config()("Xsens").has(ctl.robot().name()))
  {
    fullConfig.load(ctl.config()("Xsens")(ctl.robot().name()));
  }

  fullConfig("verbose", verbose_);
  fullConfig("bodyMappings", bodyMappings_);
  fullConfig("liveMode", liveMode_);
  fullConfig("groundingFrames", groundingFrames_);
  mc_rtc::log::info("XsensPlugin::init called with configuration:\n{}", fullConfig.dump(true, true));

  // Putting mode in datastore (true is live, false is replay), true by default
  ctl.datastore().make<bool>("XsensMode", liveMode_);

  if (liveMode_)
  {
#ifdef WITH_XSENS_STREAMING
    input_ = std::make_shared<XsensDataInputLive>(bodyMappings_, fullConfig("server"));
#else
    mc_rtc::log::error_and_throw("[XsensPlugin] LIVE mode is not supported as this plugin wasn't build with xsens_streaming library support. Please re-build the plugin to enable this feature");
#endif
  }
  else
  {
    input_ = std::make_shared<XsensDataInputDatastore>(bodyMappings_, ctl.datastore());
  }
  reset(gc);
}

void XsensPlugin::reset(mc_control::MCGlobalController& gc)
{
  mc_rtc::log::info("XsensPlugin::reset called for controller {}", gc.controller().name_);

  auto& ctl = gc.controller();
  auto& ds = ctl.datastore();
  // advertise the plugin is running
  ds.make<bool>("XsensPlugin", true);
  ds.make<bool>("XsensPlugin::Ready", false);
  ds.make<sva::PTransformd>("XsensHuman::GroundOffset", sva::PTransformd::Identity());
  // ctl.datastore().make<decltype(data_)>("XsensPlugin::Data", data_);
  ctl.datastore().make_call("XsensPlugin::GetSegmentPose",
                            [this](const std::string& segmentName)
                            { return data_->segment_poses_.at(segmentName); });
  ctl.datastore().make_call("XsensPlugin::GetSegmentVel",
                            [this](const std::string& segmentName)
                            { return data_->segment_vels_.at(segmentName); });
  ctl.datastore().make_call("XsensPlugin::GetSegmentAcc",
                            [this](const std::string& segmentName)
                            { return data_->segment_accs_.at(segmentName); });
  ctl.datastore().make_call("XsensPlugin::GetCoMpos",
                            [this]()
                            { return data_->comPosition_; });
  ctl.datastore().make_call("XsensPlugin::GetCoMvel",
                            [this]()
                            { return data_->comVelocity_; });
  ctl.datastore().make_call("XsensPlugin::GetCoMacc",
                            [this]()
                            { return data_->comAcceleration_; });
}

void XsensPlugin::before(mc_control::MCGlobalController& gc)
{
  if (!input_)
  {
    mc_rtc::log::critical("[XsensPlugin] No input how is this possible?");
    return;
  }
  auto& ctl = gc.controller();
  auto& robot = ctl.robot();
  if (input_->update())
  {
    *data_ = input_->data();

    /**
     * This ensures that the frame in-between the specified grounding frames is
     * on the ground
     */
    if (groundingFrames_.size())
    {
      auto groundingOffset = sva::PTransformd::Identity();
      auto groundingPose = sva::PTransformd::Identity();
      if (groundingFrames_.size() == 1)
      {
        groundingPose = robot.frame(groundingFrames_.front()).position();
      }
      else if (groundingFrames_.size() == 2)
      {
        groundingPose = sva::interpolate(
            robot.frame(groundingFrames_.front()).position(),
            robot.frame(groundingFrames_.back()).position(),
            0.5);
      }
      groundingOffset.translation().z() = -groundingPose.translation().z();
      for (auto& [segmentName, segmentPose] : data_->segment_poses_)
      {
        segmentPose = segmentPose * groundingOffset;
      }
    }

    gc.controller().datastore().assign("XsensPlugin::Ready", true);
  }
}

void XsensPlugin::after(mc_control::MCGlobalController& /* controller */) {}

mc_control::GlobalPlugin::GlobalPluginConfiguration XsensPlugin::configuration()
{
  mc_control::GlobalPlugin::GlobalPluginConfiguration out;
  out.should_run_before = true;
  out.should_run_after = false;
  out.should_always_run = true;
  return out;
}

}  // namespace mc_xsens_plugin

EXPORT_MC_RTC_PLUGIN("XsensPlugin", mc_xsens_plugin::XsensPlugin)
