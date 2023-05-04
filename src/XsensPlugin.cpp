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

  auto & ctl = gc.controller();
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

  // Putting mode in datastore (true is live, false is replay), true by default
  auto & mode = ctl.datastore().make<bool>("XsensMode", liveMode_);

  if (ctl.config()("Xsens").has("liveMode"))
  {
    liveMode_ = ctl.config()("Xsens")("liveMode");
    mode = liveMode_;
  }
  
  
  // advertise the plugin is running
  ctl.datastore().make<bool>("XsensPlugin", true);

  if (liveMode_)
  {
  server_.reset(new UdpServer(host, port));
  auto & data = ctl.datastore().make<XsensData>("XsensPlugin::Data");
  ctl.datastore().make_call("XsensPlugin::GetSegmentPose",
                            [&data](const std::string & segmentName) { return data.segment_poses_.at(segmentName); });  
  ctl.datastore().make_call("XsensPlugin::GetSegmentVel",
                            [&data](const std::string & segmentName) { return data.segment_vels_.at(segmentName); });  
  ctl.datastore().make_call("XsensPlugin::GetSegmentAcc",
                            [&data](const std::string & segmentName) { return data.segment_accs_.at(segmentName); });                                          
  ctl.datastore().make_call("XsensPlugin::GetCoMpos",
                            [&data]() { return data.CoMdata_.at("pose"); });
  ctl.datastore().make_call("XsensPlugin::GetCoMvel",
                            [&data]() { return data.CoMdata_.at("velocity"); });
  ctl.datastore().make_call("XsensPlugin::GetCoMacc",
                            [&data]() { return data.CoMdata_.at("acceleration"); });
  }
  else
  {
    ctl.datastore().make_call("XsensPlugin::GetSegmentPose",
                              [&ctl, this](const std::string & segmentName) { std::string linkName;
                                                                              for(const auto & body: bodyConfigurations_)
                                                                              {
                                                                                if (body.second.segmentName == segmentName)
                                                                                  {
                                                                                    linkName = body.first;
                                                                                  }
                                                                              }
                                                                            return ctl.datastore().get<sva::PTransformd>("ReplayPlugin::GetSegmentPose::"+linkName); });  
    ctl.datastore().make_call("XsensPlugin::GetSegmentVel",
                              [&ctl, this](const std::string & segmentName) { std::string linkName;
                                                                              for(const auto & body: bodyConfigurations_)
                                                                              {
                                                                                if (body.second.segmentName == segmentName)
                                                                                  {
                                                                                    linkName = body.first;
                                                                                  }
                                                                              }
                                                                              sva::MotionVecd segVel = Eigen::Vector6d(ctl.datastore().get<Eigen::VectorXd>("ReplayPlugin::GetSegmentVel::"+linkName));
                                                                              return segVel;
                                                                              });  
    ctl.datastore().make_call("XsensPlugin::GetSegmentAcc",
                              [&ctl, this](const std::string & segmentName) { std::string linkName;
                                                                              for(const auto & body: bodyConfigurations_)
                                                                              {
                                                                                if (body.second.segmentName == segmentName)
                                                                                  {
                                                                                    linkName = body.first;
                                                                                  }
                                                                              }
                                                                              sva::MotionVecd segAcc = Eigen::Vector6d(ctl.datastore().get<Eigen::VectorXd>("ReplayPlugin::GetSegmentAcc::"+linkName));
                                                                              return segAcc;
                                                                            });                                          
    ctl.datastore().make_call("XsensPlugin::GetCoMpos",
                              [&ctl]() { return ctl.datastore().get<Eigen::Vector3d>("ReplayPlugin::GetCoMpos");  });
    ctl.datastore().make_call("XsensPlugin::GetCoMvel",
                              [&ctl]() { return ctl.datastore().get<Eigen::Vector3d>("ReplayPlugin::GetCoMvel"); });
    ctl.datastore().make_call("XsensPlugin::GetCoMacc",
                              [&ctl]() { return ctl.datastore().get<Eigen::Vector3d>("ReplayPlugin::GetCoMacc"); });
  }
  

}

void XsensPlugin::reset(mc_control::MCGlobalController & controller)
{
  mc_rtc::log::info("XsensPlugin::reset called");
}

void XsensPlugin::before(mc_control::MCGlobalController & gc)
{
  if (liveMode_)
  {  
    auto & ctl = gc.controller();
    auto & data = ctl.datastore().get<XsensData>("XsensPlugin::Data");
    auto quaternions = server_->quaternions();
    auto angularKin = server_->angularSegmentKinematics();
    auto linearKin = server_->linearSegmentKinematics();
    auto grounding_offset = sva::PTransformd::Identity();
    try
    {
      grounding_offset = ctl.datastore().get<sva::PTransformd>("XsensHuman::GetGroundOffset"); 
    }
    catch(...)
    {
      mc_rtc::log::error("No datastore value for grounding offset");
    }

    // creating pose (sva::PTransformd) for each segment
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
      // applying fixed offset from configuration
      auto corrected_segpose = segpose * segoffset;
      // applying floating base offset from xsens error (displacement from feet to ground)
      auto good_pose = corrected_segpose * grounding_offset;
      data.segment_poses_[name] = good_pose;
    }

    // creating angular elements of velocity and acceleration (sva::MotionVecd)
    for(const auto & ang : angularKin)
    {
      Eigen::Vector3d angulVel{ang.angularVeloc[0], ang.angularVeloc[1], ang.angularVeloc[2]};
      Eigen::Vector3d angulAcc{ang.angularAccel[0], ang.angularAccel[1], ang.angularAccel[2]};
      const auto & name = segmentName(ang.segmentId);
      if(verbose_)
      {
        mc_rtc::log::info("Received angular kinematics message:\n\tSegment ID: {}\n\tSegment Name: "
                          "{}\n\tAngular Velocity: {}\n\tAngular Acceleration: {}",
                          ang.segmentId, name, angulVel.transpose(), angulAcc.transpose());                 
      }
      data.segment_vels_[name].angular() = Eigen::Vector3d::Zero(); //angulVel;
      data.segment_accs_[name].angular() = Eigen::Vector3d::Zero(); //angulAcc;
    }

    // creating linear elements of velocity and acceleration (sva::MotionVecd)
    for(const auto & lin : linearKin)
    {
      Eigen::Vector3d linearVel{lin.velocity[0], lin.velocity[1], lin.velocity[2]};
      Eigen::Vector3d linearAcc{lin.acceleration[0], lin.acceleration[1], lin.acceleration[2]};
      const auto & name = segmentName(lin.segmentId);
      if(verbose_)
      {
        mc_rtc::log::info("Received linear kinematics message:\n\tSegment ID: {}\n\tSegment Name: "
                          "{}\n\tLinear Velocity: {}\n\tLinear Acceleration: {}",
                          lin.segmentId, name, linearVel.transpose(), linearAcc.transpose());
      }
      data.segment_vels_[name].linear() = linearVel;
      data.segment_accs_[name].linear() = linearAcc;
    }

    auto CoMdata = server_->comData();
    data.CoMdata_["pose"] = Eigen::Vector3d{CoMdata.pos[0], CoMdata.pos[1], CoMdata.pos[2]};
    data.CoMdata_["velocity"] = Eigen::Vector3d{CoMdata.vel[0], CoMdata.vel[1], CoMdata.vel[2]};
    data.CoMdata_["acceleration"] = Eigen::Vector3d{CoMdata.acc[0], CoMdata.acc[1], CoMdata.acc[2]};
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
