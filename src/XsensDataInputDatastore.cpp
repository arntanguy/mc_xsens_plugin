#include <mc_rtc/DataStore.h>
#include <mc_rtc/logging.h>
#include <mc_xsens_plugin/XsensDataInputDatastore.h>

#include "mc_xsens_plugin/XsensSegments.h"

namespace mc_xsens_plugin
{
XsensDataInputDatastore::XsensDataInputDatastore(const XsensSegments& segments, const mc_rtc::DataStore& ds, const mc_rtc::Configuration& config)
    : XsensDataInput(segments), datastore_(ds)
{
}

bool XsensDataInputDatastore::update()
{
  XsensData recvData;
  for (const auto& [segmentName, segmentId] : segments_.segmentNameToId())
  {
    auto& ds = datastore_;
    if (ds.has("ReplayPlugin::GetSegmentPose::" + segmentName))
    {
      recvData.segment_poses_[segmentName] = ds.get<sva::PTransformd>("ReplayPlugin::GetSegmentPose::" + segmentName);
    }
    else
    {
      /* mc_rtc::log::error("No key {}", "ReplayPlugin::GetSegmentPose::" + segmentName); */
    }

    if (ds.has("ReplayPlugin::GetSegmentVel::" + segmentName))
    {
      recvData.segment_vels_[segmentName] = static_cast<Eigen::Vector6d>(ds.get<Eigen::VectorXd>("ReplayPlugin::GetSegmentVel::" + segmentName));
    }

    if (ds.has("ReplayPlugin::GetSegmentAcc::" + segmentName))
    {
      recvData.segment_accs_[segmentName] = static_cast<Eigen::Vector6d>(ds.get<Eigen::VectorXd>("ReplayPlugin::GetSegmentAcc::" + segmentName));
    }
    if (ds.has("ReplayPlugin::GetCoMpos"))
    {
      recvData.comPosition_ = ds.get<Eigen::Vector3d>("ReplayPlugin::GetCoMpos");
    }
    if (ds.has("ReplayPlugin::GetCoMvel"))
    {
      recvData.comVelocity_ = ds.get<Eigen::Vector3d>("ReplayPlugin::GetCoMvel");
    }

    if (ds.has("ReplayPlugin::GetCoMacc"))
    {
      recvData.comAcceleration_ = ds.get<Eigen::Vector3d>("ReplayPlugin::GetCoMacc");
    }
  }
  if (!recvData.segment_poses_.empty())
  {
    data_ = recvData;
  }
  return data_.segment_poses_.size();
}
}  // namespace mc_xsens_plugin
