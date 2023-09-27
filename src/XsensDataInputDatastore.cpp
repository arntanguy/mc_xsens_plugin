#include <mc_rtc/DataStore.h>
#include <mc_rtc/logging.h>
#include <mc_xsens_plugin/XsensDataInputDatastore.h>

namespace mc_xsens_plugin
{
XsensDataInputDatastore::XsensDataInputDatastore(const XsensBodyMappings& bodyMappings, const mc_rtc::DataStore& ds, const mc_rtc::Configuration& config)
    : XsensDataInput(bodyMappings), datastore_(ds)
{
}

bool XsensDataInputDatastore::update()
{
  bool success = true;
  for (const auto& bodyMapping : bodyMappings_.bodyConfigurations())
  {
    const auto& bodyName = bodyMapping.first;
    const auto& segmentName = bodyMapping.second.segmentName;

    auto& ds = datastore_;
    if (ds.has("ReplayPlugin::GetSegmentPose::" + bodyName))
    {
      data_.segment_poses_[segmentName] = ds.get<sva::PTransformd>("ReplayPlugin::GetSegmentPose::" + bodyName);
    }
    else
    {
      // mc_rtc::log::error("No key {}", "ReplayPlugin::GetSegmentPose::" + bodyName);
      success = false;
    }

    if (ds.has("ReplayPlugin::GetSegmentVel::" + bodyName))
    {
      data_.segment_vels_[segmentName] = static_cast<Eigen::Vector6d>(ds.get<Eigen::VectorXd>("ReplayPlugin::GetSegmentVel::" + bodyName));
    }

    if (ds.has("ReplayPlugin::GetSegmentAcc::" + bodyName))
    {
      data_.segment_accs_[segmentName] = static_cast<Eigen::Vector6d>(ds.get<Eigen::VectorXd>("ReplayPlugin::GetSegmentAcc::" + bodyName));
    }
    if (ds.has("ReplayPlugin::GetCoMpos"))
    {
      data_.comPosition_ = ds.get<Eigen::Vector3d>("ReplayPlugin::GetCoMpos");
    }
    if (ds.has("ReplayPlugin::GetCoMvel"))
    {
      data_.comVelocity_ = ds.get<Eigen::Vector3d>("ReplayPlugin::GetCoMvel");
    }

    if (ds.has("ReplayPlugin::GetCoMacc"))
    {
      data_.comAcceleration_ = ds.get<Eigen::Vector3d>("ReplayPlugin::GetCoMacc");
    }
  }
  return success && bodyMappings_.bodyConfigurations().size();
}
}  // namespace mc_xsens_plugin
