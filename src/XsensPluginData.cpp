#include <mc_rtc/log/Logger.h>
#include <mc_xsens_plugin/XsensPluginData.h>

namespace mc_xsens_plugin
{
void XsensData::addToLogger(mc_rtc::Logger & logger, const std::string & prefix)
{
  for(const auto & segment : segment_poses_)
  {
    const auto & segmentName = segment.first;
    const auto & segmentPose = segment.second;
    logger.addLogEntry(fmt::format("{}_segments_{}_pose", prefix, segmentName), this,
                       [&segmentPose]() -> const sva::PTransformd & { return segmentPose; });
  }

  for(const auto & segment : segment_vels_)
  {
    const auto & segmentName = segment.first;
    const auto & segmentVel = segment.second;
    logger.addLogEntry(fmt::format("{}_segments_{}_velocity", prefix, segmentName), this,
                       // XXX Log as VectorXd for backward compatibility with older logs
                       // Should be handled by InputDatastore instead
                       [&segmentVel]() -> const Eigen::VectorXd { return segmentVel.vector(); });
  }

  for(const auto & segment : segment_accs_)
  {
    const auto & segmentName = segment.first;
    const auto & segmentAcc = segment.second;
    logger.addLogEntry(fmt::format("{}_segments_{}_acceleration", prefix, segmentName), this,
                       [&segmentAcc]() -> const Eigen::VectorXd { return segmentAcc.vector(); });
  }

  logger.addLogEntry(fmt::format("{}_com_position", prefix), this,
                     [this]() -> const Eigen::Vector3d & { return comPosition_; });
  logger.addLogEntry(fmt::format("{}_com_velocity", prefix), this,
                     [this]() -> const Eigen::Vector3d & { return comVelocity_; });
  logger.addLogEntry(fmt::format("{}_com_acceleration", prefix), this,
                     [this]() -> const Eigen::Vector3d & { return comAcceleration_; });
}

void XsensData::removeFromLogger(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(this);
}
} // namespace mc_xsens_plugin
