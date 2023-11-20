#include <mc_xsens_plugin/XsensDataInputLive.h>
#include <mc_xsens_plugin/XsensSegments.h>

/** Work-around for C++17 while OpenRTM is not updated to remove the throw(...)
 * specification */
#if __cplusplus >= 201703L
/** Include C++ headers that won't work well without the specification */
#include <iostream>
#include <optional>
#include <string>
#define throw(...)
#endif

#include <xsens_streaming/udpserver.h>

#if __cplusplus >= 201703L
#undef throw
#endif

namespace mc_xsens_plugin {
XsensDataInputLive::XsensDataInputLive(const XsensSegments &segments,
                                       const mc_rtc::Configuration &config)
    : XsensDataInput(segments) {
  config("host", host_);
  config("port", port_);
  server_.reset(new UdpServer(host_, port_));
}

bool XsensDataInputLive::update() {
  auto quaternions = server_->quaternions();
  auto angularKin = server_->angularSegmentKinematics();
  auto linearKin = server_->linearSegmentKinematics();

  // creating pose (sva::PTransformd) for each segment
  for (const auto &quat : quaternions) {
    Eigen::Vector3d pos{quat.position[0], quat.position[1], quat.position[2]};
    Eigen::Quaterniond q{quat.orientation[0], quat.orientation[1],
                         quat.orientation[2], quat.orientation[3]};
    const auto &name = segments_.segmentName(quat.segmentId);
    data_.segment_poses_[name] = sva::PTransformd{q.inverse(), pos};
  }

  // creating angular elements of velocity and acceleration (sva::MotionVecd)
  for (const auto &ang : angularKin) {
    Eigen::Vector3d angulVel{ang.angularVeloc[0], ang.angularVeloc[1],
                             ang.angularVeloc[2]};
    Eigen::Vector3d angulAcc{ang.angularAccel[0], ang.angularAccel[1],
                             ang.angularAccel[2]};
    const auto &name = segments_.segmentName(ang.segmentId);
    data_.segment_vels_[name].angular() = Eigen::Vector3d::Zero(); // angulVel;
    data_.segment_accs_[name].angular() = Eigen::Vector3d::Zero(); // angulAcc;
  }

  // creating linear elements of velocity and acceleration (sva::MotionVecd)
  for (const auto &lin : linearKin) {
    Eigen::Vector3d linearVel{lin.velocity[0], lin.velocity[1],
                              lin.velocity[2]};
    Eigen::Vector3d linearAcc{lin.acceleration[0], lin.acceleration[1],
                              lin.acceleration[2]};
    const auto &name = segments_.segmentName(lin.segmentId);
    data_.segment_vels_[name].linear() = linearVel;
    data_.segment_accs_[name].linear() = linearAcc;
  }

  auto CoMdata = server_->comData();
  data_.comPosition_ =
      Eigen::Vector3d{CoMdata.pos[0], CoMdata.pos[1], CoMdata.pos[2]};
  data_.comVelocity_ =
      Eigen::Vector3d{CoMdata.vel[0], CoMdata.vel[1], CoMdata.vel[2]};
  data_.comAcceleration_ =
      Eigen::Vector3d{CoMdata.acc[0], CoMdata.acc[1], CoMdata.acc[2]};
  return true;
}

} // namespace mc_xsens_plugin
