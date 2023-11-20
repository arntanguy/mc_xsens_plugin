#pragma once

#include <mc_control/fsm/State.h>
#include <mc_tasks/EndEffectorTask.h>
#include <mc_tasks/TransformTask.h>
#include <mc_trajectory/SequenceInterpolator.h>
#include <mc_xsens_plugin/XsensBodyConfiguration.h>

namespace mc_xsens_plugin {

struct XsensPlugin;

struct XsensRetargetting : mc_control::fsm::State {
  void start(mc_control::fsm::Controller &ctl) override;
  bool run(mc_control::fsm::Controller &ctl) override;
  void teardown(mc_control::fsm::Controller &ctl) override;

  bool isActiveBody(const std::string &bodyName) const noexcept {
    return activeBodies_.empty() ||
           std::find(activeBodies_.begin(), activeBodies_.end(), bodyName) !=
               activeBodies_.end();
  }

private:
  XsensPlugin *plugin_ = nullptr;
  std::map<std::string, XsensBodyConfiguration>
      bodyConfigurations_; ///< Body configuration for this state
  std::map<std::string, std::shared_ptr<mc_tasks::TransformTask>> tasks_;
  std::map<std::string, std::shared_ptr<mc_tasks::TransformTask>> fixedTasks_;
  double fixedStiffness_ = 200;
  double fixedWeight_ = 1000;
  std::vector<std::string> ignoreJointsAry;
  std::string robot_{};
  sva::PTransformd offset_ = sva::PTransformd::Identity();
  bool fixBaseLink_ = true;
  sva::PTransformd initPosW_ =
      sva::PTransformd::Identity(); //< only used if fixBaseLink_ = true
  bool finishRequested_ = false;
  bool finishing_ = false;
  bool finished_ = false;
  std::vector<std::string> unactiveJoints_ = {};
  std::vector<std::string> activeBodies_ = {};
  std::vector<std::string> groundingFrames_ = {};

  double initialInterpolationTime_ = 5.0;
  double initialStiffnessPercent_ = 0.05;
  double initialWeightPercent_ = 0.1;
  mc_trajectory::SequenceInterpolator<double> startStiffnessInterpolator_;
  double startWeightPercent_ = 0.1;
  mc_trajectory::SequenceInterpolator<double> startWeightInterpolator_;
  double endInterpolationTime_ = 2.0;
  double endStiffnessPercent_ = 0.05;
  mc_trajectory::SequenceInterpolator<double> endStiffnessInterpolator_;
  double endWeightPercent_ = 0.1;
  mc_trajectory::SequenceInterpolator<double> endWeightInterpolator_;
  double endTime_ = 0;

  bool autoTransition_ = false;
  bool debugmode_ = false;
  double t_ = 0;
};

} // namespace mc_xsens_plugin
