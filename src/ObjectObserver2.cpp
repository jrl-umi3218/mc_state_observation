
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/version.h>
#include <SpaceVecAlg/Conversions.h>
#include <mc_state_observation/ObjectObserver2.h>

#include <mc_rtc/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/src/Geometry/Transform.h>

namespace mc_state_observation
{

ObjectObserver2::ObjectObserver2(const std::string & type, double dt) : VisionBasedObserver(type, dt) {}

void ObjectObserver2::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  VisionBasedObserver::configure(ctl, config);

  if(config.has("Object"))
  {
    object_ = static_cast<std::string>(config("Object")("robot"));
    topic_ = static_cast<std::string>(config("Object")("topic"));
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Object configuration is mandatory.", name());
  }

  subscriber_ = nh_->subscribe(topic_, 1, &ObjectObserver2::callback, this);
  subscriber2_ = nh_->subscribe("/ntm/camera", 1, &ObjectObserver2::callback2, this);

  thread_ = std::thread(std::bind(&ObjectObserver2::rosSpinner, this));

  desc_ = fmt::format("{} (Object: {}, Topic: {})", name(), object_, topic_);
}

void ObjectObserver2::reset(const mc_control::MCController & ctl)
{
  VisionBasedObserver::reset(ctl);
}

bool ObjectObserver2::run(const mc_control::MCController & ctl)
{
  bool ret = VisionBasedObserver::run(ctl);
  return ret;
}

void ObjectObserver2::updatePose()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  pose_ = poseFromTopic_;
}

void ObjectObserver2::update(mc_control::MCController & ctl)
{
  VisionBasedObserver::update(ctl);

  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if(isNewEstimatedPose_)
    {
      isNewEstimatedPose_ = false;
      truePose_ = (isFiltered_ ? estimatedPoseFiltered_ : estimatedPose_) * trueCameraPose_.pose;
      ctl.realRobot(object_).posW((isFiltered_ ? estimatedPoseFiltered_ : estimatedPose_) * camera());
    }
  }
}

void ObjectObserver2::addToLogger(const mc_control::MCController & ctl,
                                  mc_rtc::Logger & logger,
                                  const std::string & category)
{
  std::string _category = category;
  _category += "_" + object_;
  VisionBasedObserver::addToLogger(ctl, logger, _category);

  logger.addLogEntry(_category + "_posW_control", [this, &ctl]() { return ctl.robot(object_).posW(); });
  logger.addLogEntry(_category + "_posW_real", [this, &ctl]() { return ctl.realRobot(object_).posW(); });
  logger.addLogEntry(_category + "_truePose", [this, &ctl]() { return truePose_; });
  logger.addLogEntry(_category + "_trueCamera", [this, &ctl]() { return trueCameraPose_.pose; });
}

void ObjectObserver2::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  std::string _category = category;
  _category += "_" + object_;
  VisionBasedObserver::removeFromLogger(logger, category);

  logger.removeLogEntry(_category + "_posW_control");
  logger.removeLogEntry(_category + "_posW_real");
  logger.removeLogEntry(_category + "_truePose");
  logger.removeLogEntry(_category + "_trueCamera");
}

void ObjectObserver2::addToGUI(const mc_control::MCController & ctl,
                               mc_rtc::gui::StateBuilder & gui,
                               const std::vector<std::string> & category)
{
  VisionBasedObserver::addToGUI(ctl, gui, category);

  using namespace mc_rtc::gui;

  gui.addElement(category, Transform("realPose_" + object_, [this, &ctl]() { return ctl.realRobot(object_).posW(); }));
}

void ObjectObserver2::callback(const geometry_msgs::PoseStamped & msg)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(msg.pose, affine);
  const sva::PTransformd pose = sva::conversions::fromHomogeneous(affine.matrix());
  const sva::MotionVecd error = sva::transformError(pose, poseFromTopic_.pose);
  const std::lock_guard<std::mutex> lock(mutex_);
  if((poseFromTopic_.pose == sva::PTransformd::Identity())
     || (poseFromTopic_.pose != sva::PTransformd::Identity() && error.vector().norm() < 0.5))
  {
    poseFromTopic_ = {pose, msg.header.stamp.toSec()};
    isNewEstimatedPose_ = true;
    isEstimatorAlive_ = true;
  }
  else
  {
    isEstimatorAlive_ = false;
  }
}

void ObjectObserver2::callback2(const geometry_msgs::PoseStamped & msg)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(msg.pose, affine);
  trueCameraPose_ = {sva::conversions::fromHomogeneous(affine.matrix()), msg.header.stamp.toSec()};
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("Object2", mc_state_observation::ObjectObserver2)
