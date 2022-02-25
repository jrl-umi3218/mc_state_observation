#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/version.h>
#include <SpaceVecAlg/Conversions.h>
#include <mc_state_observation/SLAMObserver2.h>

#include <mc_rtc/ros.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/src/Geometry/Transform.h>

namespace mc_state_observation
{

namespace
{

bool getTransformStamped(tf2_ros::Buffer & tfBuffer,
                         const std::string & origin,
                         const std::string & to,
                         geometry_msgs::TransformStamped & transformStamped,
                         std::string & error)
{
  try
  {
    transformStamped = tfBuffer.lookupTransform(origin, to, ros::Time(0));
  }
  catch(tf2::TransformException & ex)
  {
    error = fmt::format("[SLAMObserver2] Could not get transform from \"{}\" to \"{}\"", origin, to);
    return false;
  }
  return true;
}

} // namespace

SLAMObserver2::SLAMObserver2(const std::string & type, double dt) : VisionBasedObserver(type, dt) {}

void SLAMObserver2::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  VisionBasedObserver::configure(ctl, config);

  if(config.has("Robot"))
  {
    body_ = ctl.robot(robot_).mb().bodies()[0].name();
    robots_.load({ctl.robot(robot_).module()});
  }

  if(config.has("SLAM"))
  {
    map_ = static_cast<std::string>(config("SLAM")("map"));
    estimated_ = static_cast<std::string>(config("SLAM")("estimated"));
    if(config("SLAM").has("ground"))
    {
      ground_ = static_cast<std::string>(config("SLAM")("ground"));
    }
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] SLAM configuration is mandatory.", name());
  }

  if(config.has("ExtraRobots"))
  {
    extraRobotsName_ = config("ExtraRobots");
    for(const auto & name : extraRobotsName_)
    {
      robots_.load(name, ctl.robot(name).module());
    }
  }

  desc_ = fmt::format("{} (Camera: {}, Estimated: {}, ExtraRobots: {})", name(), camera_, estimated_,
                      fmt::join(extraRobotsName_, ", "));

  thread_ = std::thread(std::bind(&SLAMObserver2::rosSpinner, this));
}

void SLAMObserver2::reset(const mc_control::MCController & ctl)
{
  VisionBasedObserver::reset(ctl);
}

bool SLAMObserver2::run(const mc_control::MCController & ctl)
{
  if(triggerInitialization_)
  {
    triggerInitialization_ = false;

    geometry_msgs::TransformStamped transformStamped;
    if(!getTransformStamped(tfBuffer_, map_, estimated_, transformStamped, error_))
    {
      isEstimatorAlive_ = false;
      return false;
    }

    const sva::PTransformd X_Slam_Estimated_Camera =
        sva::conversions::fromHomogeneous(tf2::transformToEigen(transformStamped).matrix());

    // Connect SLAM and Robot map, just for visual purpose
    const sva::PTransformd X_0_S = X_Slam_Estimated_Camera.inv() * ctl.realRobot(robot_).bodyPosW(camera_);

    auto transform = tf2::eigenToTransform(sva::conversions::toAffine(X_0_S));
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "robot_map";
    transform.child_frame_id = map_;
    tfStaticBroadcaster_.sendTransform(transform);

    filter_->reset();
  }

  geometry_msgs::TransformStamped transformStamped;
  if(!getTransformStamped(tfBuffer_, "robot_map", estimated_, transformStamped, error_))
  {
    if(isEstimatorAlive_)
    {
      const_cast<mc_control::MCController &>(ctl).datastore().remove("SLAM::Robot");
      const_cast<mc_control::MCController &>(ctl).datastore().remove("SLAM::X_S_Ground");

      for(size_t i = 0; i < robots_.robots().size(); ++i)
      {
        if(i != robots_.robotIndex())
        {
          const_cast<mc_control::MCController &>(ctl).datastore().remove(robots_.robots()[i].name()+"::X_S_Object");
        }
      }
    }
    isEstimatorAlive_ = false;
    return false;
  }
  else if(!isEstimatorAlive_)
  {
    isEstimatorAlive_ = true;

    const_cast<mc_control::MCController &>(ctl).datastore().make_call(
        "SLAM::Robot", [this]() -> const mc_rbdyn::Robot & { return robots_.robot(); });
    const_cast<mc_control::MCController &>(ctl).datastore().make_call(
        "SLAM::X_S_Ground", [this]() -> const sva::PTransformd & { return X_Slam_Ground_.pose; });

    for(size_t i = 0; i < robots_.robots().size(); ++i)
    {
      if(i != robots_.robotIndex())
      {
        const_cast<mc_control::MCController &>(ctl).datastore().make_call(
          robots_.robots()[i].name()+"::X_S_Object", [this, i]() -> const sva::PTransformd & { return robots_.robots()[i].posW(); });
      }
    }
  }

  isNewEstimatedPose_ = false;
  const sva::PTransformd pose = sva::conversions::fromHomogeneous(tf2::transformToEigen(transformStamped).matrix());
  if(pose != pose_.pose)
  {
    pose_.pose = pose;
    pose_.stamp = transformStamped.header.stamp.toSec();
    isNewEstimatedPose_ = true;
  }

  return VisionBasedObserver::run(ctl);
}

void SLAMObserver2::updatePose()
{
  geometry_msgs::TransformStamped transformStamped;
  if(ground_ != "" && getTransformStamped(tfBuffer_, map_, ground_, transformStamped, error_))
  {
    X_Slam_Ground_.pose = sva::conversions::fromHomogeneous(tf2::transformToEigen(transformStamped).matrix());
    X_Slam_Ground_.stamp = transformStamped.header.stamp.toSec();
  }
}

void SLAMObserver2::update(mc_control::MCController & ctl)
{
  VisionBasedObserver::update(ctl);

  if(isNewEstimatedPose_)
  {
    isNewEstimatedPose_ = false;

    auto & main_robot = robots_.robot();
    main_robot.mbc().q = q();
    const sva::PTransformd robot_posW =
        freeflyer() * camera().inv() * (isFiltered_ ? estimatedPoseFiltered_ : estimatedPose_);
    main_robot.posW(robot_posW);

    for(auto & robot : robots_)
    {
      if(ctl.datastore().has(robot.name()+"::X_C_Object"))
      {
      	const sva::PTransformd & X_C_Object = ctl.datastore().call<const sva::PTransformd &>(robot.name()+"::X_C_Object");
      	robot.posW(X_C_Object * (isFiltered_ ? estimatedPoseFiltered_ : estimatedPose_));
      }
      else
      {
        robot.posW(ctl.realRobot(robot.name()).posW() * ctl.realRobot().posW().inv() * main_robot.posW());
      }
      mc_rtc::ROSBridge::update_robot_publisher("SLAM_" + robot.name(), ctl.timeStep, robot);
    }
  }
}

void SLAMObserver2::addToLogger(const mc_control::MCController & ctl,
                                mc_rtc::Logger & logger,
                                const std::string & category)
{
  VisionBasedObserver::addToLogger(ctl, logger, category);
  logger.addLogEntry(category + "_LeftFootCenter", [this]() { return robots_.robot().surfacePose("LeftFootCenter"); });
  logger.addLogEntry(category + "_RightFootCenter",
                     [this]() { return robots_.robot().surfacePose("RightFootCenter"); });
  logger.addLogEntry(category + "_InternLeftHand", [this]() { return robots_.robot().surfacePose("InternLeftHand"); });
  logger.addLogEntry(category + "_InternRightHand",
                     [this]() { return robots_.robot().surfacePose("InternRightHand"); });
  logger.addLogEntry(category + "_com", [this]() { return robots_.robot().com(); });

  for(auto & robot : robots_)
  {
    const std::string & name = robot.name();
    logger.addLogEntry(category + "_posW_" + name, [this, name]() { return robots_.robot(name).posW(); });
  }
}

void SLAMObserver2::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  VisionBasedObserver::removeFromLogger(logger, category);
  logger.removeLogEntry(category + "_LeftFootCenter");
  logger.removeLogEntry(category + "_RightFootCenter");
  logger.removeLogEntry(category + "_com");

  for(auto & robot : robots_)
  {
    logger.removeLogEntry(category + "_posW_" + robot.name());
  }
}

void SLAMObserver2::addToGUI(const mc_control::MCController & ctl,
                             mc_rtc::gui::StateBuilder & gui,
                             const std::vector<std::string> & category)
{
  VisionBasedObserver::addToGUI(ctl, gui, category);

  using namespace mc_rtc::gui;

  gui.addElement(category, Transform("X_S_Ground", [this]() { return X_Slam_Ground_.pose; }),
                 Button("Initialize", [this, &ctl]() { triggerInitialization_ = true; }),
                 ArrayLabel("X_Slam_Camera", {"r", "p", "y", "x", "y", "z"}, [this]() {
                   Eigen::VectorXd ret(6);
                   ret << mc_rbdyn::rpyFromMat(pose_.pose.rotation()), pose_.pose.translation();
                   return ret;
                 }));
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("SLAM2", mc_state_observation::SLAMObserver2)
