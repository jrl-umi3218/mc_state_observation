#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_state_observation/TiltObserver.h>
#include <mc_state_observation/gui_helpers.h>
#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace mc_state_observation
{

namespace so = stateObservation;

TiltObserver::TiltObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), estimator_(alpha_, beta_, gamma_)
{
  estimator_.setSamplingTime(dt_);
  xk_.resize(9);
  xk_ << so::Vector3::Zero(), so::Vector3::Zero(), so::Vector3(0, 0, 1); // so::Vector3(0.49198, 0.66976, 0.55622);
  estimator_.setState(xk_, 0);
}

void TiltObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  updateRobotName_ = robot_;
  imuSensor_ = config("imuSensor", ctl.robot().bodySensor().name());
  updateSensorName_ = imuSensor_;
  config("alpha", alpha_);
  config("beta", beta_);
  config("gamma", gamma_);
  anchorFrameFunction_ = config("anchorFrameFunction", name() + "::" + ctl.robot(robot_).name());
  config("updateRobot", updateRobot_);
  config("updateRobotName", updateRobotName_);
  config("updateSensor", updateSensor_);
  config("updateSensorName", updateSensorName_);
  desc_ = fmt::format("{}", name_);
}

void TiltObserver::reset(const mc_control::MCController & ctl) {}

bool TiltObserver::run(const mc_control::MCController & ctl)
{
  estimator_.setAlpha(alpha_);
  estimator_.setBeta(beta_);
  estimator_.setGamma(gamma_);

  if(!ctl.datastore().has(anchorFrameFunction_))
  {
    error_ = fmt::format(
        "Observer {} requires a \"{}\" function in the datastore to provide the observer's kinematic anchor frame.\n"
        "Please refer to https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html for further details.",
        name(), anchorFrameFunction_);
    return false;
  }

  // Anchor frame defined w.r.t control robot
  // XXX what if the feet are being moved by the stabilizer?
  X_0_C_ = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, ctl.robot(robot_));
  // we want in this anchor frame:
  // - position of the IMU
  // - orientation of the IMU
  // - linear velocity of the imu
  // - angular velocity of the imu
  // - linear velocity of the control frame (derivative?)

  const auto & robot = ctl.robot(robot_);
  const auto & imu = robot.bodySensor(imuSensor_);
  auto X_0_B = robot.posW();
  auto X_0_IMU = imu.X_b_s() * robot.bodyPosW(imu.parentBody());
  X_fb_imu = X_0_IMU * X_0_B.inv();
  // auto RF = anchorFrame.oriention().transpose(); // orientation of the control anchor frame
  // auto RB = robot.posW().orientation().transpose(); // orientation of the control floating base

  // Pose of the imu in the control frame
  X_C_IMU_ = X_0_IMU * X_0_C_.inv();
  // Compute velocity of the imu in the control frame
  auto v_0_imuParent = robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())];
  auto v_0_IMU = imu.X_b_s() * v_0_imuParent;
  auto v_c_IMU = X_C_IMU_.inv() * v_0_IMU;
  imuVelC_ = v_c_IMU;

  estimator_.setSensorPositionInC(X_C_IMU_.translation());
  estimator_.setSensorOrientationInC(X_C_IMU_.rotation().transpose());
  estimator_.setSensorLinearVelocityInC(imuVelC_.linear());
  estimator_.setSensorAngularVelocityInC(imuVelC_.angular());
  // XXX how to set
  estimator_.setControlOriginVelocityInW(Eigen::Vector3d::Zero());

  auto k = estimator_.getCurrentTime();

  estimator_.setMeasurement(imu.linearAcceleration(), imu.angularVelocity(), k + 1);

  xk_ = estimator_.getEstimatedState(k + 1);

  so::Vector3 tilt = xk_.tail(3);

  // Orientation of body?
  estimatedRotationIMU_ = so::kine::mergeTiltWithYaw(tilt, X_0_IMU.rotation().transpose()).transpose();

  return true;
}

void TiltObserver::update(mc_control::MCController & ctl)
{
  if(updateRobot_)
  {
    auto & robot = ctl.realRobots().robot(updateRobot_);
    auto posW = robot.posW();
    Eigen::Matrix3d R_0_fb = estimatedRotationIMU_ * X_fb_imu.rotation();
    posW.rotation() = R_0_fb.transpose();
    mc_rtc::log::info("update robot: {}", mc_rbdyn::rpyFromMat(posW.rotation()));
    posW.translation() = Eigen::Vector3d::Zero();
    robot.posW(posW);
    robot.forwardKinematics();
  }

  if(updateSensor_)
  {
    auto & imu = ctl.robot(robot_).bodySensor(imuSensor_);
    auto & rimu = ctl.realRobot(robot_).bodySensor(imuSensor_);
    imu.orientation(Eigen::Quaterniond{estimatedRotationIMU_});
    rimu.orientation(Eigen::Quaterniond{estimatedRotationIMU_});
  }
}

void TiltObserver::addToLogger(const mc_control::MCController &, mc_rtc::Logger & logger, const std::string & category)
{
  logger.addLogEntry(category + "_imuVelC", [this]() -> const sva::MotionVecd & { return imuVelC_; });
  logger.addLogEntry(category + "_imuPoseC", [this]() -> const sva::PTransformd & { return X_C_IMU_; });
  logger.addLogEntry(category + "_imuEstRotW", [this]() { return Eigen::Quaterniond{estimatedRotationIMU_}; });
  logger.addLogEntry(category + "_controlAnchorFrame", [this]() -> const sva::PTransformd & { return X_0_C_; });
}

void TiltObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_imuVelC");
  logger.removeLogEntry(category + "_imuPoseC");
  logger.removeLogEntry(category + "_imuEstRotW");
  logger.removeLogEntry(category + "_controlAnchorFrame");
}

void TiltObserver::addToGUI(const mc_control::MCController & ctl,
                            mc_rtc::gui::StateBuilder & gui,
                            const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;
  gui.addElement(category, make_input_element("alpha", alpha_), make_input_element("beta", beta_),
                 make_input_element("gamma", gamma_));
}

} // namespace mc_state_observation
EXPORT_OBSERVER_MODULE("Tilt", mc_state_observation::TiltObserver)
