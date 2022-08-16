#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_state_observation/AttitudeObserver.h>
#include <mc_state_observation/gui_helpers.h>

namespace mc_state_observation
{

/// Sizes of the states for the state, the measurement, and the input vector
constexpr stateObservation::Index AttitudeObserver::STATE_SIZE_BASE;
constexpr stateObservation::Index AttitudeObserver::MEASUREMENT_SIZE;
constexpr stateObservation::Index AttitudeObserver::INPUT_SIZE;

namespace so = stateObservation;

AttitudeObserver::AttitudeObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), filter_(STATE_SIZE_BASE, MEASUREMENT_SIZE, INPUT_SIZE, false),
  q_(so::Matrix::Identity(STATE_SIZE_BASE, STATE_SIZE_BASE) * defaultConfig_.stateCov),
  r_(so::Matrix::Identity(MEASUREMENT_SIZE, MEASUREMENT_SIZE) * defaultConfig_.acceleroCovariance), uk_(INPUT_SIZE),
  xk_(STATE_SIZE_BASE)
{
  /// initialization of the extended Kalman filter
  imuFunctor_.setSamplingPeriod(dt_);
  filter_.setFunctor(&imuFunctor_);

  /// 3rd order control of the position and orientation
  ///(check the stability of the gain parameters)
  Kpt_ << -20, 0, 0, 0, -20, 0, 0, 0, -20;
  Kdt_ << -10, 0, 0, 0, -10, 0, 0, 0, -10;
  Kat_ << -10, 0, 0, 0, -10, 0, 0, 0, -10;
  Kpo_ << -0, 0, 0, 0, -0, 0, 0, 0, -20;
  Kdo_ << -5, 0, 0, 0, -5, 0, 0, 0, -50;
  Kao_ << -5, 0, 0, 0, -5, 0, 0, 0, -50;
}

void AttitudeObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  imuSensor_ = config("imuSensor", ctl.robot().bodySensor().name());
  if(config.has("updateSensor"))
  {
    updateSensor_ = static_cast<std::string>(config("updateSensor"));
  }
  else
  {
    updateSensor_ = imuSensor_;
  }
  datastoreName_ = config("datastoreName", name());
  config("log_kf", log_kf_);
  config("init_from_control", initFromControl_);
  if(config.has("with_gyro_bias"))
  {
    config("with_gyro_bias", withGyroBias_);
    imuFunctor_.setWithGyroBias(withGyroBias_);
    stateSize_ = imuFunctor_.getStateSize();
  }
  defaultConfig_ = config("KalmanFilter", KalmanFilterConfig{});
  config_ = defaultConfig_;
  desc_ = fmt::format("{} (sensor={})", name_, imuSensor_);
}

void AttitudeObserver::reset(const mc_control::MCController & ctl)
{
  const auto & c = config_;

  q_.noalias() = so::Matrix::Identity(stateSize_, stateSize_) * c.stateCov;
  q_.diagonal().segment<3>(indexes::angAcc).setConstant(c.orientationAccCov);
  q_.diagonal().segment<3>(indexes::linAcc).setConstant(c.linearAccCov);
  if(withGyroBias_)
  {
    q_.diagonal().segment<3>(indexes::gyroBias).setConstant(c.biasDriftCov);
  }

  r_.noalias() = so::Matrix::Identity(MEASUREMENT_SIZE, MEASUREMENT_SIZE) * c.acceleroCovariance;
  r_(3, 3) = r_(4, 4) = r_(5, 5) = c.gyroCovariance;

  filter_.reset();
  filter_.setQ(q_);
  filter_.setR(r_);
  xk_.setZero();
  if(initFromControl_)
  {
    const auto & imuSensor = ctl.robot(robot_).bodySensor(imuSensor_);
    const Eigen::Matrix3d cOri = (imuSensor.X_b_s() * ctl.robot(robot_).bodyPosW(imuSensor.parentBody())).rotation();
    xk_.segment<3>(indexes::ori) = so::kine::rotationMatrixToRotationVector(cOri.transpose());
  }

  uk_.setZero();

  filter_.setState(xk_, 0);

  Eigen::MatrixXd P = filter_.getPmatrixZero();

  P.diagonal().head<STATE_SIZE_BASE>().setConstant(c.stateInitCov);

  if(withGyroBias_)
  {
    P.diagonal().segment<3>(indexes::gyroBias).setConstant(c.biasInitCov);
  }

  filter_.setStateCovariance(P);

  lastStateInitCovariance_ = c.stateInitCov;
  m_gyrobias.setZero();
  m_rateIn.setZero();
  m_accIn.setZero();
}

bool AttitudeObserver::run(const mc_control::MCController & ctl)
{
  const auto & c = config_;
  bool ret = true;

  q_.noalias() = so::Matrix::Identity(stateSize_, stateSize_) * c.stateCov;
  q_.diagonal().segment<3>(indexes::angAcc).setConstant(c.orientationAccCov);
  q_.diagonal().segment<3>(indexes::linAcc).setConstant(c.linearAccCov);
  q_.diagonal().segment<3>(indexes::ori).setConstant(c.oriCov);
  if(withGyroBias_)
  {
    q_.diagonal().segment<3>(12).setConstant(c.biasDriftCov);
  }

  r_.noalias() = so::Matrix::Identity(MEASUREMENT_SIZE, MEASUREMENT_SIZE) * c.acceleroCovariance;
  r_(3, 3) = r_(4, 4) = r_(5, 5) = c.gyroCovariance;

  filter_.setQ(q_);
  filter_.setR(r_);

  if(lastStateInitCovariance_ != c.stateInitCov) /// if the value of the state Init Covariance has changed
  {
    filter_.setStateCovariance(so::Matrix::Identity(stateSize_, stateSize_) * c.stateInitCov);
    lastStateInitCovariance_ = c.stateInitCov;
  }

  // Get sensor values
  const mc_rbdyn::BodySensor & imu = ctl.robot(robot_).bodySensor(imuSensor_);
  m_accIn = imu.linearAcceleration();
  m_rateIn = imu.angularVelocity();

  // processing
  so::Vector6 measurement;
  if(c.compensateMode)
  {
    if(ctl.datastore().has(datastoreName_ + "::accRef"))
    {
      const Eigen::Vector3d & accRef = ctl.datastore().get<Eigen::Vector3d>(datastoreName_ + "::accRef");
      measurement.head<3>() = m_accIn - accRef;
    }
    else
    {
      measurement.head<3>() = m_accIn;
      ret = false;
    }
  }
  else
  {
    measurement.head<3>() = m_accIn;
  }

  if(withGyroBias_) /// when the bias is estimated
  {
    measurement.tail<3>() = m_rateIn;
  }
  else
  {
    measurement.tail<3>() = m_rateIn - m_gyrobias;
  }

  auto time = filter_.getCurrentTime();

  /// 3rd order jerk control
  uk_.head<3>() = Kpt_ * xk_.segment<3>(indexes::pos) + Kdt_ * xk_.segment<3>(indexes::linVel)
                  + Kat_ * xk_.segment<3>(indexes::linAcc);
  uk_.tail<3>() = Kpo_ * xk_.segment<3>(indexes::ori) + Kdo_ * xk_.segment<3>(indexes::angVel)
                  + Kao_ * xk_.segment<3>(indexes::angAcc);

  // uk_ is actually the acceleration change
  uk_ -= xk_.segment<6>(indexes::linAcc);

  filter_.setInput(uk_, time);
  filter_.setMeasurement(measurement, time + 1);

  /// set the derivation step for the finite difference method
  const so::Vector dx = filter_.stateVectorConstant(1) * 1e-8;

  Eigen::MatrixXd A = filter_.getAMatrixFD(dx);
  Eigen::MatrixXd C = filter_.getCMatrixFD(dx);

  filter_.setA(filter_.getAMatrixFD(dx));
  filter_.setC(filter_.getCMatrixFD(dx));

  /// get the estimation and give it to the array
  xk_ = filter_.getEstimatedState(time + 1);

  // result
  const so::Vector3 orientation(xk_.segment<3>(indexes::ori));
  m_orientation = c.offset * so::kine::rotationVectorToRotationMatrix(orientation);

  if(withGyroBias_)
  {
    m_gyrobias = xk_.segment<3>(indexes::gyroBias);
  }

  if(m_gyroBiasFromMeasurement_running)
  {
    m_gyroMeasurementTotal += m_rateIn;
    ++m_numberOfValues;
  }

  if(ctl.gui() && m_resetGUI)
  {
    removeFromGUI(*ctl.gui(), m_category);

    addToGUI(ctl, *ctl.gui(), m_category);
    m_resetGUI = false;
  }

  return ret;
}

void AttitudeObserver::update(mc_control::MCController & ctl)
{
  auto & sensor = ctl.robot(robot_).bodySensor(updateSensor_);
  auto & realSensor = ctl.realRobot(robot_).bodySensor(updateSensor_);
  Eigen::Quaterniond quat(m_orientation.transpose());
  sensor.orientation(quat);
  realSensor.orientation(quat);
}

void AttitudeObserver::addToLogger(const mc_control::MCController &,
                                   mc_rtc::Logger & logger,
                                   const std::string & category)
{
  logger.addLogEntry(category + "_orientation", [this]() -> sva::PTransformd {
    return sva::PTransformd{m_orientation, Eigen::Vector3d::Zero()};
  });
  logger.addLogEntry(category + "_gyroBias", [this]() { return m_gyrobias; });
  if(log_kf_)
  {
    config_.addToLogger(logger, category);
  }
}

void AttitudeObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_orientation");
  logger.removeLogEntry(category + "_gyroBias");
  if(log_kf_)
  {
    config_.removeFromLogger(logger, category);
  }
}

void AttitudeObserver::addToGUI(const mc_control::MCController & ctl,
                                mc_rtc::gui::StateBuilder & gui,
                                const std::vector<std::string> & category)
{
  m_category = category;
  using namespace mc_rtc::gui;
  auto kf_category = category;
  kf_category.push_back("KalmanFilter");
  config_.addToGUI(gui, kf_category);

  gui.addElement(category, Button("Reset config to default", [this]() { config_ = defaultConfig_; }),
                 Button("Reset",
                        [this, &ctl]() {
                          mc_rtc::log::info("[{}] Manual reset triggerred", name());
                          reset(ctl);
                        }),
                 make_rpy_label("Result", m_orientation));

  gui.addElement(category, ArrayLabel("Accelerometer", [this]() { return m_accIn; }),
                 ArrayLabel("Gyrometer", [this]() { return m_rateIn; }));

  if(m_gyroBiasFromMeasurement_running)
  {
    gui.addElement(category, mc_rtc::gui::ElementsStacking::Horizontal,
                   ArrayLabel("Gyro Bias (calibrating) :", {"x", "y", "z"},
                              [this]() -> Eigen::Vector3d { return (m_gyroMeasurementTotal / m_numberOfValues); }),
                   Button("Stop\ncalibration", [this, &ctl, &gui, &category]() { setGyroBiasToMeanValue(); }));
  }
  else
  {
    gui.addElement(category, mc_rtc::gui::ElementsStacking::Horizontal, make_xyz_input("Gyro Bias:", m_gyrobias),
                   Checkbox(
                       "EKF bias\nestimation", [this]() { return withGyroBias_; },
                       [this]() { switchWithGyroBias(!withGyroBias_); }),
                   Button("Calibrate\nfrom data\n(static robot)",
                          [this, &ctl, &gui, &category]() { startGyroBiasIdentification(); }));
  }

  auto new_category = category;
  new_category.push_back("Internal");

  gui.addElement(new_category,
                 ArrayLabel("Position", [this]() -> Eigen::Vector3d { return xk_.segment<3>(indexes::pos); }),
                 ArrayLabel("Orientation", [this]() -> Eigen::Vector3d { return xk_.segment<3>(indexes::ori); }),
                 ArrayLabel("lin vel", [this]() -> Eigen::Vector3d { return xk_.segment<3>(indexes::linVel); }),
                 ArrayLabel("Ang Vel", [this]() -> Eigen::Vector3d { return xk_.segment<3>(indexes::angVel); }),
                 ArrayLabel("Lin Acc", [this]() -> Eigen::Vector3d { return xk_.segment<3>(indexes::linAcc); }),
                 ArrayLabel("Ang Acc", [this]() -> Eigen::Vector3d { return xk_.segment<3>(indexes::angAcc); }));
}

void AttitudeObserver::setGyroBiasToMeanValue()
{
  using namespace mc_rtc::gui;

  m_gyrobias = m_gyroMeasurementTotal / m_numberOfValues;
  m_gyroBiasFromMeasurement_running = false;

  if(withGyroBias_)
  {
    xk_.segment<3>(indexes::gyroBias) = m_gyrobias;
    filter_.setCurrentState(xk_);
    Eigen::MatrixXd P = filter_.getStateCovariance();
    P.block<indexes::gyroBias, 3>(0, indexes::gyroBias).setZero();
    P.block<3, indexes::gyroBias>(indexes::gyroBias, 0).setZero();
    P.bottomLeftCorner<3, 3>() = Eigen::Vector3d(Eigen::Vector3d::Constant(config_.biasInitCov)).asDiagonal();
    filter_.setStateCovariance(P);
  }

  m_resetGUI = true;
}

void AttitudeObserver::startGyroBiasIdentification()
{
  using namespace mc_rtc::gui;

  m_numberOfValues = 0;
  m_gyroBiasFromMeasurement_running = true;
  m_gyroMeasurementTotal.setZero();

  m_resetGUI = true;
}

void AttitudeObserver::switchWithGyroBias(bool b)
{
  if(withGyroBias_ != b)
  {
    withGyroBias_ = b;
    imuFunctor_.setWithGyroBias(b);
    stateSize_ = imuFunctor_.getStateSize();
    xk_.conservativeResize(stateSize_);
    Eigen::MatrixXd P = filter_.getStateCovariance();
    P.conservativeResize(stateSize_, stateSize_);

    if(withGyroBias_)
    {
      xk_.segment<3>(indexes::gyroBias) = m_gyrobias;
      P.block<indexes::gyroBias, 3>(0, indexes::gyroBias).setZero();
      P.block<3, indexes::gyroBias>(indexes::gyroBias, 0).setZero();
      P.bottomLeftCorner<3, 3>() = Eigen::Vector3d(Eigen::Vector3d::Constant(config_.biasInitCov)).asDiagonal();
    }
    long time = filter_.getCurrentTime();
    filter_.setStateSize(stateSize_);
    filter_.setState(xk_, time);
    filter_.setStateCovariance(P);
  }
}

void AttitudeObserver::KalmanFilterConfig::addToLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.addLogEntry(category + "_covariance_state", [this]() { return stateCov; });
  logger.addLogEntry(category + "_covariance_ori_acc", [this]() { return orientationAccCov; });
  logger.addLogEntry(category + "_covariance_acc", [this]() { return acceleroCovariance; });
  logger.addLogEntry(category + "_covariance_gyr", [this]() { return gyroCovariance; });
}

void AttitudeObserver::KalmanFilterConfig::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_covariance_state");
  logger.removeLogEntry(category + "_covariance_ori_acc");
  logger.removeLogEntry(category + "_covariance_acc");
  logger.removeLogEntry(category + "_covariance_gyr");
}

void AttitudeObserver::KalmanFilterConfig::addToGUI(mc_rtc::gui::StateBuilder & gui,
                                                    const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;
  // clang-format off
  gui.addElement(category, 
    make_input_element("Compensate Mode", compensateMode),
    make_input_element("acceleroCovariance", acceleroCovariance),
    make_input_element("gyroCovariance", gyroCovariance),
    make_input_element("orientationAccCov", orientationAccCov),
    make_input_element("linearAccCov", linearAccCov),
    make_input_element("biasDriftCov", biasDriftCov),
    make_input_element("stateCov", stateCov),
    make_input_element("stateInitCov", stateInitCov),
    make_rpy_input("offset", offset));
  // clang-format on
}

void AttitudeObserver::KalmanFilterConfig::removeFromGUI(mc_rtc::gui::StateBuilder & gui,
                                                         const std::vector<std::string> & category)
{
  gui.removeCategory(category);
}

} // namespace mc_state_observation
EXPORT_OBSERVER_MODULE("Attitude", mc_state_observation::AttitudeObserver)
