#pragma once

#include <mc_observers/Observer.h>
#include <state-observation/observer/tilt-estimator.hpp>

namespace mc_state_observation
{

struct TiltObserver : public mc_observers::Observer
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
 public:
  TiltObserver(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void update(mc_control::MCController & ctl) override;

protected:
  /*! \brief Add observer from logger
   *
   * @param category Category in which to log this observer
   */
  void addToLogger(const mc_control::MCController &, mc_rtc::Logger &, const std::string & category) override;

  /*! \brief Remove observer from logger
   *
   * @param category Category in which this observer entries are logged
   */
  void removeFromLogger(mc_rtc::Logger &, const std::string & category) override;

  /*! \brief Add observer information the GUI.
   *
   * @param category Category in which to add this observer
   */
  void addToGUI(const mc_control::MCController &,
                mc_rtc::gui::StateBuilder &,
                const std::vector<std::string> & /* category */) override;
  protected:
  std::string robot_;
  bool updateRobot_ = false;
  std::string updateRobotName_;
  std::string imuSensor_;
  bool updateSensor_ = true;
  std::string updateSensorName_;
  /*!
   * parameter related to the convergence of the linear velocity
   * of the IMU expressed in the control frame
   */
  double alpha_ = 200;
  ///  parameter related to the fast convergence of the tilt
  double beta_ = 5;
  /// parameter related to the orthogonality
  double gamma_ = 15;
  std::string anchorFrameFunction_;
  stateObservation::TiltEstimator estimator_;
  
  
  // values used for computation
  sva::PTransformd X_fb_imu = sva::PTransformd::Identity();
  sva::MotionVecd imuVelC_ = sva::MotionVecd::Zero();
  sva::PTransformd X_C_IMU_ = sva::PTransformd::Identity();
  sva::PTransformd X_0_C_ = sva::PTransformd::Identity(); // control anchor frame

  // result
  // The observed tilt of the sensor
  Eigen::Matrix3d estimatedRotationIMU_;
  

  // // The RPY angles for the estimated orientation of the waist ({B})
  // TimedOrientation3D m_rpyBEst;
  // InPort<TimedOrientation3D> m_rpyBEstIn;

  // // The estimated position of the waist ({B})
  // TimedPoint3D m_pBEst;
  // InPort<TimedPoint3D> m_pBEstIn;

  // // The RPY angles for the estimated orientation of the control frame ({F})
  // TimedOrientation3D m_rpyFEst;
  // InPort<TimedOrientation3D> m_rpyFEstIn;

  // // The estimated position of the control frame ({F})
  // TimedPoint3D m_pFEst;
  // InPort<TimedPoint3D> m_pFEstIn;

  // // The observed tilt of the sensor ({S})
  // TimedOrientation3D m_rpyS;
  // OutPort<TimedOrientation3D> m_rpySOut;
  
  stateObservation::Vector3 m_pF_prev;
  /// Instance of the Tilt Estimator
  stateObservation::Vector xk_;

  bool firstSample_;
};

} // namespace mc_state_observation
