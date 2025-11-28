#pragma once

#include <mc_state_observation/TiltObserver.h>

namespace mc_state_observation
{

struct TiltObserverOdometry : public TiltObserver
{
  /// @brief Constructor for the TiltObserver.
  /// @details The parameter is given only if the Tilt Observer is used as a backup by the Kinetics Observer
  TiltObserverOdometry(const std::string & type, double dt, bool asBackup = false);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

protected:
  /*! \brief update the robot pose in the world only for visualization purpose
   *
   * @param robot Robot to update
   */

  virtual void update(mc_rbdyn::Robot & robot) override;

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

  /// @brief Updates the real robot and/or the IMU signal using our estimation results
  /// @param ctl Controller
  void update(mc_control::MCController & ctl) override;

private:
  /**
   * @brief Updates the frames that are necessary for the state estimation when   using odometry.
   * @details In particular the kinematics of the anchor in the IMU frame.
   *
   * @param ctl Controller.
   * @param updatedRobot
   */
  virtual void updateNecessaryFrames(const mc_control::MCController & ctl,
                                     const mc_rbdyn::Robot & updatedRobot) override;

  /// @brief updates the pose and the velcoity of the floating base in the world frame using our estimation results
  void updatePoseAndVel();

  /// @brief Sets the type of the odometry
  /// @param newOdometryType The new type of odometry to use.
  void setOdometryType(stateObservation::odometry::OdometryType newOdometryType);

public:
  // estimated kinematics of the IMU in the world
  stateObservation::kine::Kinematics estimatedWorldImuKine_;
  // estimated kinematics of the floating base in the world
  stateObservation::kine::Kinematics estimatedWorldFbKine_;

private:
  using TiltContact = stateObservation::odometry::LoContact;
  using ContactsDetector = measurements::ContactsDetector<stateObservation::odometry::LoContact>;
  ContactsDetector contactsDetector_;

  stateObservation::odometry::LeggedOdometryManager odometryManager_; // manager for the legged odometry
};

} // namespace mc_state_observation
