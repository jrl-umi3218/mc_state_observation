#pragma once

#include <mc_state_observation/VisionBasedObserver.h>
#include <geometry_msgs/PoseStamped.h>

namespace mc_state_observation
{

struct ObjectObserver2 : public VisionBasedObserver
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ObjectObserver2(const std::string & type, double dt);

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

  void updatePose() override;

protected:
  /*! \brief Callback for ros topic_ defined in the configuration file
   *
   * @param msg Message sent by the vision process containing estimated object information
   */
  void callback(const geometry_msgs::PoseStamped & msg);
  void callback2(const geometry_msgs::PoseStamped & msg);

  /// @{
  std::string object_ = ""; ///< Name of map TF in ROS
  std::string topic_ = ""; ///< Name of estimated camera TF in ROS
  ros::Subscriber subscriber_; ///< Subscribe to topic_ name
  ros::Subscriber subscriber2_; ///< Subscribe to topic_ name
  sva::PTransformdStamped poseFromTopic_ = {sva::PTransformd::Identity(), 0.0};
  /// @}

  // Should be deleted in "production"
  sva::PTransformdStamped trueCameraPose_ = {sva::PTransformd::Identity(), 0.0};
  sva::PTransformd truePose_ = sva::PTransformd::Identity();

};
} // namespace mc_state_observation
