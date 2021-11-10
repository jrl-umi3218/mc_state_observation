#pragma once

#include <mc_state_observation/VisionBasedObserver.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

namespace mc_state_observation
{

struct SLAMObserver2 : public VisionBasedObserver
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SLAMObserver2(const std::string & type, double dt);

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
  /// @{
  std::string body_ = ""; ///< Name of robot's freeflyer
  mc_rbdyn::Robots robots_; ///< Store robot estimated state
  /// @}

  /// @{
  std::string map_ = ""; ///< Name of map TF in ROS
  std::string estimated_ = ""; ///< Name of estimated camera TF in ROS
  bool triggerInitialization_ = false; ///< Check if the observer is initialized or not
  // sva::PTransformdStamped X_Slam_Estimated_Camera_ = {sva::PTransformd::Identity(), 0.0}; ///< Transformation to go from SLAM world to estimated camera
  /// @}

  /// @{
  std::string ground_ = "";
  sva::PTransformdStamped X_Slam_Ground_ = {sva::PTransformd::Identity(), 0.0}; ///< Ground frame in map frame
  /// @}

  /// @{
  std::vector<std::string> extraRobotsName_;
  /// @}

   /// @{
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_{tfBuffer_};
  tf2_ros::TransformBroadcaster tfBroadcaster_;
  tf2_ros::StaticTransformBroadcaster tfStaticBroadcaster_;
  /// @}
};
} // namespace mc_state_observation
