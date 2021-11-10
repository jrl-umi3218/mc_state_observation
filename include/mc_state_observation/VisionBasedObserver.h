#pragma once

#include <mc_observers/Observer.h>
#include <mc_rbdyn/Robot.h>
#include <mc_control/MCController.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <ros/ros.h>
#include <mc_state_observation/filtering.h>
#include <boost/circular_buffer.hpp>
#include <thread>

namespace sva
{
  struct PTransformdStamped
  {
    sva::PTransformd pose = sva::PTransformd::Identity();
    double stamp = 0.0;
  };
}

namespace mc_rbdyn
{
  struct RobotDataStamped
  {
    sva::PTransformd freeflyer;
    sva::PTransformd camera;
    std::vector<std::vector<double>> q;
    double stamp = 0.0;
  };
}

namespace mc_state_observation
{

struct VisionBasedObserver : public mc_observers::Observer
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VisionBasedObserver(const std::string & type, double dt);

  virtual void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  virtual void update(mc_control::MCController & ctl) override;

protected:

  /*! \brief Add observer from logger
   *
   * @param category Category in which to log this observer
   */
  virtual void addToLogger(const mc_control::MCController &, mc_rtc::Logger &, const std::string & category) override;

  /*! \brief Remove observer from logger
   *
   * @param category Category in which this observer entries are logged
   */
  virtual void removeFromLogger(mc_rtc::Logger &, const std::string & category) override;

  /*! \brief Add observer information the GUI.
   *
   * @param category Category in which to add this observer
   */
  virtual void addToGUI(const mc_control::MCController &,
                mc_rtc::gui::StateBuilder &,
                const std::vector<std::string> & /* category */) override;

  mc_rbdyn::RobotDataStamped robotCameraPoseEstimatedAtStamped(double stamp) const;

  virtual void updatePose() = 0;

  const sva::PTransformd & camera() const;

  const sva::PTransformd & freeflyer() const;

  const std::vector<std::vector<double>> & q() const;

protected:

  /// @{
  double t_ = 0.0;
  /// @}

  /// @{
  std::string robot_ = ""; ///< Name of main robot
  std::string camera_ = ""; ///< Name of main robot's camera body
  /// @}

  /// @{
  bool isEstimatorAlive_ = false;
  bool isNewEstimatedPose_ = false;
  /// @}

  /// @{
  bool isFiltered_ = false; ///< Check if a filter is apply or not
  std::unique_ptr<filter::Transform> filter_; ///< Filter based on savitzky-golay
  sva::PTransformdStamped pose_ = {sva::PTransformd::Identity(), 0.0}; ///< Estimated pose
  /// @}

  /// @{
  double past_ = 1.0; ////< Time in s to keep the last camera poses
  boost::circular_buffer<mc_rbdyn::RobotDataStamped> pastRobotData_; ///< Keep the previous camera pose
  mc_rbdyn::RobotDataStamped robotData_; ///< Robot data with corresponding stamp of pose_ (i.e. of estimatedPose_)
  sva::PTransformd estimatedPoseFiltered_ = sva::PTransformd::Identity(); ///< Estimated filtered pose
  sva::PTransformd estimatedPose_ = sva::PTransformd::Identity();
  /// @}

  /// @{
  std::shared_ptr<ros::NodeHandle> nh_ = nullptr;
  std::thread thread_;
  std::mutex mutex_;
  double rate_ = 60.0;
  void rosSpinner();
  /// @}
};

}
