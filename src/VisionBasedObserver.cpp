#include <mc_state_observation/VisionBasedObserver.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/ros.h>
#include <mc_rtc/version.h>
#include <SpaceVecAlg/Conversions.h>
#include <mc_state_observation/gui_helpers.h>

// Remove if using C++20
namespace std
{
  double lerp(double a, double b, double f)
  {
    return a + f * (b - a);
  }
}

namespace mc_state_observation
{

VisionBasedObserver::VisionBasedObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt),
  nh_(mc_rtc::ROSBridge::get_node_handle())
{
}

void VisionBasedObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  if(config.has("Robot"))
  {
    robot_ = config("Robot")("robot", ctl.robot().name());
    camera_ = static_cast<std::string>(config("Robot")(robot_)("camera"));
    if(!ctl.robot(robot_).hasBody(camera_))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("No {} body found in {}", camera_, robot_);
    }
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Robot configuration is mandatory.", name());
  }

  int m = 50;
  int d = 0;
  int n = 3;
  if(config.has("Filter"))
  {
    isFiltered_ = config("Filter")("use", false);
    m = config("Filter")("m", static_cast<int>(m));
    d = config("Filter")("d", static_cast<int>(d));
    n = config("Filter")("n", static_cast<int>(n));
  }

  auto sg_conf = gram_sg::SavitzkyGolayFilterConfig(m, m, n, d);
  filter_.reset(new filter::Transform(sg_conf));

  if(config.has("past"))
  {
    past_ = config("past");
  }

  if(config.has("rate"))
  {
    rate_ = config("rate");
  }

  pastRobotData_ = boost::circular_buffer<mc_rbdyn::RobotDataStamped>(
    static_cast<long unsigned int>(std::ceil(past_ / ctl.solver().dt())));
}

void VisionBasedObserver::reset(const mc_control::MCController &)
{
}

bool VisionBasedObserver::run(const mc_control::MCController & ctl)
{
  // Keep in memory the past camera of real robot
  pastRobotData_.push_back({ctl.realRobot().posW(), ctl.realRobot().bodyPosW(camera_),
    ctl.realRobot().mbc().q, ros::Time::now().toSec()});
  t_ += ctl.solver().dt();

  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if(!isEstimatorAlive_)
    {
      error_ = fmt::format("[{}] The estimator is no more alive.", name());
      return false;
    }
  }

  {
    const std::lock_guard<std::mutex> lock(mutex_);
    if(!isNewEstimatedPose_)
    {
      return true;
    }
  }

  updatePose();

  estimatedPose_ = pose_.pose;
  filter_->add(estimatedPose_);
  if(filter_->ready())
  {
    estimatedPoseFiltered_ = filter_->filter();
  }

  robotData_ = robotCameraPoseEstimatedAtStamped(pose_.stamp);

  return true;
}

void VisionBasedObserver::update(mc_control::MCController &)
{
}

void VisionBasedObserver::addToLogger(const mc_control::MCController & ctl,
                                 mc_rtc::Logger & logger,
                                 const std::string & category)
{
  logger.addLogEntry(category + "_estimated_pose", [this]() { return estimatedPose_; });
  logger.addLogEntry(category + "_estimated_filteredPose", [this]() { return estimatedPoseFiltered_; });

  logger.addLogEntry(category + "_camera_control", [this, &ctl]() { return ctl.robot().bodyPosW(camera_); });
  logger.addLogEntry(category + "_camera_real", [this, &ctl]() { return ctl.realRobot().bodyPosW(camera_); });
  logger.addLogEntry(category + "_camera_interpolate", [this]() { return camera(); });

  logger.addLogEntry(category + "_robot_control", [this, &ctl]() { return ctl.robot().posW(); });
  logger.addLogEntry(category + "_robot_real", [this, &ctl]() { return ctl.realRobot().posW(); });
  logger.addLogEntry(category + "_robot_interpolate", [this]() { return freeflyer(); });
}

void VisionBasedObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_estimated_pose");
  logger.removeLogEntry(category + "_estimated_filteredPose");

  logger.removeLogEntry(category + "_camera_control");
  logger.removeLogEntry(category + "_camera_real");
  logger.removeLogEntry(category + "_camera_interpolate");

  logger.removeLogEntry(category + "_robot_control");
  logger.removeLogEntry(category + "_robot_real");
  logger.removeLogEntry(category + "_robot_interpolate");

}

void VisionBasedObserver::addToGUI(const mc_control::MCController &,
                              mc_rtc::gui::StateBuilder & gui,
                              const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;

  std::vector<std::string> categoryFilter = category;
  categoryFilter.push_back("Filter");
  gui.addElement(
    categoryFilter,
    Button("Toggle filter",
            [this]() {
              isFiltered_ = !isFiltered_;
              if(isFiltered_)
              {
                filter_->reset();
              }
            }),
    Label("Apply filter:", [this]() { return (isFiltered_ ? "yes" : "no"); }),
    ArrayInput("Filter config", {"m", "d", "n"},
                [this]() { return Eigen::Vector3d(filter_->config().m, filter_->config().s, filter_->config().n); },
                [this](const Eigen::Vector3d & v) {
                  int m = static_cast<int>(v.x());
                  int d = static_cast<int>(v.y());
                  int n = static_cast<int>(v.z());
                  auto sg_conf = gram_sg::SavitzkyGolayFilterConfig(m, m, n, d);
                  filter_.reset(new filter::Transform(sg_conf));
                  filter_->reset();
                }));


  gui.addPlot(name()+"_translation", mc_rtc::gui::plot::X("t", [this]() { return t_; }),
              // mc_rtc::gui::plot::Y("x_f", [this]() { return estimatedPoseFiltered_.translation().x(); },
              //                      mc_rtc::gui::Color::Red),
              // mc_rtc::gui::plot::Y("x", [this]() { return estimatedPose_.translation().x(); },
              //                      mc_rtc::gui::Color::Red, mc_rtc::gui::plot::Style::Dotted),
              mc_rtc::gui::plot::Y("y_f", [this]() { return estimatedPoseFiltered_.translation().y(); },
                                   mc_rtc::gui::Color::Red),
              mc_rtc::gui::plot::Y("y", [this]() { return estimatedPose_.translation().y(); },
                                   mc_rtc::gui::Color::Blue, mc_rtc::gui::plot::Style::Dotted)
              // mc_rtc::gui::plot::Y("z_f", [this]() { return estimatedPoseFiltered_.translation().z(); },
              //                      mc_rtc::gui::Color::Blue),
              // mc_rtc::gui::plot::Y("z", [this]() { return estimatedPose_.translation().z(); },
              //                      mc_rtc::gui::Color::Blue, mc_rtc::gui::plot::Style::Dotted)
  );
}

mc_rbdyn::RobotDataStamped VisionBasedObserver::robotCameraPoseEstimatedAtStamped(double stamp) const
{
  mc_rbdyn::RobotDataStamped previous;

  int i = static_cast<int>(pastRobotData_.size()) - 1;
  for(; i >= 0; --i)
  {
    previous = pastRobotData_[i];
    if(previous.stamp < stamp)
    {
      break;
    }
  }

  if(i >= 0 && i <= static_cast<int>(pastRobotData_.size()) - 2)
  {
    const mc_rbdyn::RobotDataStamped & next = pastRobotData_[i + 1];
    // interpolate
    double ratio = (stamp - previous.stamp) / (next.stamp - previous.stamp);
    previous.camera = sva::interpolate(previous.camera, next.camera, ratio);
    previous.freeflyer = sva::interpolate(previous.freeflyer, next.freeflyer, ratio);

    for(size_t m = 1; m < previous.q.size(); ++m)
    {
      for(size_t n = 0; n < previous.q[m].size(); ++n)
      {
        previous.q[m][n] = std::lerp(previous.q[m][n], next.q[m][n], ratio);
      }
    }
  }

  return previous;
}


const sva::PTransformd & VisionBasedObserver::camera() const
{
  return robotData_.camera;
}

const sva::PTransformd & VisionBasedObserver::freeflyer() const
{
  return robotData_.freeflyer;
}

const std::vector<std::vector<double>> & VisionBasedObserver::q() const
{
  return robotData_.q;
}

void VisionBasedObserver::rosSpinner()
{
  mc_rtc::log::info("[{}] rosSpinner started", name());
  ros::Rate rate(rate_); // At least 2 times the rate of the estimation
  while(ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  mc_rtc::log::info("[{}] rosSpinner finished", name());
}

} // namespace mc_state_observation