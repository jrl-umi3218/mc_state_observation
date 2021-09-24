#include <mc_observers/ObserverMacros.h>
#include <mc_control/MCController.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <mc_state_observation/SLAMObserver.h>
#include <mc_state_observation/gui_helpers.h>
#include <mc_rtc/ros.h>
#include <mc_rtc/version.h>
#include <mc_rtc/constants.h>
#include <SpaceVecAlg/Conversions.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <tf2_eigen/tf2_eigen.h>
#include <random>

namespace
{

template <typename Derived>
Derived generate(const Derived& lower, const Derived& upper)
{
  assert(lower.size() == upper.size());

  std::random_device rd;  //Will be used to obtain a seed for the random number engine
  std::mt19937 gen(rd()); //Standard mersenne_twister_engine seeded with rd()

  Derived noise = Derived::Zero();
  for(int i=0; i<lower.size(); i++)
  {
    std::uniform_real_distribution<> dis(lower[i], upper[i]);
    noise(i) = dis(gen);
  }
  return noise;
}

sva::PTransformd apply(const sva::PTransformd& X, const Eigen::Vector3d& min_ori, const Eigen::Vector3d &max_ori, const Eigen::Vector3d& min_t, const Eigen::Vector3d& max_t)
{
  const Eigen::Vector3d& noise_t = generate(min_t, max_t);
  const Eigen::Vector3d& noise_ori = generate(min_ori, max_ori);
  const Eigen::Matrix3d& noise_R = mc_rbdyn::rpyToMat(noise_ori.x(), noise_ori.y(), noise_ori.z());

  Eigen::Vector3d t_noisy = X.translation() + noise_t;
  Eigen::Matrix3d R_noisy = (X.rotation() * noise_R).eval();
  return sva::PTransformd(R_noisy, t_noisy);
}

}

namespace mc_state_observation
{

SLAMObserver::SLAMObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), nh_(mc_rtc::ROSBridge::get_node_handle())
{}

void SLAMObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  if(config.has("Robot"))
  {
    robot_ = config("Robot")("robot", ctl.robot().name());
    camera_ = static_cast<std::string>(config("Robot")(robot_)("camera"));
    if(!ctl.robot(robot_).hasBody(camera_))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("No {} body found in {}", camera_, robot_);
    }
    body_ = ctl.robot(robot_).mb().bodies()[0].name();
    robots_.load({ctl.robot(robot_).module()});
  }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Robot configuration is mandatory.", name());
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
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}}] SLAM configuration is mandatory.", name());
  }

  int m = 150;
  int d = 0;
  int n = 5;
  if(config.has("Filter"))
  {
    isFiltered_ = config("Filter")("use", true);
    m = config("Filter")("m", 150);
    d = config("Filter")("d", 0);
    n = config("Filter")("n", 5);
  }

  auto sg_conf = gram_sg::SavitzkyGolayFilterConfig(m, m, n, d);
  filter_.reset(new filter::Transform(sg_conf));

  if(config.has("Publish"))
  {
    isPublished_ = config("Publish")("use", true);
  }

  if(config.has("Simulation"))
  {
    isSimulated_ = config("Simulation")("use", true);
    if(isSimulated_ && config("Simulation").has("noise"))
    {
      auto noise = config("Simulation")("noise");
      isUsingNoise_ =  noise("use", false);
      if(noise.has("translation"))
      {
        minTranslationNoise_ = noise("translation")("min", Eigen::Vector3d(-0.05, -0.05, -0.05));
        maxTranslationNoise_ = noise("translation")("max", Eigen::Vector3d(0.05, 0.05, 0.05));
      }
      if(noise.has("orientation"))
      {
        minOrientationNoise_ = noise("orientation")("min", Eigen::Vector3d(-1, -1, -1));
        minOrientationNoise_.unaryExpr(&mc_rtc::constants::toRad);
        maxOrientationNoise_ = noise("orientation")("max", Eigen::Vector3d(1, 1, 1));
        maxOrientationNoise_.unaryExpr(&mc_rtc::constants::toRad);
      }
    }
    if(isSimulated_)
    {
      estimated_ = "real/"+camera_;
      mc_rtc::log::info("[{}] Simulation mode is active so SLAM estimated link is set to {}", name(), estimated_);
    }
  }

  desc_ = fmt::format("{} (Camera: {} Estimated: {}, inSimulation: {})", name(), camera_, estimated_, isSimulated_);

  thread_ = std::thread(std::bind(&SLAMObserver::rosSpinner, this));
}

void SLAMObserver::reset(const mc_control::MCController &)
{}

bool SLAMObserver::run(const mc_control::MCController & ctl)
{
  isSLAMAlive_ = false;

  t_ += ctl.solver().dt();

  auto getTransformStamped = [this](const std::string & origin, const std::string & to, geometry_msgs::TransformStamped & transformStamped) -> bool
  {
    try
    {
      transformStamped = tfBuffer_.lookupTransform(origin, to, ros::Time(0));
    }
    catch(tf2::TransformException & ex)
    {
      error_ = ex.what();
      return false;
    }
    return true;
  };

  geometry_msgs::TransformStamped transformStamped;
  if(!isInitialized_)
  {
    std::string origin = map_;
    if(isSimulated_)
    {
      origin = "robot_map";
    }
    if(!getTransformStamped(origin, estimated_, transformStamped))
    {
      error_ = fmt::format("[{}] Could not get transform from {} to {}", name(), origin, estimated_);
      isSLAMAlive_ = false;
      return false;
    }
    X_Slam_Estimated_Camera_ = sva::conversions::fromHomogeneous(tf2::transformToEigen(transformStamped).matrix());
    return true;
  }
  else
  {
    // Connect SLAM and Robot map
    auto transform = tf2::eigenToTransform(sva::conversions::toAffine(X_0_Slam_));
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "robot_map";
    transform.child_frame_id = map_;
    tfBroadcaster_.sendTransform(transform);

    if(isSimulated_)
    {
      const sva::PTransformd X_0_FFsensor(ctl.robot().bodySensor().orientation(), ctl.robot().bodySensor().position());
      const auto & real_robot = ctl.realRobot();
      const sva::PTransformd X_0_FF = real_robot.bodyPosW(body_);
      const sva::PTransformd X_0_Camera = real_robot.bodyPosW(camera_);
      const sva::PTransformd X_Camera_Freeflyer =  X_0_FF * X_0_Camera.inv();
      X_0_Estimated_camera_ = X_Camera_Freeflyer.inv() * X_0_FFsensor;
      if(isUsingNoise_)
      {
        X_0_Estimated_camera_ = apply(X_0_Estimated_camera_, minOrientationNoise_, maxOrientationNoise_, minTranslationNoise_, maxTranslationNoise_);
      }
    }
    else
    {
      if(!getTransformStamped("robot_map", estimated_, transformStamped))
      {
        error_ = fmt::format("[{}] Could not get transform from \"{}\" to \"{}\"", name(), "robot_map", estimated_);
        return false;
      }
      X_0_Estimated_camera_ = sva::conversions::fromHomogeneous(tf2::transformToEigen(transformStamped).matrix());
      if(!getTransformStamped("robot_map", ground_, transformStamped))
      {
        error_ = fmt::format("[{}] Could not get transform from \"{}\" to \"{}\"", name(), "robot_map", ground_);
      }
      else
      {
        X_Slam_Ground_ = sva::conversions::fromHomogeneous(tf2::transformToEigen(transformStamped).matrix());
      }
    }
    return true;
  }
  t_ += ctl.solver().dt();
}

void SLAMObserver::update(mc_control::MCController & ctl)
{
  if(!isInitialized_)
  {
    if(ctl.datastore().has("SLAM::Robot"))
    {
      ctl.datastore().remove("SLAM::Robot");
    }
    return;
  }

  if(!ctl.datastore().has("SLAM::Robot"))
  {
    ctl.datastore().make_call("SLAM::Robot",
      [this]() -> const mc_rbdyn::Robot &
      {
        return robots_.robot();
      }
    );
  }

  if(!ctl.datastore().has("SLAM::isAlive"))
  {
     ctl.datastore().make_call("SLAM::isAlive",
      [this]() -> bool
      {
        return isSLAMAlive_;
      }
    );
  }

  if(!ctl.datastore().has("SLAM::X_S_Ground"))
  {
     ctl.datastore().make_call("SLAM::X_S_Ground",
      [this]() -> const sva::PTransformd &
      {
        return X_Slam_Ground_;
      }
    );
  }

  isSLAMAlive_ = true;

  const auto & real_robot = ctl.realRobot();
  auto & SLAM_robot = robots_.robot();
  SLAM_robot.mbc().q = real_robot.mbc().q;
  const sva::PTransformd X_0_FF = real_robot.bodyPosW(body_);
  const sva::PTransformd X_0_Camera = real_robot.bodyPosW(camera_);
  const sva::PTransformd X_Camera_Freeflyer =  X_0_FF * X_0_Camera.inv();

  sva::PTransformd X_0_Estimated_Freeflyer = X_Camera_Freeflyer * X_0_Estimated_camera_;
  if(isFiltered_)
  {
    filter_->add(X_0_Estimated_camera_);
    if(filter_->ready())
    {
      X_0_Filtered_estimated_camera_ = filter_->filter();
      X_0_Estimated_Freeflyer = X_Camera_Freeflyer * X_0_Filtered_estimated_camera_;
    }
  }

  SLAM_robot.posW(X_0_Estimated_Freeflyer);
  SLAM_robot.forwardKinematics();

  if(isPublished_)
  {
    mc_rtc::ROSBridge::update_robot_publisher("SLAM", ctl.timeStep, SLAM_robot);
  }
}

void SLAMObserver::addToLogger(const mc_control::MCController &, mc_rtc::Logger & logger, const std::string & category)
{
  logger.addLogEntry(category+"_LeftFoot", [this]() { return (robots_.size() == 1 ? robots_.robot().surfacePose("LeftFoot") : sva::PTransformd::Identity()); });
  logger.addLogEntry(category+"_LeftFootCenter", [this]() { return (robots_.size() == 1 ? robots_.robot().surfacePose("LeftFootCenter") : sva::PTransformd::Identity()); });
  logger.addLogEntry(category+"_RightFoot", [this]() { return (robots_.size() == 1 ? robots_.robot().surfacePose("RightFoot") : sva::PTransformd::Identity()); });
  logger.addLogEntry(category+"_RightFootCenter", [this]() { return (robots_.size() == 1 ? robots_.robot().surfacePose("RightFootCenter") : sva::PTransformd::Identity()); });
  logger.addLogEntry(category+"_com", [this]() { return (robots_.size() == 1 ? robots_.robot().com() : sva::PTransformd::Identity()); });
  logger.addLogEntry(category+"_comd", [this]() { return (robots_.size() == 1 ? robots_.robot().comVelocity() : sva::PTransformd::Identity()); });
  logger.addLogEntry(category+"_posW", [this]() { return (robots_.size() == 1 ? robots_.robot().posW() : sva::PTransformd::Identity()); });
  logger.addLogEntry(category+"_camera", [this]() { return X_0_Estimated_camera_; });
  logger.addLogEntry(category+"_cameraFiltered", [this]() { return X_0_Filtered_estimated_camera_; });
}

void SLAMObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category+"_LeftFoot");
  logger.removeLogEntry(category+"_LeftFootCenter");
  logger.removeLogEntry(category+"_RightFoot");
  logger.removeLogEntry(category+"_RightFootCenter");
  logger.removeLogEntry(category+"_com");
  logger.removeLogEntry(category+"_comd");
  logger.removeLogEntry(category+"_posW");
  logger.removeLogEntry(category+"_camera");
  logger.removeLogEntry(category+"_cameraFiltered");
}

void SLAMObserver::addToGUI(const mc_control::MCController & ctl,
                             mc_rtc::gui::StateBuilder & gui,
                             const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;

  gui.addElement(category,
    Transform("X_S_Ground", [this](){ return X_Slam_Ground_; }),
    Button("Initialize",
      [this, &ctl]()
      {
        if(!isInitialized_)
        {
          isInitialized_ = true;
          const auto & real_robot = ctl.realRobot(robot_);
          const auto & X_0_Camera = real_robot.bodyPosW(camera_);
          X_0_Slam_ = X_Slam_Estimated_Camera_.inv() * X_0_Camera;
          filter_->reset();
        }
      }),
    Transform("X_0_Slam", [this](){ return X_0_Slam_; }),
    ArrayLabel("X_Slam_Camera", {"r", "p", "y", "x", "y", "z"},
      [this]()
      {
        Eigen::VectorXd ret(6);
        sva::PTransformd X_Slam_Camera = X_0_Estimated_camera_ * X_0_Slam_.inv();
        ret << mc_rbdyn::rpyFromMat(X_Slam_Camera.rotation()), X_Slam_Camera.translation();
        return ret;
      }
    ),
    Label("Is Observer initialized?", [this](){ return (isInitialized_ ? "Yes" : "No"); }),
    Button("Delete initialization", [this](){ isInitialized_ = false; })
  );

  std::vector<std::string> categoryFilter = category;
  categoryFilter.push_back("Filter");
  gui.addElement(categoryFilter,
    Button("Toggle filter",
      [this]()
      {
        isFiltered_ = ! isFiltered_;
        if(isFiltered_)
        {
          filter_->reset();
        }
      }),
    Label("Apply filter:", [this](){ return (isFiltered_ ? "yes" : "no"); }),
    ArrayInput("Filter config", {"m", "n", "d"},
      [this]()
      {
        return Eigen::Vector3d(filter_->config().m, filter_->config().s, filter_->config().n);
      },
      [this](const Eigen::Vector3d & v)
      {
        int m = static_cast<int>(v.x());
        int d = static_cast<int>(v.y());
        int n = static_cast<int>(v.z());
        auto sg_conf = gram_sg::SavitzkyGolayFilterConfig(m, m, n, d);
        filter_.reset(new filter::Transform(sg_conf));
        filter_->reset();
      }
    )
  );

  if(isSimulated_)
  {
    std::vector<std::string> categoryNoise = category;
    categoryNoise.push_back("Noise");
    gui.addElement(categoryNoise,
      Button("Toggle noise", [this](){ isUsingNoise_ = ! isUsingNoise_; }),
      Label("Apply noise:", [this](){ return (isUsingNoise_ ? "Yes" : "No"); }),
      ArrayInput("Translation::Min", {"x", "y", "z"},
        [this]() { return minTranslationNoise_; },
        [this](const Eigen::Vector3d & v) { minTranslationNoise_ = v; }),
      ArrayInput("Translation::Max", {"x", "y", "z"},
        [this]() { return maxTranslationNoise_; },
        [this](const Eigen::Vector3d & v) { maxTranslationNoise_ = v; }),
      ArrayInput("Orientation::Min", {"r", "p", "y"},
        [this]()
        {
          Eigen::Vector3d inDegree = minOrientationNoise_;
          inDegree.unaryExpr(&mc_rtc::constants::toDeg);
          return inDegree;
        },
        [this](const Eigen::Vector3d & v)
        {
          v.unaryExpr(&mc_rtc::constants::toRad);
          minOrientationNoise_ = v;
        }
        ),
      ArrayInput("Orientation::Max", {"r", "p", "y"},
        [this]()
        {
          Eigen::Vector3d inDegree = maxOrientationNoise_;
          inDegree.unaryExpr(&mc_rtc::constants::toDeg);
          return inDegree;
        },
        [this](const Eigen::Vector3d & v)
        {
          v.unaryExpr(&mc_rtc::constants::toRad);
          maxOrientationNoise_ = v;
        }
      )
    );
  }

  gui.addPlot("SLAM::Translation",
    mc_rtc::gui::plot::X("t", [this]() { return t_; }),
    mc_rtc::gui::plot::Y("x_f", [this]() { return X_0_Filtered_estimated_camera_.translation().x(); }, mc_rtc::gui::Color::Red),
    mc_rtc::gui::plot::Y("x", [this]() { return X_0_Estimated_camera_.translation().x(); }, mc_rtc::gui::Color::Red, mc_rtc::gui::plot::Style::Dotted),
    mc_rtc::gui::plot::Y("y_f", [this]() { return X_0_Filtered_estimated_camera_.translation().y(); }, mc_rtc::gui::Color::Green),
    mc_rtc::gui::plot::Y("y", [this]() { return X_0_Estimated_camera_.translation().y(); }, mc_rtc::gui::Color::Green, mc_rtc::gui::plot::Style::Dotted),
    mc_rtc::gui::plot::Y("z_f", [this]() { return X_0_Filtered_estimated_camera_.translation().z(); }, mc_rtc::gui::Color::Blue),
    mc_rtc::gui::plot::Y("z", [this]() { return X_0_Estimated_camera_.translation().z(); }, mc_rtc::gui::Color::Blue, mc_rtc::gui::plot::Style::Dotted)
  );

  gui.addPlot("SLAM::Rotation",
    mc_rtc::gui::plot::X("t", [this]() { return t_; }),
    mc_rtc::gui::plot::Y("r_f",
      [this]()
      {
        Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(X_0_Filtered_estimated_camera_.rotation());
        return rpy.x();
      }, mc_rtc::gui::Color::Red),
    mc_rtc::gui::plot::Y("r",
      [this]()
      { Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(X_0_Estimated_camera_.rotation());
        return rpy.x();
      }, mc_rtc::gui::Color::Red, mc_rtc::gui::plot::Style::Dotted),
    mc_rtc::gui::plot::Y("p_f",
      [this]()
      {
        Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(X_0_Filtered_estimated_camera_.rotation());
        return rpy.y();
      }, mc_rtc::gui::Color::Blue),
    mc_rtc::gui::plot::Y("p",
      [this]()
      { Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(X_0_Estimated_camera_.rotation());
        return rpy.y();
      }, mc_rtc::gui::Color::Blue, mc_rtc::gui::plot::Style::Dotted),
    mc_rtc::gui::plot::Y("y_f",
      [this]()
      {
        Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(X_0_Filtered_estimated_camera_.rotation());
        return rpy.z();
      }, mc_rtc::gui::Color::Green),
    mc_rtc::gui::plot::Y("y",
      [this]()
      { Eigen::Vector3d rpy = mc_rbdyn::rpyFromMat(X_0_Estimated_camera_.rotation());
        return rpy.z();
      }, mc_rtc::gui::Color::Green, mc_rtc::gui::plot::Style::Dotted)
  );
}

void SLAMObserver::rosSpinner()
{
  mc_rtc::log::info("[{}] rosSpinner started", name());
  ros::Rate rate(30);
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }
  mc_rtc::log::info("[{}] rosSpinner finished", name());
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("SLAM", mc_state_observation::SLAMObserver)
