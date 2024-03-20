#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/io_utils.h>
#include <mc_state_observation/PressureForceSensorObserver.h>

namespace mc_state_observation
{

PressureForceSensorObserver::PressureForceSensorObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt)
{
}

void PressureForceSensorObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  if(config.has("estimatedSensor")) { forceSensor_ = static_cast<std::string>(config("estimatedSensor")); }
  else { mc_rtc::log::error_and_throw("[{}] forceSensor configuration is mandatory.", name()); }

  FSR_FL = static_cast<std::string>(config("pressureFrontLeft"));
  FSR_FR = static_cast<std::string>(config("pressureFrontRight"));
  FSR_RL = static_cast<std::string>(config("pressureRearLeft"));
  FSR_RR = static_cast<std::string>(config("pressureRearRight"));
  pressureSensors_ = {FSR_FL, FSR_FR, FSR_RL, FSR_RR};

  desc_ = fmt::format("{} (forceSensor={}, FSR_FL={}, FSR_FR={}, FSR_RL={}, FSR_RR={})", name_, forceSensor_, FSR_FL,
                      FSR_FR, FSR_RL, FSR_RR);
}

void PressureForceSensorObserver::reset(const mc_control::MCController & ctl) {}

bool PressureForceSensorObserver::run(const mc_control::MCController & ctl)
{
  auto & robot = ctl.robot(robot_);
  double DX_FL = robot.forceSensor(FSR_FL).X_p_s().translation().x();
  double DY_FL = robot.forceSensor(FSR_FL).X_p_s().translation().y();
  double DX_FR = robot.forceSensor(FSR_FR).X_p_s().translation().x();
  double DY_FR = robot.forceSensor(FSR_FR).X_p_s().translation().y();
  double DX_RL = robot.forceSensor(FSR_RL).X_p_s().translation().x();
  double DY_RL = robot.forceSensor(FSR_RL).X_p_s().translation().y();
  double DX_RR = robot.forceSensor(FSR_RR).X_p_s().translation().x();
  double DY_RR = robot.forceSensor(FSR_RR).X_p_s().translation().y();

  if(!manualOverride_)
  {
    FSR_FL_V = robot.forceSensor(FSR_FL).wrench().force().z();
    FSR_FR_V = robot.forceSensor(FSR_FR).wrench().force().z();
    FSR_RL_V = robot.forceSensor(FSR_RL).wrench().force().z();
    FSR_RR_V = robot.forceSensor(FSR_RR).wrench().force().z();
  }

  estimatedWrench_.couple() = {DY_FL * FSR_FL_V + DY_FR * FSR_FR_V + DY_RL * FSR_RL_V + DY_RR * FSR_RR_V,
                               -DX_FL * FSR_FL_V - DX_FR * FSR_FR_V - DX_RL * FSR_RL_V - DX_RR * FSR_RR_V, 0.0};
  estimatedWrench_.force().z() = FSR_FL_V + FSR_FR_V + FSR_RL_V + FSR_RR_V;

  return true;
}

void PressureForceSensorObserver::update(mc_control::MCController & ctl)
{
  auto & robot = ctl.robot(robot_);
  auto & data = *robot.data();
  auto & sensor = data.forceSensors.at(data.forceSensorsIndex.at(forceSensor_));
  sensor.wrench(estimatedWrench_);
}

void PressureForceSensorObserver::addToGUI(const mc_control::MCController & ctl,
                                           mc_rtc::gui::StateBuilder & gui,
                                           const std::vector<std::string> & category)
{
  gui.addElement(this, category,
                 mc_rtc::gui::ArrayLabel("Estimated Force", {"x [N]", "y [N]", "z [N]"},
                                         [this]() -> Eigen::Vector3d { return estimatedWrench_.force(); }));
  gui.addElement(this, category,
                 mc_rtc::gui::ArrayLabel("Estimated Couple", {"x [N.m]", "y [N.m]", "z [N.m]"},
                                         [this]() -> Eigen::Vector3d { return estimatedWrench_.couple(); }));

  gui.addElement(this, category, mc_rtc::gui::Input("Manual override", manualOverride_));
  auto addPressureSensor = [this, &category, &gui](const std::string & pressureSensorName, double & pressureValue)
  {
    gui.addElement(this, category,
                   mc_rtc::gui::NumberInput(
                       pressureSensorName, [&pressureValue]() -> double { return pressureValue; },
                       [&pressureValue](double f) { pressureValue = f; }));
  };
  addPressureSensor(FSR_FL, FSR_FL_V);
  addPressureSensor(FSR_FR, FSR_FR_V);
  addPressureSensor(FSR_RL, FSR_RL_V);
  addPressureSensor(FSR_RR, FSR_RR_V);
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("PressureForceSensor", mc_state_observation::PressureForceSensorObserver)
