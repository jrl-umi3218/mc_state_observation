#pragma once

#include <mc_observers/Observer.h>

namespace mc_state_observation
{

struct PressureForceSensorObserver : public mc_observers::Observer
{
  PressureForceSensorObserver(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void update(mc_control::MCController & ctl) override;

protected:
  /*! \brief Add observer information the GUI.
   *
   * @param category Category in which to add this observer
   */
  void addToGUI(const mc_control::MCController &,
                mc_rtc::gui::StateBuilder &,
                const std::vector<std::string> & /* category */) override;

protected:
  std::string robot_ = "";
  std::string forceSensor_ = "";
  std::string FSR_FL = "";
  std::string FSR_FR = "";
  std::string FSR_RL = "";
  std::string FSR_RR = "";
  double FSR_FL_V = 0.0;
  double FSR_FR_V = 0.0;
  double FSR_RL_V = 0.0;
  double FSR_RR_V = 0.0;
  bool manualOverride_ = false;
  std::vector<std::string> pressureSensors_;
  sva::ForceVecd estimatedWrench_ = sva::ForceVecd::Zero();
};

} // namespace mc_state_observation
