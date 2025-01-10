#pragma once

#include <mc_state_observation/measurements/Contact.h>

namespace mc_state_observation::measurements
{
/**
 * Object that contains all the functions and necessary information making easier the handling of contacts associated to
 * a sensor within the observers.

 * If the contact is detected using a thresholding on the contact force, the contact force cannot be obtained and the
 * name of the contact will be the one of the force sensor. Otherwise the name of the contact surface is used, allowing
 * the creation of contacts associated to a same sensor but a different surface.
 **/
struct ContactWithSensor : public Contact
{

public:
  inline ContactWithSensor() = default;
  // constructor if the contact is not associated to a surface
  // its name will be the name of the force sensor
  ContactWithSensor(unsigned id, std::string_view forceSensorName)
  : Contact(id, forceSensorName), forceSensor_(forceSensorName)
  {
  }

  // constructor if the contact is associated to a surface
  // its name will be the name of the force sensor
  ContactWithSensor(unsigned id, std::string_view forceSensorName, std::string_view surfaceName)
  : Contact(id, forceSensorName, surfaceName), forceSensor_(forceSensorName)
  {
  }

  inline void forceNorm(double forceNorm) { forceNorm_ = forceNorm; }

  inline const std::string & forceSensor() const noexcept { return forceSensor_; }
  inline double forceNorm() const noexcept { return forceNorm_; }

protected:
  std::string forceSensor_;
  double forceNorm_ = 0.0;
};
} // namespace mc_state_observation::measurements
