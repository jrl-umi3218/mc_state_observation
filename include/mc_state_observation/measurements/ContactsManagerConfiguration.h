#include <string>
#include <variant>
#include <vector>

namespace mc_state_observation::measurements
{
namespace internal
{

/// @brief Configuration structure that helps setting up the parameters of a ContactsManager.
/// @details The configuration is used once passed in the ContactManager's init function
template<class ConfigurationType>
struct ContactsManagerConfigurationPrvt
{
public:
  inline ContactsManagerConfigurationPrvt(const std::string & observerName) noexcept : observerName_(observerName) {}

  inline ConfigurationType & contactDetectionThreshold(double contactDetectionThreshold) noexcept
  {
    contactDetectionThreshold_ = contactDetectionThreshold;
    return static_cast<ConfigurationType &>(*this);
  }
  inline ConfigurationType & verbose(bool verbose) noexcept
  {
    verbose_ = verbose;
    return static_cast<ConfigurationType &>(*this);
  }

public:
  std::string observerName_;

  double contactDetectionThreshold_ = 0.11;
  bool verbose_ = true;
};
} // namespace internal

/// @brief Configuration structure that helps setting up the parameters of a ContactsManager whose contacts detection
/// method is based on contact surfaces.
/// @details The configuration is used once passed in the ContactManager's init function
struct ContactsManagerSurfacesConfiguration
: public internal::ContactsManagerConfigurationPrvt<ContactsManagerSurfacesConfiguration>
{
public:
  inline ContactsManagerSurfacesConfiguration(const std::string & observerName,
                                              const std::vector<std::string> & surfacesForContactDetection) noexcept
  : ContactsManagerConfigurationPrvt<ContactsManagerSurfacesConfiguration>(observerName),
    surfacesForContactDetection_(surfacesForContactDetection)
  {
  }

  inline ContactsManagerSurfacesConfiguration & contactSensorsDisabledInit(
      const std::vector<std::string> & contactSensorsDisabledsInit) noexcept
  {
    contactSensorsDisabledInit_ = contactSensorsDisabledsInit;
    return *this;
  }

  // list of admissible contact surfaces for the detection
  std::vector<std::string> surfacesForContactDetection_;
  // list of sensors that must not be used from the start of the observer
  std::vector<std::string> contactSensorsDisabledInit_ = std::vector<std::string>();
};

/// @brief Configuration structure that helps setting up the parameters of a ContactsManager whose contacts detection
/// method is based on the force sensor measurements.
/// @details The configuration is used once passed in the ContactManager's init function
struct ContactsManagerSensorsConfiguration
: public internal::ContactsManagerConfigurationPrvt<ContactsManagerSensorsConfiguration>
{

  inline ContactsManagerSensorsConfiguration(const std::string & observerName) noexcept
  : ContactsManagerConfigurationPrvt<ContactsManagerSensorsConfiguration>(observerName)
  {
  }

  inline ContactsManagerSensorsConfiguration & forceSensorsToOmit(
      const std::vector<std::string> & forceSensorsToOmit) noexcept
  {
    forceSensorsToOmit_ = forceSensorsToOmit;
    return *this;
  }
  inline ContactsManagerSensorsConfiguration & contactSensorsDisabledInit(
      const std::vector<std::string> & contactSensorsDisabledsInit) noexcept
  {
    contactSensorsDisabledInit_ = contactSensorsDisabledsInit;
    return *this;
  }

public:
  // force sensors that must not be used for the contacts detection (ex: hands when holding an object)
  std::vector<std::string> forceSensorsToOmit_;
  // list of sensors that must not be used from the start of the observer
  std::vector<std::string> contactSensorsDisabledInit_ = std::vector<std::string>();
};

/// @brief Configuration structure that helps setting up the parameters of a ContactsManager whose contacts detection
/// method is based on the solver.
/// @details The configuration is used once passed in the ContactManager's init function
struct ContactsManagerSolverConfiguration
: public internal::ContactsManagerConfigurationPrvt<ContactsManagerSolverConfiguration>
{
public:
  inline ContactsManagerSolverConfiguration(const std::string & observerName) noexcept
  : ContactsManagerConfigurationPrvt<ContactsManagerSolverConfiguration>(observerName)
  {
  }
};

} // namespace mc_state_observation::measurements
