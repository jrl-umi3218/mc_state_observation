#include <mc_rtc/gui.h>
#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/type_name.h>

namespace  mc_rtc
{
namespace gui
{

auto make_checkbox(std::string name, bool & variable)
{
  auto GetF = [&variable]()
              {
               return variable;
              };
  auto SetF = [&variable]()
              {
               variable = !variable;
              };
  return CheckboxImpl<decltype(GetF), decltype(SetF)>(name, GetF, SetF);
}

template<typename T>
auto make_number_input(std::string name, T & value)
{
  auto GetF = [&value]()
              {
               return value;
              };
  auto SetF = [&value](const T & newValue)
              {
               value = newValue;
              };
  return NumberInputImpl<decltype(GetF), decltype(SetF)>(name, GetF, SetF);
}

auto make_xyz_input(std::string name, Eigen::Vector3d & vec)
{
  auto GetF = [&vec]() -> Eigen::Vector3d { return vec; };
  auto SetF = [&vec](const Eigen::Vector3d & vecIn) { vec = vecIn; };
  return ArrayInputImpl<decltype(GetF), decltype(SetF)>(name, {"x", "y", "z"}, GetF, SetF);
}

auto make_rpy_input(std::string name, Eigen::Matrix3d & orientation)
{
  auto GetF = [&orientation]() -> Eigen::Vector3d
              {
               return mc_rbdyn::rpyFromMat(orientation) * 180./mc_rtc::constants::PI;
              };
  auto SetF = [&orientation](const Eigen::Vector3d & rpy)
              {
               orientation = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI/180);
              };
  return ArrayInputImpl<decltype(GetF), decltype(SetF)>(name,
                    {"r [deg]", "p [deg]", "y [deg]"},
                    GetF, SetF);
}

auto make_motionvecd_input(std::string name, sva::MotionVecd & vel)
{
  auto GetF = [&vel]() -> const sva::MotionVecd &
              {
               return vel;
              };
  auto SetF = [&vel](const sva::MotionVecd & newVel)
              {
                vel = newVel;
              };
  return ArrayInputImpl<decltype(GetF), decltype(SetF)>(name,
                    {"wx [rad/s]", "wy [rad/s]", "wz [rad/s]", "vx [m/s]", "vy [m/s]", "vz [m/s]"},
                    GetF, SetF);
}

auto make_admittancevecd_input(std::string name, sva::AdmittanceVecd & vel)
{
  auto GetF = [&vel]() -> Eigen::Vector6d
              {
               return vel.vector();
              };
  auto SetF = [&vel](const Eigen::Vector6d & newVel)
              {
                vel = newVel;
              };
  return ArrayInputImpl<decltype(GetF), decltype(SetF)>(name,
                    {"wx", "wy", "wz", "vx", "vy", "vz"},
                    GetF, SetF);
}

auto make_rpy_label(std::string name, const Eigen::Matrix3d & orientation)
{
  auto GetF = [&orientation]() -> Eigen::Vector3d
              {
               return mc_rbdyn::rpyFromMat(orientation) * 180./mc_rtc::constants::PI;
              };
  return ArrayLabel<decltype(GetF)>(name,
                    {"r [deg]", "p [deg]", "y [deg]"},
                    GetF);
}

auto make_label(std::string name, std::string && label)
{
  auto GetF =
    [label]()
    {
      return label;
    };
  return Label<decltype(GetF)>(name, GetF);
}

auto make_label(std::string name, const std::string & label)
{
  auto GetF =
    [&label]()
    {
      return label;
    };
  return Label<decltype(GetF)>(name, GetF);
}

auto make_input_element(const std::string & name, sva::MotionVecd & ref)
{
  return make_motionvecd_input(name, ref);
}

auto make_input_element(const std::string & name, sva::AdmittanceVecd & ref)
{
  return make_admittancevecd_input(name, ref);
}

auto make_input_element(const std::string & name, bool ref)
{
  return make_checkbox(name, ref);
}

template<typename T,
         typename std::enable_if<std::is_arithmetic<T>::value, int>::type = 0>
auto make_input_element(const std::string & name, T ref)
{
  return make_number_input(name, ref);
}

} /* gui */

} /*  mc_rtc
 */

