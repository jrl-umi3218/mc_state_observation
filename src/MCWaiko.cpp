#include <mc_observers/ObserverMacros.h>

#include "mc_state_observation/measurements/measurements.h"
#include <mc_state_observation/MCWaiko.h>
#include <mc_state_observation/gui_helpers.h>
#include <state-observation/observer/waiko-humanoid.hpp>
#include <state-observation/tools/measurements-manager/measurements.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace mc_state_observation
{

namespace so = stateObservation;

using OdometryType = measurements::OdometryType;
using LoContactsManager = stateObservation::odometry::LeggedOdometryManager::ContactsManager;

MCWaiko::MCWaiko(const std::string & type, double dt, bool asBackup)
: mc_observers::Observer(type, dt), estimator_(dt, alpha_, beta_, 1 / (2 * M_PI), 0.5, 2), odometryManager_(dt)
{
  asBackup_ = asBackup;
}

void MCWaiko::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  imuSensor_ = config("imuSensor", ctl.robot().bodySensor().name());

  config("maxAnchorFrameDiscontinuity", maxAnchorFrameDiscontinuity_);
  config("updateRobot", updateRobot_);
  config("updateSensor", updateSensor_);

  auto odomConfig = config("leggedOdometry");
  auto contactsConfig = config("contacts");
  auto filterGainsConfig = config("filterGains");

  filterGainsConfig("initAlpha", alpha_);
  filterGainsConfig("initBeta", beta_);
  filterGainsConfig("initRho", rho_);
  filterGainsConfig("initMu", mu_contacts_);
  filterGainsConfig("initLambda", lambda_contacts_);

  filterGainsConfig("finalAlpha", finalAlpha_);
  filterGainsConfig("finalBeta", finalBeta_);
  filterGainsConfig("finalRho", finalRho_);
  filterGainsConfig("finalMu", mu_contacts_final_);
  filterGainsConfig("finalLambda", lambda_contacts_final_);
  // filterGainsConfig("finalEta", eta_contacts_final_);

  anchorFrameFunction_ = "KinematicAnchorFrame::" + ctl.robot(robot_).name();
  // if a user-defined anchor frame function is given, we use it instead
  if(config.has("anchorFrameFunction"))
  {
    if(ctl.datastore().has(anchorFrameFunction_))
    {
      anchorFrameFunction_ = config("anchorFrameFunction", name() + "::" + ctl.robot(robot_).name());
    }
  }

  std::string odometryTypeStr = static_cast<std::string>(odomConfig("odometryType"));
  // we set the odometry type now because it will be necessary for the next check
  setOdometryType(stateObservation::measurements::stringToOdometryType(odometryTypeStr));

  // specific configurations for the use of odometry.
  bool correctContacts = odomConfig("correctContacts", true);

  std::vector<std::string> surfacesForContactDetection =
      contactsConfig("surfacesForContactDetection", std::vector<std::string>());

  measurements::ContactsManagerSurfacesConfiguration contactsConf(name(), surfacesForContactDetection);

  contactsManager_.init(ctl, robot_, contactsConf);

  if(odomConfig.has("kappa"))
  {
    double kappa = odomConfig("kappa");
    odometryManager_.kappa(kappa);
  }
  if(odomConfig.has("lambdaInf"))
  {
    double lambdaInf = odomConfig("lambdaInf");
    odometryManager_.lambdaInf(lambdaInf);
  }
  const auto & imu = ctl.robot(robot_).bodySensor(imuSensor_);
  const sva::PTransformd & imuXbs = imu.X_b_s();
  so::kine::Kinematics parentImuKine =
      conversions::kinematics::fromSva(imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

  // pose of the IMU's parent body in the world for the odometry robot
  const sva::PTransformd & parentPoseW = ctl.realRobot().bodyPosW(imu.parentBody());
  // velocity of the IMU's parent body in the world for the odometry robot
  const sva::MotionVecd & v_0_imuParent =
      ctl.realRobot().mbc().bodyVelW[ctl.realRobot().bodyIndexByName(imu.parentBody())];

  // kinematics of the IMU's parent body in the world for the odometry robot
  so::kine::Kinematics worldParentKine = conversions::kinematics::fromSva(parentPoseW, v_0_imuParent, true);

  // pose and velocities of the IMU in the world frame for the odometry robot
  worldImuKine_ = worldParentKine * parentImuKine;
  stateObservation::odometry::LeggedOdometryManager::Configuration loConfig(
      stateObservation::measurements::stringToOdometryType(odometryTypeStr));
  loConfig.correctContacts(correctContacts);

  odometryManager_.init(loConfig, worldImuKine_.toVector(stateObservation::kine::Kinematics::Flags::pose));
}

void MCWaiko::reset(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());

  // the updated robot has the same floating base's pose than the control robot, but its encoders are updated. We use
  // it to get more accurate local Kinematics.
  ctl.gui()->addElement(
      {"Robots"}, mc_rtc::gui::Robot(name(), [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));

  const auto & imu = robot.bodySensor(imuSensor_);

  // reset of the floating base kinematics
  poseW_ = realRobot.posW();
  velW_ = realRobot.velW();
  velW_ = sva::MotionVecd::Zero();

  // initialization of the estimator
  so::kine::Kinematics initParentImuKine = conversions::kinematics::fromSva(
      imu.X_b_s(), so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

  // kinematics of the IMU's parent body in the world for the odometry robot
  so::kine::Kinematics initWorldParentKine =
      conversions::kinematics::fromSva(realRobot.bodyPosW(imu.parentBody()), so::kine::Kinematics::Flags::pose);

  // pose and velocities of the IMU in the world frame for the odometry robot
  so ::kine::Kinematics initWorldImuKine = initWorldParentKine * initParentImuKine;
  const Eigen::Matrix3d cOri = (imu.X_b_s() * realRobot.bodyPosW(imu.parentBody())).rotation();
  so::Vector3 initX2 = initWorldImuKine.orientation.toMatrix3().transpose() * so::Vector3::UnitZ();

  estimator_.initEstimator(so::Vector3::Zero(), initX2, initWorldImuKine.orientation.toVector4(),
                           initWorldImuKine.orientation.toMatrix3().transpose() * initWorldImuKine.position());

  anchorFrameJumped_ = false;
  iter_ = 0;
  imuVelC_ = sva::MotionVecd::Zero();
  X_C_IMU_ = sva::PTransformd::Identity();

  stateObservation::kine::Kinematics initKine =
      conversions::kinematics::fromSva(ctl.realRobot().posW(), stateObservation::kine::Kinematics::Flags::pose);

  odometryManager_.replaceRobotPose(initKine.toVector(stateObservation::kine::Kinematics::Flags::pose));
}

bool MCWaiko::run(const mc_control::MCController & ctl)
{
  const auto & realRobot = ctl.realRobot(robot_);
  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  if(logger.t() > 1.0)
  {
    alpha_ = finalAlpha_;
    beta_ = finalBeta_;
    rho_ = finalRho_;
    mu_contacts_ = mu_contacts_final_;
    lambda_contacts_ = lambda_contacts_final_;
  }

  std::unordered_set<std::string> contactList;

  auto onNewContact = [&contactList](measurements::ContactWithSensor & newContact)
  { contactList.insert(newContact.name()); };

  auto onNewContactOdom = [&realRobot, &logger, this](stateObservation::odometry::LoContact & newContact)
  {
    const std::string & surfaceName = contactsManager_.contact(newContact.name()).surface();
    const sva::PTransformd & surfaceXbs = realRobot.surface(surfaceName).X_b_s();
    so::kine::Kinematics parentSurfaceKine = conversions::kinematics::fromSva(
        surfaceXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);
    const sva::PTransformd & parentPoseW = realRobot.bodyPosW(realRobot.surface(surfaceName).bodyName());
    const sva::MotionVecd & v_0_contactParent =
        realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(realRobot.surface(surfaceName).bodyName())];
    so::kine::Kinematics worldParentKine = conversions::kinematics::fromSva(parentPoseW, v_0_contactParent, true);

    stateObservation::kine::Kinematics worldContactKine = worldParentKine * parentSurfaceKine;

    newContact.contactBodyKine_ = worldContactKine.getInverse() * worldImuKine_;

    newContact.lambda(contactsManager_.contact(newContact.name()).forceMeas().norm());

    conversions::kinematics::addToLogger(logger, newContact.worldRefKine_,
                                         "Waiko_contacts_" + newContact.name() + "_refPose");
    conversions::kinematics::addToLogger(logger, newContact.worldBodyKineFromRef_,
                                         "Waiko_contacts_" + newContact.name() + "_worldImuKineFromRef");
    conversions::kinematics::addToLogger(logger, newContact.currentWorldKine_,
                                         "Waiko_contacts_" + newContact.name() + "_currentWorldContactKine");
    conversions::kinematics::addToLogger(logger, newContact.contactBodyKine_,
                                         "Waiko_contacts_" + newContact.name() + "_contactImuKine_");
    conversions::kinematics::addToLogger(logger, newContact.worldRefKineBeforeCorrection_,
                                         "Waiko_contacts_" + newContact.name() + "_refPoseBeforeCorrection");
    conversions::kinematics::addToLogger(logger, newContact.newIncomingWorldRefKine_,
                                         "Waiko_contacts_" + newContact.name() + "_newIncomingWorldRefKine");

    logger.addLogEntry("Waiko_contacts_" + newContact.name() + "_isSet", &newContact,
                       [&newContact]() -> std::string { return newContact.isSet() ? "Set" : "notSet"; });

    logger.addLogEntry("Waiko_contacts_" + newContact.name() + "_lambda", &newContact,
                       [&newContact]() -> double { return newContact.lambda(); });
    logger.addLogEntry("Waiko_contacts_" + newContact.name() + "_lifeTime", &newContact,
                       [&newContact]() -> double { return newContact.lifeTime(); });
    logger.addLogEntry("Waiko_contacts_" + newContact.name() + "_correctionWeightingCoeff", &newContact,
                       [&newContact]() -> double { return newContact.correctionWeightingCoeff(); });
  };
  auto onMaintainedContact = [&contactList](measurements::ContactWithSensor & maintainedContact)
  { contactList.insert(maintainedContact.name()); };

  auto onMaintainedContactOdom = [&realRobot, this, &ctl](stateObservation::odometry::LoContact & maintainedContact)
  {
    const std::string & surfaceName = contactsManager_.contact(maintainedContact.name()).surface();
    const sva::PTransformd & surfaceXbs = realRobot.surface(surfaceName).X_b_s();
    so::kine::Kinematics parentSurfaceKine = conversions::kinematics::fromSva(
        surfaceXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);
    const sva::PTransformd & parentPoseW = realRobot.bodyPosW(realRobot.surface(surfaceName).bodyName());
    const sva::MotionVecd & v_0_contactParent =
        realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(realRobot.surface(surfaceName).bodyName())];
    so::kine::Kinematics worldParentKine = conversions::kinematics::fromSva(parentPoseW, v_0_contactParent, true);

    stateObservation::kine::Kinematics worldContactKine = worldParentKine * parentSurfaceKine;

    maintainedContact.contactBodyKine_ = worldContactKine.getInverse() * worldImuKine_;
    const stateObservation::Vector3 & forceMeas = contactsManager_.contact(maintainedContact.name()).forceMeas();
    double forceRatio =
        forceMeas.z()
        / (forceMeas.head(2).norm() + 1e-6 * ctl.realRobot().mass() * stateObservation::cst::gravityConstant);

    maintainedContact.lambda(forceRatio);
  };

  auto onRemovedContact = [](measurements::ContactWithSensor &) {};
  auto onRemovedContactOdom = [&logger](stateObservation::odometry::LoContact & removedContact)
  {
    conversions::kinematics::removeFromLogger(logger, removedContact.worldRefKine_);
    conversions::kinematics::removeFromLogger(logger, removedContact.worldRefKineBeforeCorrection_);
    conversions::kinematics::removeFromLogger(logger, removedContact.worldBodyKineFromRef_);
    conversions::kinematics::removeFromLogger(logger, removedContact.currentWorldKine_);
    conversions::kinematics::removeFromLogger(logger, removedContact.contactBodyKine_);
    conversions::kinematics::removeFromLogger(logger, removedContact.newIncomingWorldRefKine_);
    logger.removeLogEntries(&removedContact);
  };

  contactsManager_.updateContacts(ctl, robot_, onNewContact, onMaintainedContact, onRemovedContact);
  stateObservation::odometry::LeggedOdometryManager::ContactUpdateFunctions contactUpdateFunctions =
      stateObservation::odometry::LeggedOdometryManager::ContactUpdateFunctions()
          .onNewContact(onNewContactOdom)
          .onMaintainedContact(onMaintainedContactOdom)
          .onRemovedContact(onRemovedContactOdom);

  odometryManager_.initLoop(contactList, contactUpdateFunctions, &velW_.linear(), &velW_.angular());

  runEstimator(ctl, realRobot);

  sva::PTransformd poseW;
  poseW.translation() = estWorldImuKine_.position();
  poseW.rotation() = estWorldImuKine_.orientation.toMatrix3().transpose();
  // The input robot copies the real robot to update the encoder values.
  // Then its floating base is brung back to the origin of the world frame and given zero velocities and accelerations
  // in order to ease the computations.

  iter_++;

  /* Update of the observed robot */
  my_robots_->robot().mbc().q = realRobot.mbc().q;
  update(my_robots_->robot());

  return true;
}

void MCWaiko::updateNecessaryFramesOdom(const mc_control::MCController & ctl, const mc_rbdyn::Robot & odomRobot)

{
  // pose of the floating base' frame in the world for the odometry robot
  worldFbKine_ = conversions::kinematics::fromSva(odomRobot.posW(), odomRobot.velW(), true);

  const auto & imu = ctl.robot(robot_).bodySensor(imuSensor_);
  const sva::PTransformd & imuXbs = imu.X_b_s();
  so::kine::Kinematics parentImuKine =
      conversions::kinematics::fromSva(imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

  // pose of the IMU's parent body in the world for the odometry robot
  const sva::PTransformd & parentPoseW = odomRobot.bodyPosW(imu.parentBody());
  // velocity of the IMU's parent body in the world for the odometry robot
  const sva::MotionVecd & v_0_imuParent = odomRobot.mbc().bodyVelW[odomRobot.bodyIndexByName(imu.parentBody())];

  // kinematics of the IMU's parent body in the world for the odometry robot
  so::kine::Kinematics worldParentKine = conversions::kinematics::fromSva(parentPoseW, v_0_imuParent, true);

  // pose and velocities of the IMU in the world frame for the odometry robot
  worldImuKine_ = worldParentKine * parentImuKine;

  // pose and velocities of the IMU in the floating base for the odometry robot
  fbImuKine_ = worldFbKine_.getInverse() * worldImuKine_;

  // position and linear velocity of the anchor point in the frame of the IMU.
  imuAnchorKine_ = odometryManager_.getAnchorKineIn(worldImuKine_);
}

void MCWaiko::runEstimator(const mc_control::MCController & ctl, const mc_rbdyn::Robot & odomRobot)
{
  updateNecessaryFramesOdom(ctl, odomRobot);

  const auto & imu = ctl.robot(robot_).bodySensor(imuSensor_);

  auto k = estimator_.getCurrentTime();

  // measuredOri_ = so::Matrix3(ctl.realRobot(robot_).posW().rotation().transpose());

  // The anchor frame can be obtained using 2 ways:
  // - 1: contacts are detected and can be used
  // - 2: no contact is detected, the robot is hanging. As we still need an anchor frame for the tilt estimation we
  // arbitrarily use the frame of the IMU. As we cannot perform odometry anymore as there is no contact, we cannot
  // obtain the velocity of the IMU. We will then consider it as zero and consider it as constant with the linear
  // acceleration as zero too.
  // When switching from one mode to another, we consider x1hat = x1 before the estimation to avoid discontinuities.
  if(odometryManager_.maintainedContacts().size() == 0)
  {
    estimator_.setAlpha(30);
    estimator_.setBeta(0);

    yv_.setZero();
  }
  else
  {
    estimator_.setAlpha(alpha_);
    estimator_.setBeta(beta_);
    estimator_.setRho(rho_);

    yv_ = -imu.angularVelocity().cross(imuAnchorKine_.position()) - imuAnchorKine_.linVel();
  }

  estimator_.setInput(yv_, imu.linearAcceleration(), imu.angularVelocity(), k, false);

  if(odometryManager_.maintainedContacts().size() > 0)
  {
    stateObservation::kine::Kinematics worldImuKineFromAnchor = odometryManager_.getWorldBodyKineFromAnchor(true, true);

    estimator_.addContactInput(so::WaikoHumanoid::InputWaiko::ContactInput(
                                   worldImuKineFromAnchor.orientation.toMatrix3(), worldImuKineFromAnchor.position()),
                               k);
  }

  // estimation of the state with the complementary filters
  estimator_.getEstimatedState(k + 1);

  so::kine::LocalKinematics estimatedWorldImuLocKine;
  estimatedWorldImuLocKine.position = estimator_.getEstimatedLocPosition();
  estimatedWorldImuLocKine.orientation = estimator_.getEstimatedOrientation();
  estimatedWorldImuLocKine.linVel = estimator_.getEstimatedLocLinVel();
  estimatedWorldImuLocKine.angVel = imu.angularVelocity();

  estimatedWorldImuKine_ = estimatedWorldImuLocKine;

  odometryManager_.run(stateObservation::odometry::LeggedOdometryManager::KineParams(estWorldImuKine_)
                           .attitudeMeas(estimatedWorldImuLocKine.orientation.toMatrix3())
                           .positionMeas(estimatedWorldImuKine_.position()));

  updatePoseAndVel();

  /* Backups */

  // for the Kinetics Observer
  backupFbKinematics_.push_back(conversions::kinematics::fromSva(poseW_, so::kine::Kinematics::Flags::pose));
}

void MCWaiko::updatePoseAndVel()
{
  estimatedWorldFbKine_ = estimatedWorldImuKine_ * fbImuKine_.getInverse();

  poseW_.translation() = estimatedWorldFbKine_.position();
  poseW_.rotation() = estimatedWorldFbKine_.orientation.toMatrix3().transpose();

  velW_.linear() = estimatedWorldFbKine_.linVel();
  velW_.angular() = estimatedWorldFbKine_.angVel();
}

void MCWaiko::update(mc_control::MCController & ctl)
{
  auto & realRobot = ctl.realRobot(robot_);
  if(updateRobot_)
  {
    update(realRobot);
    realRobot.forwardKinematics();
    realRobot.forwardVelocity();
  }

  if(updateSensor_)
  {
    auto & robot = ctl.robot(robot_);

    auto & imu = const_cast<mc_rbdyn::BodySensor &>(robot.bodySensor(imuSensor_));
    auto & rimu = const_cast<mc_rbdyn::BodySensor &>(realRobot.bodySensor(imuSensor_));

    imu.orientation(estimatedWorldImuKine_.orientation.toQuaternion().inverse());
    rimu.orientation(estimatedWorldImuKine_.orientation.toQuaternion().inverse());
  }
}

void MCWaiko::update(mc_rbdyn::Robot & robot)
{
  robot.posW(poseW_);
  robot.velW(velW_);
}

const so::kine::Kinematics MCWaiko::backupFb(boost::circular_buffer<so::kine::Kinematics> * koBackupFbKinematics)
{
  // new initial pose of the floating base
  so::kine::Kinematics worldResetKine = *(koBackupFbKinematics->begin());

  // original initial pose of the floating base
  so::kine::Kinematics worldFbInitBackup = backupFbKinematics_.front();

  so::kine::Kinematics fbWorldInitBackup = worldFbInitBackup.getInverse();

  // we apply the transformation from the initial pose to the intermediates pose estimated by the tilt estimator to
  // the new starting pose of the Kinetics Observer
  for(int i = 0; i < koBackupFbKinematics->size(); i++)
  {
    // Intermediary pose of the floating base estimated by the tilt estimator
    so::kine::Kinematics worldFbIntermBackup = backupFbKinematics_.at(i);

    // transformation between the initial and the intermediary pose during the backup interval
    so::kine::Kinematics initInterm = fbWorldInitBackup * worldFbIntermBackup;

    koBackupFbKinematics->at(i) = worldResetKine * initInterm;
  }

  so::Vector3 tiltLocalLinVel = poseW_.rotation() * velW_.linear();
  so::Vector3 tiltLocalAngVel = poseW_.rotation() * velW_.angular();

  // koBackupFbKinematics->back() is the new last pose of the kinetics observer
  koBackupFbKinematics->back().linVel = koBackupFbKinematics->back().orientation.toMatrix3() * tiltLocalLinVel;
  koBackupFbKinematics->back().angVel = koBackupFbKinematics->back().orientation.toMatrix3() * tiltLocalAngVel;

  return koBackupFbKinematics->back();
}

void MCWaiko::setOdometryType(stateObservation::measurements::OdometryType newOdometryType)
{
  if((newOdometryType != stateObservation::measurements::OdometryType::Odometry6d)
     && (newOdometryType != stateObservation::measurements::OdometryType::Flat))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Please choose between these two odometry types: [6D, Flat]");
  }

  odometryManager_.setOdometryType(newOdometryType);
}

void MCWaiko::addToLogger(const mc_control::MCController & ctl, mc_rtc::Logger & logger, const std::string & category)
{
  category_ = category;

  logger.addLogEntry(category + "_estimatedState_pl",
                     [this]() -> so::Vector3 { return estimator_.getEstimatedLocPosition(); });

  logger.addLogEntry(category + "_estimatedState_p",
                     [this]() -> so::Vector3 { return estimatedWorldImuKine_.position(); });

  logger.addLogEntry(category + "_estimatedState_x1",
                     [this]() -> so::Vector3 { return estimator_.getEstimatedLocLinVel(); });

  logger.addLogEntry(category + "_debug_measuredOri_",
                     [this]() -> Eigen::Quaterniond { return measuredOri_.toQuaternion().inverse(); });

  // logger.addLogEntry(category + "_debug_corrections_oriCorrection_",
  //                    [this]() -> const so::Vector3 & { return estimator_.getOriCorrection(); });
  logger.addLogEntry(category + "_debug_corrections_oriCorrFromOriMeas_",
                     [this]() -> const so::Vector3 & { return estimator_.getOriCorrFromOriMeas(); });
  logger.addLogEntry(category + "_debug_corrections_posCorrFromContactPos_",
                     [this]() -> const so::Vector3 & { return estimator_.getPosCorrectionFromContactPos(); });
  logger.addLogEntry(category + "_debug_corrections_oriCorrFromContactPos_",
                     [this]() -> const so::Vector3 & { return estimator_.geOriCorrectionFromContactPos(); });

  logger.addLogEntry(category + "_estimatedState_x2prime",
                     [this]() -> so::Vector3 { return estimator_.getEstimatedTilt().normalized(); });
  logger.addLogEntry(category + "_estimatedState_R",
                     [this]()
                     {
                       so::kine::Orientation ori;
                       ori = estimator_.getEstimatedOrientation();
                       return ori.toQuaternion().inverse();
                     });
  logger.addLogEntry(category + "_realRobotState_x1",
                     [this, &ctl]() -> so::Vector3
                     {
                       const auto & realRobot = ctl.realRobot(robot_);
                       const auto & rimu = realRobot.bodySensor(imuSensor_);

                       const sva::PTransformd & rimuXbs = rimu.X_b_s();

                       so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(
                           rimuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                       const sva::PTransformd & realRobotParentPoseW = realRobot.bodyPosW(rimu.parentBody());

                       // Compute velocity of the imu in the control frame
                       auto & realRobotV_0_imuParent =
                           realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(rimu.parentBody())];

                       so::kine::Kinematics worldParentKine =
                           conversions::kinematics::fromSva(realRobotParentPoseW, realRobotV_0_imuParent, true);

                       so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
                       return worldImuKine.orientation.toMatrix3().transpose() * worldImuKine.linVel();
                     });

  logger.addLogEntry(category + "_realRobotState_x2",
                     [this, &ctl]() -> so::Vector3
                     {
                       const auto & realRobot = ctl.realRobot(robot_);
                       const auto & rimu = realRobot.bodySensor(imuSensor_);

                       const sva::PTransformd & rimuXbs = rimu.X_b_s();

                       so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(
                           rimuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                       const sva::PTransformd & realRobotParentPoseW = realRobot.bodyPosW(rimu.parentBody());

                       // Compute velocity of the imu in the control frame
                       auto & realRobotV_0_imuParent =
                           realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(rimu.parentBody())];

                       so::kine::Kinematics worldParentKine =
                           conversions::kinematics::fromSva(realRobotParentPoseW, realRobotV_0_imuParent, true);

                       so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
                       return (worldImuKine.orientation.toMatrix3().transpose() * so::Vector3::UnitZ()).normalized();
                     });

  logger.addLogEntry(category + "_constants_gains_alpha", [this]() -> double { return estimator_.getAlpha(); });
  logger.addLogEntry(category + "_constants_gains_beta", [this]() -> double { return estimator_.getBeta(); });
  // logger.addLogEntry(category + "_constants_gains_gamma", [this]() -> double { return gamma_; });
  logger.addLogEntry(category + "_constants_gains_rho", [this]() -> double { return estimator_.getRho(); });
  logger.addLogEntry(category + "_constants_gains_contacts_mu", [this]() -> double { return mu_contacts_; });
  logger.addLogEntry(category + "_constants_gains_contacts_lambda", [this]() -> double { return lambda_contacts_; });
  // logger.addLogEntry(category + "_constants_gains_contacts_eta", [this]() -> double { return eta_contacts_; });

  logger.addLogEntry(category + "_debug_OdometryType", [this]() -> std::string
                     { return stateObservation::measurements::odometryTypeToSstring(odometryManager_.odometryType_); });

  logger.addLogEntry(category + "_IMU_AnchorFrame_pose", [this]() -> const sva::PTransformd & { return X_C_IMU_; });
  logger.addLogEntry(category + "_IMU_AnchorFrame_linVel", [this]() -> const sva::MotionVecd & { return imuVelC_; });
  logger.addLogEntry(category + "_FloatingBase_world_pose", [this]() -> const sva::PTransformd & { return poseW_; });
  logger.addLogEntry(category + "_FloatingBase_world_vel", [this]() -> const sva::MotionVecd & { return velW_; });
  logger.addLogEntry(category + "_debug_yv", [this]() -> const so::Vector3 & { return yv_; });

  logger.addLogEntry(category + "_debug_realWorldImuLocAngVel",
                     [this, &ctl]() -> so::Vector3
                     {
                       const sva::PTransformd & realImuXbs = ctl.realRobot(robot_).bodySensor(imuSensor_).X_b_s();

                       so::kine::Kinematics realParentImuKine = conversions::kinematics::fromSva(
                           realImuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                       const sva::PTransformd & realParentPoseW =
                           ctl.realRobot(robot_).bodyPosW(ctl.realRobot(robot_).bodySensor(imuSensor_).parentBody());

                       // Compute velocity of the imu in the control frame
                       auto & real_v_0_imuParent =
                           ctl.realRobot(robot_).mbc().bodyVelW[ctl.realRobot(robot_).bodyIndexByName(
                               ctl.realRobot(robot_).bodySensor(imuSensor_).parentBody())];

                       so::kine::Kinematics realWorldParentKine =
                           conversions::kinematics::fromSva(realParentPoseW, real_v_0_imuParent, true);

                       so::kine::Kinematics realWorldImuKine_ = realWorldParentKine * realParentImuKine;

                       return realWorldImuKine_.orientation.toMatrix3().transpose() * realWorldImuKine_.angVel();
                     });

  logger.addLogEntry(category + "_debug_ctlWorldImuLocAngVel",
                     [this, &ctl]() -> so::Vector3
                     {
                       const sva::PTransformd & imuXbs = ctl.robot(robot_).bodySensor(imuSensor_).X_b_s();

                       so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(
                           imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                       const sva::PTransformd & parentPoseW =
                           ctl.robot(robot_).bodyPosW(ctl.robot(robot_).bodySensor(imuSensor_).parentBody());

                       // Compute velocity of the imu in the control frame
                       auto & v_0_imuParent = ctl.robot(robot_).mbc().bodyVelW[ctl.robot(robot_).bodyIndexByName(
                           ctl.robot(robot_).bodySensor(imuSensor_).parentBody())];

                       so::kine::Kinematics worldParentKine =
                           conversions::kinematics::fromSva(parentPoseW, v_0_imuParent, true);

                       so::kine::Kinematics worldImuKine_ = worldParentKine * parentImuKine;

                       return worldImuKine_.orientation.toMatrix3().transpose() * worldImuKine_.angVel();
                     });

  logger.addLogEntry(category + "_debug_realX1",
                     [this, &ctl]() -> so::Vector3
                     {
                       const auto & realRobot = ctl.realRobot(robot_);
                       const auto & rimu = realRobot.bodySensor(imuSensor_);

                       const sva::PTransformd & rimuXbs = rimu.X_b_s();

                       so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(
                           rimuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                       const sva::PTransformd & parentPoseW = realRobot.bodyPosW(rimu.parentBody());

                       // Compute velocity of the imu in the control frame
                       auto & v_0_imuParent = realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(rimu.parentBody())];

                       so::kine::Kinematics worldParentKine =
                           conversions::kinematics::fromSva(parentPoseW, v_0_imuParent, true);

                       so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
                       return worldImuKine.orientation.toMatrix3().transpose() * worldImuKine.linVel();
                     });

  logger.addLogEntry(category + "_debug_realImuVel",
                     [this, &ctl]() -> so::Vector3
                     {
                       const auto & realRobot = ctl.realRobot(robot_);
                       const auto & rimu = realRobot.bodySensor(imuSensor_);

                       const sva::PTransformd & rimuXbs = rimu.X_b_s();

                       so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(
                           rimuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                       const sva::PTransformd & parentPoseW = realRobot.bodyPosW(rimu.parentBody());

                       auto & v_0_imuParent = realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(rimu.parentBody())];

                       so::kine::Kinematics worldParentKine =
                           conversions::kinematics::fromSva(parentPoseW, v_0_imuParent, true);

                       so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
                       return worldImuKine.linVel();
                     });

  logger.addLogEntry(category + "_debug_realBodyVel",
                     [this, &ctl]() -> so::Vector3
                     {
                       const auto & realRobot = ctl.realRobot(robot_);
                       const auto & rimu = realRobot.bodySensor(imuSensor_);

                       return realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(rimu.parentBody())].linear();
                     });

  logger.addLogEntry(category + "_debug_ctlX1",
                     [this, &ctl]() -> so::Vector3
                     {
                       const auto & robot = ctl.robot(robot_);
                       const auto & imu = robot.bodySensor(imuSensor_);

                       const sva::PTransformd & imuXbs = imu.X_b_s();

                       so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(
                           imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                       const sva::PTransformd & parentPoseW = robot.bodyPosW(imu.parentBody());

                       // Compute velocity of the imu in the control frame
                       auto & v_0_imuParent = robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())];

                       so::kine::Kinematics worldParentKine =
                           conversions::kinematics::fromSva(parentPoseW, v_0_imuParent, true);

                       so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
                       return worldImuKine.orientation.toMatrix3().transpose() * worldImuKine.linVel();
                     });

  logger.addLogEntry(category + "_debug_ctlImuVel",
                     [this, &ctl]() -> so::Vector3
                     {
                       const auto & robot = ctl.robot(robot_);
                       const auto & imu = robot.bodySensor(imuSensor_);

                       const sva::PTransformd & imuXbs = imu.X_b_s();

                       so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(
                           imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                       const sva::PTransformd & parentPoseW = robot.bodyPosW(imu.parentBody());

                       auto & v_0_imuParent = robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())];

                       so::kine::Kinematics worldParentKine =
                           conversions::kinematics::fromSva(parentPoseW, v_0_imuParent, true);

                       so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
                       return worldImuKine.linVel();
                     });

  logger.addLogEntry(category + "_debug_contactDetected", [this]() -> std::string
                     { return odometryManager_.contactsManager().contactsDetected() ? "contacts" : "no contacts"; });

  logger.addLogEntry(category + "_debug_ctlBodyVel",
                     [this, &ctl]() -> so::Vector3
                     {
                       const auto & robot = ctl.robot(robot_);
                       const auto & imu = robot.bodySensor(imuSensor_);

                       return robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())].linear();
                     });

  conversions::kinematics::addToLogger(logger, worldImuKine_, category + "_debug_worldImuKine");
  conversions::kinematics::addToLogger(logger, odometryManager_.bodyKine_, category + "_debug_worldImuKineOdom");

  conversions::kinematics::addToLogger(logger, imuAnchorKine_, category + "_debug_imuAnchorKine_");
  conversions::kinematics::addToLogger(logger, fbImuKine_, category + "_debug_fbImuKine_");

  conversions::kinematics::addToLogger(logger, worldFbKine_, category + "_debug_worldFbKine_");
  conversions::kinematics::addToLogger(logger, estimatedWorldImuKine_, category + "_debug_correctedWorldImuKine_");
}

void MCWaiko::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_imuVelC");
  logger.removeLogEntry(category + "_imuPoseC");
  logger.removeLogEntry(category + "_imuEstRotW");
  logger.removeLogEntry(category + "_controlAnchorFrame");
}

void MCWaiko::addToGUI(const mc_control::MCController &, mc_rtc::gui::StateBuilder &, const std::vector<std::string> &)
{
  using namespace mc_state_observation::gui;
  // gui.addElement(category, make_input_element("alpha", alpha_), make_input_element("beta", beta_));
}

} // namespace mc_state_observation
EXPORT_OBSERVER_MODULE("MCWaiko", mc_state_observation::MCWaiko)
