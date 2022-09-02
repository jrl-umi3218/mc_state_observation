# mc_state_observation

This package implements additional state observers for `mc_rtc`.
Some will be considered for inclusion in `mc_rtc` once they have been fully battle tested.

Here is an overview of the various observers implemented:

## Observers

### AttitudeObserver (estimation of IMU orientation)

This observer is directly inspired by the [AttitudeEstimator](https://github.com/isri-aist/hrpsys-state-observation/blob/master/include/hrpsys-state-observation/AttitudeEstimator.h) of [hrpsys-state-observation](https://github.com/isri-aist/hrpsys-state-observation) (that provides an improved replacement for the deprecated [`KalmanFilter`](https://github.com/isri-aist/hrpsys-private/tree/master/KalmanFilter) component of [hrpsys-private](https://github.com/isri-aist/hrpsys-private)). The `AttitudeEstimator` component has been heavily used on `HRP-5P`. 

Configuration options:

```yaml
robot: JVRC1                # robot to observe (defaults to the main robot)
imuSensor: Accelerometer    # sensor providing the IMU measurements (defaults to the first bodysensor)
updateSensor: Accelerometer # name of the sensor in which to write the estimated orientation (defaults to imuSensor)
log_kf: false               # whether to log the kalman filter parameters (default: false)
init_from_control: true     # whether to initialize the kalman filter's orientation from the control robot state (default: true)
KamanFilter:                # configuration of the kalman filter (default values should be reasonable in most cases)
  compensateMode: true
  offset: [0,0,0]           # Apply an orientation offset to the estimation result (rpy or matrix)
  acc_cov: 0.003
  gyr_cov: 1e-10
  ori_acc_cov: 0.003
  lin_acc_cov: 1e-13
  state_cov: 3e-14
  state_init_cov: 1e-8
```

### MocapObserverROS (estimation of the floating base from MOCAP data)

Example configuration (updates main real robot instance from MOCAP data). Note that this requires calibration of the mocap marker wrt to the robot body:
- `Calibrate`: Measures the marker frame to robot body transformation (calibration of the MOCAP markers). This assumes that the initial robot position is very well known in the controller
- `Initialize`: Establishes the link between robot map and mocap origin.

```
ObserverPipelines:
- name: MocapPipeline
  gui: true
  observers:
    - type: Encoder
    - type: MocapObserverROS
      update: true
      config:
        updateRobot: hrp5_p
        marker_tf: HRP5P
        marker_origin_tf: mocap
        body: Chest_Link2
```

### SLAMObserver (Experimental)

Estimation of the robot thanks to the estimated camera from a SLAM.

Configuration options (default value for `Filter`, `Publish` and `Simulation`):

```yaml
Robot:
  robot: robot                      # Robot name (Optionnal, by default it will be the main robot name)
  robot_name_0:                     # Must be a name of a valid robot
    camera: camera_link             # Body name of the camera of the robot name 0
  robot_name_n:                     # Must be a name of a valid robot
    camera: camera_link             # Body name of the camera of the robot name 1
  ground: ground                    # Ground frame to have a ground pose in SLAM map
SLAM:
  map: map                          # ROS TF name of SLAM map
  estimated: camera_link            # ROS TF name of estimated camera
Filter:
  use: true
  m: 100                            # savitzky-golay parameters
  d: 2                              # savitzky-golay parameters
Publish:
  use: true                         # publish estimated robot in ROS
Simulation:
  use: false                        # If true, set estimated to rea/camera of robot
  noise:
    use: false
    translation:
      min: [-0.05, -0.05, -0.05]    # [m]
      max: [0.05, 0.05, 0.05]       # [m]
    orientation:
      min: [-0.01, -0.01, -0.01]    # [degree]
      max: [0.01, 0.01, 0.01]       # [degree]
```

In your controller's configuration file in .yaml:
```yaml
ObserverPipelines:
- name: SLAMPipeline
  gui: true
  observers:
    - type: Encoder
    - type: SLAM
      update: true
      config:
        Robot:
          robot: hrp2_drc
          hrp2_drc:
            camera: xtion_link
        SLAM:
          map: map
          estimated: camera_link
        Simulation:
          use: false
```

Then from your controller you can access to the estimated robot with:
```cpp
const auto & estimatedRobot = datastore().call<const mc_rbdyn::Robot &>("SLAM::Robot");

bool isAlive = datastore().call<bool>("SLAM::isAlive");
```

### ObjectObserver (Experimental)

Estimation of the object thanks to the estimated object from a vision process.
Configuration options (default value for `Filter`, `Publish` and `Simulation`):

```yaml
Robot:
  robot: robot                      # Robot name (Optionnal, by default it will be the main robot name)
  robot_name_0:                     # Must be a name of a valid robot
    camera: camera_link             # Body name of the camera of the robot name 0
  robot_name_n:                     # Must be a name of a valid robot
    camera: camera_link             # Body name of the camera of the robot name 1
Object:
  robot: object                     # Robot name
  topic: /topic/poseStamped         # ROS topic to receive estimated object pose stamped
  inRobotMap: false                 # If the update is compute from robot camera or from robot_map (in case of choreonoid by example)
Publish:
  use: true                         # publish estimated robot in ROS
```

In your controller's configuration file in .yaml:
```yaml
ObserverPipelines:
- name: ObjectPipeline
  gui: true
  observers:
    - type: Object
      update: true
      config:
        Robot:
          robot: hrp2_drc
          hrp2_drc:
            camera: xtion_link
        Object:
          robot: cube
          topic: /process/cube/pose
```

Then from your controller you can access to the estimated robot with:
```cpp
const auto & estimatedRobot = datastore().call<const mc_rbdyn::Robot &>(name_+"::Robot");

const auto & estimatedRobot = realRobot(name_);

const auto & X_0_Object = datastore().call<const sva::PTransformd &>(name_+"::X_0_Object");

const auto & X_0_Object = realRobot(name_).posW();

const auto & X_Camera_Object = datastore().call<const sva::PTransformd &>(name_+"::X_Camera_Object");
```

## Dependencies

- [gram_savitzky_golay](https://github.com/arntanguy/gram_savitzky_golay)
- [state-observation](https://github.com/jrl-umi3218/state-observation) > 1.4.1
- [mc_rtc](https://github.com/jrl-umi3218/mc_rtc)
- Eigen3
- Boost
