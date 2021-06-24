# Streaming controllers

## Joint Group Position Controller

The joint group controller is a modified version of the `position_controllers/​JointGroupPositionController`. It adds joint position, velocity and acceleration limiting. This is done on the full vector joint commands (not per joint). The limits are enforced in a smooth and safe way for arbitrary commands so that the robot will decelerate towards a joint limit.

The input commands can be provided as positions, velocities of accelerations using the following topics:
```
/streaming_controller/command
/streaming_controller/command_acceleration
/streaming_controller/command_velocity
```
Each topic has the type `std_msgs/Float64MultiArray` where each element of the data vector corresponds to the joint command in the order specified in the controller configuration (see below).

Example controller configuration is below:
```
streaming_controller:
  type: streaming_controllers/JointGroupPositionController
  joints:
    - JOINT1
    - JOINT2
    - JOINT3
    - JOINT4
  RobotDescriptionParameter: /robot_description
  PositionSafetyFactor: 0.9
  VelocitySafetyFactor: 1.0
  AccelerationSafetyFactor: 0.05
  Kp: 200.0
  Kd: 10.0
  MyValidityChecker:
   config: "{my_package}/config/my_validy_checker_config.xml"
```

The `RobotDescriptionParameter` is used to load a URDF that contains the joint and acceleration limits.
The `PositionSafetyFactor`, `VelocitySafetyFactor` and `AccelerationSafetyFactor` are used to reduce the safety limits. They specify a percentage of the hard limit defined in the URDF.
The `Kp` and `Kd` parameters are PD gains used to deal with overshoots. These are inevitable when the robot is moving at a velocity but limited acceleration won't allow it to instantly stop.
The safety factors and PD gains can be configured using the DynamicReconfigure plugin in RQT.

The controller also allows for loading validity checking plugins. This enables to specify a validity check on every commanded state. If the check returns `false`, the command will be modified to stop the robot, e.g. to avoid self collisions. An overshoot may happen which all validity checkers you implement should account for.
You can implement a custom validity checker as a plugin using the [pluginlib](http://wiki.ros.org/pluginlib). If you implement a custom plugin named `MyValidityChecker`, the parameter name in the controller config file should also be `MyValidityChecker` (see the example config above). You can then specify further parameters required by your plugin.