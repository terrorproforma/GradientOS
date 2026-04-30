# Gradient-05 Robot Specification Sheet

Gradient-05 is a six-axis articulated robot platform built around GradientOS motion control, EtherCAT RTCore execution, and high-resolution absolute encoder feedback. This two-page specification sheet presents the current GradientOS-backed configuration and highlights ratings that are still in validation.

Values marked "in validation" are reserved for final mechanical, electrical, or metrology-backed ratings.

## Page 1: Product Overview

### Header

**GRADIENT-05**  

**High-control 6-axis robotic arm**

### Key Benefits

- Six-axis articulated arm architecture for flexible end-effector positioning.
- EtherCAT RTCore motion execution for coordinated, deterministic servo control.
- GradientOS commissioning workflows for native-home setup, drive status, and actuator diagnostics.
- Direct one-to-one joint-to-actuator mapping for transparent control and serviceability.
- 17-bit motor-side absolute encoder model for high-resolution feedback.
- Extended base and wrist rotation ranges for process development and advanced motion workflows.

### Quick Specifications


| Item                                | Unit               | Gradient-05                                              |
| ----------------------------------- | ------------------ | -------------------------------------------------------- |
| Controlled axes                     | count              | 6                                                        |
| Robot type                          | -                  | 6-axis articulated arm                                   |
| Default runtime backend             | -                  | EtherCAT RTCore                                          |
| Default IK backend                  | -                  | Numeric QuIK                                             |
| Actuator mapping                    | -                  | 1 logical joint to 1 physical actuator                   |
| Encoder resolution                  | counts / motor rev | 131,072                                                  |
| Horizontal reach, `tool0` origin    | mm                 | approx. 1,509                                            |
| Vertical reach span, `tool0` origin | mm                 | approx. 2,291                                            |
| Maximum `tool0` height above base   | mm                 | approx. 1,647                                            |
| Minimum `tool0` height below base   | mm                 | approx. -644                                             |
| Nominal kinematic chain length      | mm                 | approx. 1,790                                            |
| Maximum payload                     | kg                 | In validation                                            |
| Repeatability                       | mm                 | In validation                                            |
| Robot mass                          | kg                 | In validation                                            |
| Mounting                            | -                  | Floor or fixture mount, final interface in validation    |
| Power                               | -                  | Cabinet-dependent, final electrical rating in validation |


### Application Block

**Application:** motion-control development, robotic process automation, end-effector integration, and advanced robotic workflow R&D.  

**Controller:** GradientOS with EtherCAT RTCore.  

**Configuration:** six controlled axes with Numeric QuIK Cartesian IK.

### Page-One Copy

- Wide rotation ranges support base and wrist workflows beyond one revolution.
- Commissioning paths expose native-home, drive status, and actuator diagnostics through GradientOS.
- Cartesian jog and profile defaults are tuned for controlled development, integration, and validation workflows.

## Page 2: Technical Data

### Axis Specifications


| Axis | Joint role    | Motion range       | Rated joint speed | Peak joint speed | Gear reduction | Encoder counts / joint rev | Sign |
| ---- | ------------- | ------------------ | ----------------- | ---------------- | -------------- | -------------------------- | ---- |
| J1   | Base rotation | +/- 361.0 deg      | 180 deg/s         | 360 deg/s        | 100:1          | 13,107,200                 | -    |
| J2   | Shoulder      | +/- 108.9 deg      | 180 deg/s         | 360 deg/s        | 100:1          | 13,107,200                 | +    |
| J3   | Elbow         | -240.6 / +87.7 deg | 180 deg/s         | 360 deg/s        | 100:1          | 13,107,200                 | -    |
| J4   | Wrist roll    | +/- 361.0 deg      | 1,000 deg/s       | 2,000 deg/s      | 18:1           | 2,359,296                  | -    |
| J5   | Wrist pitch   | +/- 361.0 deg      | 1,980 deg/s       | 3,960 deg/s      | 100:11         | 1,191,564                  | -    |
| J6   | Tool roll     | +/- 573.0 deg      | 1,800 deg/s       | 3,600 deg/s      | 10:1           | 1,310,720                  | -    |


Rated and peak joint speeds are derived from the motor shaft speed through each joint gear ratio: `joint deg/s = motor RPM * 6 / gear_ratio`. Rated speed uses 3,000 RPM; peak speed uses 6,000 RPM. Controller jog, profiled-motion, thermal, and safety policy limits may command lower speeds than these derived mechanical maxima.

### Kinematic Dimensions

Current URDF joint origins in parent-link coordinates:


| Joint | X        | Y          | Z        | Axis |
| ----- | -------- | ---------- | -------- | ---- |
| J1    | 0.0 mm   | 0.0 mm     | 32.8 mm  | Z    |
| J2    | 160.0 mm | -125.5 mm  | 265.2 mm | Y    |
| J3    | 0.0 mm   | 0.0 mm     | 600.0 mm | Y    |
| J4    | 159.0 mm | 125.5 mm   | 105.0 mm | X    |
| J5    | 487.7 mm | -47.883 mm | 0.0 mm   | Y    |
| J6    | 93.3 mm  | 47.883 mm  | 0.0 mm   | X    |


URDF-defined envelope estimates:


| Envelope point                                  | Value            |
| ----------------------------------------------- | ---------------- |
| Maximum horizontal radius                       | approx. 1,509 mm |
| Maximum vertical height                         | approx. 1,647 mm |
| Minimum vertical height                         | approx. -644 mm  |
| Total vertical span                             | approx. 2,291 mm |
| Maximum straight-line distance from base origin | approx. 1,687 mm |


### Controller Defaults and Commissioning Data


| Item                                 | Value                                                                        |
| ------------------------------------ | ---------------------------------------------------------------------------- |
| Robot ID                             | `gradient-05`                                                                |
| Robot config class                   | `Gradient05Config`                                                           |
| Physical actuators                   | 6 EtherCAT axes, IDs 0-5                                                     |
| Default profile velocity             | 0.1 m/s                                                                      |
| Default profile acceleration         | 0.05 m/s^2                                                                   |
| Max Cartesian jog linear cap         | 0.2 m/s                                                                      |
| Max Cartesian jog angular cap        | 180 deg/s                                                                    |
| Motor rated speed                    | 3,000 RPM                                                                    |
| Motor peak speed                     | 6,000 RPM                                                                    |
| Drive-native ratios                  | J1 100:1, J2 100:1, J3 100:1, J4 18:1, J5 100:11, J6 10:1                    |
| Absolute encoder model in GradientOS | 17-bit motor-side count model                                                |
| Startup drive overrides              | None at robot layer; drive-family defaults live in the drive profile catalog |


## Ratings in Validation

These fields should remain marked as "in validation" until backed by mechanical design, electrical design, production CAD, or metrology/test evidence.


| Field                              | Publication requirement             | Best source                                            |
| ---------------------------------- | ----------------------------------- | ------------------------------------------------------ |
| Maximum payload                    | Rated payload claim                 | Mechanical calculation plus lift/trajectory validation |
| Repeatability                      | Accuracy/repeatability claim        | Calibration procedure and metrology report             |
| Robot mass                         | Installation and floor-loading data | Final CAD assembly or weighed build                    |
| Allowable wrist moment and inertia | End-effector/tooling compatibility  | Wrist gearbox, actuator ratings, and validation        |
| Tool flange drawing                | EOAT interface data                 | Production CAD drawing/export                          |
| Base mounting pattern              | Cell layout and installation data   | Final base plate drawing                               |
| Power requirements                 | Facility planning data              | Drive cabinet, PSU, breaker, and field wiring design   |
| Environmental rating               | Industrial/environmental claim      | Sealing design and validation                          |
| Cable and air routing              | Process package compatibility       | Harness and cable-dress design                         |
| Production render/drawing          | Visual/spec-sheet drawing package   | Production CAD/STL assets                              |


## Source Files

This sheet was built from the following GradientOS sources:

- `src/gradient_os/arm_controller/robots/gradient05/config.py`
- `robots/gradient-05/gradient-05.urdf`
- `robots/gradient-05/robot.json`
- `robots/gradient-05/dh_params.csv`
- `src/gradient_os/arm_controller/robots/base.py`
- `src/gradient_os/arm_controller/command_api.py`

## Verification Status


| Spec area                                         | Status                                            |
| ------------------------------------------------- | ------------------------------------------------- |
| Axis count, limits, actuator mapping, gear ratios | Sourced from runtime config                       |
| URDF joint origins and envelope estimates         | Derived from current URDF                         |
| Payload, mass, repeatability, wrist moments       | Missing from repo, requires CAD/test verification |
| Final production drawing dimensions               | Missing from repo, requires CAD drawing export    |
