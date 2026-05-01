# Advanced Metal Research Rosie Robot Specification Sheet

Advanced Metal Research presents Rosie: a six-axis articulated robot platform with GradientOS motion control, EtherCAT RTCore execution, and high-resolution absolute encoder feedback. The sheet uses the Rosie logo, Rosie mascot, and robot dimension artwork with a warm cream editorial treatment, deep red signage accents, and clean technical cards.

## Brand Direction

- **Company:** Advanced Metal Research, 13020 Yukon Ave, Hawthorne, CA 90250
- **Tagline:** The next industrial workforce
- **Robot:** Rosie
- **Background:** warm cream `#FAF6EE` with subtle paper texture
- **Accent:** deep Rosie red `#B81C1C`
- **Typography:** Playfair Display headings, DM Sans body
- **Hero:** Rosie mascot prominent over the Rosie isometric robot layer; cropped robot dimension drawing used for the technical robot dimensions visual
- **Intro strip:** red americana-style strip with star dividers
- **Layout:** clean cards, strong hierarchy, no decorative clutter

## Page 1: Rosie Overview

### Header

**ADVANCED METAL RESEARCH**  

**The next industrial workforce**  

**Rosie**

### Intro Strip

**GradientOS integrated star EtherCAT RTCore star six-axis control star numeric QuIK IK**

### Headline Specifications


| Headline spec    | Rosie            |
| ---------------- | ---------------- |
| Nominal payload  | 15 kg            |
| Horizontal reach | approx. 1,509 mm |
| Vertical reach   | approx. 1,638 mm |
| Repeatability    | 0.08 mm          |


### Key Benefits

- Full industrial robotics controller, open source.
- No hidden software fees or limited software functionality.
- Open GradientOS stack makes integration easier and simpler.
- Six-axis articulated arm architecture for flexible end-effector positioning.
- EtherCAT RTCore motion execution for coordinated, deterministic servo control.
- GradientOS commissioning workflows for native-home setup, drive status, and actuator diagnostics.
- Direct one-to-one joint-to-actuator mapping for transparent control and serviceability.
- Expandable IO for cell and end-effector integration.
- Hollow shoulder cable pass-through, 62 mm diameter.
- Hollow wrist cable pass-through, 33 mm diameter.

### Quick Specifications


| Item                              | Unit               | Rosie                                       |
| --------------------------------- | ------------------ | ------------------------------------------- |
| Controlled axes                   | count              | 6                                           |
| Robot type                        | -                  | 6-axis articulated arm                      |
| Default runtime backend           | -                  | EtherCAT RTCore                             |
| Default IK backend                | -                  | Numeric QuIK                                |
| Actuator mapping                  | -                  | 1 logical joint to 1 physical actuator      |
| Encoder resolution                | counts / motor rev | 131,072                                     |
| Horizontal reach, `tool0` origin  | mm                 | approx. 1,509                               |
| Vertical reach, `tool0` origin    | mm                 | approx. 1,638                               |
| Total vertical envelope span      | mm                 | approx. 2,291                               |
| Maximum `tool0` height above base | mm                 | approx. 1,638 headline / 1,647 sampled URDF |
| Minimum `tool0` height below base | mm                 | approx. -644                                |
| Nominal kinematic chain length    | mm                 | approx. 1,790                               |
| Nominal payload                   | kg                 | 15                                          |
| Repeatability                     | mm                 | 0.08                                        |
| Expandable IO                     | -                  | Supported                                   |
| Power                             | -                  | 200-240 VAC 3PH, 35A                        |
| Hollow shoulder pass-through      | mm                 | 62 diameter                                 |
| Hollow wrist pass-through         | mm                 | 33 diameter                                 |


### Application Block

**Application:** motion-control development, robotic process automation, end-effector integration, and advanced robotic workflow R&D.

**Controller:** GradientOS with EtherCAT RTCore, delivered as an open-source full industrial robotics controller.

**Configuration:** six controlled axes with Numeric QuIK Cartesian IK.

### Page-One Copy

- Wide rotation ranges support base and wrist workflows beyond one revolution.
- Commissioning paths expose native-home, drive status, and actuator diagnostics through GradientOS.
- Cartesian jog and profile defaults are tuned for controlled development, integration, and validation workflows.

## Page 2: Technical Data

### Axis Specifications


| Axis | Joint role    | Motion range       | Rated joint speed | Peak joint speed | Encoder counts / joint rev | Sign |
| ---- | ------------- | ------------------ | ----------------- | ---------------- | -------------------------- | ---- |
| J1   | Base rotation | +/- 361.0 deg      | 180 deg/s         | 360 deg/s        | 13,107,200                 | -    |
| J2   | Shoulder      | +/- 108.9 deg      | 180 deg/s         | 360 deg/s        | 13,107,200                 | +    |
| J3   | Elbow         | -240.6 / +87.7 deg | 180 deg/s         | 360 deg/s        | 13,107,200                 | -    |
| J4   | Wrist roll    | +/- 361.0 deg      | 1,000 deg/s       | 2,000 deg/s      | 2,359,296                  | -    |
| J5   | Wrist pitch   | +/- 361.0 deg      | 1,333 deg/s       | 2,667 deg/s      | 1,769,472                  | -    |
| J6   | Tool roll     | +/- 573.0 deg      | 1,800 deg/s       | 3,600 deg/s      | 1,310,720                  | -    |


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


URDF-defined envelope values:


| Envelope point                                  | Value            |
| ----------------------------------------------- | ---------------- |
| Maximum horizontal radius                       | approx. 1,509 mm |
| Maximum vertical height                         | approx. 1,647 mm |
| Minimum vertical height                         | approx. -644 mm  |
| Total vertical envelope span                    | approx. 2,291 mm |
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
| Drive-native ratios                  | J1 100:1, J2 100:1, J3 100:1, J4 18:1, J5 27:2, J6 10:1                      |
| Power                                | 200-240 VAC 3PH, 35A                                                         |
| Expandable IO                        | Supported for cell and end-effector integration                              |
| Hollow cable routing                 | 62 mm shoulder pass-through; 33 mm wrist pass-through                        |
| Absolute encoder model in GradientOS | 17-bit motor-side count model                                                |
| Startup drive overrides              | None at robot layer; drive-family defaults live in the drive profile catalog |


## Page 3: Mechanical Interfaces

Drawings on the third page document the mounting, wrist, and tool-adapter interfaces used to integrate Rosie into a cell.

### Arm Envelope

Side projection of Rosie with key envelope dimensions:


| Dimension           | Value     |
| ------------------- | --------- |
| Wrist tool envelope | 346.17 mm |
| Wrist housing span  | 253.9 mm  |
| Tool-side offset    | 145.67 mm |
| Mount plate width   | 350.5 mm  |
| Base column width   | 150 mm    |


### Base Mount

Floor or pedestal mount footprint and bolt pattern:


| Dimension       | Value        |
| --------------- | ------------ |
| Mount plate     | 220 x 220 mm |
| Bolt circle     | 140 mm PCD   |
| Anchor bore     | Ø25.5 mm     |
| Plate thickness | 5 mm         |


### Wrist Flange

Hollow tool flange with high-density mount pattern:


| Dimension       | Value      |
| --------------- | ---------- |
| Hollow bore     | Ø33 mm     |
| Bolt pattern    | 85 x 85 mm |
| Threaded mounts | M5 x 6     |
| Flange width    | 92.3 mm    |
| Overall height  | 121.83 mm  |


### ISO50 Tool Adapter

Optional end-effector adapter, eight-position bolt circle:


| Dimension       | Value  |
| --------------- | ------ |
| Outer diameter  | Ø70 mm |
| Bolt circle PCD | Ø50 mm |
| Through bore    | Ø38 mm |
| Stack height    | 10 mm  |


## Company

Footer address: Advanced Metal Research | 13020 Yukon Ave | Hawthorne, CA 90250

## Source Files

This sheet was built from the following GradientOS sources:

- `src/gradient_os/arm_controller/robots/gradient05/config.py`
- `robots/gradient-05/gradient-05.urdf`
- `robots/gradient-05/robot.json`
- `robots/gradient-05/dh_params.csv`
- `src/gradient_os/arm_controller/robots/base.py`
- `src/gradient_os/arm_controller/command_api.py`

## Verification Status


| Spec area                                         | Status                      |
| ------------------------------------------------- | --------------------------- |
| Axis count, limits, actuator mapping, gear ratios | Sourced from runtime config |
| URDF joint origins and envelope values            | Derived from current URDF   |
| Final production drawing dimensions               | Requires CAD drawing export |
