# A6-EC Manufacturer Notes 2026-04-15

Status: user-forwarded vendor correspondence note. This is not the project's canonical SOP by itself, but it is important source material for the current A6-EC migration and should be read before changing drive-native ratio or homing behavior.

## Source

These notes summarize the manufacturer's written reply provided by the user on 2026-04-15.

## Key Clarifications

- "Load" and `RM` in rotary mode refer to the output or load shaft after gearbox reduction, not the motor shaft.
- `RM = Encoder Resolution * Mechanical Gear Ratio Numerator / Mechanical Gear Ratio Denominator`.
- When `C10.1A / C10.1C` are zero, `RM` is determined by `C10.18 / C10.19`.
- The manufacturer recommends configuring the actual mechanical ratio in `C10.18 / C10.19`.
- If `C10.18 / C10.19` are left at `1:1`, `6064` and `U40.28` remain motor-side and the host must do the output-shaft conversion itself.
- `6091` is not the rotary-mode reconstruction source. It is the electronic gear ratio for command-unit conversion.

## Ratio Change Rule

- After changing `C10.18 / C10.19`, a fresh homing procedure is required.
- No power cycle is required to make that ratio change take effect.
- The manufacturer's framing is:
  - ratio change changes the coordinate system immediately
  - the home position must then be re-established with homing

## Homing vs Manual 607C Write

- Manually writing `607C` alone is not enough to establish a valid homing/reference state.
- After successful HM Method 35:
  - `6064` immediately reflects the new coordinate system
  - `6041 bit 12 = 1`
  - `6041 bit 15 = 1`
- With only a direct `607C` write:
  - `6064` does not change immediately
  - bits 12 and 15 are not set
- Vendor conclusion: HM Method 35 must still be executed even if `607C` is manually written.

## 607C Semantics

- In rotary mode, `607C` should stay in the range `0 .. RM-1`.
- Negative `607C` values may be accepted at a low level, but the manufacturer says they are semantically incorrect in rotary mode.
- For seam-adjacent homes, prefer a positive value near `RM-1` instead of a negative value.

## Valid Trust State

- The manufacturer defines a valid post-home trust state as:
  - `bit 12 = 1`
  - `bit 15 = 1`
  - `bit 13 = 0`
  - no active alarms
- The manufacturer explicitly identifies `0x9650` as an expected successful state for HM Method 35 completion.

## Persistence Notes

- `607C` persistence depends on `C13.10`.
- Recommended setting is the default `C13.10 = 1`, which enables automatic saving to EEPROM.
- The manufacturer says homing persistence is also affected by this setting.

## F31.10 Scope

- `F31.10` is described as an encoder-data recovery tool, not a normal commissioning step.
- Typical uses:
  - clearing encoder battery or multi-turn related faults
  - rebuilding after catastrophic data loss
- After `F31.10 = 4`, physical homing must be performed again.

## Recommended Vendor Flow

1. Set `C00.07 = 4` (Absolute Rotation Mode).
2. Set `C10.18 / C10.19` to the true mechanical ratio.
3. Keep `C13.10 = 1`.
4. Move the mechanism to the desired zero position.
5. Set `607C` to the desired home value.
6. Switch to HM mode (`6060 = 6`).
7. Set `6098 = 35`.
8. Trigger the controlword sequence `6 -> 7 -> 15 -> 31`.
9. Wait for `6041 bit 12 = 1` and `bit 15 = 1`.
10. Return to CSP (`6060 = 8`) for normal operation.

## Project-Relevant Implications

- This vendor note strongly supports making the drive own output-shaft semantics instead of keeping A6-EC in a permanent host-conversion posture.
- It also means that post-ratio-change commissioning should be modeled as:
  - startup/config change
  - fresh HM35 re-home
  - optional later restart or power-cycle only for persistence validation
- A power cycle is not part of the required homing-completion sequence after changing `C10.18 / C10.19`.


---
RAW EMAIL RESPONSES BELOW 


-Q1: Is C00.07 = 4 (Absolute Rotation Mode) suitable for rotary joints?


Yes, C00.07 = 4 corresponds to Absolute Rotation Mode, which is specifically designed for rotary mechanisms with unlimited travel range (such as turntables or robot joints).

Key Features:

The single-turn position range is 0 to (RM-1), where RM is the number of encoder pulses corresponding to one full revolution of the load.

If the motor is dragged and rotated while powered off, the position data will not be lost, provided the rotation does not exceed 32,767 turns.

Note: The setting range for the Origin Offset parameter (607Ch) must be between 0 and (RM-1).


-Q2: What is the correct procedure for "setting the current position as the origin"?

There are two methods:

Method A: Use HM Mode 35 (Recommended)

Ensure the motor is stationary and inactive.

Set 6060h = 6, 6098h = 35, and 60E6h = 0.

First, write your desired origin position value into 607Ch.

Trigger the process by writing the following sequence to the Control Word (6040h): 6 → 7 → 15 → 31.

Once the homing process is complete, the value in 6064h will be directly overwritten with the value from 607Ch.

Method B: Directly write to 607Ch

Calculate the specific value you want for the "Current Position = Origin Offset" setting.

Write this value directly into 607Ch.

The drive will automatically save this value to its EEPROM.


-Q3: Which object is used for the persistent storage of the Origin Offset?

607Ch (Origin Offset) is the correct object for persistent storage.

After a homing operation is completed using an HM mode, the drive automatically calculates the offset and saves it to the EEPROM.

Manually writing a value to 607Ch will also result in automatic saving.

60B0h (Position Offset) is a runtime offset; it is not saved upon power-off, so do not use it for this purpose.


-Q4: During normal CSP (Cyclic Synchronous Position) operation, which object serves as the authoritative position reference?

6064h (Position Feedback) is the authoritative position reference you are looking for. It already represents the "semantic coordinate system established after homing," so no further calculations involving 607Ch are required.

Meaning of each object:

Table

Object | Meaning

6064h | Position Feedback (in command units); Authoritative Position

U40.16 | Absolute Position Feedback (in command units)

U40.20/U40.22 | Encoder Multi-turn Information (Low/High 32 bits)

U40.2A/U40.2C | Rotational Mode Position Feedback (in encoder units) (Low/High 32 bits)


-Q5: How do I determine if homing was successful?

Check the following bits within 6041h (Statusword):

Table

Bit Name | Meaning

Bit 12 | Homing Attained: 1 = The homing operation is fully complete.

Bit 15 | Homing Reference Attained: 1 = The absolute coordinate system has been successfully established.

Bit 13 | Homing Error: 1 = An error occurred during the homing process.

Verification Logic: If Bit 12 = 1 AND Bit 15 = 1, then homing was successful.


-Q6: Are Bit 15 and Bit 12 of 0x6041 interpreted correctly?

Yes, that is correct.

Bit 12: The homing operation is complete.

Bit 15: The absolute coordinate system has been successfully established (this remains valid even after a power cycle/restart).

You must check both bits; only when both are set to 1 is the "reference considered valid and trustworthy."


-Q7: Does 607Ch need to be explicitly saved?

No, you do not need to manually invoke 1010h to save it.

Once homing is complete in HM (Homing) mode, the drive automatically saves the homing offset to the EEPROM.

1010h:01 is the general command for saving parameters, typically used to save changes after manually modifying parameters. However, the saving of 607Ch is performed automatically.


-Q8: Is Homing Method 35 correct?

Yes, setting 6098h = 35 corresponds to the "silent" method of "setting the current position as the home position." If the motor is stationary, calculations are performed based on parameter 60E6h:

60E6h = 0 (Absolute): 6064h is overwritten with the value of 607Ch.

60E6h = 1 (Relative): 6064h = Original Value + 607Ch.

You should simply use 60E6h = 0.


-Q9: Is there a recommended procedure?

Standard Commissioning Procedure:

Set C00.07 = 4 (Rotary Mode), then power cycle the device.

Manually rotate the joint to the desired "Home Position."

Switch to HM (Homing) Mode: Set 6060h = 6.

Set 6098h = 35 and 60E6h = 0.

Write 607Ch = 0 (if you wish to designate the current position as the zero point).

Sequentially write the following values ​​to the Control Word (6040h): 6 → 7 → 15 → 31.

Wait until both Bit 12 and Bit 15 of the Status Word (6041h) turn to 1.

Switch back to CSP (Cyclic Synchronous Position) Mode: Set 6060h = 8 to resume normal motion.

After powering off and restarting, 6064h will automatically restore to the correct position; re-homing is not required.


-Q10: How do I distinguish between the underlying coordinate system and the user coordinate system?

A simple explanation:

Encoder Coordinate System: U40.20 / U40.22 (Raw multi-turn encoder data).

User / Semantic Coordinate System: 6064h (This value already incorporates the offset defined in 607Ch).

The host controller only needs to read 6064h; the drive has already calculated the underlying offset for you.


email 2: 

1. Meaning of "Load" and Definition of RM

"The load completes one full rotation" refers to the output shaft (or load shaft) completing one full revolution after passing through the gearbox reduction mechanism.

Definition of RM:

• RM = Encoder Resolution × Mechanical Gear Ratio Numerator / Mechanical Gear Ratio Denominator

• When the gear ratio is 1:1, RM is equal to the encoder resolution.

• RM is determined by parameters C10.18 / C10.19 (when C10.1A / C10.1C is set to 0).

2. Is it necessary to configure the actual output gear ratio within the drive?

It is recommended to set the actual gear ratio (C10.18 / C10.19) within the drive. The benefits are:

• 6064h directly represents the output shaft position, eliminating the need for conversion by the host controller.

• U40.28 (Rotary Mode Position Feedback) also reflects the output shaft position.

• The coordinate system is automatically and correctly restored after a power cycle.

If the gear ratio is not set within the drive (i.e., kept at 1:1), the following issues will arise:

• 6064h reflects the position on the motor side, not the output shaft position.

• After executing Homing Method 35, the Home Offset will not align with the actual mechanical position.

• The host controller must handle all position conversions independently.

3. Which parameters are used to define absolute position reconstruction in Rotary Mode?

• C10.18 / C10.19: Mechanical Gear Ratio Numerator / Denominator; used to calculate the single-turn position range (RM).

• C10.1A / C10.1C: Mechanical Absolute Position Limit (takes precedence over the gear ratio parameters).

• 607C: Home Offset; this value is persisted to the EEPROM.

• 60E6: Homing Position Calculation Method (Absolute / Relative); applicable when using Homing Method 35.

Note: 6091 represents the Electronic Gear Ratio; it is used for command unit conversion and does not affect position reconstruction in Rotary Mode.

4. Is it necessary to perform a homing procedure again after changing the gear ratio parameters?

Yes, you must perform the homing procedure again. The knowledge base explicitly states: When the mechanical gear ratio is modified, the absolute mechanical position undergoes a sudden shift; therefore, a homing operation must be performed again after the modification.

• No power cycle (power off/on) is required.

• The value in 607C remains intact, but the coordinate system has changed; consequently, the home position must be re-established.

5. Comparison: Manually Writing to 607C vs. HM Method 35

Manually writing to 607C alone is insufficient to establish a valid homing reference point status.

After executing HM Method 35:

• 6064h immediately reflects the new coordinate system.

• Bit 12 of 6041 is set to 1 (Homing Attained).

• Bit 15 of 6041 is set to 1 (Absolute Coordinate System Established).

When only writing to 607C:

• 6064h does not change immediately.

• Bits 12 and 15 are not set.

Conclusion: Even if 607C is manually written, HM Method 35 must still be executed to establish a valid homing status.

6. Interpretation of Negative Values in 607C

In rotary mode, the valid range for 607C should be 0 to (RM-1).

It has been observed that the system permits writing negative values to 607C and allows them to be read back; this is a low-level implementation detail. However:

• Negative values are semantically incorrect in rotary mode.

• They may lead to anomalies in position calculations.

Recommendation: Always set 607C within the range of 0 to (RM-1). If the home position is located near the "wrap-around seam" (the transition point), set a positive value close to RM-1 rather than using a negative value. 7. The Exact Difference Between 6064 and U40.16

• 6064h: Position feedback (command units)—the application-layer position value after conversion by the electronic gearing ratio.

• U40.16 (0x6063): Position feedback (encoder units)—the raw encoder pulse count.

After homing and a subsequent power cycle (reboot):

• Provided the gearing ratio is configured correctly, the two values should be consistent once converted.

• 6064h represents the authoritative application position, while U40.16 represents the underlying raw data.

8. The "Trust Model" Following the Completion of a Homing Operation

Conditions for a Valid Trust State:

• Bit 12 = 1 (Homing action completed)

• Bit 15 = 1 (Absolute coordinate system successfully established)

• Bit 13 = 0 (No homing errors)

• No active alarm states

No other additional conditions are required. EEPROM saving is controlled by parameter C13.10; the default value of 1 ensures automatic saving.

9. Interpretation of Status Word 0x9650

0x9650 in binary is 1001011001010000.

Analysis:

• Bit 12 = 1 ✓ (Homing completed)

• Bit 15 = 1 ✓ (Absolute coordinate system successfully established)

• Bit 10 = 1 (Position reached)

• The basic status bits indicate that the servo is enabled and operating normally.

This represents the expected successful state, indicating that Homing Method 35 has completed correctly and the absolute coordinate system has been successfully established.

10. The Function of Parameter 0x2013:17 (C13.10)

C13.10 serves as the EtherCAT Parameter Storage Selection:

• 0: Do not save to EEPROM

• 1: Save to EEPROM (Default value)

• 2: Write only when manually saved

Impact:

• Determines whether parameters written via communication commands are persisted (saved permanently).

• The persistence of parameter 607C depends on this setting.

• The persistence of the results from Homing Method 35 is also affected by this setting.

Recommendation: Retain the default value of 1 to ensure that the homing offset is automatically saved after the homing operation. 11. Function of F31.10

F31.10 Encoder Data Reset Options:

• 1: Read Encoder Data

• 2: Write Encoder Data

• 3: Reset Encoder Faults

• 4: Reset Encoder Faults and Multi-turn Data (Most commonly used)

Purpose:

• To clear the Er20.8 alarm when the battery is connected for the first time.

• To reset the system after an encoder multi-turn overflow (ErA0.1) occurs.

• To rebuild data following catastrophic data loss.

Note: After executing F31.10 = 4, a physical homing procedure must be performed again!

12. Recommended Implementation Strategy

Initial Configuration

• Set C00.07 = 4 (Absolute Rotation Mode).

• Set C10.18 / C10.19 to the actual gear ratio.

• Ensure C13.10 = 1 (Automatic Parameter Saving).

Initial Homing Calibration

• Move the mechanical system to the desired zero position.

• Set 607C to the desired home position value (e.g., 0 or a specific reference value).

• Switch to HM Mode (6060h = 6).

• Set 6098h = 35 (Method 35).

• Trigger the homing sequence (Control Word sequence: 6 → 7 → 15 → 31).

• Wait until 6041h displays Bit 12 = 1 and Bit 15 = 1.

Power-off and Restart Recovery

• No manual intervention is required; the system recovers automatically.

• 6064h directly displays the correct application position.

• Check that Bit 15 = 1 to confirm the coordinate system is valid.

Normal Operation

• Switch to CSP Mode (6060h = 8).

• Read 6064h as the authoritative position feedback.

email 3::

{our model b: Model B:
- Even after correct ratio setup and successful HM35, the host should reconstruct authoritative multi-turn semantic position from `U40.20/U40.22` plus host-side offsets/anchors
- `6064h` is not sufficient by itself for multi-turn planning or restart recovery}

--Core Conclusion: Model B is the correct choice.

Even if parameters C00.07 = 4 and C10.18/C10.19 are configured correctly, and the HM35 command is executed successfully, the host controller must still utilize the values ​​from U40.20/U40.22 to reconstruct the authoritative multi-turn semantic position.

Reason: In absolute rotation mode, parameter 6064h exhibits a "sawtooth wave" characteristic—after the output shaft completes one full revolution, the value jumps abruptly from (RM-1) back to 0, thereby failing to maintain multi-turn continuity.

--Question 1: Multi-turn continuity of 6064h

Answer: 6064h does not maintain unique continuity across multiple turns of rotation.

In absolute rotation mode, the single-turn position range for the rotating load is strictly defined as 0 to (RM-1).

• When the motor rotates in the forward direction: 6064h increments from 0 up to RM-1, then instantly jumps back to 0, exhibiting a periodic forward sawtooth waveform.

• When the motor rotates in the reverse direction: 6064h decrements from RM-1 down to 0, then instantly jumps back to RM-1, exhibiting a periodic reverse sawtooth waveform.

Even when modulo wrapping occurs, 6064h can still be considered authoritative data for the single-turn position; however, it cannot be directly utilized for multi-turn motion planning.

--Question 2: Using 6064h for motion planning

Answer: For rotary joints where the software limit range exceeds one full revolution, directly using 6064h for motion planning is incorrect.

Correct Procedure:

• The host controller should read U40.20/U40.22 (encoder multi-turn information) to acquire continuous multi-turn position data.

• By incorporating the gear ratio parameter, convert the encoder's multi-turn data into the semantic position of the output shaft.

• 6064h may be used for UI display purposes to show the single-turn position, but the multi-turn position must be reconstructed by the host controller.

--Question 3: Can 6064h be trusted after a system restart if Bit 15 = 1 but Bit 12 = 0?

Answer: It cannot be directly trusted. Status Word Bit Definitions:

• Bit 12 = 1: Homing operation fully completed.

• Bit 15 = 1: Absolute coordinate system successfully established.

In Absolute Rotary Mode, Bit 15 may remain at 1 after a restart (since the absolute coordinate system has already been established); however, if Bit 12 = 0, it indicates that the drive's internal state may not have fully recovered. Recommendation:

• Check whether the multi-turn data in U40.20/U40.22 is plausible.

• Verify that the position data matches the values ​​recorded prior to power-off before utilizing the system.

-- Question 4: Target Position Commands in CSP Mode

Answer: 607Ah is the only recommended object for target position commands.

In CSP Mode:

• 607Ah (Target Position): The target position must be written via this object.

• 60B0h (Position Offset): Optional; used for runtime position offsetting.

Issuing position commands directly via the U40.20/U40.22 register series is not supported. These are read-only feedback objects intended solely for reading encoder multi-turn data.

-- Question 5: Why U40.20/U40.22 are continuous while 6064h is not

Answer: This is the expected behavior in Absolute Rotary Mode and does not indicate a configuration error.

Explanation:

• U40.20/U40.22: Raw encoder multi-turn data; these objects record the cumulative number of rotations of the motor shaft and are inherently continuous.

• 6064h: In Rotary Mode, this object is designed to provide single-turn position feedback (ranging from 0 to RM-1), resulting in a sawtooth waveform.

If continuous multi-turn position data is required, the host controller should utilize U40.20/U40.22—applying the appropriate gear ratio for conversion—rather than relying on 6064h. --Question 6: Relationships Between Registers

Registers:

6064h: Position Feedback (Command Units); in Rotary Mode, this represents the single-turn position [0, RM-1].

6063h (U40.16): Position Feedback (Encoder Units); represents the raw encoder pulse count.

U40.20/U40.22: Encoder Multi-turn Information (Low/High 32 bits); represents continuous multi-turn data.

U40.28: Rotary Mode Position Feedback (Command Units) — Low 32 bits.

U40.2A/U40.2C: Rotary Mode Position Feedback (Encoder Units) — Low/High 32 bits.


Recommended Host-side Processing Rules:

Read U40.20/U40.22 as the authoritative multi-turn position data, and use the gear ratio to convert it into the output shaft position; verify the homing status via Bits 12 and 15 of register 6041h; and write the target position command to register 607Ah.


Complete Architecture Reference:

1. Position Reading: Read U40.20/U40.22 to obtain continuous multi-turn encoder data.

2. Position Conversion: On the host side, combine with the gear ratio (C10.18/C10.19) to convert the data into the semantic position of the output shaft.

3. Status Verification: Read 6041h to confirm that Bit 12 = 1 and Bit 15 = 1.

4. Target Writing: Write the target position to 607Ah (in Command Units).

5. Power-off Recovery: After restarting, directly read U40.20/U40.22; no re-homing is required.

EMAIL 4:

Q1. Issue regarding the retention of Bit 15 status after a power restart
(a) Is Bit 15 always cleared after a power-down and restart?
Yes, this is the intended behavior of the firmware design, not a fault. In the current firmware version of the A6-EC, Bit 15 is cleared following a power-down and restart. This behavior is determined by the firmware version; currently, there are no configuration parameters available to alter this behavior.
(b) Is there a specific firmware version or parameter that allows Bit 15 to be retained?
Not at present. On the drive side, there is currently no supported configuration scheme that allows Bit 15 to remain set to 1 after a power-down and restart. Your host-side verification scheme serves as a correct and valid alternative.
(c) Is the host-side verification scheme a recommended approach?
Yes, it is recommended. Your consistency check scheme—verifying the modulo RM consistency between U40.20/.22 and 6064h—is both correct and reliable. Provided the check passes, the current coordinate system data can be accepted as valid, eliminating the need to wait for Bit 15 to become 1. This scheme is robust and capable of correctly identifying valid state data from the pre-power-down condition.
Q2. Signals that explicitly indicate "U40.20/U40.22 data is no longer reliable"
(a) Primary signals indicating a data loss fault
Yes, the following fault codes should be primarily monitored:
Er20.8 - Encoder battery failure
Er20.9 - Encoder multi-turn abnormality
ErA0.1 - Encoder multi-turn overflow
ALF9.0 - Encoder battery warning
Additionally, you may monitor 0x603F (Error Code Register) and 0x203F (Extended Error Code). (b) Direct Objects Indicating Unreliable Data
In addition to fault codes, the following flag bits can also be monitored:
U41.05 Bit 13 - Multi-turn Fault
U41.05 Bit 14 - Battery Fault
U41.05 Bit 15 - Battery Warning
C10.15 - Multi-turn Overflow Flag
(c) Recommended Tolerance Range for Plausibility Checks
The tolerance of 16 count units you have adopted is reasonable. Read jitter in a stationary state typically falls within a few count units; a tolerance of 16 counts is significantly higher than the actual jitter and effectively prevents false positives.
Q3. Preconditions for HM35
Your implementation of the two-stage transaction is correct: first, disable the enable signal; wait for the state transition to complete; and then execute HM35.
Regarding the stabilization waiting time: 500 milliseconds is a reasonable and conservative setting. The drive documentation does not explicitly specify a minimum waiting time; your 500ms strategy ensures state stability and represents a recommended, conservative approach.
Core Requirements: The motor must be in a stationary state AND have exited the "OperationEnabled" state.
