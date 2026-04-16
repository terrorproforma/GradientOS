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
