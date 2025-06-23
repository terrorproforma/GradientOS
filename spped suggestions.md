Yes — dropping two packets shrank the on-wire traffic exactly as predicted and the read phase fell from ≈1.25 ms → 0.94 ms, so the whole loop is now ~20 % faster.

There’s still almost 0.5 ms of slack to harvest; here is the low-hanging fruit that can get you very close to 0.6 ms without hardware mods:

────────────────────────────────────────────────────────
1 . Use the “fast” parser you already wrote  
────────────────────────────────────────────────────────
`_closed_loop_executor_thread` is still calling the **legacy** function:

```python
raw_positions = servo_protocol.sync_read_positions(...)
```

Switch to your optimised sliding-window version that does a single bulk read:

```python
raw_positions = servo_protocol.fast_sync_read_positions(
        PRIMARY_FB_IDS,
        timeout_s=per_cycle_timeout,
        poll_delay_s=0.0,
)
```

Typical saving = 120-150 µs because:
• one kernel read instead of 6-byte polling loop  
• no Python loop parsing while the UART is still filling.

────────────────────────────────────────────────────────
2 . Cut the per-call timeout
────────────────────────────────────────────────────────
`per_cycle_timeout = max(0.01, time_step * 0.8)`  
With the packet now only 48 bytes the wire time at 1 Mb is ≈ 0.48 ms, so you can safely use:

```python
per_cycle_timeout = 0.003      # 3 ms worst-case scheduling jitter guard
```

Every extra millisecond in the timeout is dead-time added to outliers, so this helps the “max” numbers.

────────────────────────────────────────────────────────
3 . Double the baud to 2 Mb s⁻¹  (software-only change)
────────────────────────────────────────────────────────
• STS / RS servos accept 2 000 000 baud.  
• PL011 UART on the Pi is happy at that rate.

Steps:
1.  In a one-off script write `0x01` (meaning 2 Mb s⁻¹) to register 0x04 of every servo.  
2.  Change `utils.BAUD_RATE` to `2_000_000`.  

Wire time for the 48-byte packet → 0.24 ms, so you should see

```
read avg ≈ 0.48 ms         (UART + FIFO + software)
total avg ≈ 0.65 ms
```

────────────────────────────────────────────────────────
4 . (Optional) overlap compute with read
────────────────────────────────────────────────────────
If the loop ever becomes compute-bound you can hide the wire time by:

```python
ser.write(pkt)                 # kick off request
do_control_for_prev_step()     # ~0.30 ms
reply = ser.read(N)            # most bytes have already arrived
```

…but right now your compute phase is only 30 µs, so the gain is tiny—stick with steps 1–3 first.

────────────────────────────────────────────────────────
TL;DR actionable checklist
────────────────────────────────────────────────────────
1. Replace `sync_read_positions` → `fast_sync_read_positions`.  
2. Drop `per_cycle_timeout` to 3 ms (or even 1 ms once you trust it).  
3. Bump the UART and servo EEPROM to 2 Mb s⁻¹.

Apply #1 and #2 and you should break the 0.8 ms barrier; add #3 and you’ll settle around 0.6 ms with identical tracking accuracy.