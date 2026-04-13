# A6-EC Servo Drive Manual: Chapter 11 Parameter List

Source PDF: `docs/resources/chapter 11 - parameter list - A6-EC_series_servo_drive_manual (2).pdf`.

This revision fixes the first-pass extraction problems by rebuilding the parameter tables into real HTML tables. The long `11.3` reference text is kept in manual-style formatting so formulas and dense notes remain intact.

## Contents

- `11.1` Parameter Group Description
- `11.2` Parameter List
- `Parameters (2000h/C00)`
- `Basic Gain Parameters (2001h/C01)`
- `Advanced Gain Parameters (2002h/C02)`
- `Instruction Parameters (2003h/C03)`
- `I/O Parameters (2004h/C04)`
- `Stop Mode (2005h/C05)`
- `Protection Parameters (2006h/C06)`
- `Auto-tuning Parameters (2007h/C07)`
- `Communication Parameters (200Ah/C0A)`
- `Homing Touch Probe Parameters (2010h/C10)`
- `EtherCAT Parameters (2013h/C13)`
- `Motor Parameters (2020h/R20)`
- `Drive Parameters (2021h/R21)`
- `Motor Gain Parameters (2022h/R22)`
- `Parameters of Control in Progress (2030h/F30)`
- `Parameters of Control in Progress (2031h/F31)`
- `Running Monitoring Parameters (2040h/U40)`
- `Status Monitoring Parameters (2041h/U41)`
- `Version Parameters (2042h/U42)`
- `11.2.2` Common Parameters in Group `6000h`
- `11.3` Description of Parameters
- `11.3.1 Group C00`
- `11.3.2 Group C01`
- `11.3.3 Group C03`
- `11.3.4 Group C05`
- `11.3.5 Group C06`
- `11.3.6 Group C0A`
- `11.3.7 Group C13`
- `11.3.8 Group R21`
- `11.3.9 Group F30`
- `11.3.10 Group F31`
- `11.3.11 Group U40`
- `11.3.12 Group 6000`

## 11.1 Parameter Group Description

Parameter access address: index+subindex, both in hexadecimal format.
The CiA402 protocol has the following constraints on the address of system parameters.

| Index | Description |
|---|---|
| `0001h—0FFFh` | Data type description |
| `1000h—1FFFh` | CoE communication object |
| `2000h—5FFFh` | Manufacturer specific object |
| `6000h—9FFFh` | Sub-protocol object |
| `A000h—FFFFh` | Reserved |

## 11.2 Parameter List

### 11.2.1 Common Parameters in Group `2000h`

#### Parameters (2000h/C00)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>01h</code></td>
<td><code>C00.00</code></td>
<td>Control mode</td>
<td>10: EtherCAT</td>
<td>Range: 0-10<br>Default: 10<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>02h</code></td>
<td><code>C00.01</code></td>
<td>Motor rotating direction</td>
<td>0: CCW 1: CW</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Upon re-power-on</td>
</tr>
<tr>
<td><code>05h</code></td>
<td><code>C00.04</code></td>
<td>Auto-tuning mode</td>
<td>0: Manual mode 1: Standard mode 2: Positioning mode</td>
<td>Range: 0-2<br>Default: 1<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>06h</code></td>
<td><code>C00.05</code></td>
<td>Stiffness level</td>
<td>-</td>
<td>Range: 1-31<br>Default: 12<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>07h</code></td>
<td><code>C00.06</code></td>
<td>Load inertia ratio</td>
<td>-</td>
<td>Range: 0-12000<br>Default: 100<br>Unit: %<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>08h</code></td>
<td><code>C00.07</code></td>
<td>Absolute mode</td>
<td>0: Incremental position mode 1: Absolute position linear mode 2: Absolute position linear infinite mode 3: Absolute position single-turn mode 4: Absolute position rotation mode 5: Absolute mechanical single-turn mode (operating direction selectable)</td>
<td>Range: 0-5<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Upon re-power-on</td>
</tr>
<tr>
<td><code>11h</code></td>
<td><code>C00.10</code></td>
<td>Bleeder resistor selection</td>
<td>0: Internal bleeder resistor 1: External bleeder resistor 2: No bleeder resistor 3: Capacitor bleeder resistor</td>
<td>Range: 0-3<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>12h</code></td>
<td><code>C00.11</code></td>
<td>Bleeder resistor power</td>
<td>-</td>
<td>Range: 1-65535<br>Default: 50<br>Unit: W<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>13h</code></td>
<td><code>C00.12</code></td>
<td>Bleeder resistor resistance</td>
<td>-</td>
<td>Range: 1-65535<br>Default: 50<br>Unit: Ω<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>14h</code></td>
<td><code>C00.13</code></td>
<td>Bleeder resistor heat dissipation coeffi-cient</td>
<td>-</td>
<td>Range: 1-100<br>Default: 30<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>15h</code></td>
<td><code>C00.14</code></td>
<td>Brake enable switch</td>
<td>-</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>17h</code></td>
<td><code>C00.16</code></td>
<td>Panel display</td>
<td>0: Default display - 1: Speed display 2: Torque display 3: Voltage display 4: Load rate display</td>
<td>Range: 0-4<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>32h</code></td>
<td><code>C00.31</code></td>
<td>Super user</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

Note: For details about parameters above, refer to  section 11.3.1 "Group C00".

#### Basic Gain Parameters (2001h/C01)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>01h</code></td>
<td><code>C01.00</code></td>
<td>1st position loop gain</td>
<td>-</td>
<td>Range: 0-20000<br>Default: 400<br>Unit: 0.1rad/ s<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>02h</code></td>
<td><code>C01.01</code></td>
<td>1st speed loop gain</td>
<td>-</td>
<td>Range: 1-20000<br>Default: 250<br>Unit: 0.1Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>03h</code></td>
<td><code>C01.02</code></td>
<td>1st speed loop integral time parameter</td>
<td>-</td>
<td>Range: 1-51200<br>Default: 3184<br>Unit: 0.01ms<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>04h</code></td>
<td><code>C01.03</code></td>
<td>1st torque reference filter cutoff frequency</td>
<td>-</td>
<td>Range: 5-16000<br>Default: 200<br>Unit: Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>09h</code></td>
<td><code>C01.08</code></td>
<td>2nd position loop gain</td>
<td>-</td>
<td>Range: 0-20000<br>Default: 560<br>Unit: 0.1rad/ s<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Ah</code></td>
<td><code>C01.09</code></td>
<td>2nd speed loop gain</td>
<td>-</td>
<td>Range: 1-20000<br>Default: 350<br>Unit: 0.1Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Bh</code></td>
<td><code>C01.0A</code></td>
<td>2nd speed loop integral time parameter</td>
<td>-</td>
<td>Range: 1-51200<br>Default: 2274<br>Unit: 0.01ms<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Ch</code></td>
<td><code>C01.0B</code></td>
<td>2nd torque reference filter cutoff frequency</td>
<td>-</td>
<td>Range: 5-16000<br>Default: 280<br>Unit: Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>11h</code></td>
<td><code>C01.10</code></td>
<td>Speed feedback filter</td>
<td>0: Internal setting 1: Low-pass filter 2: Overlapping average filter 3: Speed observer 4: No filter</td>
<td>Range: 0-4<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>12h</code></td>
<td><code>C01.11</code></td>
<td>Cutoff frequency of speed feedback low-pass filter</td>
<td>-</td>
<td>Range: 10-16000<br>Default: 8000<br>Unit: Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>13h</code></td>
<td><code>C01.12</code></td>
<td>0: No filter 1: 2 times filter Speed feedback 2: 4 times filter overlapping 3: 8 times filter average filter time 4: 16 times filter constant 5: 32 times filter 6: 64 times filter</td>
<td>-</td>
<td>Range: 0-6<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>14h</code></td>
<td><code>C01.13</code></td>
<td>Speed feedforward source</td>
<td>0: No feedforward 1: Internal reference 2: Model tracking 5: Communication</td>
<td>Range: 0-5<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>15h</code></td>
<td><code>C01.14</code></td>
<td>Speed feedforward percentage</td>
<td>-</td>
<td>Range: 0-2000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>16h</code></td>
<td><code>C01.15</code></td>
<td>Speed feedforward filter cutoff frequency</td>
<td>-</td>
<td>Range: 5-16000<br>Default: 318<br>Unit: Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>17h</code></td>
<td><code>C01.16</code></td>
<td>Torque feedforward source</td>
<td>0: No feedforward 1: Internal reference 2: Model tracking 5: Communication</td>
<td>Range: 0-5<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>18h</code></td>
<td><code>C01.17</code></td>
<td>Torque feedforward percentage</td>
<td>-</td>
<td>Range: 0-2000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>19h</code></td>
<td><code>C01.18</code></td>
<td>Torque feedforward filter cutoff frequency</td>
<td>-</td>
<td>Range: 5-16000<br>Default: 318<br>Unit: Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>1Ch</code></td>
<td><code>C01.1B</code></td>
<td>PDFF control coefficient</td>
<td>-</td>
<td>Range: 0-1000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>1Dh</code></td>
<td><code>C01.1C</code></td>
<td>Damping factor control coefficient</td>
<td>-</td>
<td>Range: 0-1000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>21h</code></td>
<td><code>C01.20</code></td>
<td>Position reference overlapping average filter time constant A</td>
<td>-</td>
<td>Range: 0-1280<br>Default: 0<br>Unit: 0.1ms<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>22h</code></td>
<td><code>C01.21</code></td>
<td>Position reference overlapping average filter time constant B</td>
<td>-</td>
<td>Range: 0-1280<br>Default: 0<br>Unit: 0.1ms<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>23h</code></td>
<td><code>C01.22</code></td>
<td>Position reference low-pass filter time constant A</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: 0.1ms<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>24h</code></td>
<td><code>C01.23</code></td>
<td>Position reference low-pass filter time constant B</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: 0.1ms<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>25h</code></td>
<td><code>C01.24</code></td>
<td>1st notch filter frequency of position reference</td>
<td>-</td>
<td>Range: 0-2000<br>Default: 0<br>Unit: 0.1Hz<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>26h</code></td>
<td><code>C01.25</code></td>
<td>1st notch filter width of position reference</td>
<td>-</td>
<td>Range: 0-1000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>27h</code></td>
<td><code>C01.26</code></td>
<td>1st notch filter depth of position reference</td>
<td>-</td>
<td>Range: 10-1000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>28h</code></td>
<td><code>C01.27</code></td>
<td>2nd notch filter frequency of position reference</td>
<td>-</td>
<td>Range: 0-2000<br>Default: 0<br>Unit: 0.1Hz<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>29h</code></td>
<td><code>C01.28</code></td>
<td>2nd notch filter width of position reference</td>
<td>-</td>
<td>Range: 0-1000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>2Ah</code></td>
<td><code>C01.29</code></td>
<td>2nd notch filter depth of position reference</td>
<td>-</td>
<td>Range: 10-1000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>2Bh</code></td>
<td><code>C01.2A</code></td>
<td>Position reference pre-charge filter time constant</td>
<td>-</td>
<td>Range: 0-1280<br>Default: 0<br>Unit: 0.1ms<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>31h</code></td>
<td><code>C01.30</code></td>
<td>Adaptive notch mode</td>
<td>0: Disabled 1: 1st notch 2: 2nd notch 3: Notch parameter reset 4: R esonance frequency tested only</td>
<td>Range: 0-4<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>32h</code></td>
<td><code>C01.31</code></td>
<td>Adaptive notch test times</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: Times<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>39h</code></td>
<td><code>C01.38</code></td>
<td>Gain switchover mode</td>
<td>0: Fixed to the 1st gain set 1: DI switchover 2: DI P-PI switchover 3: Torque reference 4: Speed reference 5: Speed feedback 6: S peed reference change rate 7: Position deviation 8: Position reference</td>
<td>Range: 0-8<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>3Ah</code></td>
<td><code>C01.39</code></td>
<td>Gain switchover time</td>
<td>-</td>
<td>Range: 10-10000<br>Default: 50<br>Unit: 0.1ms<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>3Bh</code></td>
<td><code>C01.3A</code></td>
<td>Gain switchover threshold</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 10<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>3Ch</code></td>
<td><code>C01.3B</code></td>
<td>Gain switchover loop width</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 10<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>41h</code></td>
<td><code>C01.40</code></td>
<td>Frequency of the 1st notch</td>
<td>-</td>
<td>Range: 10-8000<br>Default: 8000<br>Unit: Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>42h</code></td>
<td><code>C01.41</code></td>
<td>Width level of the 1st notch</td>
<td>-</td>
<td>Range: 0-4000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>43h</code></td>
<td><code>C01.42</code></td>
<td>Depth level of the 1st notch</td>
<td>-</td>
<td>Range: 10-1000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>44h</code></td>
<td><code>C01.43</code></td>
<td>Frequency of the 2nd notch</td>
<td>-</td>
<td>Range: 10-8000<br>Default: 8000<br>Unit: Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>45h</code></td>
<td><code>C01.44</code></td>
<td>Width level of the 2nd notch</td>
<td>-</td>
<td>Range: 0-4000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>46h</code></td>
<td><code>C01.45</code></td>
<td>Depth level of the 2nd notch</td>
<td>-</td>
<td>Range: 10-1000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>47h</code></td>
<td><code>C01.46</code></td>
<td>Frequency of the 3rd notch</td>
<td>-</td>
<td>Range: 10-8000<br>Default: 8000<br>Unit: Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>48h</code></td>
<td><code>C01.47</code></td>
<td>Width level of the 3rd notch</td>
<td>-</td>
<td>Range: 0-4000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>49h</code></td>
<td><code>C01.48</code></td>
<td>Depth level of the 3rd notch</td>
<td>-</td>
<td>Range: 10-1000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>4Ah</code></td>
<td><code>C01.49</code></td>
<td>Frequency of the 4th notch</td>
<td>-</td>
<td>Range: 10-8000<br>Default: 8000<br>Unit: Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>4Bh</code></td>
<td><code>C01.4A</code></td>
<td>Width level of the 4th notch</td>
<td>-</td>
<td>Range: 0-4000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>4Ch</code></td>
<td><code>C01.4B</code></td>
<td>Depth level of the 4th notch</td>
<td>-</td>
<td>Range: 10-1000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>4Dh</code></td>
<td><code>C01.4C</code></td>
<td>Frequency of the 5th notch</td>
<td>-</td>
<td>Range: 10-8000<br>Default: 8000<br>Unit: Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>4Eh</code></td>
<td><code>C01.4D</code></td>
<td>Width level of the 5th notch</td>
<td>-</td>
<td>Range: 0-4000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>4Fh</code></td>
<td><code>C01.4E</code></td>
<td>Depth level of the 5th notch</td>
<td>-</td>
<td>Range: 10-1000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

Note: For details about parameters above, refer to  section 11.3.2 "Group C01".

#### Advanced Gain Parameters (2002h/C02)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>01h</code></td>
<td><code>C02.00</code></td>
<td>Model Name tracking control</td>
<td>0: Disabled Options 1: S ingle mass model tracking</td>
<td>Range: Value Range Default<br>Default: 0-1<br>Unit: 0<br>Type: -<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>02h</code></td>
<td><code>C02.01</code></td>
<td>Model tracking control gain</td>
<td>-</td>
<td>Range: Value Range Default<br>Default: 10-20000<br>Unit: 500<br>Type: 0.1rad/ U16 s<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>03h</code></td>
<td><code>C02.02</code></td>
<td>Model tracking inertia correction coefficient</td>
<td>-</td>
<td>Range: 10-8000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>31h</code></td>
<td><code>C02.30</code></td>
<td>Speed observer gain</td>
<td>-</td>
<td>Range: 0-40000<br>Default: 0<br>Unit: 0.1Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>32h</code></td>
<td><code>C02.31</code></td>
<td>Speed observer inertia correction</td>
<td>-</td>
<td>Range: 10-8000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>33h</code></td>
<td><code>C02.32</code></td>
<td>Speed observer speed feedback cutoff frequency</td>
<td>-</td>
<td>Range: 0-16000<br>Default: 0<br>Unit: Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>39h</code></td>
<td><code>C02.38</code></td>
<td>Frequency for vibration suppression 1</td>
<td>-</td>
<td>Range: 10-20000<br>Default: 1000<br>Unit: 0.1Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>3Ah</code></td>
<td><code>C02.39</code></td>
<td>Inertia correction for vibration suppression 1</td>
<td>-</td>
<td>Range: 10-8000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>3Bh</code></td>
<td><code>C02.3A</code></td>
<td>Low-pass filter correction for vibration suppression 1</td>
<td>-</td>
<td>Range: -9999-9999<br>Default: 0<br>Unit: 0.1Hz<br>Type: I16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>3Ch</code></td>
<td><code>C02.3B</code></td>
<td>Correction of high-pass filter 1 for vibration sup- pression 1</td>
<td>-</td>
<td>Range: -9999-9999<br>Default: 0<br>Unit: 0.1Hz<br>Type: I16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>3Dh</code></td>
<td><code>C02.3C</code></td>
<td>Frequency of high- pass filter 2 for vibration sup- pression 1</td>
<td>-</td>
<td>Range: 10-50000<br>Default: 20000<br>Unit: 0.1Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>3Eh</code></td>
<td><code>C02.3D</code></td>
<td>Ratio of compensation 1 for vibration suppression 1</td>
<td>-</td>
<td>Range: 0-20000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>3Fh</code></td>
<td><code>C02.3E</code></td>
<td>Ratio of compensation 2 for vibration suppression 1</td>
<td>-</td>
<td>Range: 0-20000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>41h</code></td>
<td><code>C02.40</code></td>
<td>Frequency for vibration suppression 2</td>
<td>-</td>
<td>Range: 10-20000<br>Default: 1000<br>Unit: 0.1Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>42h</code></td>
<td><code>C02.41</code></td>
<td>Inertia correction for vibration suppression 2</td>
<td>- Value Range Default</td>
<td>Range: 10-8000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>43h</code></td>
<td><code>C02.42</code></td>
<td>Low-pass filter correction for vibration suppression 2</td>
<td>-</td>
<td>Range: -9999-9999<br>Default: 0<br>Unit: 0.1Hz<br>Type: I16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>44h</code></td>
<td><code>C02.43</code></td>
<td>Correction of high-pass filter 1 for vibration sup- pression 2</td>
<td>-</td>
<td>Range: -9999-9999<br>Default: 0<br>Unit: 0.1Hz<br>Type: I16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>45h</code></td>
<td><code>C02.44</code></td>
<td>Frequency of high- pass filter 2 for vibration sup- pression 2</td>
<td>-</td>
<td>Range: 10-50000<br>Default: 20000<br>Unit: 0.1Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>46h</code></td>
<td><code>C02.45</code></td>
<td>Ratio of compensation 1 for vibration suppression 2</td>
<td>-</td>
<td>Range: 0-20000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>47h</code></td>
<td><code>C02.46</code></td>
<td>Ratio of compensation 2 for vibration suppression 2</td>
<td>-</td>
<td>Range: 0-20000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>49h</code></td>
<td><code>C02.48</code></td>
<td>Frequency for vibration suppression 3</td>
<td>-</td>
<td>Range: 10-20000<br>Default: 1000<br>Unit: 0.1Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>4Ah</code></td>
<td><code>C02.49</code></td>
<td>Inertia correction for vibration suppression 3</td>
<td>-</td>
<td>Range: 10-8000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>4Bh</code></td>
<td><code>C02.4A</code></td>
<td>Low-pass filter correction for vibration suppression 3</td>
<td>-</td>
<td>Range: -9999-9999<br>Default: 0<br>Unit: 0.1Hz<br>Type: I16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>4Ch</code></td>
<td><code>C02.4B</code></td>
<td>Correction of high-pass filter 1 for vibration sup- pression 3</td>
<td>-</td>
<td>Range: -9999-9999<br>Default: 0<br>Unit: 0.1Hz<br>Type: I16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>4Dh</code></td>
<td><code>C02.4C</code></td>
<td>Frequency of high- pass filter 2 for vibration sup- pression 3</td>
<td>-</td>
<td>Range: 10-50000<br>Default: 20000<br>Unit: 0.1Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>4Eh</code></td>
<td><code>C02.4D</code></td>
<td>Ratio of compensation 1 for vibration suppression 3</td>
<td>- Value Range Default</td>
<td>Range: 0-20000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>4Fh</code></td>
<td><code>C02.4E</code></td>
<td>Ratio of compensation 2 for vibration suppression 3</td>
<td>-</td>
<td>Range: 0-20000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>61h</code></td>
<td><code>C02.60</code></td>
<td>Disturbance observer gain</td>
<td>-</td>
<td>Range: 0-40000<br>Default: 0<br>Unit: 0.1Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>62h</code></td>
<td><code>C02.61</code></td>
<td>Disturbance observer inertia correction coefficient</td>
<td>-</td>
<td>Range: 1-10000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>63h</code></td>
<td><code>C02.62</code></td>
<td>Disturbance observer low-pass cutoff frequency</td>
<td>-</td>
<td>Range: 0-16000<br>Default: 0<br>Unit: Hz<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>64h</code></td>
<td><code>C02.63</code></td>
<td>Disturbance observer compensation torque per- centage</td>
<td>-</td>
<td>Range: 0-2000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>69h</code></td>
<td><code>C02.68</code></td>
<td>Friction compensation switch and relevant setting</td>
<td>-</td>
<td>Range: 0-255<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6Ah</code></td>
<td><code>C02.69</code></td>
<td>Friction compensation speed threshold</td>
<td>-</td>
<td>Range: -<br>Default: 0-5000<br>Unit: 20<br>Type: 0.1rpm U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6Bh</code></td>
<td><code>C02.6A</code></td>
<td>Static friction compensation</td>
<td>-</td>
<td>Range: 0-2000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6Ch</code></td>
<td><code>C02.6B</code></td>
<td>Forward friction compensation of coulomb friction</td>
<td>-</td>
<td>Range: 0-2000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6Dh</code></td>
<td><code>C02.6C</code></td>
<td>Reverse friction compensation of coulomb friction</td>
<td>-</td>
<td>Range: -2000-0<br>Default: 0<br>Unit: 0.1%<br>Type: I16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6Eh</code></td>
<td><code>C02.6D</code></td>
<td>Viscous friction torque for rated speed</td>
<td>-</td>
<td>Range: 0-2000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6Fh</code></td>
<td><code>C02.6E</code></td>
<td>Friction compensation filter time</td>
<td>-</td>
<td>Range: Value Range Default<br>Default: 0-65535<br>Unit: 0<br>Type: 0.01ms U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>70h</code></td>
<td><code>C02.6F</code></td>
<td>Friction compensation threshold for zero speed</td>
<td>-</td>
<td>Range: -<br>Default: 0-1000<br>Unit: 10<br>Type: 0.1rpm U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

#### Instruction Parameters (2003h/C03)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>22h</code></td>
<td><code>C03.21</code></td>
<td>Speed reference</td>
<td>-</td>
<td>Range: -8000-8000<br>Default: 100<br>Unit: rpm<br>Type: I16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>23h</code></td>
<td><code>C03.22</code></td>
<td>Acceleration rate</td>
<td>-</td>
<td>Range: 0-3600000<br>Default: 10<br>Unit: ms<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>25h</code></td>
<td><code>C03.24</code></td>
<td>Deceleration rate</td>
<td>-</td>
<td>Range: 0-3600000<br>Default: 10<br>Unit: ms<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>28h</code></td>
<td><code>C03.27</code></td>
<td>Internal positive speed limit</td>
<td>-</td>
<td>Range: 0-8000<br>Default: 6000<br>Unit: rpm<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>29h</code></td>
<td><code>C03.28</code></td>
<td>Internal negative speed limit</td>
<td>-</td>
<td>Range: 0-8000<br>Default: 6000<br>Unit: rpm<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>2Ch</code></td>
<td><code>C03.2B</code></td>
<td>Speed reach threshold</td>
<td>-</td>
<td>Range: 0-8000<br>Default: 1000<br>Unit: rpm<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>2Dh</code></td>
<td><code>C03.2C</code></td>
<td>Speed synchronization threshold</td>
<td>-</td>
<td>Range: 0-1000<br>Default: 10<br>Unit: rpm<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>2Eh</code></td>
<td><code>C03.2D</code></td>
<td>Speed rotation threshold</td>
<td>-</td>
<td>Range: 0-1000<br>Default: 20<br>Unit: rpm<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>2Fh</code></td>
<td><code>C03.2E</code></td>
<td>Zero speed output threshold</td>
<td>-</td>
<td>Range: 0-1000<br>Default: 10<br>Unit: rpm<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>42h</code></td>
<td><code>C03.41</code></td>
<td>Torque reference</td>
<td>-</td>
<td>Range: -4000-4000<br>Default: 0<br>Unit: 0.1%<br>Type: I16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>44h</code></td>
<td><code>C03.43</code></td>
<td>Internal positive torque limit</td>
<td>-</td>
<td>Range: 0-4000<br>Default: 3000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>45h</code></td>
<td><code>C03.44</code></td>
<td>Internal negative torque limit</td>
<td>-</td>
<td>Range: 0-4000<br>Default: 3000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>48h</code></td>
<td><code>C03.47</code></td>
<td>Positive speed limit in torque mode</td>
<td>-</td>
<td>Range: 0-8000<br>Default: 3000<br>Unit: rpm<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>49h</code></td>
<td><code>C03.48</code></td>
<td>Negative speed limit in torque mode</td>
<td>-</td>
<td>Range: 0-8000<br>Default: 3000<br>Unit: rpm<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>4Ah</code></td>
<td><code>C03.49</code></td>
<td>Reference value for torque reach</td>
<td>-</td>
<td>Range: 0-4000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>4Bh</code></td>
<td><code>C03.4A</code></td>
<td>Valid value for torque reached</td>
<td>-</td>
<td>Range: 0-4000<br>Default: 200<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>4Ch</code></td>
<td><code>C03.4B</code></td>
<td>Invalid value for torque reached</td>
<td>-</td>
<td>Range: 0-4000<br>Default: 100<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

Note: For details about parameters above, refer to  section 11.3.3 "Group C03".

#### I/O Parameters (2004h/C04)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>01h</code></td>
<td><code>C04.00</code></td>
<td>DI1 function selection</td>
<td>0: No definition 1: S-ON 2: Fault reset 4: Emergency stop 5: Home switch 6: Forward overtravel 7: Reverse overtravel 30: Probe 1 31: Probe 2</td>
<td>Range: 0-32<br>Default: 6<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>02h</code></td>
<td><code>C04.01</code></td>
<td>DI1 logic selection</td>
<td>0: Active low 1: Active high</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>03h</code></td>
<td><code>C04.02</code></td>
<td>DI1 filter time</td>
<td>-</td>
<td>Range: -<br>Default: 0-65535<br>Unit: 150<br>Type: 0.01ms U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>05h</code></td>
<td><code>C04.04</code></td>
<td>DI2 function selection</td>
<td>0: No definition 1: S-ON 2: Fault reset 4: Emergency stop 5: Home switch 6: Forward overtravel 7: Reverse overtravel 30: Probe 1 31: Probe 2</td>
<td>Range: 0-32<br>Default: 7<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>06h</code></td>
<td><code>C04.05</code></td>
<td>DI2 logic selection</td>
<td>0: Active low 1: Active high</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>09h</code></td>
<td><code>C04.08</code></td>
<td>DI3 function selection</td>
<td>0: No definition 1: S-ON 2: Fault reset 4: Emergency stop 5: Home switch 6: Forward overtravel 7: Reverse overtravel 30: Probe 1 31: Probe 2</td>
<td>Range: 0-32<br>Default: 5<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Ah</code></td>
<td><code>C04.09</code></td>
<td>DI3 logic selection</td>
<td>0: Active low 1: Active high</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Bh</code></td>
<td><code>C04.0A</code></td>
<td>DI3 filter time</td>
<td>-</td>
<td>Range: -<br>Default: 0-65535<br>Unit: 150<br>Type: 0.01ms U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Dh</code></td>
<td><code>C04.0C</code></td>
<td>DI4 function selection</td>
<td>0: No definition 1: S-ON 2: Fault reset 4: Emergency stop 5: Home switch 6: Forward overtravel 7: Reverse overtravel 30: Probe 1 31: Probe 2</td>
<td>Range: 0-32<br>Default: 31<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Eh</code></td>
<td><code>C04.0D</code></td>
<td>DI4 logic selection</td>
<td>0: Active low 1: Active high</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Fh</code></td>
<td><code>C04.0E</code></td>
<td>DI4 filter time</td>
<td>-</td>
<td>Range: -<br>Default: 0-65535<br>Unit: 150<br>Type: 0.01ms U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>11h</code></td>
<td><code>C04.10</code></td>
<td>DI5 function selection</td>
<td>0: No definition 1: S-ON 2: Fault reset 4: Emergency stop 5: Home switch 6: Forward overtravel 7: Reverse overtravel 30: Probe 1 31: Probe 2</td>
<td>Range: 0-32<br>Default: 30<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>12h</code></td>
<td><code>C04.11</code></td>
<td>DI5 logic selection</td>
<td>0: Active low 1: Active high</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>13h</code></td>
<td><code>C04.12</code></td>
<td>DI5 filter time</td>
<td>-</td>
<td>Range: -<br>Default: 0-65535<br>Unit: 150<br>Type: 0.01ms U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>31h</code></td>
<td><code>C04.30</code></td>
<td>DO1 function selection</td>
<td>0: No definition 1: Servo ready 2: Motor rotation 9: Brake output 10: Alarm 11: Fault 32: EDM safety state</td>
<td>Range: 0-20<br>Default: 1<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>32h</code></td>
<td><code>C04.31</code></td>
<td>DO1 logic selection</td>
<td>0: Active low 1: Active high</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>33h</code></td>
<td><code>C04.32</code></td>
<td>DO2 function selection</td>
<td>0: No definition 1: Servo ready 2: Motor rotation 9: Brake output 10: Alarm 11: Fault 32: EDM safety state</td>
<td>Range: 0-20<br>Default: 4<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>34h</code></td>
<td><code>C04.33</code></td>
<td>DO2 logic selection</td>
<td>0: Active low 1: Active high</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>35h</code></td>
<td><code>C04.34</code></td>
<td>DO3 function selection</td>
<td>0: No definition 1: Servo ready 2: Motor rotation 9: Brake output 10: Alarm 11: Fault 32: EDM safety state</td>
<td>Range: 0-20<br>Default: 3<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>36h</code></td>
<td><code>C04.35</code></td>
<td>DO3 logic selection</td>
<td>0: Active low 1: Active high</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

#### Stop Mode (2005h/C05)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>03h</code></td>
<td><code>C05.02</code></td>
<td>Stop mode at overtravel</td>
<td>0: C oast to stop, keeping de-energized status 1: S top at zero speed, keeping position lock status 2: S top at zero speed, keeping de-energized status 3: R amp to stop as defined by 6085h, keeping de- energized status 4: R amp to stop as defined by 6085h, keeping position lock status 5: D ynamic braking stop, keeping de-energized status 6: D ynamic braking stop, keeping dynamic braking status 7: N ot responding to overtravel</td>
<td>Range: 0-7<br>Default: 1<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>04h</code></td>
<td><code>C05.03</code></td>
<td>Stop mode at No. 1 fault</td>
<td>0: C oast to stop, keeping de-energized status 1: Dynamic braking stop, keeping de-energized status 2: D ynamic braking stop, keeping dynamic braking status</td>
<td>Range: 0-2<br>Default: 2<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Dh</code></td>
<td><code>C05.0C</code></td>
<td>Limit for stop at emergency-stop torque</td>
<td>-</td>
<td>Range: 0-3000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Eh</code></td>
<td><code>C05.0D</code></td>
<td>Maximum downtime</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 10000<br>Unit: ms<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>11h</code></td>
<td><code>C05.10</code></td>
<td>Delay from brake close to motor de-energized</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 100<br>Unit: ms<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>12h</code></td>
<td><code>C05.11</code></td>
<td>Speed threshold at brake closing</td>
<td>-</td>
<td>Range: 10-3000<br>Default: 30<br>Unit: rpm<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>13h</code></td>
<td><code>C05.12</code></td>
<td>Maximum waiting time with S-ON off at brake closing</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 100<br>Unit: ms<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>14h</code></td>
<td><code>C05.13</code></td>
<td>Delay from brake on to command received</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 100<br>Unit: ms<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>15h</code></td>
<td><code>C05.14</code></td>
<td>Energizing delay of DB relay</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 20<br>Unit: ms<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

Note: For details about parameters above, refer to  section 11.3.4 "Group C05".

#### Protection Parameters (2006h/C06)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>04h</code></td>
<td><code>C06.03</code></td>
<td>Threshold of excessive speed</td>
<td>-</td>
<td>Range: 0-9000<br>Default: 0<br>Unit: rpm<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>05h</code></td>
<td><code>C06.04</code></td>
<td>Input phase loss detection</td>
<td>0: Enabled 1: Disabled</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>06h</code></td>
<td><code>C06.05</code></td>
<td>Retentive at power 0: Non-retentive failure 1: Retentive</td>
<td>-</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>08h</code></td>
<td><code>C06.07</code></td>
<td>Mechanical limit position</td>
<td>0: Inactive 1: Enabled 2: Enabled after homing</td>
<td>Range: 0-2<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>09h</code></td>
<td><code>C06.08</code></td>
<td>Mechanical PL</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 2 31 -1<br>Unit: Unit in application<br>Type: I32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Bh</code></td>
<td><code>C06.0A</code></td>
<td>Mechanical NL</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: ﹣2 31<br>Unit: Unit in application<br>Type: I32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>11h</code></td>
<td><code>C06.10</code></td>
<td>Drive overload protection threshold</td>
<td>-</td>
<td>Range: 0-3500<br>Default: 1150<br>Unit: 0.1%<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>12h</code></td>
<td><code>C06.11</code></td>
<td>Motor overload protection threshold</td>
<td>-</td>
<td>Range: 0-3500<br>Default: 1150<br>Unit: 0.1%<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>13h</code></td>
<td><code>C06.12</code></td>
<td>Motor Name locked- rotor detection</td>
<td>0: Inactive Options 1: Enabled</td>
<td>Range: 0-1<br>Default: 1<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>14h</code></td>
<td><code>C06.13</code></td>
<td>Motor locked- rotor detection time</td>
<td>-</td>
<td>Range: 0-3000<br>Default: 200<br>Unit: ms<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>15h</code></td>
<td><code>C06.14</code></td>
<td>Motor locked- rotor detection speed</td>
<td>-</td>
<td>Range: 0-1000<br>Default: 10<br>Unit: rpm<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>16h</code></td>
<td><code>C06.15</code></td>
<td>Output phase loss 0: Inactive detection 1: Enabled</td>
<td>-</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>1Dh</code></td>
<td><code>C06.1C</code></td>
<td>Encoder communication fault tolerance threshold</td>
<td>-</td>
<td>Range: 0-88<br>Default: 3<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>21h</code></td>
<td><code>C06.20</code></td>
<td>Protection from out of control</td>
<td>0: Inactive 1: Enabled</td>
<td>Range: 0-1<br>Default: 1<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

Note: For details about parameters above, refer to  section 11.3.5 "Group C06".

#### Auto-tuning Parameters (2007h/C07)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>01h</code></td>
<td><code>C07.00</code></td>
<td>Offline inertia auto- tuning mode setting</td>
<td>-</td>
<td>Range: 0-785<br>Default: 769<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>02h</code></td>
<td><code>C07.01</code></td>
<td>Offline inertia auto-tuning speed reference</td>
<td>-</td>
<td>Range: 50-1000<br>Default: 500<br>Unit: rpm<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>03h</code></td>
<td><code>C07.02</code></td>
<td>Acceleration/ Deceleration time for offline inertia auto- tuning</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 100<br>Unit: ms<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>04h</code></td>
<td><code>C07.03</code></td>
<td>Offline inertia auto- tuning target torque</td>
<td>-</td>
<td>Range: 1-1500<br>Default: 150<br>Unit: 0.1%<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>05h</code></td>
<td><code>C07.04</code></td>
<td>Offline inertia auto- tuning revolutions</td>
<td>-</td>
<td>Range: 10-65535<br>Default: 200<br>Unit: 0.01r<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

#### Communication Parameters (200Ah/C0A)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>09h</code></td>
<td><code>C0A.08</code></td>
<td>Commissioning software communication station ID</td>
<td>-</td>
<td>Range: 1-255<br>Default: 1<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Ah</code></td>
<td><code>C0A.09</code></td>
<td>Commissioning software communication baud rate</td>
<td>0: 1200bps 1: 2400bps 2: 4800bps 3: 9600bps 4: 19200bps 5: 38400bps 6: 57600bps 7: 115200bps</td>
<td>Range: 0-7<br>Default: 7<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Upon re-power-on</td>
</tr>
<tr>
<td><code>0Bh</code></td>
<td><code>C0A.0A</code></td>
<td>Commissioning software communication format</td>
<td>0: No parity, 1 stop bit 1: Odd parity, 1 stop bit 2: Even parity, 1 stop bit 3: No parity, 2 stop bits 4: Odd parity, 2 stop bits 5: Even parity, 2 stop bits</td>
<td>Range: 0-5<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Upon re-power-on</td>
</tr>
<tr>
<td><code>0Ch</code></td>
<td><code>C0A.0B</code></td>
<td>Commissioning software communication response time</td>
<td>-</td>
<td>Range: 1-1000<br>Default: 1<br>Unit: ms<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Dh</code></td>
<td><code>C0A.0C</code></td>
<td>Commissioning software communication timeout</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Eh</code></td>
<td><code>C0A.0D</code></td>
<td>Commissioning software communication storage</td>
<td>0: No storage 1: Storage</td>
<td>Range: 0-1<br>Default: 1<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Fh</code></td>
<td><code>C0A.0E</code></td>
<td>Commissioning software data format</td>
<td>0: L ow 16 bits before high 16 bits 1: H igh 16 bits before low 16 bits</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

Note: For details about parameters above, refer to  section 11.3.6 "Group C0A".

#### Homing Touch Probe Parameters (2010h/C10)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>01h</code></td>
<td><code>C10.00</code></td>
<td>Homing enable</td>
<td>0: Inactive 1: W ritten through communication 2: DI trigger 3: C urrent position as home</td>
<td>Range: 0-3<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>09h</code></td>
<td><code>C10.08</code></td>
<td>Homing timeout interval</td>
<td>-</td>
<td>Range: -<br>Default: 0-(2 32 -1) 60000<br>Unit: ms<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>11h</code></td>
<td><code>C10.10</code></td>
<td>Multi-turn absolute position offset (low 32 bits)</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: P<br>Type: I32<br>Modification: At stop<br>Effective: Upon re- power- on</td>
</tr>
<tr>
<td><code>13h</code></td>
<td><code>C10.12</code></td>
<td>Multi-turn absolute position offset (high 32 bits)</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: P<br>Type: I32<br>Modification: At stop<br>Effective: Upon re- power- on</td>
</tr>
<tr>
<td><code>15h</code></td>
<td><code>C10.14</code></td>
<td>Multi-turn revolutions data offset</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: Rev<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>16h</code></td>
<td><code>C10.15</code></td>
<td>Multi-turn overflow flag</td>
<td>-</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>17h</code></td>
<td><code>C10.16</code></td>
<td>Reference running mode in rotation mode</td>
<td>0: Nearest 1: A lways in forward direction 2: A lways in reverse direction 3: A lways in current direction 4: Not specified</td>
<td>Range: 0-4<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>19h</code></td>
<td><code>C10.18</code></td>
<td>Numerator of electronic gear ratio in rotation mode</td>
<td>-</td>
<td>Range: 1-65535<br>Default: 1<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>1Ah</code></td>
<td><code>C10.19</code></td>
<td>Denominator of electronic gear ratio in rotation mode</td>
<td>-</td>
<td>Range: 1-65535<br>Default: 1<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>1Bh</code></td>
<td><code>C10.1A</code></td>
<td>Upper limit of mechanical absolute position in ro-tation mode (low 32 bits)</td>
<td>-</td>
<td>Range: 0-(2 32 -1)<br>Default: 0<br>Unit: P<br>Type: U32<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>1Dh</code></td>
<td><code>C10.1C</code></td>
<td>Upper limit of mechanical absolute position in ro-tation mode (high 32 bits)</td>
<td>-</td>
<td>Range: 0-(2 32 -1)<br>Default: 0<br>Unit: P<br>Type: U32<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>1Fh</code></td>
<td><code>C10.1E</code></td>
<td>Single-turn homing absolute value offset</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: Unit in application<br>Type: I32<br>Modification: At stop<br>Effective: Upon re- power- on</td>
</tr>
<tr>
<td><code>31h</code></td>
<td><code>C10.30</code></td>
<td>Torque limit of homing upon hit- and-stop</td>
<td>-</td>
<td>Range: 0-3000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>32h</code></td>
<td><code>C10.31</code></td>
<td>Speed for homing upon hit-and-stop</td>
<td>-</td>
<td>Range: 0-1000<br>Default: 10<br>Unit: rpm<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>33h</code></td>
<td><code>C10.32</code></td>
<td>Number of times for homing upon hit- and-stop</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 30<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

#### EtherCAT Parameters (2013h/C13)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>01h</code></td>
<td><code>C13.00</code></td>
<td>EtherCAT slave name</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>02h</code></td>
<td><code>C13.01</code></td>
<td>EtherCAT slave alias</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>03h</code></td>
<td><code>C13.02</code></td>
<td>EtherCAT sync loss threshold</td>
<td>-</td>
<td>Range: 1-20<br>Default: 8<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>04h</code></td>
<td><code>C13.03</code></td>
<td>EtherCAT synchronization detection mode</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>05h</code></td>
<td><code>C13.04</code></td>
<td>EtherCAT sync loss count</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>06h</code></td>
<td><code>C13.05</code></td>
<td>EtherCAT synchronization mode setting</td>
<td>-</td>
<td>Range: 0-2<br>Default: 1<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>07h</code></td>
<td><code>C13.06</code></td>
<td>EtherCAT synchronization error threshold</td>
<td>-</td>
<td>Range: 0-6000<br>Default: 3000<br>Unit: ns<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>08h</code></td>
<td><code>C13.07</code></td>
<td>Occurrence count of excessive position ref- erence increment in sync position mode</td>
<td>-</td>
<td>Range: 1-30<br>Default: 5<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>09h</code></td>
<td><code>C13.08</code></td>
<td>EtherCAT enhanced link selection</td>
<td>0: Inactive 1: Enabled</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Upon re-power-on</td>
</tr>
<tr>
<td><code>0Ah</code></td>
<td><code>C13.09</code></td>
<td>Maximum errors and invalid frames of EtherCAT port 0 per unit time</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Bh</code></td>
<td><code>C13.0A</code></td>
<td>Maximum errors and invalid frames of EtherCAT port 1 per unit time</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Ch</code></td>
<td><code>C13.0B</code></td>
<td>Max. transfer error of EtherCAT port per unit time</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Dh</code></td>
<td><code>C13.0C</code></td>
<td>Max. EtherCAT data frame processing unit error per unit time</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Eh</code></td>
<td><code>C13.0D</code></td>
<td>Max. link loss value of EtherCAT port per unit time</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Fh</code></td>
<td><code>C13.0E</code></td>
<td>EtherCAT state machine status and port connection status</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>10h</code></td>
<td><code>C13.0F</code></td>
<td>EtherCAT AL status code</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>11h</code></td>
<td><code>C13.10</code></td>
<td>EtherCAT parameter storage</td>
<td>0: No storage 1: Storage</td>
<td>Range: 0-1<br>Default: 1<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>12h</code></td>
<td><code>C13.11</code></td>
<td>EtherCAT IRQ loss threshold</td>
<td>-</td>
<td>Range: 0-10<br>Default: 5<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>13h</code></td>
<td><code>C13.12</code></td>
<td>EtherCAT IRQ loss count</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>1Ah</code></td>
<td><code>C13.19</code></td>
<td>Use of the loop network</td>
<td>-</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

Note: For details about parameters above, refer to  section 11.3.7 "Group C13".

#### Motor Parameters (2020h/R20)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>01h</code></td>
<td><code>R20.00</code></td>
<td>Motor model</td>
<td>-</td>
<td>Range: -<br>Default: 0-65535<br>Unit: 20000<br>Type: -<br>Modification: At stop<br>Effective: Upon re-power-on</td>
</tr>
<tr>
<td><code>23h</code></td>
<td><code>R20.22</code></td>
<td>Encoder type</td>
<td>-</td>
<td>Range: -<br>Default: 0-65535<br>Unit: 0<br>Type: -<br>Modification: Read only<br>Effective: Upon re-power-on</td>
</tr>
</tbody>
</table>

#### Drive Parameters (2021h/R21)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>01h</code></td>
<td><code>R21.00</code></td>
<td>Drive model</td>
<td>-</td>
<td>Range: -<br>Default: 0-65535<br>Unit: 3<br>Type: -<br>Modification: U16<br>Effective: At stop</td>
</tr>
<tr>
<td><code>02h</code></td>
<td><code>R21.01</code></td>
<td>Internal drive model</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 3<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Dh</code></td>
<td><code>R21.0C</code></td>
<td>Drive voltage class</td>
<td>-</td>
<td>Range: 0-2<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Eh</code></td>
<td><code>R21.0D</code></td>
<td>Rated drive power</td>
<td>- 1-(2 -1)</td>
<td>Range: 32<br>Default: 40<br>Unit: 0.01kW<br>Type: U32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>10h</code></td>
<td><code>R21.0F</code></td>
<td>Rated output current of drive</td>
<td>-</td>
<td>Range: 1-(2 32 -1)<br>Default: 280<br>Unit: 0.01A<br>Type: U32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>12h</code></td>
<td><code>R21.11</code></td>
<td>Maximum output current of drive</td>
<td>-</td>
<td>Range: 1-(2 32 -1)<br>Default: 980<br>Unit: 0.01A<br>Type: U32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>14h</code></td>
<td><code>R21.13</code></td>
<td>Internal bleeder resistor power</td>
<td>-</td>
<td>Range: 1-65535<br>Default: 40<br>Unit: W<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>15h</code></td>
<td><code>R21.14</code></td>
<td>Internal bleeder resistor resistance</td>
<td>-</td>
<td>Range: 1-65535<br>Default: 50<br>Unit: Ω<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

Note: For details about parameters above, refer to  section 11.3.8 "Group R21".

#### Motor Gain Parameters (2022h/R22)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>01h</code></td>
<td><code>R22.00</code></td>
<td>Current loop mode</td>
<td>0: Standard mode 1: P erformance mode</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Upon re-power-on</td>
</tr>
<tr>
<td><code>02h</code></td>
<td><code>R22.01</code></td>
<td>Current loop response level</td>
<td>-</td>
<td>Range: 0-4000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: At stop<br>Effective: Upon re-power-on</td>
</tr>
<tr>
<td><code>21h</code></td>
<td><code>R22.20</code></td>
<td>MTPA field-weakening switch</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 256<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>22h</code></td>
<td><code>R22.21</code></td>
<td>Field-weakening depth</td>
<td>-</td>
<td>Range: 500-2000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>23h</code></td>
<td><code>R22.22</code></td>
<td>Field-weakening proportional gain</td>
<td>-</td>
<td>Range: 10-1000<br>Default: 100<br>Unit: Hz<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>24h</code></td>
<td><code>R22.23</code></td>
<td>Field-weakening integral gain</td>
<td>-</td>
<td>Range: 0-8000<br>Default: 100<br>Unit: 0.1%<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>25h</code></td>
<td><code>R22.24</code></td>
<td>Cutoff frequency of d axis current low-pass filter</td>
<td>-</td>
<td>Range: 0-16000<br>Default: 0<br>Unit: Hz<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>26h</code></td>
<td><code>R22.25</code></td>
<td>Field-weakening d axis current limit</td>
<td>-</td>
<td>Range: 0-3000<br>Default: 1500<br>Unit: 0.1%<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>31h</code></td>
<td><code>R22.30</code></td>
<td>Dead zone compensation</td>
<td>-</td>
<td>Range: 0-2000<br>Default: 1000<br>Unit: 0.1%<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

#### Parameters of Control in Progress (2030h/F30)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>01h</code></td>
<td><code>F30.00</code></td>
<td>JOG enabling in velocity mode</td>
<td>-</td>
<td>Range: 0~8000<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>02h</code></td>
<td><code>F30.01</code></td>
<td>JOG enabling in position mode</td>
<td>-</td>
<td>Range: 0~8000<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>03h</code></td>
<td><code>F30.02</code></td>
<td>JOG velocity reference</td>
<td>-</td>
<td>Range: 0~8000<br>Default: 100<br>Unit: rpm<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>04h</code></td>
<td><code>F30.03</code></td>
<td>JOG acceleration/ deceleration time</td>
<td>-</td>
<td>Range: 0~3600000<br>Default: 100<br>Unit: ms<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>06h</code></td>
<td><code>F30.05</code></td>
<td>JOG distance in position mode</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 20000<br>Unit: Unit in application<br>Type: I32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>11h</code></td>
<td><code>F30.10</code></td>
<td>Inertia auto-tuning selection</td>
<td>0: Disabled 1: Enabled</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>12h</code></td>
<td><code>F30.11</code></td>
<td>Initial angle auto-tuning 0: Disabled selection 1: Enabled</td>
<td>-</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

Note: For details about parameters above, refer to  section 11.3.9 "Group F30".

#### Parameters of Control in Progress (2031h/F31)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>01h</code></td>
<td><code>F31.00</code></td>
<td>Fault reset</td>
<td>0: Inactive 1: Reset</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>02h</code></td>
<td><code>F31.01</code></td>
<td>Software reset</td>
<td>0: Inactive 1: Reset</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>03h</code></td>
<td><code>F31.02</code></td>
<td>Parameter initialization</td>
<td>0: Inactive 1: R estore default settings of parameters 2: R estore default settings of the object dictionary</td>
<td>Range: 0-2<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>04h</code></td>
<td><code>F31.03</code></td>
<td>Drive motor parameter reset</td>
<td>0: Inactive 1: F actory reset drive parameters 2: F actory reset motor parameters</td>
<td>Range: 0-2<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>05h</code></td>
<td><code>F31.04</code></td>
<td>Fault record initialization</td>
<td>0: Inactive 1: Fault record clearing</td>
<td>Range: 0-1<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>11h</code></td>
<td><code>F31.10</code></td>
<td>0: Inactive 1: Read encoder 2: Write encoder Encoder data reset 3: Reset encoder fault 4: R eset encoder fault and multi-turn data 16: Operation failed</td>
<td>-</td>
<td>Range: 0-31<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: At stop<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

Note: For details about parameters above, refer to  section 11.3.10 "Group F31".

#### Running Monitoring Parameters (2040h/U40)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>01h</code></td>
<td><code>U40.00</code></td>
<td>Speed reference</td>
<td>-</td>
<td>Range: -9000- 9000<br>Default: 0<br>Unit: rpm<br>Type: I16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>02h</code></td>
<td><code>U40.01</code></td>
<td>Speed feedback</td>
<td>-</td>
<td>Range: -9000- 9000<br>Default: 0<br>Unit: rpm<br>Type: I16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>03h</code></td>
<td><code>U40.02</code></td>
<td>Torque reference</td>
<td>-</td>
<td>Range: -4000- 4000<br>Default: 0<br>Unit: 0.1%<br>Type: I16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>04h</code></td>
<td><code>U40.03</code></td>
<td>Torque feedback</td>
<td>-</td>
<td>Range: -4000- 4000<br>Default: 0<br>Unit: 0.1%<br>Type: I16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>05h</code></td>
<td><code>U40.04</code></td>
<td>DI status</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>06h</code></td>
<td><code>U40.05</code></td>
<td>DO status</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>07h</code></td>
<td><code>U40.06</code></td>
<td>Bus voltage</td>
<td>-</td>
<td>Range: 0-9000<br>Default: 0<br>Unit: 0.1V<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>08h</code></td>
<td><code>U40.07</code></td>
<td>Average load ratio</td>
<td>-</td>
<td>Range: 0-4000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>09h</code></td>
<td><code>U40.08</code></td>
<td>Electrical angle</td>
<td>-</td>
<td>Range: 0-36000<br>Default: 0<br>Unit: 0.01°<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Ah</code></td>
<td><code>U40.09</code></td>
<td>Mechanical angle</td>
<td>-</td>
<td>Range: 0-36000<br>Default: 0<br>Unit: 0.01°<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Dh</code></td>
<td><code>U40.0C</code></td>
<td>RMS value of phase current</td>
<td>-</td>
<td>Range: -9000- 9000<br>Default: 0<br>Unit: 0.1A<br>Type: I16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>11h</code></td>
<td><code>U40.10</code></td>
<td>Position deviation counter</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: P<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>15h</code></td>
<td><code>U40.14</code></td>
<td>Absolute position reference</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: Unit in application<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>17h</code></td>
<td><code>U40.16</code></td>
<td>Absolute position feedback (reference unit)</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: Unit in application<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>19h</code></td>
<td><code>U40.18</code></td>
<td>Absolute position feedback (encoder unit)</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: P<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>1Bh</code></td>
<td><code>U40.1A</code></td>
<td>Absolute position feedback (encoder unit)</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: P<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>1Dh</code></td>
<td><code>U40.1C</code></td>
<td>Encoder single-turn data</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: P<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>1Fh</code></td>
<td><code>U40.1E</code></td>
<td>Encoder multi-turn position data</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: Rev<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>20h</code></td>
<td><code>U40.1F</code></td>
<td>Encoder initial angle</td>
<td>-</td>
<td>Range: 0-36000<br>Default: 0<br>Unit: 0.01°<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>21h</code></td>
<td><code>U40.20</code></td>
<td>Encoder multi-turn data (low 32 bits)</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: P<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>23h</code></td>
<td><code>U40.22</code></td>
<td>Encoder multi-turn data (high 32 bits)</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: P<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>25h</code></td>
<td><code>U40.24</code></td>
<td>Absolute position feedback (encoder unit) (low 32 bits)</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: P<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>27h</code></td>
<td><code>U40.26</code></td>
<td>Absolute position feedback (encoder unit) (high 32 bits)</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: P<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>29h</code></td>
<td><code>U40.28</code></td>
<td>Position feedback in rotation mode (reference unit) (low 32 bits)</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: Unit in application<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>2Bh</code></td>
<td><code>U40.2A</code></td>
<td>Position feedback in rotation mode (encoder unit) (low 32 bits)</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: P<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>2Dh</code></td>
<td><code>U40.2C</code></td>
<td>Position feedback in rotation mode (encoder unit) (high 32 bits)</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: P<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>31h</code></td>
<td><code>U40.30</code></td>
<td>Heatsink temperature</td>
<td>-</td>
<td>Range: -9000- 9000<br>Default: 0<br>Unit: 0.1℃<br>Type: I16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>35h</code></td>
<td><code>U40.34</code></td>
<td>Offline inertia auto- tuning value</td>
<td>-</td>
<td>Range: 0-12000<br>Default: 0<br>Unit: %<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>37h</code></td>
<td><code>U40.36</code></td>
<td>Instantaneous value in phase U current</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: 0.001A<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>39h</code></td>
<td><code>U40.38</code></td>
<td>Instantaneous value in phase V current</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: 0.001A<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>3Bh</code></td>
<td><code>U40.3A</code></td>
<td>Synchronization cycle measured value</td>
<td>-</td>
<td>Range: 0-(2 31 -1)<br>Default: 0<br>Unit: 10ns<br>Type: U32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>3Dh</code></td>
<td><code>U40.3C</code></td>
<td>SYNC and IRQ phase value</td>
<td>-</td>
<td>Range: ﹣2 31 - (2 31 -1)<br>Default: 0<br>Unit: 10ns<br>Type: I32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>3Fh</code></td>
<td><code>U40.3E</code></td>
<td>Drive accumulated heat</td>
<td>-</td>
<td>Range: 0-2000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>40h</code></td>
<td><code>U40.3F</code></td>
<td>Motor accumulated heat</td>
<td>-</td>
<td>Range: 0-2000<br>Default: 0<br>Unit: 0.1%<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

Note: For details about parameters above, refer to  section 11.3.11 "Group U40".

#### Status Monitoring Parameters (2041h/U41)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>01h</code></td>
<td><code>U41.00</code></td>
<td>MCU system status</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>02h</code></td>
<td><code>U41.01</code></td>
<td>MCU fault state</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>05h</code></td>
<td><code>U41.04</code></td>
<td>Encoder system status</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>06h</code></td>
<td><code>U41.05</code></td>
<td>Encoder fault state</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>07h</code></td>
<td><code>U41.06</code></td>
<td>Group number of abnormal parameter</td>
<td>-</td>
<td>Range: 0-255<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>08h</code></td>
<td><code>U41.07</code></td>
<td>Offset of the abnormal parameter within the parameter group</td>
<td>-</td>
<td>Range: 0-255<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Bh</code></td>
<td><code>U41.0A</code></td>
<td>Servo Status</td>
<td>-</td>
<td>Range: 0-3<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Ch</code></td>
<td><code>U41.0B</code></td>
<td>Servo running mode</td>
<td>-</td>
<td>Range: 0-9<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Dh</code></td>
<td><code>U41.0C</code></td>
<td>Servo running time</td>
<td>-</td>
<td>Range: 0-(2 32 -1)<br>Default: 0<br>Unit: 0.1s<br>Type: U32<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

#### Version Parameters (2042h/U42)

<table>
<thead>
<tr><th>Index</th><th>Parameter</th><th>Name</th><th>Options</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>01h</code></td>
<td><code>U42.00</code></td>
<td>ARM version</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: 0.01<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>03h</code></td>
<td><code>U42.02</code></td>
<td>Encoder version</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: 0.01<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>04h</code></td>
<td><code>U42.03</code></td>
<td>ARM-based machine</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: 0.01<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>06h</code></td>
<td><code>U42.05</code></td>
<td>Internal software version</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: 0.01<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Bh</code></td>
<td><code>U42.0A</code></td>
<td>EtherCAT CoE version</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: 0.01<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>0Ch</code></td>
<td><code>U42.0B</code></td>
<td>EtherCAT XML version</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: 0.01<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>11h</code></td>
<td><code>U42.10</code></td>
<td>Drive model</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>12h</code></td>
<td><code>U42.11</code></td>
<td>Motor model</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>13h</code></td>
<td><code>U42.12</code></td>
<td>Encoder model</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>14h</code></td>
<td><code>U42.13</code></td>
<td>Power supply unit model identification</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>15h</code></td>
<td><code>U42.14</code></td>
<td>Inverter model identification 1</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>16h</code></td>
<td><code>U42.15</code></td>
<td>Inverter model identification 2</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>17h</code></td>
<td><code>U42.16</code></td>
<td>Servo version</td>
<td>-</td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: Read only<br>Effective: Immediately</td>
</tr>
</tbody>
</table>

### 11.2.2 Common Parameters in Group `6000h`

Parameter group `6000h` contains supported sub-protocol DSP 402 related objects.

<table>
<thead>
<tr><th>Index</th><th>Subindex</th><th>Name</th><th>Access</th><th>PDO Mapping</th><th>Type</th><th>Details</th></tr>
</thead>
<tbody>
<tr>
<td><code>603Fh</code></td>
<td><code>0</code></td>
<td>Error code</td>
<td>RO</td>
<td>TPDO</td>
<td><code>U16</code></td>
<td>Range: -<br>Default: -<br>Unit: -<br>Type: U16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>6040h</code></td>
<td><code>0</code></td>
<td>Control word</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U16</code></td>
<td>Range: 0-65535<br>Default: -<br>Unit: -<br>Type: U16<br>Modification: 0<br>Effective: During - Immedi- operation ately</td>
</tr>
<tr>
<td><code>6041h</code></td>
<td><code>0</code></td>
<td>Status word</td>
<td>RO</td>
<td>TPDO</td>
<td><code>U16</code></td>
<td>Range: -<br>Default: -<br>Unit: -<br>Type: U16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>605Ah</code></td>
<td><code>0</code></td>
<td>Quick stop option code</td>
<td>RW</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 0-7<br>Default: 2<br>Unit: -<br>Type: I16<br>Modification: During operation<br>Effective: Upon re-power-on</td>
</tr>
<tr>
<td><code>605Ch</code></td>
<td><code>0</code></td>
<td>Stop mode upon servo-off</td>
<td>RW</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: -4-1<br>Default: 0<br>Unit: -<br>Type: I16<br>Modification: During operation<br>Effective: Upon re-power-on</td>
</tr>
<tr>
<td><code>605Dh</code></td>
<td><code>0</code></td>
<td>Halt option code</td>
<td>RW</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 1-3<br>Default: 1<br>Unit: -<br>Type: I16<br>Modification: During operation<br>Effective: Upon re-power-on</td>
</tr>
<tr>
<td><code>605Eh</code></td>
<td><code>0</code></td>
<td>Stop mode at No. 2 fault</td>
<td>RW</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 5-3<br>Default: 2<br>Unit: -<br>Type: I16<br>Modification: During operation<br>Effective: Upon re-power-on</td>
</tr>
<tr>
<td><code>6060h</code></td>
<td><code>0</code></td>
<td>Servo mode</td>
<td>RW</td>
<td>RPDO</td>
<td><code>I8</code></td>
<td>Range: 0-10<br>Default: 0<br>Unit: -<br>Type: I8<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6061h</code></td>
<td><code>0</code></td>
<td>Modes of operation display</td>
<td>RO</td>
<td>TPDO</td>
<td><code>I8</code></td>
<td>Range: -<br>Default: -<br>Unit: - -<br>Type: I8<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>6062h</code></td>
<td><code>0</code></td>
<td>Position reference</td>
<td>RO</td>
<td>TPDO</td>
<td><code>I32</code></td>
<td>Range: -<br>Default: -<br>Unit: Reference unit -<br>Type: I32<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>6063h</code></td>
<td><code>0</code></td>
<td>Position feedback</td>
<td>RO</td>
<td>TPDO</td>
<td><code>I32</code></td>
<td>Range: -<br>Default: -<br>Unit: Encoder unit -<br>Type: I32<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>6064h</code></td>
<td><code>0</code></td>
<td>Position feedback</td>
<td>RO</td>
<td>TPDO</td>
<td><code>I32</code></td>
<td>Range: -<br>Default: -<br>Unit: Reference unit -<br>Type: I32<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>6065h</code></td>
<td><code>0</code></td>
<td>Following error window</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U32</code></td>
<td>Range: 0-(2 32 -1)<br>Default: 0<br>Unit: Reference unit<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6066h</code></td>
<td><code>0</code></td>
<td>Following error time out</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U16</code></td>
<td>Range: 0-65535<br>Default: 0<br>Unit: ms<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6067h</code></td>
<td><code>0</code></td>
<td>Position reach threshold</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U32</code></td>
<td>Range: 0-(2 32 -1)<br>Default: 734<br>Unit: Reference unit<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6068h</code></td>
<td><code>0</code></td>
<td>Position window time</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U16</code></td>
<td>Range: 0-65535<br>Default: 0<br>Unit: ms<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>606Ch</code></td>
<td><code>0</code></td>
<td>Actual speed</td>
<td>RO</td>
<td>TPDO</td>
<td><code>I32</code></td>
<td>Range: -<br>Default: -<br>Unit: Reference unit/s -<br>Type: I32<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>606Dh</code></td>
<td><code>0</code></td>
<td>Speed reach threshold</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U16</code></td>
<td>Range: 0-65535<br>Default: 10<br>Unit: rpm<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>606Eh</code></td>
<td><code>0</code></td>
<td>Velocity window time</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U16</code></td>
<td>Range: 0-65535<br>Default: 0<br>Unit: ms<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>606Fh</code></td>
<td><code>0</code></td>
<td>Velocity threshold</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U16</code></td>
<td>Range: 0-65535<br>Default: 10<br>Unit: rpm<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6070h</code></td>
<td><code>0</code></td>
<td>Velocity threshold time</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U16</code></td>
<td>Range: 0-65535<br>Default: 0<br>Unit: ms<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6071h</code></td>
<td><code>0</code></td>
<td>Target torque</td>
<td>RW</td>
<td>RPDO</td>
<td><code>I16</code></td>
<td>Range: 4000- 4000<br>Default: 0<br>Unit: 0.1%<br>Type: I16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6072h</code></td>
<td><code>0</code></td>
<td>Max. torque</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U16</code></td>
<td>Range: 0-4000<br>Default: 3500<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6074h</code></td>
<td><code>0</code></td>
<td>Torque reference</td>
<td>RO</td>
<td>TPDO</td>
<td><code>I16</code></td>
<td>Range: 0<br>Default: -<br>Unit: 0.1% -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>6077h</code></td>
<td><code>0</code></td>
<td>Actual torque</td>
<td>RO</td>
<td>TPDO</td>
<td><code>I16</code></td>
<td>Range: 0<br>Default: -<br>Unit: 0.1% -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>607Ah</code></td>
<td><code>0</code></td>
<td>Target position</td>
<td>RW</td>
<td>RPDO</td>
<td><code>I32</code></td>
<td>Range: -2 31 - (2 31 -1)<br>Default: 0<br>Unit: Reference unit<br>Type: I32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>607Ch</code></td>
<td><code>0</code></td>
<td>Home offset</td>
<td>RW</td>
<td>RPDO</td>
<td><code>I32</code></td>
<td>Range: -2 31 - (2 31 -1)<br>Default: 0<br>Unit: Reference unit<br>Type: I32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>607Dh</code></td>
<td><code>0</code></td>
<td>Highest sub-index supported</td>
<td>RO</td>
<td>NO</td>
<td><code>U8</code></td>
<td>Range: -<br>Default: 0x02<br>Unit: -<br>Type: U8<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>607Dh</code></td>
<td><code>1</code></td>
<td>Minimum software position limit</td>
<td>RW</td>
<td>RPDO</td>
<td><code>I32</code></td>
<td>Range: -2 31 - (2 31 -1)<br>Default: -2 31<br>Unit: Reference unit<br>Type: I32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>607Dh</code></td>
<td><code>2</code></td>
<td>Maximum software position limit</td>
<td>RW</td>
<td>RPDO</td>
<td><code>I32</code></td>
<td>Range: -2 31 - (2 31 -1)<br>Default: 2 31 -1<br>Unit: Reference unit<br>Type: I32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>607Eh</code></td>
<td><code>0</code></td>
<td>Reference polarity</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U8</code></td>
<td>Range: 0-255<br>Default: 0<br>Unit: -<br>Type: U8<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>607Fh</code></td>
<td><code>0</code></td>
<td>Maximum speed</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U32</code></td>
<td>Range: 0-(2 32 -1)<br>Default: 104857600<br>Unit: Reference unit/s<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6081h</code></td>
<td><code>0</code></td>
<td>Profile operating speed</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U32</code></td>
<td>Range: 0-(2 32 -1)<br>Default: 1747627<br>Unit: User velocity<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6083h</code></td>
<td><code>0</code></td>
<td>Profile acceleration rate</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U32</code></td>
<td>Range: 0-(2 32 -1)<br>Default: 174762666<br>Unit: Reference unit/s 2<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6084h</code></td>
<td><code>0</code></td>
<td>Profile deceleration rate</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U32</code></td>
<td>Range: 0-(2 32 -1)<br>Default: 174762666<br>Unit: Reference unit/s 2<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6085h</code></td>
<td><code>0</code></td>
<td>Quick stop deceleration</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U32</code></td>
<td>Range: 0-(2 32 -1)<br>Default: 2 31 -1<br>Unit: Reference unit/s 2<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6086h</code></td>
<td><code>0</code></td>
<td>Motion profile type</td>
<td>RW</td>
<td>RPDO</td>
<td><code>I16</code></td>
<td>Range: 32767- 32767<br>Default: 0<br>Unit: -<br>Type: I16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6087h</code></td>
<td><code>0</code></td>
<td>Torque slope</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U32</code></td>
<td>Range: 0-(2 32 -1)<br>Default: 2 32 -1<br>Unit: 0.1%/s<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6091h</code></td>
<td><code>1</code></td>
<td>Motor revolutions</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U32</code></td>
<td>Range: 0-(2 32 -1)<br>Default: 1<br>Unit: -<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6091h</code></td>
<td><code>2</code></td>
<td>Shaft revolutions</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U32</code></td>
<td>Range: 1-(2 32 -1)<br>Default: 1<br>Unit: -<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6098h</code></td>
<td><code>0</code></td>
<td>Homing method</td>
<td>RW</td>
<td>RPDO</td>
<td><code>I8</code></td>
<td>Range: 2-35<br>Default: 1<br>Unit: -<br>Type: I8<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6098h</code></td>
<td><code>0</code></td>
<td>Highest sub-index supported</td>
<td>RO</td>
<td>NO</td>
<td><code>U8</code></td>
<td>Range: 2<br>Default: -<br>Unit: - -<br>Type: U8<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>6099h</code></td>
<td><code>1</code></td>
<td>Speed during search for switch</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U32</code></td>
<td>Range: 0-(2 32 -1)<br>Default: 1747627<br>Unit: Reference unit/s<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>6099h</code></td>
<td><code>2</code></td>
<td>Speed during search for zero</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U32</code></td>
<td>Range: 174763<br>Default: -<br>Unit: Reference 10-(2 32 -1) unit/s<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>609Ah</code></td>
<td><code>0</code></td>
<td>Homing acceleration</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U32</code></td>
<td>Range: 0-(2 32 -1)<br>Default: 1747626667<br>Unit: Reference unit/s 2<br>Type: U32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>60B0h</code></td>
<td><code>0</code></td>
<td>Position offset</td>
<td>RW</td>
<td>RPDO</td>
<td><code>I32</code></td>
<td>Range: -2 31 - (2 31 -1)<br>Default: 0<br>Unit: Reference unit<br>Type: I32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>60B1h</code></td>
<td><code>0</code></td>
<td>Speed deviation</td>
<td>RW</td>
<td>RPDO</td>
<td><code>I32</code></td>
<td>Range: -2 31 - (2 31 -1)<br>Default: 0<br>Unit: Reference unit/s<br>Type: I32<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>60B2h</code></td>
<td><code>0</code></td>
<td>Torque offset</td>
<td>RW</td>
<td>RPDO</td>
<td><code>I16</code></td>
<td>Range: 4000- 4000<br>Default: 0<br>Unit: 0.10%<br>Type: I16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>60B8h</code></td>
<td><code>0</code></td>
<td>Touch probe function</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U16</code></td>
<td>Range: 0-65535<br>Default: 0<br>Unit: -<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>60B9h</code></td>
<td><code>0</code></td>
<td>Touch probe status</td>
<td>RW</td>
<td>TPDO</td>
<td><code>U16</code></td>
<td>Range: 0<br>Default: -<br>Unit: - -<br>Type: U16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60BAh</code></td>
<td><code>0</code></td>
<td>Touch probe 1 positive edge</td>
<td>RW</td>
<td>TPDO</td>
<td><code>I32</code></td>
<td>Range: 0<br>Default: -<br>Unit: Reference unit -<br>Type: I32<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60BBh</code></td>
<td><code>0</code></td>
<td>Touch probe 1 negative edge</td>
<td>RW</td>
<td>TPDO</td>
<td><code>I32</code></td>
<td>Range: 0<br>Default: -<br>Unit: Reference unit -<br>Type: I32<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60BCh</code></td>
<td><code>0</code></td>
<td>Touch probe 2 positive edge</td>
<td>RW</td>
<td>TPDO</td>
<td><code>I32</code></td>
<td>Range: 0<br>Default: -<br>Unit: Reference unit -<br>Type: I32<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60BDh</code></td>
<td><code>0</code></td>
<td>Touch probe 2 negative edge</td>
<td>RW</td>
<td>TPDO</td>
<td><code>I32</code></td>
<td>Range: 0<br>Default: -<br>Unit: Reference unit -<br>Type: I32<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60C5h</code></td>
<td><code>0</code></td>
<td>Max. acceleration</td>
<td>RW</td>
<td>RPDO</td>
<td><code>-</code></td>
<td>Range: -<br>Default: -<br>Unit: -<br>Type: -<br>Modification: -<br>Effective: 2 31 -1</td>
</tr>
<tr>
<td><code>60C6h</code></td>
<td><code>0</code></td>
<td>Max. deceleration</td>
<td>RW</td>
<td>RPDO</td>
<td><code>-</code></td>
<td>Range: -<br>Default: -<br>Unit: -<br>Type: -<br>Modification: -<br>Effective: 2 31 -1</td>
</tr>
<tr>
<td><code>60D5h</code></td>
<td><code>0</code></td>
<td>Touch probe 1 positive edge counter</td>
<td>RO</td>
<td>TPDO</td>
<td><code>U16</code></td>
<td>Range: 0<br>Default: -<br>Unit: - -<br>Type: U16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60D6h</code></td>
<td><code>0</code></td>
<td>Touch probe 1 negative edge counter</td>
<td>RO</td>
<td>TPDO</td>
<td><code>U16</code></td>
<td>Range: 0<br>Default: -<br>Unit: - -<br>Type: U16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60D7h</code></td>
<td><code>0</code></td>
<td>Touch probe 2 positive edge counter</td>
<td>RO</td>
<td>TPDO</td>
<td><code>U16</code></td>
<td>Range: 0<br>Default: -<br>Unit: - -<br>Type: U16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60D8h</code></td>
<td><code>0</code></td>
<td>Touch probe 2 negative edge counter</td>
<td>RO</td>
<td>TPDO</td>
<td><code>U16</code></td>
<td>Range: 0<br>Default: -<br>Unit: - -<br>Type: U16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E0h</code></td>
<td><code>0</code></td>
<td>Positive torque limit</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U16</code></td>
<td>Range: 0-4000<br>Default: 3500<br>Unit: 0.1%<br>Type: U16<br>Modification: During<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>60E1h</code></td>
<td><code>0</code></td>
<td>Negative torque limit</td>
<td>RW</td>
<td>RPDO</td>
<td><code>U16</code></td>
<td>Range: 0-4000<br>Default: 3500<br>Unit: 0.1%<br>Type: U16<br>Modification: During operation<br>Effective: Immediately</td>
</tr>
<tr>
<td><code>60E1h</code></td>
<td><code>0</code></td>
<td>Highest sub-index supported</td>
<td>RO</td>
<td>NO</td>
<td><code>U8</code></td>
<td>Range: 22<br>Default: -<br>Unit: - -<br>Type: U8<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E1h</code></td>
<td><code>1</code></td>
<td>1st supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 1<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E1h</code></td>
<td><code>2</code></td>
<td>2nd supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 2<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>3</code></td>
<td>3rd supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 3<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>4</code></td>
<td>4th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 4<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>5</code></td>
<td>5th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 5<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>6</code></td>
<td>6th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 6<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>7</code></td>
<td>7th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 7<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>8</code></td>
<td>8th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 8<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>9</code></td>
<td>9th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 9<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>A</code></td>
<td>10th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 10<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>B</code></td>
<td>11th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 11<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>C</code></td>
<td>12th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 12<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>D</code></td>
<td>13th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 13<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>E</code></td>
<td>14th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 14<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>F</code></td>
<td>15th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 17<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>10</code></td>
<td>16th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 18<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>11</code></td>
<td>17th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 19<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>12</code></td>
<td>18th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 20<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>13</code></td>
<td>19th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 21<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>14</code></td>
<td>20th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 22<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>15</code></td>
<td>21th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 23<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>16</code></td>
<td>22th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 24<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>17</code></td>
<td>23th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 25<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>18</code></td>
<td>24th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 26<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>19</code></td>
<td>25th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 27<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>1A</code></td>
<td>26th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 28<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>1B</code></td>
<td>27th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 29<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>1C</code></td>
<td>28th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 30<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>1D</code></td>
<td>29th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 33<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>1E</code></td>
<td>30th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 34<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E3h</code></td>
<td><code>1F</code></td>
<td>31th supported homing method</td>
<td>RO</td>
<td>NO</td>
<td><code>I16</code></td>
<td>Range: 35<br>Default: -<br>Unit: - -<br>Type: I16<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60E6h</code></td>
<td><code>0</code></td>
<td>-</td>
<td>RW</td>
<td>NO</td>
<td><code>U16</code></td>
<td>Range: -<br>Default: -<br>Unit: -<br>Type: U16<br>Modification: 0-1<br>Effective: 0</td>
</tr>
<tr>
<td><code>60F4h</code></td>
<td><code>0</code></td>
<td>Position deviation</td>
<td>RO</td>
<td>TPDO</td>
<td><code>I32</code></td>
<td>Range: -<br>Default: -<br>Unit: Reference unit -<br>Type: I32<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60FCh</code></td>
<td><code>0</code></td>
<td>Position reference</td>
<td>RO</td>
<td>TPDO</td>
<td><code>I32</code></td>
<td>Range: -<br>Default: -<br>Unit: Encoder unit -<br>Type: I32<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60FDh</code></td>
<td><code>0</code></td>
<td>DI status</td>
<td>RO</td>
<td>TPDO</td>
<td><code>U32</code></td>
<td>Range: -<br>Default: -<br>Unit: - -<br>Type: U32<br>Modification: -<br>Effective: -</td>
</tr>
<tr>
<td><code>60FFh</code></td>
<td><code>0</code></td>
<td>Target velocity</td>
<td>RW</td>
<td>RPDO</td>
<td><code>I32</code></td>
<td>Range: (-2 31 -1)- (2 31 -1)<br>Default: -<br>Unit: Reference unit/s<br>Type: I32<br>Modification: 0<br>Effective: During - Immedi- operation ately</td>
</tr>
<tr>
<td><code>6502h</code></td>
<td><code>0</code></td>
<td>Supported drive modes</td>
<td>RO</td>
<td>NO</td>
<td><code>U32</code></td>
<td>Range: -<br>Default: 941<br>Unit: -<br>Type: U32<br>Modification: -<br>Effective: -</td>
</tr>
</tbody>
</table>

## 11.3 Description of Parameters

### 11.3.1 Group C00

```text
C00.05: Stiffness level
y   Defines the stiffness level of the servo system. The higher the stiffness level, the stronger the gains and
the quicker the response will be. But an excessively high stiffness level will cause vibration. The setpoint 0
indicates the lowest stiffness and 41 indicates the highest stiffness.

C00.06: Load inertia ratio
y   Defines the mechanical load inertia ratio relative to the motor moment of inertia.
y   When C00.06 is set to 0, it indicates the motor carries no load; if it is set to 1.00, it indicates the
mechanical load inertia is the same as the motor moment of inertia.
y   When the value of C00.06 is equal to the actual inertia ratio, the value of speed loop gain can represent
the maximum follow-up frequency of actual speed loop.
```

### 11.3.2 Group C01

```text
C01.00: 1st position loop gain
y   Defines the proportional gain of the position loop.
y   This parameter determines the responsiveness of the position loop. A high setpoint shortens the
positioning time. Note that an excessively high setpoint may cause vibration.
y   The 1st gain set includes C01.00, C01.01, C01.02, and C01.03.

C01.01: 1st speed loop gain
y   Defines the speed loop proportional gain.
y   This parameter determines the responsiveness of the speed loop. The higher the setpoint, the faster the
speed loop response is. Note that an excessively high setpoint may cause vibration.
y   In the position control mode, the position loop gain must be increased together with the speed loop gain.

C01.02: 1st speed loop integral time
y   Defines the speed loop integral time constant.
y   The lower the setpoint, the better the integral action, and the quicker will the deviation value be close to 0.
y   There is no integral action when C01.02 is set to 512.00 ms

C01.08: 2nd position loop gain
y   Defines the 2nd gain of the position loop.
y   The 2nd gain set includes C01.08, C01.09, C01.0A, and C01.0B.
y   For details about gain switchover, see section 7.5 "Gain Switchover".

C01.0B: 2nd torque reference filter cutoff frequency
y   Defines the torque reference filter time constant.
y   Low-pass filtering of torque references helps to smoothen torque references and reduce vibration.
y   Pay attention to the responsiveness during setting as an excessively high setpoint lowers down the
responsiveness.

NOTICE
y The servo drive offers two low-pass filters for torque references. By default, the 1st filter is
used.
y Gain switchover can be used in the position or speed control mode. Once certain
conditions are satisfied, the servo drive can switch to filter 2.

C01.11: Cutoff frequency of speed feedback low-pass filter
y     Defines the cutoff frequency for first-order low-pass filtering on the speed feedback.
y     The lower the setpoint, the weaker the speed feedback fluctuation, and the longer the feedback delay will
be.
y     Setting this parameter to 8000 Hz negates the filtering effect.

C01.12: Speed feedback overlapping average filter time constant
y     Defines the moving average filtering times for speed feedback.
y     The higher the setpoint, the weaker the speed feedback fluctuation, but the longer the feedback delay will
be.
y     When C01.12 is set to a value higher than 0, C01.11 (Cutoff frequency of speed feedback low-pass filter)
is invalid.

C01.13: Speed feedforward source
y     Defines the source of the speed loop feedforward signal.
y     In the position control mode, the speed feedforward control can improve the position reference
responsiveness.

Speed Feedforward
Setpoint                                                           Remarks
Source
0          No feedforward                                          -
The speed corresponding to the position reference (encoder unit)
1         Internal reference
is defined as the speed feedforward source.
Model tracking control can improve the responsiveness and
shorten the positioning time.
It is only available in the position control mode.
2           Model tracking
It must be used with C02.00. When C02.00 is set to 1, the speed
feedforward is sourced from the speed feedforward output of
model tracking.
In CSP, 60B1h is used as the source of the external speed
feedforward signal.
5          Communication
Bit 6 of 607Eh can specify the polarity of the speed feedforward
signal (60B1h).

C01.14: Speed feedforward percentage
y   In the position control mode, speed feedforward is the value of C01.14 multiplied by the speed
feedforward signal, which is part of the speed reference. Increasing the setpoint improves the
responsiveness to position references and reduces the position deviation during operation at a constant
speed.
y   Set C01.15 to a fixed value first, and then gradually increase the value of C01.14 from 0 to a certain
setpoint at which speed feedforward achieves the desired effect.
y   Adjust C01.15 and C01.14 repeatedly until a balanced setting is achieved.
y   For the speed feedforward function and speed feedforward signal selection, see C01.13 (Speed
feedforward source selection).

C01.15: Speed feedforward filter cutoff frequency
y   Defines the speed feedforward smoothing filter time.

C01.16: Torque feedforward source
y   Defines whether to enable the internal torque feedforward function in a non-torque control mode.
y   The torque feedforward function can improve the torque reference responsiveness and reduce the
position deviation during operation at constant acceleration/deceleration rate.

Torque Feedforward
Setpoint                                                          Remarks
Source
0           No feedforward                                          -
The torque feedforward signal source is the speed reference.
In the position control mode, the speed reference is output from the
1          Internal reference
position controller. In the speed control mode, the speed reference
is output from the user speed reference.
It must be used with C02.00. When C02.00 is set to 1, the torque
2           Model tracking       feedforward is sourced from the torque feedforward output of
model tracking.
In CSP, 60B1h is used as the source of the external torque
feedforward signal.
5           Communication
Bit 6 of 607Eh can specify the polarity of the torque feedforward
signal (60B1h).

y   Torque feedforward parameters include C01.17 (Torque feedforward percentage) and C01.18 (Torque
feedforward cutoff frequency).
y   In a non-torque control mode, the control block diagram of torque feedforward is as follows:

Torque feedforward
control

Speed reference             Speed loop                Current loop
Motor
control                   control
−
Speed feedback

Speed detection                           Encoder

Figure 11-1 Torque feedforward control

C01.17: Torque feedforward percentage
y   In control modes other than torque control, torque feedforward is the product of torque feedforward
signal multiplied by C01.17 and is part of the torque reference. Increasing the setpoint improves the
responsiveness to variable speed references and position references and reduces the position deviation
during operation at a constant speed.

C01.18: Torque feedforward filter cutoff frequency
y   Defines the filter time constant of torque feedforward.

C01.1B: PDFF control coefficient
y   Defines the control method of the speed loop.
y   When the setpoint is 100.0, PI control (default control mode of the speed loop) is applied to the speed
loop, which features fast dynamic response.
y   When the setpoint is 0.0, speed loop integral action is enhanced, which filters out low-frequency
interference but also slows down the dynamic response.
y   C01.1B can be used to keep a good responsiveness of the speed loop, with the anti-interference capacity
in low-frequency bands improved and the speed feedback overshoot not increased.

C01.30: Adaptive notch mode
Setpoint:
0: Adaptive notch not updated
1: One adaptive notch activated (3rd notch)
2: Two adaptive notches activated (3rd and 4th notches)
3: Adaptive notch cleared, values of the 3rd and 4th notches restored to default settings
4: Resonance point tested only, displayed in C01.31, C01.32, and C01.33
Description:
y   Defines the operation mode of the adaptive notch.

C01.38: Gain switchover mode

Gain Switchover
Setpoint                                                                  Remarks
Condition
0      Fixed to the 1st gain set   The 1st gain set applies.
Gains are switched through bit 26 of 60FE.
Bit 26 signal inactive: 1st gain set
1      DI switchover               Bit 26 signal active: 2nd gain set
If the bit 26 signal cannot be allocated to a DI terminal, the 1st gain set
applies.
Gains are switched through bit 26 of 60FE.
Bit 26 signal inactive: 1st gain set
Bit 26 signal active: 2nd gain set (The 2nd speed loop integral (C01.0A) is
2      DI P-PI switchover
forced to be 512 ms.)
If the bit 26 signal cannot be allocated to a DI terminal, the 1st gain set
applies.
When the absolute value of the torque reference exceeds (threshold + loop
width, %) in the last 1st gain set, the drive switches to the 2nd gain set.
3      Torque reference            When the absolute value of the torque reference is less than (threshold –
loop width, %) and this status lasts within the delay (C01.39) in the last 2nd
gain set, the drive returns to the 1st gain set.
When the absolute value of the speed reference exceeds (threshold + loop
width, rpm) in the last 1st gain set, the drive switches to the 2nd gain set.
4      Speed reference             When the absolute value of the speed reference is less than (threshold –
loop width, rpm) and this status lasts within the delay (C01.39) in the last
2nd gain set, the drive returns to the 1st gain set.
It is valid only in the position control mode.
When the absolute value of the actual speed exceeds (threshold + loop
width, rpm) in the last 1st gain set, the drive switches to the 2nd gain set.
5      Speed feedback              When the absolute value of the actual speed is less than (threshold – loop
width, rpm) and this status lasts within the delay (C01.39) in the last 2nd
gain set, the drive returns to the 1st gain set.
The 1st gain set applies when the drive is not in the position control mode.
It is valid only in non-speed control modes.
When the absolute value of the change rate in the speed reference
exceeds (threshold + loop width, 10 rpm/s) in the last 1st gain set, the
Speed reference change      drive switches to the 2nd gain set.
rate                        When the absolute value of the change rate in the speed reference is less
than (threshold – loop width, 10 rpm/s) and this status lasts within the
delay (C01.39) in the last 2nd gain set, the drive returns to the 1st gain set.
The 1st gain set applies in the speed control mode.

Gain Switchover
Setpoint                                                                Remarks
Condition
When the absolute value of the position deviation exceeds (threshold +
loop width, encoder unit) in the last 1st gain set, the drive switches to the
2nd gain set.
7      Position deviation         When the absolute value of the position deviation is less than (threshold +
loop width, encoder unit) and this status lasts within the delay (C01.39) in
the last 2nd gain set, the drive returns to the 1st gain set.
The 1st gain set applies when the drive is not in the position control mode.
It is valid only in the position control mode.
When the position reference is not 0 in the last 1st gain set, the drive
switches to the 2nd gain set.
8      Position reference
When the position reference is 0 and this status lasts within the delay
(C01.39) in the last 2nd gain set, the drive returns to the 1st gain set.
The 1st gain set applies when the drive is not in the position control mode.

C01.39: Gain switchover time
Defines the duration when the drive switches from the 2nd gain set to the 1st gain set.

C01.3A: Gain switchover threshold
y   Defines the gain switchover threshold.
y   Gain switchover is affected by both the threshold and the loop width, as defined by C01.38. The unit of
gain switchover threshold varies with the switchover condition.
y   Set C01.3A to a value greater than or equal to C01.3B. If C01.3A is set to a value less than C01.3B, the
servo drive sets C01.3A to the same value as C01.3B.

C01.3B: Gain switchover loop width
y   Defines the gain switchover loop width.
y   Gain switchover is affected by both the threshold and the loop width. The unit of gain switchover
threshold varies with the switchover condition.
y   Set C01.3A to a value greater than or equal to C01.3B. If C01.3A is set to a value less than C01.3B, the
servo drive sets C01.3A to the same value as C01.3B.

C01.40: Frequency of the 1st notch
y   Defines the center frequency of the notch, which is the mechanical resonance frequency.
y   In the torque control mode, setting the notch frequency to 8000 Hz deactivates the notch function.

C01.41: Width level of the 1st notch
y   Defines the width level of the notch. Use the default value in general cases.
y   Width level is the ratio of the notch width to the notch center frequency.

C01.42: Depth level of the 1st notch
y     Defines the depth level of the notch.
y     The depth level of the notch is the ratio between the input to the output at the notch center frequency.
y     The higher the setpoint, the lower the notch depth and the weaker the mechanical resonance
suppression will be. Note that an excessively high setpoint may cause system instability.
y     For the use of notch, see 7.14 "Vibration Suppression".

C01.45: Depth level of the 2nd notch
y     Description of the 2nd notch parameters is the same as that of the 1st notch parameters.

NOTICE
y The 1st and 2nd notches can be set manually or configured as adaptive notches (C01.30
= 1 or 2). In this case, the parameters are automatically set by the drive, while the other
three notches can be set manually.
```

### 11.3.3 Group C03

```text
C03.21: Speed reference
y     It is the speed reference in the local speed mode, which is invalid in EtherCAT mode.

C03.22: Acceleration rate
y     It is the acceleration ramp time of the speed reference in the local speed mode, which is invalid in
EtherCAT mode.

C03.24: Deceleration rate
y     It is the deceleration ramp time of the speed reference in the local speed mode, which is invalid in
EtherCAT mode.

C03.27: Internal positive speed limit
y     It is the PL of the speed reference in the local speed mode, which is invalid in EtherCAT mode.

C03.28: Internal negative speed limit
y     It is the NL of the speed reference in the local speed mode, which is invalid in EtherCAT mode.

C03.43: Internal positive torque limit
y     It is valid only in the local torque mode. For torque limit in EtherCAT mode, use 60E0h/60E1h/6072h.
Use the torque limit with caution as an excessively low limit value may lead to insufficient motor torque
output.
y     If the setpoint exceeds the maximum torque of the servo motor and servo drive, the actual torque is
limited to the maximum torque of the servo motor and servo drive.

C03.44: Internal negative torque limit
y     It is valid only in the local torque mode. For torque limit in EtherCAT mode, use 60E0h/60E1h/6072h.

Use the torque limit with caution as an excessively low limit value may lead to insufficient motor torque
output.
y   If the setpoint exceeds the maximum torque of the servo motor and servo drive, the actual torque is
limited to the maximum torque of the servo motor and servo drive.

C03.47: Positive speed limit in torque mode
y   It is valid only in the local torque mode. Use 607F for the speed limit in the EtherCAT, CST, and PT modes.

C03.48: Negative speed limit in torque mode
y   It is valid only in the local torque mode. Use 607F for the speed limit in the EtherCAT, CST, and PT modes.

C03.4B: Invalid value for torque reached
y   The torque reached function is used to judge whether the actual torque reference reaches the range of
the valid value for torque reached. If yes, the servo drive outputs the corresponding flag (bit 10 of the
status word) to the host controller.
A: Actual torque reference (U40.02)
B: Base value for torque reach (C03.49)
C: Valid value for torque reach (C03.4A)
D: Invalid value for torque reach (C03.4B)
C and D are offsets on the basis of B.
The torque reach signal is activated only when the actual torque reference meets the condition: |A| ≥ B +
C. Otherwise, the torque reach signal remains inactive.
The torque reach signal is deactivated only when the actual torque reference meets the condition: |A| < B +
D.

Actual torque

B+C
A
B+D

B
Time
﹣B

﹣(B+D)

﹣(B+C)

Torque reached output
6041.bit10       OFF     ON               OFF              ON          OFF
```

### 11.3.4 Group C05

```text
C05.0D: Maximum downtime
y   Defines the maximum time taken by the motor in decelerating from 6000 RPM to 0 RPM when the stop
mode is set to "Ramp to stop as defined by 6084h/609Ah (HM)" or "Ramp to stop as defined by 6085h".
```

### 11.3.5 Group C06

```text
C06.04: Input phase loss detection
Servo drives support three-phase 380 V power supplies. When voltage fluctuation or phase loss occurs on the
power supply, power input phase loss protection will be triggered by the servo drive based on the setting of
C06.04.
y   C06.04 = 0: The servo drive reports Er81.0 (Phase loss fault) when the servo drive is set to 3 kW.
y   C06.04 = 1: The servo drive does not report Er81.0 (Phase loss fault) when the servo drive is set to 3 kW,
with deration of 80%.

C06.11: Motor overload protection threshold
y   Determines the motor overload duration before Er41.0 (Motor overload) is reported.
y   You can change the setpoint to advance or delay the time when overload protection is triggered based
on the motor temperature. The setpoint 50% indicates the time is cut by half; 150% indicates the time is
prolonged by 50%.
y   Set this parameter based on the actual temperature of the motor.

C06.20: Protection from out of control
y   Sets whether to enable the runaway protection function.
```

### 11.3.6 Group C0A

```text
C0A.09: Commissioning software communication baud rate
y   Defines the communication rate between the servo drive and the host controller.
y   The baud rate set in the servo drive must be the same as that in the host controller. Otherwise,
communication will fail.

C0A.0A: Commissioning software communication format
y   Defines the data check mode between the servo drive and the host controller during communication.
y   The data format of the servo drive and the host controller must be the same; otherwise, the
communication fails.
```

### 11.3.7 Group C13

```text
C13.00: EtherCAT slave name
y   Indicates the station number assigned to the slave by the master during EtherCAT communication.

C13.01: EtherCAT slave alias
y   Indicates the station number assigned to the slave EtherCAT communication since the master cannot
automatically assign station numbers.
y   C13.01 = 0: The master assigns the station numbers by default. C13.01 ≠ 0: The set station number
applies by default, with the one assigned by master deactivated.

C13.05: EtherCAT synchronization mode setting
y   Defines the synchronization work mode:

Setpoint        Function                                          Remarks
Manufacturer
0                                                    Manufacturer Function
Function
Applies to the scenarios where the synchronization performance
1           Sync 1
indicator of the host controller jitters for 1 us.
Applies to the scenarios where the synchronization performance
2           Sync 2
indicator of the host controller jitters for more than 1 us.

y   In the work mode, the synchronization cycle must be an integer multiple of 125 μs. Otherwise, the serve
drive will report Er74.0 (EtherCAT synchronization cycle setting is incorrect.)

C13.06: EtherCAT synchronization error threshold
y   Defines the permissible jitter range of synchronization signals when the servo drive works in synchronization
mode 1 (C13.05 = 1).

C13.08: EtherCAT enhanced link selection
y   When a redundant loop network is used, the EtherCAT Enhanced Link Check function must be enabled
(C13.08 = 1), which will take effect upon next power-on of the servo drive.
y   When a loop network is used, both C13.08 and C13.19 need to be set to 1.
```

### 11.3.8 Group R21

```text
R21.00: Drive model
Setpoint:
2: 200EC             3: 400EC             5: 750EC           6: 1000EC

Description:
y   Sets the SN of the servo drive. The following table lists the servo drive SNs.

Servo Drive
Setpoint                                                      Remarks
SN

2       200EC        The rated drive power is 0.2 kW. The main circuit inputs single-phase 220 V.

3       400EC        The rated drive power is 0.4 kW. The main circuit inputs single-phase 220 V.

5       750EC        The rated drive power is 0.75 kW. The main circuit inputs single-phase 220 V.

The rated drive power is 1.0 kW. The main circuit inputs single-phase or three-
phase 220 V.
6      1000EC
(The main circuit of the servo drive supports single-phase 220 V power supplies
without derating.)

If the voltage input to the main circuit of the servo drive does not comply with the preceding
specifications, a fault or damage occurs.
```

### 11.3.9 Group F30

```text
F30.03 JOG acceleration/deceleration time
y   Acceleration/Deceleration time setpoint for jog in velocity mode, which can be enabled through
parameter F30.00 on the panel or through the software

F30.10: Inertia auto-tuning selection
y   Used to enable offline inertia auto-tuning through the keypad.
y   In the parameter display mode, switch to F30.10 and press the SET key to enable offline inertia auto-
tuning. For details about offline inertia auto-tuning, see section 7.2 "Inertia Auto-tuning".
```

### 11.3.10 Group F31

```text
F31.00: Fault reset
y   Defines whether to enable fault reset.

Setpoint       Function                                           Remarks
0        No operation                                             -
When a No.1 or No.2 resettable fault occurs, you can enable the fault reset
function in the non-operational state after rectifying the fault cause and
1           Enable
stopping the keypad from displaying the fault.
When a No.3 warning occurs, you can enable the fault reset function directly.

y   For fault classification, see section 10.1.3 "List of faults and alarms".
y   The fault reset function, once enabled, stops the keypad from displaying the fault only. It does not
activate modifications made on parameters.
y   This function is not applicable to non-resettable faults. Use this function with caution in cases where the
fault causes are not rectified.

F31.01 Software reset
y   Defines whether to enable fault reset.

Setpoint       Function                                           Remarks
0        No operation                                             -
Programs in the drive are reset automatically (similar to the program reset
1           Enable        upon power-on) after the software reset function is enabled, without the
need for a power cycle.

y   Software reset conditions: The servo drive is disabled, and there is no non-resettable fault such as No.1
fault.

F31.10: Encoder data reset
y   The absolute position saved by the encoder changes abruptly after multi-turn data reset. In this case,
perform mechanical homing.
```

### 11.3.11 Group U40

```text
U40.00: Speed reference
y   Indicates the present speed reference (accurate to 1 RPM) of the drive in the position and speed control
modes.

U40.01: Speed feedback
y   Indicates the actual motor speed after round-off, which is accurate to 1 rpm.
y   This parameter is a 32-bit integer, which is displayed as a decimal on the keypad.

U40.02: Actual torque reference
y   Indicates the present torque reference (accurate to 0.1%). The value 100.0% corresponds to the rated
torque of the motor.

U40.04: DI status
y   Indicates the level status of five DIs without filtering.
y   Upper LED segments ON: high level (indicated by "1")
Lower LED segments ON: low level (indicated by "0") In cases where DI1 is low level and DI2 to DI5 are high
level, the corresponding binary value is 11110, and the value of U40.04 read in the software tool is 30.
y   The keypad displays as follows:
DI4 DI2
DI5 DI3 DI1

H H H H L          H: High
1 1 1 1 0          L: Low

U40.05: DO state
y   Indicates the level status of three DOs without filtering.
y   Upper LED segments ON: high level (indicated by "1")
Lower LED segments ON: low level (indicated by "0") In cases where DO1 is low level and DO2 to DO3 are
high level, the corresponding binary value is 110, and the value of U40.05 read in the software tool is 6.
y   The keypad displays as follows:

DO2
DO3 DO1

H H L          H: High
1 1 0          L: Low

U40.06: Bus voltage
y   Indicates the DC bus voltage of the main circuit input voltage after rectification, which is accurate to 0.1 V.

U40.07: Average load ratio
y   Indicates the percentage of the average load torque to the rated torque of the motor, which is accurate
to 0.1%. The value 100.0% corresponds to the rated torque of the motor.

U40.08: Electrical angle
y   Indicates the present electrical angle of the motor, which is accurate to 0.1°.
y   The electrical angle variation range is ±360.0° when the motor rotates.
y   If the motor has four pairs of poles, each revolution generates four rounds of angle changes from 0° to
359.9°.
y   Similarly, if the motor has five pairs of poles, each revolution generates five rounds of angle changes from
0° to 359.9°.

U40.09: Mechanical angle
y   Indicates present mechanical angle (encoder unit) of the motor. The value 0 indicates that the mechanical
angle is 0°.

U40.0C: RMS value of phase current
y   Indicates the RMS value of the phase current of the servo motor, which is accurate to 0.1 A.

U40.10: Position deviation counter
y   Counts the position pulses fed back by the encoder in any control mode.
y   This parameter is a 32-bit integer, which is displayed as a decimal on the keypad.

U40.30: Heatsink temperature
y   Indicates the temperature of the module inside the servo drive, which can be used as a reference for
estimating the actual temperature of the servo drive.
```

### 11.3.12 Group 6000

```text
603Fh: Fault code
y   When a fault described in the DSP402 profile occurs on the drive, 603Fh is as described in DSP402.
y   When a fault specified by the user occurs on the servo drive, 603Fh is 0xFF00. The value of 603Fh is in
hexadecimal.

y   In addition, the object dictionary 203Fh displays auxiliary bytes of fault code in hexadecimal.
y   203Fh is a UInt32 value, in which the high 16 bits indicate the internal fault code of the manufacturer,
and the low 16 bits indicate the external fault code of the manufacturer.

605Ah: Quick stop option code
0: Coast to stop, keeping de-energized status
1: Ramp to stop as defined by 6084h/609Ah (HM), keeping de-energized status
2: Ramp to stop as defined by 6085h, keeping de-energized status
3: Stop at emergency stop torque, keeping de-energized status
5: Ramp to stop as defined by 6084h/609Ah (HM), keeping position lock status
6: Ramp to stop as defined by 6085h, keeping position lock status
7: Stop at emergency stop torque, keeping position lock status

605Ch: Stop mode at S-ON OFF
–4: Ramp to stop as defined by 6085h, keeping dynamic braking status
–3: Stop at zero speed, keeping dynamic braking status
–2: Ramp to stop as defined by 6084h/609Ah (HM), keeping dynamic braking status
–1: Dynamic braking stop, keeping dynamic braking status
0: Coast to stop, keeping de-energized status
1: Ramp to stop as defined by 6084h/609Ah (HM), keeping de-energized status

605Dh: Stop option code
1: Ramp to stop as defined by 6084h/609Ah (HM), keeping position lock status
2: Ramp to stop as defined by 6085h, keeping position lock status
3: Stop at emergency stop torque, keeping position lock status

605Eh: Stop mode at No. 2 fault
-5: Stop at zero speed, keeping dynamic braking status
–4: Stop at emergency stop torque, keeping dynamic braking status
-3: Ramp to stop as defined by 6085h, keeping dynamic braking status
–2: Ramp to stop as defined by 6084h/609Ah (HM), keeping dynamic braking status
–1: Dynamic braking stop, keeping dynamic braking status
0: Coast to stop, keeping de-energized status
1: Ramp to stop as defined by 6084h/609Ah (HM), keeping de-energized status
2: Ramp to stop as defined by 6085h, keeping de-energized status
3: Stop at emergency stop torque, keeping de-energized status
4: Dynamic braking stop, keeping de-energized status

6060h: Modes of operation
Setpoint:
1: Profile position (PP) mode
3: Profile velocity (PV) mode

4: Profile torque (PT) mode
6: Homing mode (HM)
8: Cyclic synchronous position (CSP) mode
9: Cyclic synchronous velocity (CSV) mode
10: Cyclic synchronous torque (CST) mode
Others: N/A

Description:
y   If an unsupported operation mode is selected through an SDO, an SDO error will be returned.
y   If an unsupported operation mode is selected through a PDO, the change of the operation mode will be
invalid.

6061h: Modes of operation display
1: PP mode
3: PV mode
4: PT mode
6: HM
8: CSP mode
9: CSV mode
10: CST mode

6064h: Position actual value
y   Position actual value in user-defined unit (6064h) x Gear ratio (6091h) = Position actual value in encoder
unit (6063h)

6065h: Following error window
y   When the difference value between position reference (6062h) and position actual value (6064h) keeps
exceeding ±6065h after the time defined by 6066h elapses, Er47.0 (Position deviation too large) occurs.

6066h: Following error time out
y   Defines the time lapse to trigger excessive position deviation, used with 6065h.

6067h: Max. profile velocity
y   Defines the threshold for position reach.
y   If the difference between the position reference value (6062h) and the position actual value (6064h) is
within ±6067h and the time reaches 6068h, the position is reached. In this case, bit 10 of 6041h is set to
1 in PP mode.
y   This flag bit is meaningful only when the S-ON signal is active in PP mode.

6068h: Position window time
y   Defines the window time for position reach, which must be used together with 6067h.

606Dh: Velocity window
y   Defines the threshold for speed reach.

y   If the difference value between the target speed (60FFh) and the actual speed (606Ch) is within ±606Dh
and the time reaches 606Eh, the speed is reached and bit 10 of the status word 6041h is set to 1 in the
PV mode.
y   This flag bit is meaningful only when the S-ON signal is active in PV mode.

606Fh: Velocity threshold
y   Defines the threshold for determining whether the user velocity is 0.
y   When the velocity actual value (606Ch) is within ±606Fh and the time reaches the value set by 6070h, the
user velocity is 0. When either condition is not met, the user velocity is not 0.
y   This flag bit is valid only in PV mode.
y   It is not related to the S-ON state.

6070h: Velocity threshold time
y   Defines the time window for determining whether the user velocity is 0, which must be used together
with 606Fh.

6071h: Target torque
y   Defines the target torque of the servo drive in PT mode.
y   The value 1000 corresponds to the rated torque of the motor.

6072h: Max. torque
y   Defines the maximum torque reference limit.
y   The value 1000 corresponds to the rated torque of the motor.

6074h: Torque reference value
y   Defines the target torque value.
y   The value 1000 corresponds to the rated torque of the motor.

6077h: Torque actual value
y   Indicates the internal torque feedback of the servo drive.
y   The value 1000 corresponds to the rated torque of the motor.

607Ah: Target position
y   Defines the target position of the servo drive in PP mode.
y   When bit 6 of 6040h is set to 0, 607Ah indicates the absolute target position of current segment. After
positioning of the current segment is done, the value of 6064h will be the same as the value of 607Ah.
y   When bit 6 of 6040h is set to 1, 607Ah indicates the target incremental displacement of the current
segment. After positioning of current segment is done, the incremental displacement will be the same as
the value of 607Ah.

607Ch: Home offset
y   Defines the physical location of mechanical zero that deviates from the home of the motor in position
control modes (profile position mode, interpolation mode, and homing mode).

y   The home offset in active under the following conditions: The device is powered on, the homing
operation is complete, and bit 15 of 6041h is set to 1.
y   After homing is done, the position actual value (6064h) will be the same as the value of 607Ch.
y   If 607Ch is set to a value outside 607Dh (Software position limit), Er84.3 (Home setting error) will occur.

607D.01h: Minimum software position limit
y   Defines the minimum software position limit relative to the mechanical zero.
y   Minimum software position limit = (607D.01h)
y   The software position limit is used to judge the absolute position. When homing is not performed, the
internal software position limit is inactivated.

607D.02h: Maximum software position limit
y   Defines the maximum software position limit relative to the mechanical zero.
y   Maximum software position limit = (607D.02h)

607Eh: Polarity
y   Defines the polarity of position or speed references.
y   When bit 7 is 1, it indicates the position reference is multiplied by "–1" and the motor direction is
reversed in the standard position mode or interpolation mode.
y   When bit 6 is 1, it indicates the speed reference (60FFh) is multiplied by "–1" and the motor direction is
reversed in the speed mode.
y   When bit 5 is 1, it indicates the torque reference (6071h) is multiplied by "–1" and the motor direction is
reversed in the torque mode.
y   Other bits are meaningless.

607Fh: Max. profile velocity
y   Defines the maximum operating speed in user-defined unit.

6081h: Profile velocity
y   Defines the constant operating speed of the target position in PP mode
y   The setpoint takes effect after the slave receives the displacement reference.

6083h: Profile acceleration
y   Defines the acceleration rate in the acceleration stage of the displacement reference in PP mode.
y   The following formula applies if a motor equipped with 17-bit encoder needs to run at 400 RPM (6081h:
400 x 131072/60) with acceleration rate being 400 RPM/s (6083h: 400 x 131072/60) and deceleration
rate being 200 RPM/s (6084h: 200 x 131072/60) under a gear ratio of 1:1:
Acceleration time tup = Δ6081h/Δ6083h = 1 (s).
Deceleration time tdown = Δ6081h/Δ6084h=2 (s).
y   The setpoint 0 will be forcibly changed to 1.

6084h: Profile deceleration

y   Defines the deceleration rate in the deceleration stage of the displacement reference in PP mode.
y   The following formula applies if a motor equipped with 17-bit encoder needs to run at 400 RPM (6081h:
400 x 131072/60) with acceleration rate being 400 RPM/s (6083h: 400 x 131072/60) and deceleration
rate being 200 RPM/s (6084h: 200 x 131072/60) under a gear ratio of 1:1:
Acceleration time tup = Δ6081h/Δ6083h = 1 (s).
Deceleration time tdown = Δ6081h/Δ6084h=2 (s).
y   The setpoint 0 will be forcibly changed to 1.

6085h: Quick stop deceleration
y   Defines the deceleration rate when the quick stop command (6040h set to 0x0002) is active and 605Ah
(Quick stop option code) is set to 2 or 5.
y   The setpoint 0 will be forcibly changed to 1.

6087h: Torque slope
y   Defines the acceleration rate (torque reference increment per second) of the torque reference in PT
mode.
y   In PT mode, if 605Ah is set to 1, 2, 5, or 6, or 605Dh is set to 1 or 2, the servo drive decelerates to stop as
defined by 6087h.
y   If the setpoint exceeds the torque reference limit, the limit value will be used.
y   The setpoint 0 will be forcibly changed to 1.

6091.01h: Torque slope
y   Defines the numerator of the gear ratio.
y   The gear ratio is used to establish the proportional relationship between the load shaft displacement
designated by the user and the motor shaft displacement.
y   The relationship between motor position feedback (encoder unit) and load shaft position feedback
(reference unit) is as follows.
Motor position feedback = Load shaft position feedback x Gear ratio
The relationship between the motor speed (rpm) and the load shaft speed (reference unit/s) is as follows.
Motor speed (rpm) = Load shaft speed x 6091h x 60/Encoder resolutions
y   The relationship between the motor acceleration (rpm/ms) and the load shaft acceleration (reference unit/
s2) is as follows.
Motor acceleration (rpm/ms) = Load shaft acceleration x 6091h x 1000/Encoder resolutions/60
6091.02h: Shaft revolutions
y   Defines the denominator of the gear ratio.

6098h: Homing method
y   For details, see Table 4-1 "Mode lists".

6099.01h: Speed during search for switch
y   Defines the speed during search for the deceleration point signal. A large setpoint helps prevent homing
timeout.

6099.02h: Speed during search for zero
y   Defines the speed in searching for the home signal. Setting this speed to a low value prevents overshoot
during stop at high speed, avoiding excessive deviation between the stop position and the set mechanical
home.

609Ah: Homing acceleration
y   Defines the acceleration rate in homing mode.

60B8h: Touch Probe Function

Bit                                Name                                            Description
Touch probe 1 function selection
0       0: Disabled
1: Enabled
Touch probe 1 trigger mode
0: S ingle trigger mode (Latches the position at the first
trigger event.)
bit0 to bit5: Probe 1 related settings
1: Continuous trigger
When DI is used as the probe trigger
Touch probe 1 trigger signal selection                      signal, the DI source cannot be
2       0: DI signal                                                changed after the probe is enabled.
1: Z signal                                                 For an absolute encoder, the Z signal
3       Reserved                                                    refers to the zero point of motor single-
turn position feedback.
Touch probe 1 positive edge
4       0: Switch off latching at positive edge
1: Enable latching at positive edge
Touch probe 1 negative edge
5       0: Switch off latching at negative edge
1: Enable latching at negative edge
6 to 7    Reserved
Touch probe 2 function selection
8       0: Disabled
1: Enabled
Touch probe 2 trigger mode
0: S ingle trigger mode (Latches the position at the first
trigger event.)                                          bit8 to bit13: Probe 2 related settings
1: Continuous trigger
Touch probe 2 trigger signal selection
10       0: DI signal
1: Z signal
11       Reserved

Bit                               Name                                          Description
Touch probe 2 positive edge
12       0: Switch off latching at positive edge
1: Enable latching at positive edge
bit8 to bit13: Probe 2 related settings
Touch probe 2 negative edge
13       0: Switch off latching at negative edge
1: Enable latching at negative edge
14 to 15   Reserved

60BAh: Touch probe 1 positive edge
y    Indicates the position feedback value (reference unit) latched at positive edge of touch probe 1 signal.

60BBh: Touch probe 1 negative edge
y    Indicates the position feedback value (reference unit) latched at negative edge of touch probe 1 signal.

60BCh: Touch probe 2 positive edge
y    Indicates the position feedback value (reference unit) latched at positive edge of touch probe 2 signal.

60BDh: Touch probe 2 negative edge
y    Indicates the position feedback value (reference unit) latched at negative edge of touch probe 2 signal.

60C5h: Max. acceleration
y    Defines the maximum permissible deceleration in PP mode, PV mode, and homing mode.
y    The setpoint 0 will be forcibly changed to 1.

60C6h: Max. deceleration
y    Defines the maximum permissible deceleration in PP mode, PV mode, and homing mode.
y    The setpoint 0 will be forcibly changed to 1.

60D5h: Touch probe 1 positive edge counter
y    The counting value is added by "1" each time this object is triggered.

60D6h: Touch probe 1 negative edge counter
y    The counting value is added by "1" each time this object is triggered.

60D7h: Touch probe 2 positive edge counter
y    The counting value is added by "2" each time this object is triggered.

60D8h: Touch probe 2 negative edge counter
y    The counting value is added by "2" each time this object is triggered.

60E0h: Positive torque limit
y    Defines the maximum torque limit of the servo drive in the forward direction.

60E1h: Negative torque limit
y   Defines the maximum torque limit of the servo drive in the reverse direction.

60E3.01h: 1st supported homing method
y   Bit 0 to bit 7: The low 8 bits indicate the supported homing method. Set 6098h to the corresponding
value.
y   Bit 8: Relative position homing
0: Not supported
1: Supported
y   Bit 9: Absolute position homing
0: Not supported
1: Supported
y   Bit 10 to bit 15: N/A

60E6h: Actual position calculation method
y   Defines the method for calculating the mechanical position after homing is completed. After homing is
triggered, changes in 60E6h will be blocked.

60F4h: Position deviation
y   This object indicates the position deviation (in reference unit).

60FCh: Position reference value
y   Indicates the position reference (encoder unit).
y   If no warning is detected when the S-ON signal is active, the relationship between the position reference
in reference unit and that in encoder unit is as follows:
60FCh (in encoder unit) = 6062h (in reference unit) x 6091h

60FDh: DI status
y   Indicates current DI logic of the drive. 0: Inactive; 1: Active

Bit                                                Description

0                                           Reverse overtravel active

1                                           Forward overtravel active

2                                              Home signal active

3 to 15                                                    N/A

16                                               DI1 input active

17                                               DI2 input active

18                                               DI3 input active

19                                               DI4 input active

Bit                                         Description

20                                        DI5 input active

21 to 26                                           NA

60FFh: Target velocity
y   Defines the target velocity in CSV and PV mode.

6502h: Supported drive modes
y   Defines the target velocity in CSV and PV mode.
```

