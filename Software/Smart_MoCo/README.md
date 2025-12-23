# Smart MoCo Software

This is the software running on the STM32F042F6P6 embedded in the motor controller from this repository.

## CAN

Smart MoCo is controlled and reports telemetry via CAN.
Each motor controller has an ID 0x00-0x7F which corresponds to the upper two nybbles of the CAN IDs used by that motor controller.
The low nybble of the CAN ID is the ID of the telemetry/command message.
All control modes assume the limit switch A is in the low direction and limit switch B is in the high direction. I.e., with conventional current flowing from M+ to M- the motor will eventually contact limit switch B. Therefore Limit B should always be greater than Limit A. Swap the limit switch wires if this is not the case.
All control modes assume the encoder and motor direction are the same. I.e., with conventional current flowing from M+ to M- the absolute encoder will increase in duty cycle and quadurature encoder phase A rising edge preceeds B. Swap the motor polarity if this is not the case.
Ignore Limit drive commands only ignore limit switches. To disable soft limits set them to unreachable positions with e.g., xx5 80 00 00 00 7F FF FF FF (INT32_MIN, INT32_MAX).
If parameters are not set, Smart MoCo will not drive the motor and request each parameter with a remote CAN frame every 200ms. This is to account for lost parameter messages.
PID and error gain parameters are reset when the control mode changes between open loop, target position, and target velocity
Updated PID and error gain messages should be sent after the previous target message and before a target message that changes the control mode.

Data parameter information is given as (Unit Datatype) and all parameters are little endian (least-significant-byte first).

| Datatype | C         | Bytes |
|----------|-----------|-------|
| u8       | uint8_t   | 1     |
| u16      | uint16_t  | 2     |
| u32      | uint32_t  | 4     |
| i8       | int8_t    | 1     |
| i16      | int16_t   | 2     |
| i32      | int32_t   | 4     |
| f16      | float16_t | 2     |
| f32      | float32_t | 4     |
| f64      | float64_t | 8     |

### Telemetry CAN Messages

| Name                | ID  | D0                 | D1  | D2  | D3  | D4                              | D5  | D6               | D7                                                 |
|---------------------|-----|--------------------|-----|-----|-----|---------------------------------|-----|------------------|----------------------------------------------------|
| Position            | xx0 | Angle (step i32)   | ... | ... | ... | Angular Velocity (step/sec i16) | ... | Current (A/8 u8) | Limit A, Limit B, Soft Limit A, Soft Limit B, 0000 |
| Position Calibrated | xx1 |                    |     |     |     |                                 |     |                  |                                                    |
| Command Error       | xxD | Command ID (u8)    |     |     |     |                                 |     |                  |                                                    |
| Echo Reply          | xxF | Payload (u64)      | ... | ... | ... | ...                             | ... | ...              | ...                                                |

### Debug Telemetry
When debug telemetry is enabled it is sent periodically.
| Name     | ID  | Type |
|----------|-----|------|
| Tick     | 7F0 | u64  |
| Angle    | 7F1 | i64  |
| Velocity | 7F2 | f64  |
| Current  | 7F3 | f64  |
| P Out    | 7F4 | f64  |
| I Out    | 7F5 | f64  |
| D Out    | 7F6 | f64  |
| Error    | 7F7 | f64  |
| DeltaT   | 7F8 | f64  |

### Command CAN Messages and Required Parameters

| Name                          | ID  | D0                       | D1                       | D2                               | D3                      | D4                    | D5  | D6  | D7  | 3 | 4 | 5 |
|-------------------------------|-----|--------------------------|--------------------------|----------------------------------|-------------------------|-----------------------|-----|-----|-----|---|---|---|
| Open Loop                     | xx2 | 0                        | Duty Cycle (1/32768 i16) | ...                              |                         |                       |     |     |     | x | x | x |
| Open Loop Ignore Limit        | xx2 | 1                        | Duty Cycle (1/32768 i16) | ...                              |                         |                       |     |     |     | x | x |   |
| Target Position               | xx2 | 2                        | Error Gain (1/1024 u16)  | ...                              | Position (step i32)     | ...                   | ... | ... |     | x | x | x |
| Target Position Ignore Limit  | xx2 | 3                        | Error Gain (1/1024 u16)  | ...                              | Position (step i32)     | ...                   | ... | ... |     | x | x |   |
| Target Velocity               | xx2 | 4                        | Error Gain (1/1024 u16)  | ...                              | Velocity (step/sec i32) | ...                   | ... | ... |     | x | x | x |
| Target Velocity Ignore Limit  | xx2 | 5                        | Error Gain (1/1024 u16)  | ...                              | Velocity (step/sec i32) | ...                   | ... | ... |     | x | x |   |
| Target Current                | xx2 | 6                        | Error Gain (1/1024 u16)  | ...                              | Current (ADC i16)       | ...                   |     |     |     | x | x | x |
| Target Current Ignore Limit   | xx2 | 7                        | Error Gain (1/1024 u16)  | ...                              | Current (ADC i16)       | ...                   |     |     |     | x | x |   |
| Set Low-Pass Smoothing Factor | xx3 | Alpha (1/65536 u16)      | ...                      |                                  |                         |                       |     |     |     |   |   |   |
| Set PID                       | xx4 | P (1/256 u16)            | ...                      | I (sec/256 u16)                  | ...                     | D (1/256/sec u16)     | ... |     |     |   |   |   |
| Set Soft Limit Position       | xx5 | A Position (step i32)    | ...                      | ...                              | ...                     | B Position (step i32) | ... | ... | ... |   |   |   |
| Start Position Calibration    | xx6 | Duty Cycle (1/32768 i16) | ...                      | Limit Switch Position (step i32) | ...                     | ...                   | ... |     |     |   |   |   |
| Debug Telemetry               | xx7 | Enable (u8)              |                          |                                  |                         |                       |     |     |     |   |   |   |
| Stop and Reset                | xxC |                          |                          |                                  |                         |                       |     |     |     |   |   |   |
| Echo Request                  | xxE | Payload (u64)            | ...                      | ...                              | ...                     | ...                   | ... | ... | ... |   |   |   |
