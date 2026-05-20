# Smart MoCo Software

This is the software running on the STM32F042F6P6 embedded in the motor controller from this repository.

## CAN

Smart MoCo is controlled and reports telemetry via CAN.
CAN has been tested with the [ACAN_T4](https://github.com/pierremolinaro/acan-t4) library with `ACAN_T4_Settings(125000)` and [gs-usb](https://python-can.readthedocs.io/en/4.0.0/interfaces/gs_usb.html) library with `can.Bus(channel=0, interface="gs_usb", bitrate=125000)`.

| Parameter                    | Value  |
|------------------------------|--------|
| Time Quantum                 | 500ns  |
| Time Quanta in Bit Segment 1 | 8      |
| Time Quanta in Bit Segment 2 | 7      |
| Time for one Bit             | 8000ns |
| Baud Rate                    | 125000 |
| ReSynchronization Jump Width | 4      |

Each motor controller has an Device ID (DID) 0x00-0x1E which corresponds to the upper 5 bits of the CAN IDs used by that motor controller.
The low 6 bits of the CAN ID is the ID of the telemetry/command message (MID).
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

| Name                | MID | D0                  | D1  | D2  | D3  | D4                    | D5  | D6               | D7                                                 |
|---------------------|-----|---------------------|-----|-----|-----|-----------------------|-----|------------------|----------------------------------------------------|
| Position            |  30 | Position (step i32) | ... | ... | ... | Velocity (step/s i16) | ... | Current (A/8 u8) | 0000, Soft Limit B, Soft Limit A, Limit B, Limit A |
| Position Calibrated |  31 |                     |     |     |     |                       |     |                  |                                                    |
| Command Error       |  32 | Command ID (u8)     |     |     |     |                       |     |                  |                                                    |
| Echo Reply          |  3F | Payload (u64)       | ... | ... | ... | ...                   | ... | ...              | ...                                                |

### Debug Telemetry

When debug telemetry is enabled it is sent periodically.

| Name     | ID  | Type |
|----------|-----|------|
| Tick     | 7F0 | i32  |
| Position | 7F1 | i32  |
| Velocity | 7F2 | f32  |
| Current  | 7F3 | f32  |
| P Out    | 7F4 | f32  |
| I Out    | 7F5 | f32  |
| D Out    | 7F6 | f32  |
| Error    | 7F7 | f32  |
| DeltaT   | 7F8 | f32  |

### Command CAN Messages and Required Parameters

| Name                         | MID | D0                         | D1  | D2                               | D3  | D4                    | D5  | D6                    | D7  | 1 | 2 | 3 | 4 | 5 | 8 |
|------------------------------|-----|----------------------------|-----|----------------------------------|-----|-----------------------|-----|-----------------------|-----|---|---|---|---|---|---|
| Stop and Reset               |  00 |                            |     |                                  |     |                       |     |                       |     |   |   |   |   |   |   |
| Set Ramp Rate                |  01 | Ramp Rate (1/s f32)        | ... | ...                              | ... |                       |     |                       |     |   |   |   |   |   |   |
| Set PI                       |  02 | P (f32)                    | ... | ...                              | ... | I (s f32)             | ... | ...                   | ... |   |   |   |   |   |   |
| Set D                        |  03 | D (1/s f32)                | ... | ...                              | ... |                       | ... |                       |     |   |   |   |   |   |   |
| Ignore Limit Switch          |  04 | 000000, B, A               |     |                                  |     |                       |     |                       |     |   |   |   |   |   |   |
| Set Soft Limit Position      |  05 | A Position (step i32)      | ... | ...                              | ... | B Position (step i32) | ... | ...                   | ... |   |   |   |   |   |   |
| Start Position Calibration   |  06 | Duty Cycle (1/32768 i16)   | ... | Limit Switch Position (step i32) | ... | ...                   | ... |                       |     | x |   |   |   |   | x |
| Debug Telemetry              |  07 | 0000000, Enable            |     |                                  |     |                       |     |                       |     |   |   |   |   |   |   |
| Set Duty Cycle Range         |  08 | Fwd Max (1/32768 i16)      | ... | Fwd Min (1/32768 i16)            | ... | Rev Min (1/32768 i16) | ... | Rev Max (1/32768 i16) | ... |   |   |   |   |   |   |
| Echo Request                 |  0F | Payload (u64)              | ... | ...                              | ... | ...                   | ... | ...                   | ... |   |   |   |   |   |   |
| Open Loop                    |  10 | Duty Cycle (1/32768 i16)   | ... |                                  |     |                       |     |                       |     | x |   |   | x | x | x |
| Target Position              |  11 | Feed-Forward (1/32758 i16) | ... | Position (step i32)              | ... | ...                   | ... |                       |     |   | x | x | x | x | x |
| Target Velocity              |  12 | Feed-Forward (1/32758 i16) | ... | Velocity (step/s f32)            | ... | ...                   | ... |                       |     |   | x | x | x | x | x |
| Target Current               |  13 | Feed-Forward (1/32758 i16) | ... | Current (ADC i16)                | ... |                       |     |                       |     |   | x | x | x | x | x |


### LED Error Code Colors

|  Color  | Meaning                      |
|---------|------------------------------|
| Green   | Normal                       |
| Yellow  | Echo request (ping)          |
| Cyan    | Other CAN message (not ping) |
| Purple  | CAN Initialization error     |
| Red     | Malformed request            |