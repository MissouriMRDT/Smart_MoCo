# Smart_MoCo_Hardware

Motor controller breakout with VNH ICs, a microcontroller, and CANbus. 
Transmits encoder, limit switch, and motor controller data over CAN. 

## 2027 SMoCo CAN Interface

The Smart Motor Controller has a set of custom command and telemetry packages which are used to control and read data from it. 

### CAN Packet IDs

All packets are identified in relation to the predetermined packet ID of the SMoCo which is set at the beginning of the can.h header file: 
  ```c++
  #define CAN_ID 0
  ```
### CAN Packet Data
As this is buit to support CAN classic, the expected packet size is 8 bytes which are partitioned variously throughout the packets. Not all packets utilize the full 8 bytes used by CAN, any remaining bytes can be filled with 0s and will not be processed.
### TX Packet Lookup
|ID|Name|Description|Data|
|---|---|---|---|
|CAN_ID + 0x01|Target Position|Sets the mode to position driving and sets the target position.|[feedForward(int16), targetPosition(int32)]
|CAN_ID + 0x02|Target Velocity|Sets the mode to velocity driving and sets the target velocity.|[feedForward(int16), targetVelocity(int32)]
|CAN_ID + 0x03|Target Current|Sets the mode to current driving and sets the target current.|[feedForward(int16), targetCurrent(int32)]
|CAN_ID + 0x04|PI|Sets the proportion and integral scalar values.|[p(float), i(float)]
|CAN_ID + 0x05|D|Sets the derivative scalar value.|[d(float)]
|CAN_ID + 0x06|Soft Limit Enable|Enables or disables the soft limits.|[en(uint8)]
|CAN_ID + 0x07|Soft Limit|Sets the position of the high and low soft limits.|[alim(int32), blim(int32)]
|CAN_ID + 0x08|Ramp Rate|Sets the ramp rate.|[ramp(float)]
|CAN_ID + 0x09|Duty Cycle Range|Sets the lower and upper limits of the duty cycle input.|[fwdMax(int16), fwdMin(int16), revMax(int16), revMin(int16)]
|CAN_ID + 0x0A|Duty Cycle|Sets the duty cycle of the driver.|[dutyCycle(uint16)]
|CAN_ID + 0x0B|Calibrate|Begins calibration setting the speed and the set position.|[cDutyCycle(uint16), cPos(uint32)]

**CAN RX Lookup**
|ID|Name|Description|Data|
|---|---|---|---|
|CAN_ID + 0x00|Telemetry|Returns the current position velocity and current status of the motor.|[rPos(int32), rVel(int16), rCur(uint8)]
