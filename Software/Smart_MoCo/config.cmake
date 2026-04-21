# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE

# (0x7F to 0x00) The high two nybbles of all CAN IDs associated with this device. This should be unique for each Smart MoCo on the CAN bus.
SMOCO_ID=0x08

# Comment out to use absolute PWM encoder on PA5 and uncomment to use quadrature encoder on PA1 and PA5.
# QUADRATURE_ENCODER

# Absolute encoder resolution (steps traveled as PWM duty cycle increases from 0% to 100%)
ABSOLUTE_ENCODER_RESOLUTION=4096

#[[
If the following are all true:
1. The range of the absolute encoder crosses over the zero point.
2. The motor controller can be powered up with the encoder on either side of
    that zero point.
3. The absolute position must correlate with the same joint position every
    time the motor controller powers up.
Then follow these steps to change this value to a position out of the range of
the joint.
1. Power off the motor controller.
2. Comment out ABSOLUTE_ENCODER_STARTUP_THRESHOLD.
3. Move the joint in the positive direction as far as possible.
4. Power on the motor controller and upload the software.
5. Set ABSOLUTE_ENCODER_STARTUP_THRESHOLD to the reported joint position plus
    several steps to get this value out of the joint's range.
]]
# ABSOLUTE_ENCODER_STARTUP_THRESHOLD=1600

)