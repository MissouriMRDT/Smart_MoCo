#ifndef __MOCO_CONFIG_H
#define __MOCO_CONFIG_H

// (0x7F to 0x00) The high two nybbles of all CAN IDs associated with this
// device. This should be unique for each Smart MoCo on the CAN bus.
#define MOCO_ID 0x0B

// Comment out to use absolute PWM encoder on PA5 and uncomment to use
// quadrature encoder on PA1 and PA5.
// #define QUADRATURE_ENCODER 1

// Telemetry interval (ms)
#define TELEMETRY_INTERVAL 500

// Rate to capture debug telemetry (ms)
#define DEBUG_TELEMETRY_INTERVAL 100

// Missing parameter request interval (ms)
#define PARAMETER_REQUEST_INTERVAL 500

#endif // __MOCO_CONFIG_H
