#ifndef __SMOCO_H
#define __SMOCO_H

#include <stdbool.h>
#include <stdint.h>

typedef union {
  struct __attribute__((__packed__)) {
    int32_t position;
    int16_t velocity;
    uint8_t current;
    uint8_t flags;
  } SMOCO_MID_POSITION_;
  struct __attribute__((__packed__)) {
    uint8_t commandID;
  } SMOCO_MID_ERROR_;
  struct __attribute__((__packed__)) {
    uint64_t payload;
  } SMOCO_MID_ECHO_REPLY_;
  struct __attribute__((__packed__)) {
    float rampRate;
  } SMOCO_MID_RAMP_RATE_;
  struct __attribute__((__packed__)) {
    float p;
    float i;
  } SMOCO_MID_PI_;
  struct __attribute__((__packed__)) {
    float d;
  } SMOCO_MID_D_;
  struct __attribute__((__packed__)) {
    uint8_t ab;
  } SMOCO_MID_IGNORE_LIMIT_;
  struct __attribute__((__packed__)) {
    int32_t aPosition;
    int32_t bPosition;
  } SMOCO_MID_SOFT_LIMIT_;
  struct __attribute__((__packed__)) {
    int16_t dutyCycle;
    int32_t limitSwitchPosition;
  } SMOCO_MID_CALIBRATE_;
  struct __attribute__((__packed__)) {
    bool enable;
  } SMOCO_MID_DEBUG_;
  struct __attribute__((__packed__)) {
    int16_t fwdMax;
    int16_t fwdMin;
    int16_t revMin;
    int16_t revMax;
  } SMOCO_MID_DUTY_CYCLE_RANGE_;
  struct __attribute__((__packed__)) {
    uint64_t payload;
  } SMOCO_MID_ECHO_REQUEST_;
  struct __attribute__((__packed__)) {
    int16_t dutyCycle;
  } SMOCO_MID_OPEN_LOOP_;
  struct __attribute__((__packed__)) {
    int16_t feedForward;
    int32_t position;
  } SMOCO_MID_TARGET_POSITION_;
  struct __attribute__((__packed__)) {
    int16_t feedForward;
    float velocity;
  } SMOCO_MID_TARGET_VELOCITY_;
  struct __attribute__((__packed__)) {
    int16_t feedForward;
    int16_t current;
  } SMOCO_MID_TARGET_CURRENT_;
} SMOCOMessage;
typedef struct __attribute__((__packed__)) {
  uint32_t tick;
  int32_t position;
  float velocity;
  float current;
  float pOut;
  float iOut;
  float dOut;
  float error;
  float deltaT;
} DebugTelemetry;

#define SMOCO_WIDTH_DID 5
#define SMOCO_WIDTH_MID 6

#define SMOCO_MID_POSITION 0x30
#define SMOCO_MID_POSITION_CALIBRATED 0x31
#define SMOCO_MID_ERROR 0x32
#define SMOCO_MID_ECHO_REPLY 0x3F
#define SMOCO_MID_STOP 0x00
#define SMOCO_MID_RAMP_RATE 0x01
#define SMOCO_MID_PI 0x02
#define SMOCO_MID_D 0x03
#define SMOCO_MID_IGNORE_LIMIT 0x04
#define SMOCO_MID_SOFT_LIMIT 0x05
#define SMOCO_MID_CALIBRATE 0x06
#define SMOCO_MID_DEBUG 0x07
#define SMOCO_MID_DUTY_CYCLE_RANGE 0x08
#define SMOCO_MID_ECHO_REQUEST 0x0F
#define SMOCO_MID_OPEN_LOOP 0x10
#define SMOCO_MID_TARGET_POSITION 0x11
#define SMOCO_MID_TARGET_VELOCITY 0x12
#define SMOCO_MID_TARGET_CURRENT 0x13

static const uint32_t SMOCO_WIDTH[1 << SMOCO_WIDTH_MID] = {
    [SMOCO_MID_POSITION] = 8,
    [SMOCO_MID_POSITION_CALIBRATED] = 0,
    [SMOCO_MID_ERROR] = 1,
    [SMOCO_MID_ECHO_REPLY] = 8,
    [SMOCO_MID_STOP] = 0,
    [SMOCO_MID_RAMP_RATE] = 4,
    [SMOCO_MID_PI] = 8,
    [SMOCO_MID_D] = 4,
    [SMOCO_MID_IGNORE_LIMIT] = 1,
    [SMOCO_MID_SOFT_LIMIT] = 8,
    [SMOCO_MID_CALIBRATE] = 6,
    [SMOCO_MID_DEBUG] = 1,
    [SMOCO_MID_DUTY_CYCLE_RANGE] = 8,
    [SMOCO_MID_ECHO_REQUEST] = 8,
    [SMOCO_MID_OPEN_LOOP] = 2,
    [SMOCO_MID_TARGET_POSITION] = 6,
    [SMOCO_MID_TARGET_VELOCITY] = 6,
    [SMOCO_MID_TARGET_CURRENT] = 4,
};

#define SMOCO_ID_DEBUG 0x7F0
#define SMOCO_WIDTH_DEBUG 4

#endif /* __SMOCO_H */
