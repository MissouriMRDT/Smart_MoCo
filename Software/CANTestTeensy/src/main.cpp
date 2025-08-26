#include <ACAN_T4.h>
#include <Arduino.h>

// Refer to i.MX RT1060 Processor Reference Manual
// 23.6.1.1 (OCOTP memory map) and Table 22-9 (Fusemap Descriptions)
uint32_t UUID0 = *(volatile uint32_t *)0x401F4410;
uint32_t UUID1 = *(volatile uint32_t *)0x401F4420;
uint32_t MAC = *(volatile uint32_t *)0x401F4620;
bool FIRST_TRANSMITTER = MAC == 0xE511A751;
uint32_t ID = 0b11111111111 & MAC; // Least significant 11 bits.

uint32_t sendTime = 0;
uint8_t counter = 0;

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = FIRST_TRANSMITTER ? 0 : 2; i < 5; i++) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
  Serial.printf("UUID %08X %08X\n", UUID0, UUID1);
  Serial.printf("MAC %08X\n", MAC);
  Serial.flush();

  ACAN_T4_Settings settings(125 * 1000); // 125 kbit/s
  settings.mRxPin = 23;
  settings.mTxPin = 22;
  Serial.printf("Actual Bit Rate: %d, Exact Bit Rate: %d, PPM Error: %d, "
                "Prescaler: %d, PROP: %d, Time Seg 1: %d, Time Sec 2: %d, Sync "
                "Jump Width: %d\n",
                settings.actualBitRate(), settings.actualBitRate(),
                settings.ppmFromWishedBitRate(), settings.mBitRatePrescaler,
                settings.mPropagationSegment, settings.mPhaseSegment1,
                settings.mPhaseSegment2, settings.mRJW);
  const uint32_t errorCode = ACAN_T4::can1.begin(settings);
  if (0 == errorCode) {
    Serial.println("can1 ok");
  } else {
    Serial.print("Error can1: 0x");
    Serial.println(errorCode, HEX);
  }

  sendTime = FIRST_TRANSMITTER ? millis() + 5000 : 0xFFFFFFFF;
}

void loop() {
  CANMessage rxMessage;
  if (ACAN_T4::can1.receive(rxMessage)) {
    Serial.printf("RX id: 0x%03X, data: 0x%02X\n", rxMessage.id,
                  rxMessage.data[0]);
    counter = (rxMessage.data[0] + 1) % 0xFF;
    sendTime = millis() + 1000;

    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (millis() > sendTime) {
    CANMessage txMessage{id : ID, ext : false, rtr : false, len : 1};
    txMessage.data[0] = counter;
    Serial.printf("TX id: 0x%03X, data: 0x%02X, success: %d\n", ID,
                  txMessage.data[0], ACAN_T4::can1.tryToSend(txMessage));
    sendTime = FIRST_TRANSMITTER ? millis() + 5000 : 0xFFFFFFFF;

    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
  }
}
