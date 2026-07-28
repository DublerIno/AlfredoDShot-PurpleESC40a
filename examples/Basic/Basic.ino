/*
  Basic - bidirectional DShot on one pin, printing RPM.

  Wiring: GPIO 4 -> ESC signal, with a 1k pull-up to 3V3. Share ground.
*/

#include <AlfredoDShot.h>

const int PIN_ESC = 4;
const uint8_t MOTOR_POLES = 14;

AlfredoDShot esc;

void setup() {
  Serial.begin(115200);
  esc.begin(PIN_ESC, DSHOT600, true, MOTOR_POLES);

  // Arm: two seconds of zero throttle at 1 kHz.
  for (int i = 0; i < 2000; i++) {
    esc.send(0);
    delayMicroseconds(1000);
  }
  esc.resetStats();
}

void loop() {
  static uint32_t next = micros();
  static uint32_t lastPrint = 0;

  if ((int32_t)(micros() - next) < 0) return;
  next += 1000;  // 1 kHz

  esc.sendThrottle(0.08f);  // 8 %

  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.printf("rpm %8.0f   erpm %6lu   loss %.2f%%\n", esc.rpm(),
                  (unsigned long)esc.erpm(), esc.lossPercent());
  }
}
