#include <Arduino.h>
#include <AlfredoDShot.h>

// Left ESC signal in the battlebot project.
static constexpr int ESC_PIN = 2;
static constexpr uint8_t MOTOR_POLES = 14;

AlfredoDShot esc;

void setup() {
  AlfredoDShot::releaseBootloader(ESC_PIN);
  Serial.begin(115200);
  delay(50);
  bool ok = esc.begin(ESC_PIN, DSHOT600, true, MOTOR_POLES);
  Serial.printf("AlfredoDShot begin=%u pin=%d\n", ok ? 1U : 0U, ESC_PIN);
}

void loop() {
  static uint32_t nextUs = micros();
  static uint32_t lastPrint = 0;
  static uint32_t nextEdtSequence = 0;
  static uint32_t edtWaitUntil = 0;

  if ((int32_t)(micros() - nextUs) < 0) return;
  nextUs += 1000; // 1 kHz, as in the upstream example

  // AM32 accepts EDT/configuration commands with the motor stopped. First
  // complete the library's zero-throttle arming period, then send one stop
  // frame followed by six consecutive EDT-enable commands.
  bool fresh;
  if (esc.isArmed() && (int32_t)(millis() - nextEdtSequence) >= 0) {
    esc.send(0);
    esc.command(DSHOT_CMD_EDT_ENABLE, 6);
    edtWaitUntil = millis() + 1000;
    nextEdtSequence = millis() + 5000;
  }

  if ((int32_t)(millis() - edtWaitUntil) < 0)
    fresh = esc.send(0);
  else
    fresh = esc.sendThrottle(0.20f);

  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    const auto st = esc.stats();
    Serial.printf("fresh:%u armed:%u status:%u echo:%u rpm:%.0f erpm:%lu "
                  "loss:%.1f%% edt:%u V:%.2f A:%.2f sent:%lu ok:%lu no:%lu "
                  "frame:0x%04X\n",
                  fresh ? 1U : 0U, esc.isArmed() ? 1U : 0U,
                  (unsigned)esc.status(), (unsigned)esc.echoPulses(), esc.rpm(),
                  (unsigned long)esc.erpm(), esc.lossPercent(),
                  esc.edtSeen() ? 1U : 0U, esc.voltage(), esc.current(),
                  (unsigned long)st.sent, (unsigned long)st.ok,
                  (unsigned long)st.noReply, (unsigned)esc.lastFrame());
  }
}
