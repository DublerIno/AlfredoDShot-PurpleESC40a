# AlfredoDShot

Bidirectional DShot for the ESP32, built on the RMT peripheral and aimed at
[AM32](https://github.com/am32-firmware/AM32) ESCs.

- One GPIO per ESC — TX and RX share the pad, like a 1-Wire bus.
- eRPM telemetry decoded from the GCR reply, plus Extended DShot Telemetry.
- DShot150 / 300 / 600 / 1200, inverted frame and inverted CRC.
- No tasks, no timers, no DMA. Two files.

## Wiring

Use **one** GPIO per ESC. Any free GPIO works — on the ESP32-S3, 4, 5, 6, 7 and
8 are plain I/O with no boot-strapping duty, so they are all good picks.

```
  ESP32-S3                                            AM32 ESC

                             3V3
                              |
                             [ ] 1 kΩ  pull-up
                              |
  GPIO 8  -----[ 33 Ω ]-------+--------------------------- signal
               series         |
                              |
  GND  ------------------------------------------------- ground

                                     ESC runs from its own battery
```

Repeat on another GPIO for each additional ESC. **Each ESC needs its own pair of
resistors** — the pull-up belongs on the ESC side of the series resistor, so it
holds the bus itself rather than just the ESP's pad.

| | |
|---|---|
| **1 kΩ pull-up to 3V3** | **Required.** The ESP drives the line open-drain so the ESC can pull it low to answer. The pull-up provides the rising edge. 1 kΩ–2.2 kΩ is the useful range; the ESP's internal ~45 kΩ pull-up is *far* too slow (a DShot600 telemetry bit is only 1.33 µs) and telemetry will not decode without an external one. |
| **33 Ω series resistor** | Damps ringing on the signal wire and limits current if both ends ever drive at once. Put it at the ESP end. 33–100 Ω all work; it costs nothing at these edge rates and it is cheap insurance on a robot. |
| **Shared ground** | Required, and it must be a real signal ground — run a dedicated ground wire from the ESC to the ESP, not through the motor power return. |
| **Power** | Do not feed the ESC's BEC into the ESP's 3V3 rail unless you know it is 3.3 V. Most AM32 ESCs have no BEC at all. |

The same circuit also carries the AM32 configurator — see
`AM32_ConfiguratorLink` below. No adapter and no rewiring needed to change ESC
settings.

### Why one pin and not two

Two pins wired to the same ESC signal line are the same electrical node, so the
transmitting pin still has to be open-drain or tri-stated while the ESC talks —
otherwise the ESP's push-pull driver fights the ESC. Once TX is open-drain, the
second pin adds nothing but a wasted GPIO. Binding an RMT TX channel *and* an
RMT RX channel to one pad (`io_loop_back` + `io_od_mode`) is exactly what
ESP-IDF's own 1-Wire driver does, and it is the same problem.

The ESP32-S3 has 4 TX-capable and 4 RX-capable RMT channels, so this scheme
supports 4 bidirectional ESCs — the whole GPIO 4–7 block.

## Quick start

```cpp
#include <AlfredoDShot.h>

AlfredoDShot esc;

void setup() {
  esc.begin(4, DSHOT600, /*bidirectional=*/true, /*motorPoles=*/14);
  for (int i = 0; i < 2000; i++) { esc.send(0); delayMicroseconds(1000); }  // arm
}

void loop() {
  static uint32_t next = micros();
  if ((int32_t)(micros() - next) < 0) return;
  next += 1000;                       // 1 kHz control loop

  if (esc.sendThrottle(0.10f)) {      // true when fresh telemetry arrived
    Serial.println(esc.rpm());
  }
}
```

`send()` is the whole protocol: it harvests the reply to the previous frame,
then transmits the next one. Call it at a steady rate. Anything from 1 kHz to
~7 kHz works at DShot600.

## API

| | |
|---|---|
| `begin(pin, mode, bidirectional, motorPoles)` | `mode` is `DSHOT150/300/600/1200`. `motorPoles` is the magnet count — 14 for a typical 12N14P outrunner. |
| `send(value)` | `0` = stop, `48..2047` = throttle. Returns `true` if telemetry decoded. |
| `sendThrottle(0.0..1.0)` | Same, scaled. |
| `command(cmd, repeat = 6)` | Queue a special command. AM32 needs 6 consecutive frames for most of them; beacons fire immediately. Send with the motor stopped. |
| `rpm()` / `erpm()` / `periodUs()` | Shaft RPM, electrical RPM, raw commutation period. |
| `status()` | `DSHOT_RX_OK`, `NO_REPLY`, `FRAMING`, `BAD_GCR`, `BAD_CRC`, `IDLE`. |
| `stats()` / `lossPercent()` / `resetStats()` | Link health counters. |
| `temperatureC()` / `voltage()` / `current()` / `stress()` / `escStatus()` | EDT values. Send `DSHOT_CMD_EDT_ENABLE` first; AM32 then interleaves them with eRPM frames at a few Hz. |
| `setRxIdleTimeoutUs(us)` | How long the receiver waits for the line to go quiet, default 60 µs. |
| `echoPulses()` | Wiring check — see below. |

## Troubleshooting

The receiver shares the pin with the transmitter, so it reads our *own* frame
back off the wire before the ESC ever replies. `echoPulses()` reports how much
of it survived the round trip, which separates wiring faults from protocol
faults:

| `echoPulses()` | Meaning |
|---|---|
| **31** | The pin is driven and released cleanly. Wiring is good; any remaining problem is on the ESC side. |
| **0** | Nothing on the wire at all. The RMT output is not reaching the pad, or the line is shorted. |
| **1–30** | Edges are being lost. Almost always a missing or too-weak pull-up: the line can be pulled low but cannot rise fast enough. |

### Dead after an ESP reset (NO-REPLY, throttle ignored)

The ESC is parked in its bootloader. AM32 reboots ~0.5 s after the signal stops,
and its bootloader only runs the motor firmware if it sees the line go **low** —
but the pull-up beats its internal pull-down, so a booting ESP leaves it stuck.

Fix: `AlfredoDShot::releaseBootloader(pin)` as the first line of `setup()`, as
both DShot examples do. `AM32_ConfiguratorLink` needs the opposite — don't call
it there.

### No telemetry at all

If `echoPulses()` is 31 but every frame is `NO-REPLY`:

- **Check the pull-up again.** AM32 only enters bidirectional mode when it sees
  the line sitting *high* between frames (it wants ~100 consecutive high reads
  while disarmed). A line that idles low keeps it in plain DShot mode forever.
  AM32 needs no configurator setting for this — it auto-detects — but it cannot
  detect what the wiring does not present.
- **Check the ground.** A missing signal ground is the other way to get a line
  the ESC cannot read.
- Three rising beeps and nothing else is AM32's power-on chime. It means the
  ESC powered up but never saw a valid frame, so it never armed.

## Examples

- **`AS5600_TelemetryCheck`** — compares reported eRPM against an AS5600
  magnetic encoder on the motor shaft. This is the one to run first: it prints
  the error between the two RPM figures, the implied pole count, and a
  breakdown of exactly how frames are being lost.
- **`Basic`** — twenty lines, prints RPM.
- **`AM32_ConfiguratorLink`** — read and write ESC settings over the *same
  wire*, no adapter. Flash it and the ESP32-S3 appears to the
  [am32.ca](https://am32.ca) web configurator as a Direct Connect linker; flash
  a DShot sketch again to run the motor. Needs `USB Mode: USB-OTG (TinyUSB)`
  and `USB CDC On Boot: Disabled` — it refuses to build otherwise.

  To connect: leave the ESC **unpowered**, hit Connect in the configurator,
  *then* apply the battery. AM32's bootloader only listens briefly at power-up,
  so it has to come up after the configurator is already listening.

If `poles?` in the AS5600 example settles on a clean even number that is not
what you passed to `begin()`, that is your motor's real pole count.

## How the telemetry path works

The receiver is armed **before** the frame goes out, so a single capture covers
our own transmission, the ~30 µs turnaround, and the ESC's reply. The
alternative — arming the receiver from the transmit-done interrupt — has to win
a race against that 30 µs window on every single frame, and loses some. Here
there is no race at all; the decoder just steps over the outgoing frame by
finding the turnaround gap.

Decoding follows the bidirectional DShot spec: RMT run lengths are converted
back to the 21-bit line stream, `value ^ (value >> 1)` undoes the transition
coding, four 5-bit GCR quintets map back to nibbles, and the inverted 4-bit
checksum gates the result. A frame that fails any of those steps is counted and
discarded rather than reported.

The decoder was verified against a synthetic ESC encoder over all 2048
representable eRPM values at ±12 % timing jitter, at all four DShot rates.

## Notes and limits

- Requires ESP32 Arduino core 3.x (ESP-IDF 5 RMT driver).
- `send()` will briefly block if you call it faster than a capture can
  complete (~135 µs at DShot600). At normal loop rates it never blocks.
- Non-bidirectional mode (`bidirectional=false`) allocates only a TX channel
  and drives the pin push-pull — no pull-up needed.
- eRPM `0` is a real reading: AM32 sends a specific code for "not spinning".
  Check `status()`, not `rpm() > 0`, to tell that apart from a lost frame.
