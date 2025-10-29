# Death Matter Controller

**ESP32-C3 Matter controller for Death, the Fortune Telling Skeleton**

A Matter-compatible occupancy sensor that controls Death's animatronic skull via UART, exposing 12 operational states as Matter endpoints for Apple Home integration.

> **Part of the [Death Fortune Teller Project](https://github.com/copperdogma/death-fortune-teller)** - This controller sits inside Death's skull and bridges Matter/Apple Home control to Death's main ESP32-WROVER brain.

## 🎯 Project Overview

This is the **Matter controller** that sits inside Death's skull. It acts as a bridge between Apple Home (via Matter) and Death's ESP32-WROVER brain (via UART), enabling remote control of the fortune telling sequence through Siri or the Home app.

### What It Does

- **Detects occupancy** - PIR sensors detect NEAR (skull proximity) and FAR (walking away) events
- **Matter integration** - 12 Matter endpoints represent Death's operational states
- **UART control** - Sends commands to Death's main ESP32 over UART (TX=5, RX=6, 115200 baud)
- **Apple Home ready** - Successfully commissioned and verified with Apple Home on iOS 18

## ✨ Key Features

- **Matter-compatible** - Full Matter 1.0 support with dynamic endpoints
- **Apple Home integration** - Commissioned via QR code, controllable via Siri
- **12 operational states** - Complete fortune telling sequence exposed as Matter endpoints
- **Dual PIR sensing** - NEAR and FAR motion detection for flow control
- **UART bridge** - Real-time state commands sent to Death's main controller
- **Production-ready** - Factory credentials, proper attestation certificates, tested and working

## 🏗️ Hardware

- **Controller**: ESP32-C3 Super Mini (4MB flash, 384KB RAM)
- **Sensors**: 2x PIR motion sensors (NEAR/FAR detection)
- **Communication**: UART to Death's ESP32-WROVER (115200 baud)
- **LED**: Single status LED on GPIO 8
- **Button**: Reset/commissioning button on GPIO 9

## 📡 Matter Endpoints

The controller exposes 12 On/Off endpoints representing Death's operational states:

1. **FAR Motion** - Detects person walking away
2. **NEAR Motion** - Detects person approaching skull
3. **Play Welcome** - Welcome audio sequence
4. **Wait For Near** - Standby state waiting for proximity
5. **Play Finger Prompt** - Audio prompt for finger placement
6. **Mouth Open Wait Finger** - Jaw opens, waiting for finger sensor
7. **Finger Detected** - Capacitive sensor activated
8. **Snap With Finger** - Jaw snaps shut with finger present
9. **Snap No Finger** - Jaw snaps shut without finger
10. **Fortune Flow** - Active fortune telling sequence
11. **Fortune Done** - Fortune printed, sequence complete
12. **Cooldown** - Post-sequence delay before next cycle

## 🔌 UART Protocol

Commands are sent to Death's ESP32-WROVER at **115200 baud**:

```
NEAR    → "NEAR\n"    - Person detected near skull
FAR     → "FAR\n"     - Person walking away
[STATE] → "[STATE_NAME]\n" - Trigger specific state manually
```

In addition to the state triggers, the controller now sends two reliability handshakes:

- `CMD_BOOT_HELLO` (0x0D) is emitted immediately after boot and every second until the WROVER replies with `RSP_BOOT_ACK` (0x90).
- `CMD_FABRIC_HELLO` (0x0E) is emitted every second after Matter commissioning completes until `RSP_FABRIC_ACK` (0x91) arrives.

Death's main controller handles the full state machine, audio playback, servo control, LED effects, and thermal printing. This Matter controller acts as a remote trigger for these operations.

## 🚀 Quick Start

### Prerequisites

1. **ESP-IDF 5.4.1** and **ESP-Matter** installed (see [SETUP-GUIDE.md](SETUP-GUIDE.md))
2. **ESP32-C3 Super Mini** connected via USB
3. **Death's main controller** configured and running

### Build & Flash

```bash
# Set up environment
source ~/esp/esp-idf/export.sh && source ~/esp/esp-matter/export.sh

# Build firmware
cd firmware
idf.py set-target esp32c3
idf.py build

# Flash to device
idf.py -p /dev/cu.usbmodem101 flash

# Monitor output
idf.py -p /dev/cu.usbmodem101 monitor
```

### Commission to Apple Home

1. **Check QR code** - Device prints QR code at boot
2. **Scan with iPhone** - Use Home app to scan QR code
3. **Connect to WiFi** - Provide network credentials during pairing
4. **Control Death** - Toggle endpoints in Home app or use Siri!

For detailed setup instructions, see **[SETUP-GUIDE.md](SETUP-GUIDE.md)**.

## 📚 Documentation

- **[SETUP-GUIDE.md](SETUP-GUIDE.md)** - Complete setup from zero to commissioned device
- **[docs/MATTER-TECHNICAL-GUIDE.md](docs/MATTER-TECHNICAL-GUIDE.md)** - Deep dive into Matter protocol and Apple Home integration
- **[templates/occupancy-sensor/](templates/occupancy-sensor/)** - Source code templates

## 🔧 Architecture

```
┌─────────────────┐         ┌──────────────────┐         ┌─────────────┐
│   Apple Home    │ ◄─────► │  Matter Controller│ ◄─────► │    Death    │
│   (Matter)      │  BLE    │   (ESP32-C3)     │  UART   │  (ESP32)    │
└─────────────────┘         └──────────────────┘         └─────────────┘
                                   │                               │
                                   ▼                               ▼
                            ┌──────────────┐          ┌──────────────────┐
                            │  PIR Sensors │          │  Audio, Servo,   │
                            │  (NEAR/FAR)  │          │  Printer, LED    │
                            └──────────────┘          └──────────────────┘
```

### Flow Example

1. **Person approaches** → NEAR sensor triggers → "NEAR" sent via UART
2. **Death detects** → Plays welcome audio → Opens jaw
3. **Home app toggle** → Matter endpoint "Play Finger Prompt" → Death prompts for finger
4. **Finger detected** → Death prints fortune → Matter "Fortune Done" endpoint toggles
5. **Person leaves** → FAR sensor triggers → "FAR" sent → Cooldown begins

## 🔒 Security

- **Factory credentials** - Unique device certificates per build
- **Matter attestation** - Proper PAA/PAI/DAC certificate chain
- **Secure commissioning** - SPAKE2+ passcode authentication
- **Credential management** - All sensitive files excluded via `.gitignore`

**⚠️ NEVER commit credentials or certificates!** Every device must have unique Matter credentials.

## 🐛 Troubleshooting

### Device won't pair with Apple Home

- **Check BLE advertising** - Monitor should show "CHIPoBLE advertising started"
- **Verify factory partition** - Ensure `firmware/131b_1234/` partition is flashed
- **Factory reset** - Erase flash and re-flash if commissioning data corrupted

### UART communication issues

- **Verify baud rate** - Both devices must use 115200 baud
- **Check wiring** - TX/RX pins (5/6) properly connected
- **Monitor UART** - Check serial output for command transmission

### Memory issues / crashes

- **BLE memory** - Reduced BLE buffer counts for constrained RAM
- **Hardware AES disabled** - Software AES used to avoid heap allocation failures
- **mbedTLS dynamic buffers** - Enabled for CASE handshake memory management

See **[SETUP-GUIDE.md](SETUP-GUIDE.md)** troubleshooting section for detailed solutions.

## 🎓 Development Notes

### Key Technical Decisions

- **ESP32-C3 over ESP32** - Smaller, lower power, sufficient for Matter controller role
- **Dynamic endpoints** - 12 endpoints created at runtime (not static configuration)
- **UART over WiFi** - Simpler than full Matter integration in Death's main controller
- **Occupancy sensor device type** - Logical fit for motion detection use case

### Project Status

✅ **Complete and Working**

- All 12 endpoints created successfully
- Successfully commissioned to Apple Home
- BLE advertising working
- WiFi connectivity established
- CASE handshake completing
- UART communication verified

### Future Enhancements

- [ ] Battery power support with power management
- [ ] Thread network support (currently WiFi-only)
- [ ] Additional sensor endpoints (audio state, printer status)
- [ ] Multi-device Matter fabric support

## 📝 License

MIT License - See [LICENSE](LICENSE) for details.

## 🙏 Acknowledgments

- **Death Project** - The animatronic skeleton this controller serves
- **ESP-IDF & ESP-Matter** - Espressif's excellent Matter SDK
- **Apple Home** - Successfully commissioned and verified integration
- Built with extensive AI-assisted debugging and iteration

---

**Death awaits Matter commands.** Start with [SETUP-GUIDE.md](SETUP-GUIDE.md)! 💀🎭
