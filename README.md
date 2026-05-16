# POTBot

**Smart camera systems for tracking climate change underwater in fish and habitats.**

POTBot is an open-source, Arduino-based automated underwater camera and environmental monitoring system. Originally developed for deployment inside commercial western rock lobster pots, the system records video of marine habitats and faunal communities at fishing locations, logs water temperature, and captures GPS coordinates on retrieval.

The system operates autonomously for approximately one month of continuous fishing activity. When spent, units are simply swapped out for fresh ones and data collection continues at minimal cost.

## How It Works

1. **Power on** — the unit acquires a GPS lock and enters standby.
2. **Deployment** — a conductivity-based wet switch detects immersion and triggers video recording.
3. **In water** — the camera records for a configurable duration (default 40 min). Water temperature is logged periodically. If deployed in darkness, a short supplementary video is recorded at dawn.
4. **Retrieval** — when removed from water, the GPS activates and records the pot's position.
5. **Data download** — videos are accessed via USB from the Mobius camera; metadata (GPS, timestamps, temperatures) is read through the Arduino Serial Monitor.

## Repository Structure

```
POTBot/
├── docs/
│   ├── POTBot_Operations_Manual.docx   # Full setup and operations manual
│   └── Flow_Diagram_PB_IV.jpg          # Operational state diagram
├── Version10/
│   └── PB_CodeV10/
│       └── PB_CodeV10.ino              # Current firmware (V10)
├── Pot_code/                           # Earlier firmware versions
├── Final-Report 2011-021.pdf           # Original FRDC research report
└── README.md
```

## Hardware

| Component | Description |
|-----------|-------------|
| Microcontroller | Adafruit ItsyBitsy M0 Express |
| Camera | Mobius ActionCam |
| GPS | Serial GPS module (9600 baud) |
| Real-Time Clock | Gravity RTC (I²C) |
| Temperature Sensor | Dallas DS18B20 (OneWire) |
| Flash Storage | Adafruit SPI Flash (onboard FAT filesystem) |
| Status LED | Adafruit DotStar |
| Wet Switch | Conductivity-based (analog) |

## Getting Started

### Prerequisites

- [Arduino IDE](https://www.arduino.cc/en/software) (latest version)
- Adafruit SAMD board support (install via Boards Manager)
- Required libraries (install via Library Manager):
  - OneWire
  - DallasTemperature
  - Adafruit SleepyDog
  - Adafruit SPIFlash
  - TinyGPS++
  - Adafruit DotStar
  - RTClib
- GravityRTC library (install from ZIP — included in the repo or available in the docs)

### Upload Firmware

1. Open `Version10/PB_CodeV10/PB_CodeV10.ino` in Arduino IDE.
2. Select **Adafruit ItsyBitsy M0** as the board.
3. Select the correct COM port.
4. Click Upload.

For detailed setup, deployment, and data retrieval instructions, see the **[Operations Manual](docs/POTBot_Operations_Manual.docx)**.

## Configurable Parameters

| Parameter | Default | Unit | Description |
|-----------|---------|------|-------------|
| `VidTime` | 2400 | seconds | Video recording duration after deployment |
| `GPSTime` | 240 | minutes | GPS active time after retrieval |
| `Dawn` | 7 | hour (24h) | Start of daylight window |
| `Dusk` | 17 | hour (24h) | End of daylight window |

## Applications

POTBot systems have been used to:

- Augment spatial habitat mapping off the west coast of Western Australia
- Document spatial and habitat distributions of marine teleosts across the west coast bioregion
- Serve as a robust, automated alternative for BRUV (Baited Remote Underwater Video) monitoring with geographic synchronisation of footage

## Licence

This project is open-source freeware. See individual files for any specific terms.

## Contributing

Contributions, bug reports, and feature requests are welcome. Please open an issue or submit a pull request.
