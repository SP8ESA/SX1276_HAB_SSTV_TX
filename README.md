# RA-02 SSTV Balloon Transmitter

Stratospheric balloon SSTV image transmitter using SX1276 (RA-02) module + Raspberry Pi Pico.

[![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc/4.0/)

## Project Description

SSTV (Slow Scan Television) image transmitter for stratospheric balloon missions, designed for 2m (144 MHz) and 70cm (430 MHz) amateur radio bands.

### Features

- **USB Mass Storage** - Pico appears as USB drive for easy image upload
- **JPEG Support** - Automatic JPEG decoding
- **Multiple Images** - Cycles through all JPEG files alphabetically
- **Two SSTV Modes** - Robot36 (36s) and PD120 (126s)
- **Horus v2 Telemetry** - 4FSK telemetry between SSTV images
- **Configuration File** - Easy setup via config.txt
- **PPM Correction** - Precise frequency tuning for crystal offset
- **Auto-recovery** - GP5 jumper for factory reset

## Author

Kacper Kidała SP8ESA

Code generated with assistance from Claude Opus 4.5

## Hardware

### Required Components

| Component | Description |
|-----------|-------------|
| Raspberry Pi Pico | RP2040 microcontroller |
| RA-02 (SX1276) | 433 MHz LoRa module |
| Wire Antenna | 1/4 wave for 2m or 70cm band |

### Frequency Bands

| Band | Frequency Range |
|------|-----------------|
| 2m | 144.000 - 146.000 MHz |
| 70cm | 430.000 - 440.000 MHz |

**Note:** RA-02 module is designed for 433 MHz. For 2m band operation, different module may be required (e.g., SX1278 for 137-525 MHz).

## Wiring Diagram

See [WIRING.txt](WIRING.txt) for detailed connection diagram.

```
Raspberry Pi Pico              RA-02 Module (SX1276)
=================              =====================
GPIO 10 (SPI1 SCK)  ────────── SCK
GPIO 11 (SPI1 TX)   ────────── MOSI
GPIO 12 (SPI1 RX)   ────────── MISO
GPIO 13            ────────── NSS (CS)
GPIO 14            ────────── RESET
GPIO 15            ────────── DIO0

3V3 (Pin 36)       ────────── VCC (3.3V!)
GND                ────────── GND

GPIO 5             ────────── GND (jumper for factory reset)

USB                ────────── To computer
```

⚠️ **WARNING:** RA-02 module operates at 3.3V. Do NOT connect to 5V!

## Building

### Requirements

- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) 2.0+ (or VS Code Pico Extension)
- CMake 3.13+
- ARM GCC toolchain

### Clone

```bash
git clone https://github.com/SP8ESA/SX1276_HAB_SSTV_TX.git
cd SX1276_HAB_SSTV_TX
```

### Build

```bash
mkdir build && cd build
cmake ..
make -j4
```

### Flash

```bash
# Hold BOOTSEL and connect USB
cp SX1276_HAB_SSTV_TX.uf2 /media/$USER/RPI-RP2/
```

## Usage

### Uploading Images

1. Connect Pico to computer via USB
2. Wait for "SSTV" drive to appear (may take a few seconds)
3. Copy JPEG files to the drive
4. Files will be transmitted in alphabetical order

### Image Requirements

- **Format:** JPEG (baseline, not progressive)
- **Size:** Any (automatically scaled to 320×240)
- **Naming:** Standard 8.3 filenames (e.g., `IMAGE1.JPG`)

### Configuration

Edit `config.txt` on the USB drive:

```
freq=434.500
ppm=0.0
interval=3
mode=robot36
horus=0
horus_id=256
horus_count=1
```

| Parameter | Description | Default |
|-----------|-------------|---------|
| `freq` | TX frequency in MHz | 434.500 |
| `ppm` | Crystal PPM correction | 0.0 |
| `interval` | Seconds between transmissions | 3 |
| `mode` | SSTV mode: `robot36` or `pd120` | robot36 |
| `horus` | Enable Horus v2 telemetry: `0` or `1` | 0 |
| `horus_id` | Horus payload ID (256-65535 for custom) | 256 |
| `horus_count` | Number of Horus packets per SSTV cycle | 1 |

### Horus Binary v2 Telemetry

The transmitter supports **Horus Binary v2** 4FSK telemetry. When enabled, telemetry packets are transmitted before each SSTV image.

**Horus v2 Specifications:**
- 100 baud 4FSK
- Golay FEC for error correction
- Standard Horus audio frequencies (1200-2010 Hz)
- Compatible with [Horus-GUI](https://github.com/projecthorus/horus-gui) decoder

**Packet Contents:**
- Payload ID and packet counter
- Time (hours, minutes, seconds)
- Position (latitude, longitude, altitude)
- Speed and satellite count
- Temperature and battery voltage

**Note:** Currently sends simulated test data. To use real GPS/sensor data, modify the `horus_update_telemetry()` function with actual readings.

### SSTV Modes

| Mode | Resolution | Time | VIS Code |
|------|------------|------|----------|
| Robot36 | 320×240 | 36s | 0x08 |
| PD120 | 640×496 | 126s | 0x5F |

## Troubleshooting

### Files Become Read-Only

Sometimes the flash filesystem can become corrupted and files appear read-only.

**Solution:**
1. Disconnect Pico from USB
2. Connect GP5 pin to GND with a jumper wire
3. Connect Pico to USB while holding the jumper
4. Wait for "Format complete" message (check UART output)
5. Remove the jumper and reset Pico
6. Re-upload your images and config.txt

### JPEG Decode Failed

TJpgDec library doesn't support progressive JPEG. Convert your images:

```bash
# Using ImageMagick
convert input.jpg -interlace none output.jpg

# Using ffmpeg
ffmpeg -i input.jpg -pix_fmt yuvj420p output.jpg
```

### No USB Drive Appears

- Wait 5-10 seconds after connecting
- Try different USB cable (data cable, not charge-only)
- Check UART output for debug messages

## Technical Specifications

| Parameter | Value |
|-----------|-------|
| Modulation | FM (AFSK for SSTV) |
| Output power | +14 dBm (25 mW) typical |
| Frequency accuracy | Depends on crystal + PPM correction |
| Image buffer | 320×240 YCbCr |
| Flash storage | 256 KB for images |

## Balloon Mission Notes

- Use Robot36 for faster updates, PD120 for better quality
- Test PPM correction on ground before launch
- Ensure antenna is properly matched for chosen frequency
- Consider duty cycle and thermal management at altitude

## Warning

⚠️ **Transmission on amateur radio frequencies requires appropriate license!**

Make sure you have a valid amateur radio license and comply with regulations in your country. Balloon missions may require additional coordination with aviation authorities.

## License

This project is licensed under CC BY-NC 4.0 (Creative Commons Attribution-NonCommercial).

- Non-commercial use (including amateur radio) - ✅ OK
- Modifications allowed - ✅ OK  
- Commercial use requires author's permission

This project uses:
- [TinyUSB](https://github.com/hathach/tinyusb) - MIT License
- [TJpgDec](http://elm-chan.org/fsw/tjpgd/00index.html) - Custom permissive license
- [Raspberry Pi Pico SDK](https://github.com/raspberrypi/pico-sdk) - BSD-3-Clause

73 de SP8ESA
