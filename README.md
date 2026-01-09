# RA-02 SSTV Balloon Transmitter

Stratospheric balloon SSTV image transmitter with Horus v2 telemetry using SX1276 (RA-02) module + Raspberry Pi Pico.

[![License: CC BY-NC 4.0](https://img.shields.io/badge/License-CC%20BY--NC%204.0-lightgrey.svg)](https://creativecommons.org/licenses/by-nc/4.0/)

## Project Description

SSTV (Slow Scan Television) image transmitter with integrated Horus Binary v2 4FSK telemetry for stratospheric balloon missions. Designed for 70cm (430 MHz) amateur radio band.

### Features

- **USB Mass Storage** - Pico appears as USB drive for easy image upload
- **JPEG Support** - Automatic JPEG decoding with scaling
- **Multiple Images** - Cycles through all JPEG files alphabetically
- **Two SSTV Modes** - Robot36 (36s) and PD120 (126s)
- **Horus v2 Telemetry** - 4FSK telemetry for position tracking
- **GPS Integration** - ATGM336H GPS module support
- **Image Overlay** - Callsign, label, and image counter on transmitted images
- **Configuration File** - Easy setup via config.txt
- **PPM Correction** - Precise frequency tuning for crystal offset
- **Separate Frequencies** - Independent SSTV and Horus frequencies
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
| ATGM336H | GPS module (optional but recommended) |
| Wire Antenna | 1/4 wave for 70cm band (~17cm) |

### Frequency Band

| Band | Frequency Range |
|------|-----------------|
| 70cm | 430.000 - 440.000 MHz |

**Note:** RA-02 module is designed for 433 MHz band.

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

Raspberry Pi Pico              ATGM336H GPS Module
=================              ===================
GPIO 1 (UART0 RX)  ────────── TX
3V3                ────────── VCC
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
sstv_freq=433.400
horus_freq=437.600
ppm=0.0
interval=3
mode=robot36
horus=1
horus_id=256
horus_count=1
sstv_callsign=N0CALL
sstv_label=
sstv_font_size=2
sstv_counter=1
```

| Parameter | Description | Default |
|-----------|-------------|---------|
| `sstv_freq` | SSTV TX frequency in MHz | 433.400 |
| `horus_freq` | Horus TX frequency in MHz | 437.600 |
| `ppm` | Crystal PPM correction | 0.0 |
| `interval` | Seconds between transmissions | 3 |
| `mode` | SSTV mode: `robot36` or `pd120` | robot36 |
| `horus` | Enable Horus v2 telemetry: `0` or `1` | 1 |
| `horus_id` | Horus payload ID (256-65535 for custom) | 256 |
| `horus_count` | Number of Horus packets per SSTV cycle | 1 |
| `sstv_callsign` | Callsign overlay (top-left corner) | N0CALL |
| `sstv_label` | Label overlay (bottom-left corner) | (empty) |
| `sstv_font_size` | Font size for overlays (1-4) | 2 |
| `sstv_counter` | Show image counter (1/12, 2/12...): `0` or `1` | 1 |

### Image Overlay

The transmitter automatically overlays text on images before transmission:

- **Top-left:** Callsign (from `sstv_callsign`)
- **Top-right:** Image counter (e.g., "3/12") if `sstv_counter=1` and multiple images
- **Bottom-left:** Custom label (from `sstv_label`)

Font size can be adjusted with `sstv_font_size` (1=small, 4=large).

### Horus Binary v2 Telemetry

The transmitter supports **Horus Binary v2** 4FSK telemetry. When enabled, telemetry packets are transmitted before each SSTV image on a separate frequency.

**Horus v2 Specifications:**
- 100 baud 4FSK (direct RF frequency shifting)
- 270 Hz tone spacing
- Golay FEC for error correction
- +1200 Hz audio offset for USB reception
- Compatible with [Horus-GUI](https://github.com/projecthorus/horus-gui) decoder

**USB Reception:**
Tune your SDR to the `horus_freq` and use USB mode. The lowest tone will be at +1200 Hz audio, highest at +2010 Hz. This allows direct reception with standard Horus-GUI settings.

**Packet Contents:**
- Payload ID and packet counter
- Time (hours, minutes, seconds) from GPS
- Position (latitude, longitude, altitude) from GPS
- Speed and satellite count
- Core temperature and battery voltage (ADC)

**GPS Integration:**
Connect ATGM336H GPS module to GPIO1 (UART0 RX) for real position data. Without GPS, simulated test coordinates are transmitted.

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
| SSTV Modulation | FM (1200-2300 Hz audio) |
| Horus Modulation | 4FSK (270 Hz spacing) |
| Output power | +20 dBm (100 mW) max |
| Frequency accuracy | Depends on crystal + PPM correction |
| Image buffer | 320×240 YCbCr |
| Flash storage | 256 KB for images |
| GPS | ATGM336H via UART0 (9600 baud) |

## Balloon Mission Notes

- Use Robot36 for faster updates, PD120 for better quality
- Enable Horus telemetry for real-time position tracking
- Test PPM correction on ground before launch
- Ensure antenna is properly matched for chosen frequency
- Consider duty cycle and thermal management at altitude
- GPS may need clear sky view for lock

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
