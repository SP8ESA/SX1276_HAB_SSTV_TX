# Copilot Instructions for SX1276_HAB_SSTV_TX

## Project Overview

Stratospheric balloon SSTV image transmitter with Horus v2 4FSK telemetry. Runs on **Raspberry Pi Pico (RP2040)** with **SX1276 (RA-02)** radio module for 70cm amateur band (430-440 MHz).

**Author:** Kacper Kidała SP8ESA  
**License:** CC BY-NC 4.0

## Architecture

```
main.c              - Core application: SSTV encoding, Horus telemetry, GPS parsing, radio control
├── msc_disk.c/h    - USB Mass Storage + FAT12 filesystem on flash (last 256KB)
├── jpeg_tjpgdec.c  - JPEG decoding wrapper using TJpgDec library
├── src/tjpgd.*     - TJpgDec library (elm-chan)
├── usb_descriptors.c - TinyUSB device descriptors
└── tusb_config.h   - TinyUSB configuration (MSC only)
```

### Key Data Flows

1. **Image Pipeline:** USB (host) → Flash (FAT12) → JPEG decode → YCbCr buffers → SSTV FM modulation → SX1276
2. **Telemetry:** GPS (UART0) → NMEA parse → Horus v2 packet → Golay FEC → 4FSK symbols → SX1276
3. **Config:** `config.txt` on flash → parsed at startup → runtime variables

### Image Buffers

```c
uint8_t image_Y[240][320];   // Luminance
uint8_t image_RY[240][320];  // Cr (R-Y)
uint8_t image_BY[240][320];  // Cb (B-Y)
```

## Build & Flash

### VS Code Tasks (preferred)

- **Compile:** Run task `Compile Project` (uses ninja)
- **Flash:** Run task `Run Project` (uses picotool)

### Manual Build

```bash
mkdir build && cd build
cmake ..
ninja  # or make -j4
```

### Flash to Pico

```bash
# Method 1: picotool (with debugger)
picotool load build/SX1276_HAB_SSTV_TX.elf -fx

# Method 2: UF2 drag-drop
# Hold BOOTSEL, connect USB, copy build/SX1276_HAB_SSTV_TX.uf2 to RPI-RP2
```

## Hardware Pin Mappings

| Function | GPIO | Notes |
|----------|------|-------|
| SPI1 SCK | 10 | SX1276 clock |
| SPI1 MOSI | 11 | SX1276 data in |
| SPI1 MISO | 12 | SX1276 data out |
| SX1276 CS | 13 | Chip select |
| SX1276 RST | 14 | Reset (active low) |
| SX1276 DIO0 | 15 | TX done IRQ |
| GPS RX | 1 | UART0, 9600 baud |
| GPS GND | 2,3,4 | Power control (LOW=ON) |
| Factory Reset | 5 | Jumper to GND = format flash |

## Coding Conventions

### Radio Control Pattern

Always use sleep after mode changes:
```c
sx_write(REG_OP_MODE, MODE_SLEEP);
sleep_ms(10);
sx_write(REG_OP_MODE, MODE_STDBY);
```

### USB Task Servicing

Long operations must call `tud_task()` to keep USB responsive:
```c
while (transmitting) {
    feed_fifo();  // Includes tud_task()
    // ... work ...
}
```

### GPS Processing

Call `gps_process()` frequently in loops - it's non-blocking:
```c
for (int i = 0; i < delay_ms / 10; i++) {
    tud_task();
    gps_process();
    sleep_ms(10);
}
```

### Frequency Calculation with PPM

```c
double corrected_freq = (double)freq_hz * (1.0 + ppm_correction / 1000000.0);
uint32_t frf = (uint32_t)(corrected_freq / FSTEP_HZ);  // FSTEP_HZ = 61.035
```

## Critical Implementation Details

### SSTV Modes

- **Robot36:** 320×240, 36s, VIS code 0x08, Y + alternating RY/BY lines
- **PD120:** 640×496, 126s, VIS code 0x5F, full YCrCb per line

### Horus v2 Telemetry

- 4FSK (direct RF frequency shifting)
- 100 baud, 270 Hz tone spacing
- +1200 Hz USB offset (for SDR reception)
- Golay (23,12) FEC with interleaving and scrambling
- Must match `horusdemodlib` encoding exactly

### Flash Filesystem

- FAT12, 256KB at flash offset 0x1C0000
- 512-byte sectors, single cluster per sector
- Max 16 root directory entries
- Write requires 4KB sector erase/program

## Testing Checklist

- [ ] SX1276 version register returns 0x12
- [ ] USB drive "SSTV" appears within 10s
- [ ] JPEG files decode without "progressive" error
- [ ] Horus packets decode in horus-gui
- [ ] PPM correction matches SDR frequency display

## Common Issues

| Symptom | Cause | Fix |
|---------|-------|-----|
| Files read-only | Flash corruption | GP5→GND jumper, reconnect USB |
| JPEG decode fail | Progressive JPEG | Convert: `convert img.jpg -interlace none out.jpg` |
| No USB drive | stdio_usb conflict | Verify `pico_enable_stdio_usb(... 0)` in CMakeLists.txt |
| Frequency drift | Crystal offset | Adjust `ppm` in config.txt |
