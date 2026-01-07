#include <stdio.h>
#include <math.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "tusb.h"
#include "msc_disk.h"

// JPEG loader implemented in jpeg_tjpgdec.c
bool load_jpeg_image(const uint8_t* jpeg_data, uint32_t size);

#define PIN_SCK   10
#define PIN_MOSI  11
#define PIN_MISO  12
#define PIN_CS    13
#define PIN_RST   14
#define PIN_DIO0  15

// SX1276 Registers
#define REG_FIFO           0x00
#define REG_OP_MODE        0x01
#define REG_BITRATE_MSB    0x02
#define REG_BITRATE_LSB    0x03
#define REG_FDEV_MSB       0x04
#define REG_FDEV_LSB       0x05
#define REG_FRF_MSB        0x06
#define REG_FRF_MID        0x07
#define REG_FRF_LSB        0x08
#define REG_PA_CONFIG      0x09
#define REG_PA_RAMP        0x0A
#define REG_IRQ_FLAGS2     0x3F
#define REG_PREAMBLE_MSB   0x25
#define REG_PREAMBLE_LSB   0x26
#define REG_SYNC_CONFIG    0x27
#define REG_PACKET_CONFIG1 0x30
#define REG_PACKET_CONFIG2 0x31
#define REG_PAYLOAD_LEN    0x32
#define REG_FIFO_THRESH    0x35
#define REG_DIO_MAPPING1   0x40
#define REG_VERSION        0x42

#define MODE_SLEEP  0x00
#define MODE_STDBY  0x01
#define MODE_TX     0x03

#define IRQ2_FIFO_FULL     0x80
#define IRQ2_FIFO_EMPTY    0x40
#define IRQ2_FIFO_LEVEL    0x20

#define BASE_FREQ_HZ  434500000UL
#define FSTEP_HZ      61.035f

// Runtime config (can be overridden from config.txt)
static uint32_t base_freq_hz = BASE_FREQ_HZ;
static float ppm_correction = 0.0f;
static uint32_t tx_interval_sec = 3;

// SSTV mode selection
typedef enum {
    SSTV_ROBOT36 = 0,
    SSTV_PD120 = 1
} sstv_mode_t;
static sstv_mode_t sstv_mode = SSTV_ROBOT36;

// FM deviation - try lower for cleaner signal
// Standard SSTV is ±2.4 kHz but let's try ±1.5 kHz
#define FM_DEVIATION_HZ  1500

// SSTV Robot36 frequencies
#define FREQ_VIS_BIT1    1100
#define FREQ_VIS_BIT0    1300
#define FREQ_SYNC        1200
#define FREQ_PORCH       1500
#define FREQ_BLACK       1500
#define FREQ_WHITE       2300

#define IMG_WIDTH   320
#define IMG_HEIGHT  240

// Image buffers - filled from BMP or test pattern
// Full resolution for all channels (needed for PD120)
uint8_t image_Y[IMG_HEIGHT][IMG_WIDTH];
uint8_t image_RY[IMG_HEIGHT][IMG_WIDTH];
uint8_t image_BY[IMG_HEIGHT][IMG_WIDTH];
static bool image_loaded = false;

// RGB to YCbCr conversion
void rgb_to_ycbcr(uint8_t r, uint8_t g, uint8_t b, uint8_t* y, uint8_t* cb, uint8_t* cr) {
    int Y  = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
    int Cb = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
    int Cr = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
    *y  = (Y  < 0) ? 0 : (Y  > 255) ? 255 : Y;
    *cb = (Cb < 0) ? 0 : (Cb > 255) ? 255 : Cb;
    *cr = (Cr < 0) ? 0 : (Cr > 255) ? 255 : Cr;
}

// Load BMP into YCbCr buffers with scaling to 320x240 if needed
bool load_bmp_image(const uint8_t* bmp_data, uint32_t size) {
    if (size < 54) return false;
    
    // Check BMP header
    if (bmp_data[0] != 'B' || bmp_data[1] != 'M') {
        printf("Not a BMP file\n");
        return false;
    }
    
    // Get image info
    uint32_t data_offset = bmp_data[10] | (bmp_data[11] << 8) | 
                           (bmp_data[12] << 16) | (bmp_data[13] << 24);
    int32_t src_width = bmp_data[18] | (bmp_data[19] << 8) | 
                    (bmp_data[20] << 16) | (bmp_data[21] << 24);
    int32_t src_height = bmp_data[22] | (bmp_data[23] << 8) | 
                     (bmp_data[24] << 16) | (bmp_data[25] << 24);
    uint16_t bpp = bmp_data[28] | (bmp_data[29] << 8);
    
    printf("BMP: %dx%d, %d bpp, offset=%d\n", src_width, src_height, bpp, data_offset);
    
    if (src_width <= 0 || src_width > 4096) {
        printf("Invalid width\n");
        return false;
    }
    if (bpp != 24) {
        printf("Need 24-bit BMP\n");
        return false;
    }
    
    // BMP rows are padded to 4 bytes
    int row_size = ((src_width * 3 + 3) / 4) * 4;
    bool bottom_up = (src_height > 0);
    if (src_height < 0) src_height = -src_height;
    
    if (src_height <= 0 || src_height > 4096) {
        printf("Invalid height\n");
        return false;
    }
    
    const uint8_t* pixels = bmp_data + data_offset;
    
    // Scale image to 320x240 using nearest-neighbor sampling
    bool needs_scale = (src_width != IMG_WIDTH || src_height != IMG_HEIGHT);
    if (needs_scale) {
        printf("Scaling from %dx%d to %dx%d\n", src_width, src_height, IMG_WIDTH, IMG_HEIGHT);
    }
    
    for (int y = 0; y < IMG_HEIGHT; y++) {
        // Map destination y to source y
        int src_y = (y * src_height) / IMG_HEIGHT;
        if (bottom_up) src_y = src_height - 1 - src_y;
        if (src_y < 0) src_y = 0;
        if (src_y >= src_height) src_y = src_height - 1;
        
        const uint8_t* row = pixels + src_y * row_size;
        
        for (int x = 0; x < IMG_WIDTH; x++) {
            // Map destination x to source x
            int src_x = (x * src_width) / IMG_WIDTH;
            if (src_x < 0) src_x = 0;
            if (src_x >= src_width) src_x = src_width - 1;
            
            uint8_t b = row[src_x * 3 + 0];
            uint8_t g = row[src_x * 3 + 1];
            uint8_t r = row[src_x * 3 + 2];
            
            uint8_t Y, Cb, Cr;
            rgb_to_ycbcr(r, g, b, &Y, &Cb, &Cr);
            
            image_Y[y][x] = Y;
            image_RY[y][x] = Cr;
            image_BY[y][x] = Cb;
        }
    }
    
    printf("BMP loaded OK%s\n", needs_scale ? " (scaled)" : "");
    return true;
}

void generate_test_image(void) {
    printf("Generating test image...\n");
    const uint8_t bars_Y[8]  = {235, 210, 170, 145, 106, 81, 41, 16};
    const uint8_t bars_Cb[8] = {128, 16,  166, 54,  202, 90, 240, 128};
    const uint8_t bars_Cr[8] = {128, 146, 16,  34,  222, 240, 110, 128};
    
    for (int y = 0; y < IMG_HEIGHT; y++) {
        for (int x = 0; x < IMG_WIDTH; x++) {
            int bar = x / 40;
            if (bar > 7) bar = 7;
            image_Y[y][x] = bars_Y[bar];
            image_RY[y][x] = bars_Cr[bar];
            image_BY[y][x] = bars_Cb[bar];
        }
    }
    printf("Done.\n");
}

// SX1276 functions
static inline void cs_sel() { gpio_put(PIN_CS, 0); }
static inline void cs_desel() { gpio_put(PIN_CS, 1); }

void sx_write(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {reg | 0x80, val};
    cs_sel();
    spi_write_blocking(spi1, tx, 2);
    cs_desel();
}

uint8_t sx_read(uint8_t reg) {
    uint8_t tx[2] = {reg & 0x7F, 0};
    uint8_t rx[2];
    cs_sel();
    spi_write_read_blocking(spi1, tx, rx, 2);
    cs_desel();
    return rx[1];
}

void fill_fifo(int count) {
    cs_sel();
    uint8_t cmd = REG_FIFO | 0x80;
    spi_write_blocking(spi1, &cmd, 1);
    for (int i = 0; i < count; i++) {
        uint8_t pattern = 0xAA;
        spi_write_blocking(spi1, &pattern, 1);
    }
    cs_desel();
}

void feed_fifo(void) {
    uint8_t flags = sx_read(REG_IRQ_FLAGS2);
    if (!(flags & IRQ2_FIFO_LEVEL)) {
        fill_fifo(32);
    }
    // Service TinyUSB while doing long transmissions so the host remains responsive
    tud_task();
}

// FSK Tone - bitrate = 2 * audio_freq
static bool tx_active = false;
static uint32_t current_freq = 0;

void set_bitrate(uint32_t br) {
    uint32_t reg = 32000000UL / br;
    sx_write(REG_BITRATE_MSB, (reg >> 8) & 0xFF);
    sx_write(REG_BITRATE_LSB, reg & 0xFF);
}

void tone(uint32_t freq_hz) {
    if (freq_hz == 0) return;
    uint32_t bitrate = freq_hz * 2;
    
    if (!tx_active) {
        set_bitrate(bitrate);
        fill_fifo(48);
        sx_write(REG_OP_MODE, MODE_TX);
        tx_active = true;
        current_freq = freq_hz;
    } else if (freq_hz != current_freq) {
        set_bitrate(bitrate);
        current_freq = freq_hz;
    }
    feed_fifo();
}

void tone_off(void) {
    sx_write(REG_OP_MODE, MODE_STDBY);
    tx_active = false;
    current_freq = 0;
}

void tone_ms(uint32_t freq_hz, uint32_t ms) {
    tone(freq_hz);
    uint32_t start = time_us_32();
    while (time_us_32() - start < ms * 1000) {
        feed_fifo();
        sleep_us(200);
    }
}

void tone_us(uint32_t freq_hz, uint32_t us) {
    tone(freq_hz);
    uint32_t start = time_us_32();
    while (time_us_32() - start < us) {
        feed_fifo();
    }
}

void init_radio(void) {
    gpio_put(PIN_RST, 0);
    sleep_ms(10);
    gpio_put(PIN_RST, 1);
    sleep_ms(10);
    
    printf("SX1276: 0x%02X\n", sx_read(REG_VERSION));
    
    sx_write(REG_OP_MODE, MODE_SLEEP);
    sleep_ms(10);
    sx_write(REG_OP_MODE, 0x00);
    sleep_ms(10);
    
    // Apply PPM correction to frequency
    double corrected_freq = (double)base_freq_hz * (1.0 + ppm_correction / 1000000.0);
    uint32_t frf = (uint32_t)(corrected_freq / FSTEP_HZ);
    sx_write(REG_FRF_MSB, (frf >> 16) & 0xFF);
    sx_write(REG_FRF_MID, (frf >> 8) & 0xFF);
    sx_write(REG_FRF_LSB, frf & 0xFF);
    
    printf("Freq: %.3f MHz (PPM: %.1f)\n", corrected_freq / 1000000.0, ppm_correction);
    
    // FM deviation
    uint16_t fdev = (uint16_t)(FM_DEVIATION_HZ / FSTEP_HZ);
    printf("FDEV: %d (%.0f Hz)\n", fdev, fdev * FSTEP_HZ);
    sx_write(REG_FDEV_MSB, (fdev >> 8) & 0x3F);
    sx_write(REG_FDEV_LSB, fdev & 0xFF);
    
    set_bitrate(2400);
    
    sx_write(REG_PA_CONFIG, 0x8F);
    sx_write(REG_PA_RAMP, 0x09);
    
    sx_write(REG_PREAMBLE_MSB, 0x00);
    sx_write(REG_PREAMBLE_LSB, 0x00);
    sx_write(REG_SYNC_CONFIG, 0x00);
    sx_write(REG_PACKET_CONFIG1, 0x00);
    sx_write(REG_PAYLOAD_LEN, 0x00);
    
    uint8_t pkt2 = sx_read(REG_PACKET_CONFIG2);
    sx_write(REG_PACKET_CONFIG2, pkt2 & 0xF8);
    
    sx_write(REG_FIFO_THRESH, 0x80 | 15);
    
    sx_write(REG_OP_MODE, MODE_STDBY);
    sleep_ms(5);
    
    printf("Radio ready\n");
}

// SSTV
uint32_t pixel_to_freq(uint8_t pixel) {
    return 1500 + (pixel * 800 / 255);
}

void send_vis_code(uint8_t vis) {
    printf("VIS (0x%02X)...\n", vis);
    tone_ms(1900, 300);
    tone_ms(1200, 10);
    tone_ms(1900, 300);
    tone_ms(1200, 30);
    
    uint8_t parity = 0;
    for (int i = 0; i < 7; i++) {
        uint8_t bit = (vis >> i) & 1;
        parity ^= bit;
        tone_ms(bit ? 1100 : 1300, 30);
    }
    tone_ms(parity ? 1100 : 1300, 30);
    tone_ms(1200, 30);
}

void send_robot36_line_pair(int pair) {
    int y_even = pair * 2;
    int y_odd = pair * 2 + 1;
    uint32_t line_start;
    uint32_t pixel_time;
    
    // === EVEN LINE: Y + R-Y ===
    // Sync: 9ms @ 1200 Hz
    tone_us(1200, 9000);
    // Porch: 3ms @ 1500 Hz
    tone_us(1500, 3000);
    
    // Y scan: 88ms total for 320 pixels = 275us per pixel
    line_start = time_us_32();
    for (int x = 0; x < IMG_WIDTH; x++) {
        tone(pixel_to_freq(image_Y[y_even][x]));
        // Wait until correct time for this pixel
        pixel_time = line_start + (x + 1) * 88000 / IMG_WIDTH;
        while (time_us_32() < pixel_time) {
            // tight loop
        }
        if (x % 64 == 0) feed_fifo();
    }
    
    // Separator: 4.5ms @ 1500 Hz
    tone_us(1500, 4500);
    // Even parity: 1.5ms @ 1900 Hz
    tone_us(1900, 1500);
    
    // R-Y scan: 44ms total for 320 pixels = 137.5us per pixel
    line_start = time_us_32();
    for (int x = 0; x < IMG_WIDTH; x++) {
        tone(pixel_to_freq(image_RY[y_even][x]));
        pixel_time = line_start + (x + 1) * 44000 / IMG_WIDTH;
        while (time_us_32() < pixel_time) {
            // tight loop
        }
        if (x % 64 == 0) feed_fifo();
    }
    
    // === ODD LINE: Y + B-Y ===
    // Sync: 9ms @ 1200 Hz
    tone_us(1200, 9000);
    // Porch: 3ms @ 1500 Hz
    tone_us(1500, 3000);
    
    // Y scan: 88ms
    line_start = time_us_32();
    for (int x = 0; x < IMG_WIDTH; x++) {
        tone(pixel_to_freq(image_Y[y_odd][x]));
        pixel_time = line_start + (x + 1) * 88000 / IMG_WIDTH;
        while (time_us_32() < pixel_time) {
            // tight loop
        }
        if (x % 64 == 0) feed_fifo();
    }
    
    // Separator: 4.5ms @ 1500 Hz
    tone_us(1500, 4500);
    // Odd parity: 1.5ms @ 2300 Hz
    tone_us(2300, 1500);
    
    // B-Y scan: 44ms
    line_start = time_us_32();
    for (int x = 0; x < IMG_WIDTH; x++) {
        tone(pixel_to_freq(image_BY[y_odd][x]));
        pixel_time = line_start + (x + 1) * 44000 / IMG_WIDTH;
        while (time_us_32() < pixel_time) {
            // tight loop
        }
        if (x % 64 == 0) feed_fifo();
    }
}

// PD120: 640x496, 126.08s total
// Each "line" is actually a LINE PAIR: Y0, R-Y (avg), B-Y (avg), Y1
// 248 line pairs for 496 lines total
// Sync 20ms@1200Hz, porch 2.08ms@1500Hz, then 4 components each 121.6ms
void send_pd120_line_pair(int pair) {
    // Source lines in our 320x240 buffer
    // PD120 has 248 pairs = 496 lines, we scale from 240 lines
    int y0 = (pair * 2) * 240 / 496;      // even line
    int y1 = (pair * 2 + 1) * 240 / 496;  // odd line
    if (y0 >= IMG_HEIGHT) y0 = IMG_HEIGHT - 1;
    if (y1 >= IMG_HEIGHT) y1 = IMG_HEIGHT - 1;
    
    uint32_t line_start;
    uint32_t pixel_time;
    
    // Sync: 20ms @ 1200 Hz
    tone_us(1200, 20000);
    // Porch: 2.08ms @ 1500 Hz
    tone_us(1500, 2080);
    
    // Y0 (even line luminance): 121.6ms for 640 pixels
    line_start = time_us_32();
    for (int x = 0; x < 640; x++) {
        int src_x = x * IMG_WIDTH / 640;
        if (src_x >= IMG_WIDTH) src_x = IMG_WIDTH - 1;
        tone(pixel_to_freq(image_Y[y0][src_x]));
        pixel_time = line_start + (x + 1) * 121600 / 640;
        while (time_us_32() < pixel_time) {
            // tight loop
        }
        if (x % 128 == 0) feed_fifo();
    }
    
    // R-Y (chrominance average of lines y0 and y1): 121.6ms for 640 pixels
    line_start = time_us_32();
    for (int x = 0; x < 640; x++) {
        int src_x = x * IMG_WIDTH / 640;
        if (src_x >= IMG_WIDTH) src_x = IMG_WIDTH - 1;
        // Average R-Y from both lines
        uint8_t ry_avg = (image_RY[y0][src_x] + image_RY[y1][src_x]) / 2;
        tone(pixel_to_freq(ry_avg));
        pixel_time = line_start + (x + 1) * 121600 / 640;
        while (time_us_32() < pixel_time) {
            // tight loop
        }
        if (x % 128 == 0) feed_fifo();
    }
    
    // B-Y (chrominance average of lines y0 and y1): 121.6ms for 640 pixels
    line_start = time_us_32();
    for (int x = 0; x < 640; x++) {
        int src_x = x * IMG_WIDTH / 640;
        if (src_x >= IMG_WIDTH) src_x = IMG_WIDTH - 1;
        // Average B-Y from both lines
        uint8_t by_avg = (image_BY[y0][src_x] + image_BY[y1][src_x]) / 2;
        tone(pixel_to_freq(by_avg));
        pixel_time = line_start + (x + 1) * 121600 / 640;
        while (time_us_32() < pixel_time) {
            // tight loop
        }
        if (x % 128 == 0) feed_fifo();
    }
    
    // Y1 (odd line luminance): 121.6ms for 640 pixels
    line_start = time_us_32();
    for (int x = 0; x < 640; x++) {
        int src_x = x * IMG_WIDTH / 640;
        if (src_x >= IMG_WIDTH) src_x = IMG_WIDTH - 1;
        tone(pixel_to_freq(image_Y[y1][src_x]));
        pixel_time = line_start + (x + 1) * 121600 / 640;
        while (time_us_32() < pixel_time) {
            // tight loop
        }
        if (x % 128 == 0) feed_fifo();
    }
}

void send_sstv_image(void) {
    if (sstv_mode == SSTV_PD120) {
        printf("=== SSTV TX (PD120) ===\n");
        send_vis_code(0x5F);  // PD120 VIS = 95
        for (int pair = 0; pair < 248; pair++) {
            send_pd120_line_pair(pair);
            if ((pair + 1) % 50 == 0) printf("  %d%%\n", (pair + 1) * 100 / 248);
        }
    } else {
        printf("=== SSTV TX (Robot36) ===\n");
        send_vis_code(0x08);  // Robot36 VIS = 8
        for (int pair = 0; pair < 120; pair++) {
            send_robot36_line_pair(pair);
            if ((pair + 1) % 30 == 0) printf("  %d%%\n", (pair + 1) * 100 / 120);
        }
    }
    tone_ms(1500, 100);
    tone_off();
    printf("=== Done ===\n\n");
}

int main(void) {
    stdio_init_all();
    
    // Check GP5 for force-format jumper (active low)
    gpio_init(5);
    gpio_set_dir(5, GPIO_IN);
    gpio_pull_up(5);
    sleep_ms(10); // let pull-up settle
    
    // USB MSC init
    tusb_init();
    msc_disk_init();
    
    // If GP5 is grounded, force format the flash
    if (!gpio_get(5)) {
        printf("GP5 grounded — forcing flash format...\n");
        msc_disk_format();
        const char* cfg_name_fmt = "config.txt";
        const char* default_cfg_fmt = "freq=434.500\nppm=0.0\ninterval=3\nmode=robot36\n";
        msc_disk_create_file_if_missing(cfg_name_fmt, (const uint8_t*)default_cfg_fmt, strlen(default_cfg_fmt));
        printf("Format complete. Remove GP5 jumper and reset.\n");
    }

    // Wait for USB host to enumerate/mount the device before starting radio
    printf("Waiting for USB host to mount device (10s timeout)...\n");
    uint32_t mount_start = to_ms_since_boot(get_absolute_time());
    while (!tud_mounted() && (to_ms_since_boot(get_absolute_time()) - mount_start) < 10000) {
        tud_task();
        sleep_ms(10);
    }
    if (tud_mounted()) {
        printf("USB host mounted. Continuing startup.\n");
    } else {
        printf("USB host did not mount within timeout; continuing anyway.\n");
    }

    // Ensure config file exists; create with default if missing
    const char* cfg_name = "config.txt";
    const char* default_cfg = "freq=434.500\nppm=0.0\ninterval=3\nmode=robot36\n";
    if (msc_disk_create_file_if_missing(cfg_name, (const uint8_t*)default_cfg, strlen(default_cfg))) {
        uint32_t cfg_size;
        const uint8_t* cfg_data = msc_disk_find_file(cfg_name, &cfg_size);
        if (cfg_data && cfg_size > 0) {
            // Parse config: freq=xxx.xxx, ppm=x.x, interval=x, mode=robot36|pd120
            bool config_valid = false;
            float freq_mhz = 0.0f;
            float ppm_val = 0.0f;
            uint32_t interval_val = 3;
            sstv_mode_t mode_val = SSTV_ROBOT36;
            
            // Simple line-by-line parser
            const char* p = (const char*)cfg_data;
            const char* end = p + cfg_size;
            while (p < end) {
                // Skip whitespace
                while (p < end && (*p == ' ' || *p == '\t')) p++;
                if (p >= end) break;
                
                // Check for freq=
                if (strncmp(p, "freq=", 5) == 0) {
                    p += 5;
                    freq_mhz = 0.0f;
                    float decimal = 0.0f;
                    float decimal_div = 1.0f;
                    bool in_decimal = false;
                    while (p < end && *p != '\n' && *p != '\r') {
                        if (*p >= '0' && *p <= '9') {
                            if (in_decimal) {
                                decimal_div *= 10.0f;
                                decimal += (*p - '0') / decimal_div;
                            } else {
                                freq_mhz = freq_mhz * 10.0f + (*p - '0');
                            }
                        } else if (*p == '.') {
                            in_decimal = true;
                        }
                        p++;
                    }
                    freq_mhz += decimal;
                    if (freq_mhz >= 1.0f && freq_mhz <= 1000.0f) {
                        config_valid = true;
                    }
                }
                // Check for ppm=
                else if (strncmp(p, "ppm=", 4) == 0) {
                    p += 4;
                    ppm_val = 0.0f;
                    float decimal = 0.0f;
                    float decimal_div = 1.0f;
                    bool in_decimal = false;
                    bool negative = false;
                    if (p < end && *p == '-') { negative = true; p++; }
                    while (p < end && *p != '\n' && *p != '\r') {
                        if (*p >= '0' && *p <= '9') {
                            if (in_decimal) {
                                decimal_div *= 10.0f;
                                decimal += (*p - '0') / decimal_div;
                            } else {
                                ppm_val = ppm_val * 10.0f + (*p - '0');
                            }
                        } else if (*p == '.') {
                            in_decimal = true;
                        }
                        p++;
                    }
                    ppm_val += decimal;
                    if (negative) ppm_val = -ppm_val;
                }
                // Check for interval=
                else if (strncmp(p, "interval=", 9) == 0) {
                    p += 9;
                    interval_val = 0;
                    while (p < end && *p != '\n' && *p != '\r') {
                        if (*p >= '0' && *p <= '9') {
                            interval_val = interval_val * 10 + (*p - '0');
                        }
                        p++;
                    }
                    if (interval_val < 1) interval_val = 1;
                    if (interval_val > 3600) interval_val = 3600;
                }
                // Check for mode=
                else if (strncmp(p, "mode=", 5) == 0) {
                    p += 5;
                    if (strncmp(p, "pd120", 5) == 0 || strncmp(p, "PD120", 5) == 0) {
                        mode_val = SSTV_PD120;
                    } else {
                        mode_val = SSTV_ROBOT36;  // default to robot36
                    }
                    // Skip to end of line
                    while (p < end && *p != '\n' && *p != '\r') p++;
                }
                
                // Skip to next line
                while (p < end && *p != '\n') p++;
                if (p < end) p++;
            }

            if (config_valid && freq_mhz >= 1.0f && freq_mhz <= 1000.0f) {
                base_freq_hz = (uint32_t)(freq_mhz * 1000000.0f);
                ppm_correction = ppm_val;
                tx_interval_sec = interval_val;
                sstv_mode = mode_val;
                printf("Config OK: freq=%.3f MHz, ppm=%.1f, interval=%lu s, mode=%s\n", 
                       freq_mhz, ppm_correction, tx_interval_sec,
                       sstv_mode == SSTV_PD120 ? "PD120" : "Robot36");
            } else {
                // invalid config -> overwrite with default
                printf("Config invalid, overwriting with default\n");
                msc_disk_overwrite_file(cfg_name, (const uint8_t*)default_cfg, strlen(default_cfg));
                base_freq_hz = BASE_FREQ_HZ;
                ppm_correction = 0.0f;
                tx_interval_sec = 3;
                sstv_mode = SSTV_ROBOT36;
            }
        }
    }
    
    spi_init(spi1, 10000000);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    
    gpio_init(PIN_RST);
    gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_put(PIN_RST, 1);
    
    sleep_ms(1000);
    printf("\n=== SSTV Robot36 USB ===\n");
    printf("434.500 MHz, dev +/-1.5kHz\n");
    printf("Copy 320x240 24-bit BMP to USB drive\n\n");
    
    init_radio();
    generate_test_image();  // default test pattern
    
    printf("Waiting for BMP file...\n");
    
    // Check if there are JPEGs in flash; if so, build a sorted list and prepare to cycle
    ImageEntry img_list[16];
    bool img_bad[16] = {false};  // Track files that fail to decode
    int img_count = msc_disk_list_jpegs(img_list, 16);
    int curr_img_idx = 0;
    bool use_jpeg_cycle = false;

    printf("Found %d JPEG files:\n", img_count);
    for (int i = 0; i < img_count; i++) {
        printf("  [%d] %s (cluster=%u, size=%lu)\n", i, img_list[i].name, img_list[i].cluster, img_list[i].size);
    }

    if (img_count > 0) {
        use_jpeg_cycle = true;
        // Try to find first working image
        bool found = false;
        for (int tries = 0; tries < img_count && !found; tries++) {
            uint32_t jpg_size;
            const uint8_t* jpg_data = msc_disk_get_jpeg_by_index(curr_img_idx, &jpg_size);
            if (jpg_data && load_jpeg_image(jpg_data, jpg_size)) {
                image_loaded = true;
                found = true;
                printf("Loaded JPEG '%s'\n", img_list[curr_img_idx].name);
            } else {
                printf("JPEG '%s' decode failed, skipping\n", img_list[curr_img_idx].name);
                img_bad[curr_img_idx] = true;
                curr_img_idx = (curr_img_idx + 1) % img_count;
            }
        }
        if (!found) {
            printf("No valid JPEG found\n");
        }
    } else {
        // Fallback: check for single BMP
        uint32_t bmp_size;
        const uint8_t* bmp_data = msc_disk_find_bmp(&bmp_size);
        if (bmp_data && load_bmp_image(bmp_data, bmp_size)) {
            image_loaded = true;
            printf("Found existing BMP in flash!\n");
        }
    }
    
    uint32_t last_change = 0;
    bool pending_check = false;
    bool formatted_once = false;
    
    while (1) {
        tud_task();  // USB handling
        
        // Detect disk write
        if (msc_disk_has_new_image()) {
            last_change = time_us_32();
            pending_check = true;
        }
        
        // Wait 2 seconds after last write before checking
        if (pending_check && (time_us_32() - last_change > 2000000)) {
            pending_check = false;
            printf("Checking for new files...\n");
            // Rebuild JPEG list (sorted) and if present, use cycle mode
            int new_count = msc_disk_list_jpegs(img_list, 16);
            printf("Found %d JPEG files after update\n", new_count);
            if (new_count > 0) {
                // Reset bad flags and index if files changed
                if (new_count != img_count) {
                    curr_img_idx = 0;
                    for (int i = 0; i < 16; i++) img_bad[i] = false;
                    printf("File list changed, resetting\n");
                }
                img_count = new_count;
                use_jpeg_cycle = true;
                
                // Find first working image
                bool found = false;
                for (int tries = 0; tries < img_count && !found; tries++) {
                    if (img_bad[curr_img_idx]) {
                        curr_img_idx = (curr_img_idx + 1) % img_count;
                        continue;
                    }
                    uint32_t jpg_size;
                    const uint8_t* jpg_data = msc_disk_get_jpeg_by_index(curr_img_idx, &jpg_size);
                    if (jpg_data && load_jpeg_image(jpg_data, jpg_size)) {
                        image_loaded = true;
                        found = true;
                        printf("Loaded JPEG[%d] '%s'\n", curr_img_idx, img_list[curr_img_idx].name);
                    } else {
                        printf("JPEG '%s' decode failed, skipping\n", img_list[curr_img_idx].name);
                        img_bad[curr_img_idx] = true;
                        curr_img_idx = (curr_img_idx + 1) % img_count;
                    }
                }
                if (!found) {
                    printf("No valid JPEG found\n");
                    image_loaded = false;
                }
            } else {
                // try single BMP
                use_jpeg_cycle = false;
                uint32_t bmp_size2;
                const uint8_t* bmp_data2 = msc_disk_find_bmp(&bmp_size2);
                if (bmp_data2) {
                    printf("Found BMP, size=%lu\n", bmp_size2);
                    if (load_bmp_image(bmp_data2, bmp_size2)) {
                        image_loaded = true;
                        printf("Image loaded!\n");
                    }
                } else {
                    printf("No BMP/JPEG found\n");
                    image_loaded = false;

                    // If we detected a write (pending_check was triggered) but can't read
                    // any usable filesystem/files, attempt to reformat once so host and
                    // Pico agree on filesystem layout. This avoids cases where the host
                    // formatted the volume in an incompatible way.
                    if (!formatted_once) {
                        printf("No usable filesystem detected after host write — formatting flash to default layout...\n");
                        msc_disk_format();
                        formatted_once = true;

                        // Recreate config.txt with default and reset config values
                        const char* cfg_name2 = "config.txt";
                        const char* default_cfg2 = "freq=434.500\nppm=0.0\ninterval=3\nmode=robot36\n";
                        msc_disk_create_file_if_missing(cfg_name2, (const uint8_t*)default_cfg2, strlen(default_cfg2));
                        base_freq_hz = BASE_FREQ_HZ;
                        ppm_correction = 0.0f;
                        tx_interval_sec = 3;
                        sstv_mode = SSTV_ROBOT36;
                        printf("Format complete; default config restored.\n");
                    }
                }
            }
        }
        
        // Transmit if image loaded, then wait
        if (image_loaded) {
            printf("\n=== TX START (file %d/%d: %s) ===\n", curr_img_idx + 1, img_count > 0 ? img_count : 1,
                   img_count > 0 ? img_list[curr_img_idx].name : "test");
            send_sstv_image();
            printf("TX done. Next TX in %lu s...\n", tx_interval_sec);
            
            // After sending, if we're cycling JPEGs, advance to next good one
            if (use_jpeg_cycle && img_count > 1) {
                // Find next non-bad image
                bool found_next = false;
                for (int tries = 0; tries < img_count && !found_next; tries++) {
                    curr_img_idx = (curr_img_idx + 1) % img_count;
                    if (img_bad[curr_img_idx]) {
                        printf("Skipping bad file '%s'\n", img_list[curr_img_idx].name);
                        continue;
                    }
                    uint32_t next_size;
                    const uint8_t* next_data = msc_disk_get_jpeg_by_index(curr_img_idx, &next_size);
                    if (next_data && load_jpeg_image(next_data, next_size)) {
                        printf("Preloaded '%s' (%lu bytes)\n", img_list[curr_img_idx].name, next_size);
                        found_next = true;
                    } else {
                        printf("Failed to decode '%s', marking as bad\n", img_list[curr_img_idx].name);
                        img_bad[curr_img_idx] = true;
                    }
                }
                if (!found_next) {
                    printf("No more valid images to cycle\n");
                    // Keep current image loaded, will transmit same one
                }
            }

            // Wait tx_interval_sec seconds, but keep USB alive
            for (uint32_t i = 0; i < tx_interval_sec * 100; i++) {
                tud_task();
                sleep_ms(10);
                if (msc_disk_has_new_image()) {
                    last_change = time_us_32();
                    pending_check = true;
                }
            }
        } else {
            // No image loaded - try to load one if we have JPEGs
            if (use_jpeg_cycle && img_count > 0) {
                uint32_t jpg_size;
                const uint8_t* jpg_data = msc_disk_get_jpeg_by_index(curr_img_idx, &jpg_size);
                if (jpg_data && load_jpeg_image(jpg_data, jpg_size)) {
                    image_loaded = true;
                    printf("Recovered: loaded JPEG[%d]\n", curr_img_idx);
                }
            }
        }
        
        sleep_ms(10);
    }
}
