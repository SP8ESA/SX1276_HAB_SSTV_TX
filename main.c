#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "tusb.h"
#include "msc_disk.h"

// JPEG loader implemented in jpeg_tjpgdec.c
bool load_jpeg_image(const uint8_t* jpeg_data, uint32_t size);

// Forward declarations for radio functions (used by Horus before definition)
void tone(uint32_t freq_hz);
void tone_off(void);
void tone_ms(uint32_t freq_hz, uint32_t ms);
void tone_us(uint32_t freq_hz, uint32_t us);
void feed_fifo(void);
void sx_write(uint8_t reg, uint8_t val);
uint8_t sx_read(uint8_t reg);
void set_bitrate(uint32_t br);
void init_radio(void);

#define PIN_SCK   10
#define PIN_MOSI  11
#define PIN_MISO  12
#define PIN_CS    13
#define PIN_RST   14
#define PIN_DIO0  15
#define PIN_DIO2  16    // Data input for continuous FSK mode

// GPS ATGM336H pins
#define GPS_UART       uart0
#define GPS_UART_RX    1        // GP1 - GPS TX -> Pico RX
#define GPS_GND_PIN1   2        // GP2 - GPS GND control
#define GPS_GND_PIN2   3        // GP3 - GPS GND control  
#define GPS_GND_PIN3   4        // GP4 - GPS GND control
#define GPS_BAUD       9600


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

// ============================================================================
// HORUS BINARY v2 - 4FSK Telemetry (TRUE RF 4FSK, not AFSK)
// ============================================================================
// 100 baud, 4FSK, 270 Hz shift between tones
// Direct frequency shifting for SDR reception

#define HORUS_BAUD_RATE    100      // 100 symbols/sec
#define HORUS_TONE_SPACING 270      // Hz between tones (standard Horus)
#define HORUS_SYMBOL_US    10000    // 1/100 baud = 10ms per symbol

// Horus v2 configuration
static bool horus_enabled = false;
static uint16_t horus_payload_id = 256;  // 256+ for custom payloads
static uint32_t horus_packet_count = 0;
static uint32_t horus_tx_count = 1;      // How many packets per interval

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

// ============================================================================
// HORUS BINARY v2 IMPLEMENTATION
// Based on horus_l2.c by David Rowe VK5DGR
// https://github.com/projecthorus/horusdemodlib/blob/master/src/horus_l2.c
// ============================================================================

// ============================================================================
// VSYS voltage measurement using ADC
// Pi Pico has VSYS connected to ADC3 (GPIO29) through 3:1 voltage divider
// ============================================================================
static bool adc_initialized = false;

static void init_vsys_adc(void) {
    if (!adc_initialized) {
        adc_init();
        adc_gpio_init(29);  // GPIO29 = ADC3 = VSYS/3
        adc_initialized = true;
    }
}

// Read VSYS voltage in volts (returns value like 3.7, 4.2, 5.0)
static float read_vsys_voltage(void) {
    init_vsys_adc();
    adc_select_input(3);  // ADC3 = VSYS/3
    
    // Average multiple readings for stability
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += adc_read();
    }
    uint16_t adc_raw = sum / 16;
    
    // ADC is 12-bit (0-4095), reference is 3.3V
    // VSYS is divided by 3 before ADC
    // So: VSYS = (adc_raw / 4095) * 3.3V * 3
    float vsys = (adc_raw / 4095.0f) * 3.3f * 3.0f;
    return vsys;
}

// Convert VSYS voltage to Horus battery field (0-255, decoded as value/51)
static uint8_t vsys_to_horus_battery(float vsys) {
    // Horus decoder: voltage = battery / 51.0
    // So: battery = voltage * 51
    int raw = (int)(vsys * 51.0f + 0.5f);
    if (raw < 0) raw = 0;
    if (raw > 255) raw = 255;
    return (uint8_t)raw;
}

// Read internal temperature sensor (RP2040 core temperature)
static float read_core_temperature(void) {
    init_vsys_adc();  // Ensure ADC is initialized
    adc_set_temp_sensor_enabled(true);
    adc_select_input(4);  // ADC4 = internal temperature sensor
    
    // Average multiple readings
    uint32_t sum = 0;
    for (int i = 0; i < 16; i++) {
        sum += adc_read();
    }
    uint16_t adc_raw = sum / 16;
    
    // Convert to temperature
    // T = 27 - (ADC_voltage - 0.706) / 0.001721
    float adc_voltage = (adc_raw / 4095.0f) * 3.3f;
    float temp_c = 27.0f - (adc_voltage - 0.706f) / 0.001721f;
    
    return temp_c;
}

// ============================================================================
// GPS ATGM336H - NMEA Parser
// ============================================================================

// GPS data structure
typedef struct {
    bool valid;              // GPS fix valid
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    float latitude;          // Decimal degrees, + = North
    float longitude;         // Decimal degrees, + = East
    uint16_t altitude;       // Meters
    uint8_t satellites;
    uint8_t speed_knots;     // Speed in knots
} gps_data_t;

static gps_data_t gps_data = {0};
static char nmea_buffer[128];
static uint8_t nmea_idx = 0;
static bool gps_initialized = false;

// Initialize GPS power control pins and UART
static void gps_init(void) {
    if (gps_initialized) return;
    
    // Setup GPS GND control pins (active LOW = GPS ON)
    gpio_init(GPS_GND_PIN1);
    gpio_init(GPS_GND_PIN2);
    gpio_init(GPS_GND_PIN3);
    gpio_set_dir(GPS_GND_PIN1, GPIO_OUT);
    gpio_set_dir(GPS_GND_PIN2, GPIO_OUT);
    gpio_set_dir(GPS_GND_PIN3, GPIO_OUT);
    
    // Turn GPS ON (GND pins LOW)
    gpio_put(GPS_GND_PIN1, 0);
    gpio_put(GPS_GND_PIN2, 0);
    gpio_put(GPS_GND_PIN3, 0);
    
    // Setup UART for GPS
    uart_init(GPS_UART, GPS_BAUD);
    gpio_set_function(GPS_UART_RX, GPIO_FUNC_UART);
    
    gps_initialized = true;
}

// Turn GPS power ON
static void gps_power_on(void) {
    gpio_put(GPS_GND_PIN1, 0);
    gpio_put(GPS_GND_PIN2, 0);
    gpio_put(GPS_GND_PIN3, 0);
}

// Turn GPS power OFF
static void gps_power_off(void) {
    gpio_put(GPS_GND_PIN1, 1);
    gpio_put(GPS_GND_PIN2, 1);
    gpio_put(GPS_GND_PIN3, 1);
}

// Parse NMEA latitude/longitude field (format: DDMM.MMMM or DDDMM.MMMM)
static float nmea_parse_coord(const char *str, int deg_digits) {
    if (!str || !*str) return 0.0f;
    
    char deg_str[4] = {0};
    strncpy(deg_str, str, deg_digits);
    int degrees = atoi(deg_str);
    
    float minutes = atof(str + deg_digits);
    return degrees + minutes / 60.0f;
}

// Parse NMEA time field (HHMMSS.sss)
static void nmea_parse_time(const char *str) {
    if (!str || strlen(str) < 6) return;
    
    char buf[3] = {0};
    buf[0] = str[0]; buf[1] = str[1];
    gps_data.hours = atoi(buf);
    
    buf[0] = str[2]; buf[1] = str[3];
    gps_data.minutes = atoi(buf);
    
    buf[0] = str[4]; buf[1] = str[5];
    gps_data.seconds = atoi(buf);
}

// Get next field from NMEA sentence (modifies string)
static char* nmea_next_field(char **ptr) {
    if (!ptr || !*ptr) return NULL;
    char *start = *ptr;
    char *comma = strchr(start, ',');
    if (comma) {
        *comma = '\0';
        *ptr = comma + 1;
    } else {
        *ptr = NULL;
    }
    return start;
}

// Parse GGA sentence (position + altitude + satellites)
// $GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,47.0,M,,*47
static void nmea_parse_gga(char *sentence) {
    char *ptr = sentence;
    char *field;
    
    // Skip $GPGGA,
    field = nmea_next_field(&ptr);  // $GPGGA
    
    // Time
    field = nmea_next_field(&ptr);
    nmea_parse_time(field);
    
    // Latitude
    field = nmea_next_field(&ptr);
    float lat = nmea_parse_coord(field, 2);
    
    // N/S
    field = nmea_next_field(&ptr);
    if (field && *field == 'S') lat = -lat;
    gps_data.latitude = lat;
    
    // Longitude
    field = nmea_next_field(&ptr);
    float lon = nmea_parse_coord(field, 3);
    
    // E/W
    field = nmea_next_field(&ptr);
    if (field && *field == 'W') lon = -lon;
    gps_data.longitude = lon;
    
    // Fix quality (0=invalid, 1=GPS, 2=DGPS)
    field = nmea_next_field(&ptr);
    int fix = field ? atoi(field) : 0;
    gps_data.valid = (fix > 0);
    
    // Number of satellites
    field = nmea_next_field(&ptr);
    gps_data.satellites = field ? atoi(field) : 0;
    
    // HDOP (skip)
    field = nmea_next_field(&ptr);
    
    // Altitude
    field = nmea_next_field(&ptr);
    gps_data.altitude = field ? (uint16_t)atof(field) : 0;
}

// Parse RMC sentence (position + speed + time)
// $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
static void nmea_parse_rmc(char *sentence) {
    char *ptr = sentence;
    char *field;
    
    // Skip $GPRMC,
    field = nmea_next_field(&ptr);  // $GPRMC
    
    // Time
    field = nmea_next_field(&ptr);
    nmea_parse_time(field);
    
    // Status (A=valid, V=invalid)
    field = nmea_next_field(&ptr);
    bool valid = (field && *field == 'A');
    
    // Latitude
    field = nmea_next_field(&ptr);
    float lat = nmea_parse_coord(field, 2);
    
    // N/S
    field = nmea_next_field(&ptr);
    if (field && *field == 'S') lat = -lat;
    
    // Longitude
    field = nmea_next_field(&ptr);
    float lon = nmea_parse_coord(field, 3);
    
    // E/W
    field = nmea_next_field(&ptr);
    if (field && *field == 'W') lon = -lon;
    
    // Speed in knots
    field = nmea_next_field(&ptr);
    uint8_t speed = field ? (uint8_t)atof(field) : 0;
    
    // Only update if valid
    if (valid) {
        gps_data.valid = true;
        gps_data.latitude = lat;
        gps_data.longitude = lon;
        gps_data.speed_knots = speed;
    }
}

// Process complete NMEA sentence
static void nmea_process_sentence(char *sentence) {
    // Check for GGA (has altitude and satellites)
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        nmea_parse_gga(sentence);
    }
    // Check for RMC (has speed)
    else if (strncmp(sentence, "$GPRMC", 6) == 0 || strncmp(sentence, "$GNRMC", 6) == 0) {
        nmea_parse_rmc(sentence);
    }
}

// Process incoming GPS data - call frequently
static void gps_process(void) {
    while (uart_is_readable(GPS_UART)) {
        char c = uart_getc(GPS_UART);
        
        if (c == '$') {
            // Start of new sentence
            nmea_idx = 0;
            nmea_buffer[nmea_idx++] = c;
        }
        else if (c == '\r' || c == '\n') {
            // End of sentence
            if (nmea_idx > 0) {
                nmea_buffer[nmea_idx] = '\0';
                nmea_process_sentence(nmea_buffer);
                nmea_idx = 0;
            }
        }
        else if (nmea_idx < sizeof(nmea_buffer) - 1) {
            nmea_buffer[nmea_idx++] = c;
        }
    }
}

// Get current GPS data (copies to provided structure)
static void gps_get_data(gps_data_t *data) {
    if (data) {
        *data = gps_data;
    }
}

// ============================================================================

// Golay (23,12) constants - polynomial is 0xC75 (AE3 reversed)
#define GOLAY_POLYNOMIAL 0xC75

// Golay syndrome calculator - exactly as in horusdemodlib
static int golay23_syndrome(int c) {
    int x;
    for (x = 11; x >= 0; x--) {
        if (c & ((1 << 11) << x)) {
            c ^= GOLAY_POLYNOMIAL << x;
        }
    }
    return c;
}

// CRC16 CCITT (same as used by Horus)
static uint16_t horus_crc16(uint8_t *data, uint8_t length) {
    uint8_t x;
    uint16_t crc = 0xFFFF;
    while (length--) {
        x = crc >> 8 ^ *data++;
        x ^= x >> 4;
        crc = (crc << 8) ^ ((uint16_t)(x << 12)) ^ ((uint16_t)(x << 5)) ^ ((uint16_t)x);
    }
    return crc;
}

// Prime numbers for algebraic interleaver - MUST match horusdemodlib exactly!
static const uint16_t primes[] = {
    2,      3,      5,      7,      11,     13,     17,     19,     23,     29, 
    31,     37,     41,     43,     47,     53,     59,     61,     67,     71, 
    73,     79,     83,     89,     97,     101,    103,    107,    109,    113, 
    127,    131,    137,    139,    149,    151,    157,    163,    167,    173, 
    179,    181,    191,    193,    197,    199,    211,    223,    227,    229, 
    233,    239,    241,    251,    257,    263,    269,    271,    277,    281, 
    283,    293,    307,    311,    313,    317,    331,    337,    347,    349,
    379,    383,    389,    757,    761,    769,    773
};

// Algebraic interleaver (in-place) - exactly as in horusdemodlib
// Uses LSB-first bit ordering within bytes
static void horus_interleave(uint8_t *inout, int nbytes) {
    uint16_t nbits = (uint16_t)nbytes * 8;
    uint32_t i, j, n, ibit, ibyte, ishift, jbyte, jshift;
    uint32_t b;
    uint8_t out[64];  // Max packet size
    
    memset(out, 0, nbytes);
    
    // Find nearest prime less than nbits (co-prime with nbits)
    i = 1;
    uint16_t imax = sizeof(primes) / sizeof(uint16_t);
    while ((primes[i] < nbits) && (i < imax))
        i++;
    b = primes[i - 1];
    
    for (n = 0; n < nbits; n++) {
        i = n;
        j = (b * i) % nbits;  // Must be 32-bit to avoid overflow
        
        // Read bit i (LSB-first within byte)
        ibyte = i / 8;
        ishift = i % 8;  // LSB-first
        ibit = (inout[ibyte] >> ishift) & 0x1;
        
        // Write to bit j position (LSB-first within byte)
        jbyte = j / 8;
        jshift = j % 8;  // LSB-first
        out[jbyte] |= ibit << jshift;
    }
    
    memcpy(inout, out, nbytes);
}

// DVB 16-bit additive scrambler (in-place) - exactly as in horusdemodlib
// Uses LSB-first bit ordering within bytes
static void horus_scramble(uint8_t *inout, int nbytes) {
    int nbits = nbytes * 8;
    int i, ibit, ibits, ibyte, ishift, mask;
    uint16_t scrambler = 0x4a80;  // Init at start of every frame
    uint16_t scrambler_out;
    
    for (i = 0; i < nbits; i++) {
        // Generate scrambler output bit
        scrambler_out = ((scrambler & 0x2) >> 1) ^ (scrambler & 0x1);
        
        // Modify i-th bit by XOR with scrambler output (LSB-first)
        ibyte = i / 8;
        ishift = i % 8;  // LSB-first
        ibit = (inout[ibyte] >> ishift) & 0x1;
        ibits = ibit ^ scrambler_out;
        
        // Clear and set bit
        mask = 1 << ishift;
        inout[ibyte] &= ~mask;
        inout[ibyte] |= ibits << ishift;
        
        // Update scrambler LFSR
        scrambler >>= 1;
        scrambler |= scrambler_out << 14;
    }
}

// Calculate number of TX data bytes for given payload size
static int horus_get_num_tx_bytes(int num_payload_bytes) {
    int num_payload_bits = num_payload_bytes * 8;
    int num_golay_codewords = num_payload_bits / 12;
    if (num_payload_bits % 12)
        num_golay_codewords++;
    
    int num_tx_bits = 2 * 8 + num_payload_bits + num_golay_codewords * 11;  // 2 = UW size
    int num_tx_bytes = num_tx_bits / 8;
    if (num_tx_bits % 8)
        num_tx_bytes++;
    
    return num_tx_bytes;
}

// Encode packet with Golay FEC, interleaving, scrambling
// Output includes unique word ($$) at start
// EXACTLY matches horusdemodlib/src/horus_l2.c horus_l2_encode_tx_packet()
static int horus_l2_encode_packet(uint8_t *output, uint8_t *input, int num_payload_bytes) {
    static const uint8_t uw[] = {'$', '$'};
    int num_tx_bytes = horus_get_num_tx_bytes(num_payload_bytes);
    uint8_t *pout = output;
    int ninbit, ningolay, nparitybits;
    int32_t ingolay, paritybyte, inbit, golayparity;
    int ninbyte, shift, golayparitybit, i;
    int num_payload_bits = num_payload_bytes * 8;
    
    // Copy unique word
    memcpy(pout, uw, sizeof(uw));
    pout += sizeof(uw);
    
    // Copy payload data
    memcpy(pout, input, num_payload_bytes);
    pout += num_payload_bytes;
    
    // Generate Golay parity bits
    // Read input bits one at a time, MSB first
    // Fill input Golay codeword, find output parity, write MSB first
    ninbit = 0;
    ingolay = 0;
    ningolay = 0;
    paritybyte = 0;
    nparitybits = 0;
    
    while (ninbit < num_payload_bits) {
        // Extract input data bit (MSB first)
        ninbyte = ninbit / 8;
        shift = 7 - (ninbit % 8);  // MSB first
        inbit = (input[ninbyte] >> shift) & 0x1;
        ninbit++;
        
        // Build up input golay codeword
        ingolay = ingolay | inbit;
        ningolay++;
        
        // When we get 12 bits do a Golay encode
        if (ningolay % 12) {
            ingolay <<= 1;
        } else {
            // Get parity for this 12-bit word
            golayparity = golay23_syndrome(ingolay << 11);
            ingolay = 0;
            
            // Write 11 parity bits to output data (MSB first)
            for (i = 0; i < 11; i++) {
                golayparitybit = (golayparity >> (10 - i)) & 0x1;
                paritybyte = paritybyte | golayparitybit;
                nparitybits++;
                if (nparitybits % 8) {
                    paritybyte <<= 1;
                } else {
                    // Full byte ready
                    *pout++ = (uint8_t)paritybyte;
                    paritybyte = 0;
                }
            }
        }
    }
    
    // Complete final Golay encode if we have partially finished ingolay
    if (ningolay % 12) {
        ingolay >>= 1;  // Remove extra shift from last iteration
        golayparity = golay23_syndrome(ingolay << 12);
        
        // Write parity bits to output data
        for (i = 0; i < 11; i++) {
            golayparitybit = (golayparity >> (10 - i)) & 0x1;
            paritybyte = paritybyte | golayparitybit;
            nparitybits++;
            if (nparitybits % 8) {
                paritybyte <<= 1;
            } else {
                // Full byte ready
                *pout++ = (uint8_t)paritybyte;
                paritybyte = 0;
            }
        }
    }
    
    // Final partial parity byte (use MS bits first)
    if (nparitybits % 8) {
        paritybyte <<= 7 - (nparitybits % 8);
        *pout++ = (uint8_t)paritybyte;
    }
    
    // Interleave (skip UW - don't interleave $$)
    horus_interleave(&output[sizeof(uw)], num_tx_bytes - 2);
    
    // Scramble (skip UW - don't scramble $$)
    horus_scramble(&output[sizeof(uw)], num_tx_bytes - 2);
    
    return num_tx_bytes;
}

// Horus v2 packet structure (32 bytes)
// Based on: https://github.com/projecthorus/horusdemodlib/wiki/4-Packet-Format-Details
typedef struct __attribute__((packed)) {
    uint16_t payload_id;     // 0-1
    uint16_t counter;        // 2-3
    uint8_t hours;           // 4
    uint8_t minutes;         // 5
    uint8_t seconds;         // 6
    float latitude;          // 7-10 (IEEE 754 float)
    float longitude;         // 11-14 (IEEE 754 float)
    uint16_t altitude;       // 15-16
    uint8_t speed;           // 17
    uint8_t sats;            // 18
    int8_t temperature;      // 19
    uint8_t battery;         // 20
    uint8_t custom[9];       // 21-29 (custom data)
    uint16_t checksum;       // 30-31
} horus_v2_packet_t;

// Telemetry data - defaults to zeros when no GPS fix
static horus_v2_packet_t horus_packet_data = {
    .payload_id = 256,
    .counter = 0,
    .hours = 0,
    .minutes = 0,
    .seconds = 0,
    .latitude = 0.0f,
    .longitude = 0.0f,
    .altitude = 0,
    .speed = 0,
    .sats = 0,
    .temperature = 20,
    .battery = 189,      // Will be updated with real VSYS measurement
    .custom = {0},
    .checksum = 0
};

// Build and encode Horus v2 packet
static int build_horus_v2_encoded(uint8_t *output) {
    // Update packet fields
    horus_packet_data.payload_id = horus_payload_id;
    horus_packet_data.counter = (uint16_t)(horus_packet_count & 0xFFFF);
    
    // Get GPS data
    gps_data_t gps;
    gps_get_data(&gps);
    
    // Update from GPS if valid
    if (gps.valid) {
        horus_packet_data.hours = gps.hours;
        horus_packet_data.minutes = gps.minutes;
        horus_packet_data.seconds = gps.seconds;
        horus_packet_data.latitude = gps.latitude;
        horus_packet_data.longitude = gps.longitude;
        horus_packet_data.altitude = gps.altitude;
        horus_packet_data.speed = gps.speed_knots;
        horus_packet_data.sats = gps.satellites;
    } else {
        // No GPS fix - use time since boot, zero coordinates
        uint32_t now_s = time_us_32() / 1000000;
        horus_packet_data.hours = (now_s / 3600) % 24;
        horus_packet_data.minutes = (now_s / 60) % 60;
        horus_packet_data.seconds = now_s % 60;
        horus_packet_data.latitude = 0.0f;
        horus_packet_data.longitude = 0.0f;
        horus_packet_data.altitude = 0;
        horus_packet_data.speed = 0;
        horus_packet_data.sats = 0;
    }
    
    // Read real VSYS voltage and convert to Horus format
    float vsys = read_vsys_voltage();
    horus_packet_data.battery = vsys_to_horus_battery(vsys);
    
    // Read core temperature
    float temp = read_core_temperature();
    horus_packet_data.temperature = (int8_t)temp;  // Truncate to integer
    
    // Calculate CRC over first 30 bytes
    horus_packet_data.checksum = horus_crc16((uint8_t*)&horus_packet_data, 30);
    
    // Encode with Golay FEC
    return horus_l2_encode_packet(output, (uint8_t*)&horus_packet_data, 32);
}

// ============================================================================
// TRUE 4FSK Transmission - Direct frequency control
// ============================================================================

// Set carrier frequency directly (for 4FSK)
static void horus_set_freq(uint32_t freq_hz) {
    uint32_t frf = (uint32_t)((double)freq_hz / FSTEP_HZ);
    sx_write(REG_FRF_MSB, (frf >> 16) & 0xFF);
    sx_write(REG_FRF_MID, (frf >> 8) & 0xFF);
    sx_write(REG_FRF_LSB, frf & 0xFF);
}

// Set FSK deviation in Hz
static void horus_set_fdev(uint16_t fdev_hz) {
    uint16_t fdev = (uint16_t)((float)fdev_hz / FSTEP_HZ);
    sx_write(REG_FDEV_MSB, (fdev >> 8) & 0x3F);
    sx_write(REG_FDEV_LSB, fdev & 0xFF);
}

// Setup radio for TRUE 4FSK - STDBY/TX switching (good power)
static void horus_tx_start(void) {
    // Stop any current transmission
    tone_off();
    
    // Go to sleep to change mode
    sx_write(REG_OP_MODE, MODE_SLEEP);
    sleep_ms(1);
    
    // FSK mode, standby
    sx_write(REG_OP_MODE, 0x01);
    sleep_ms(1);
    
    // PA config - same as init_radio
    sx_write(REG_PA_CONFIG, 0x8F);
    sx_write(REG_PA_RAMP, 0x09);
    
    // Zero deviation - pure carrier
    sx_write(REG_FDEV_MSB, 0x00);
    sx_write(REG_FDEV_LSB, 0x00);
    
    // Set initial frequency
    double corrected_freq = (double)base_freq_hz * (1.0 + ppm_correction / 1000000.0);
    horus_set_freq((uint32_t)corrected_freq);
}

// Stop 4FSK and restore FM mode for SSTV  
static void horus_tx_stop(void) {
    // Go to standby
    sx_write(REG_OP_MODE, MODE_STDBY);
    sleep_ms(1);
    
    // Full radio reinit to restore all settings properly
    init_radio();
}

// Transmit single 4FSK symbol - wait for mode ready, compensate timing
// Symbol 0 = base (lowest), 1 = +270Hz, 2 = +540Hz, 3 = +810Hz (highest)
__attribute__((noinline)) static void horus_tx_symbol(uint8_t symbol) {
    uint32_t start_time = time_us_32();
    
    // Calculate frequency for this symbol
    uint32_t base = (uint32_t)((double)base_freq_hz * (1.0 + ppm_correction / 1000000.0));
    uint32_t freq = base + (symbol & 0x03) * HORUS_TONE_SPACING;
    uint32_t frf = (uint32_t)((double)freq / FSTEP_HZ);
    
    // STDBY and wait for mode ready
    sx_write(REG_OP_MODE, MODE_STDBY);
    while (!(sx_read(0x3E) & 0x80)) {}  // Wait for ModeReady (RegIrqFlags1 bit 7)
    
    // Write FRF registers
    sx_write(REG_FRF_MSB, (frf >> 16) & 0xFF);
    sx_write(REG_FRF_MID, (frf >> 8) & 0xFF);
    sx_write(REG_FRF_LSB, frf & 0xFF);
    
    // TX and wait for mode ready
    sx_write(REG_OP_MODE, MODE_TX);
    while (!(sx_read(0x3E) & 0x80)) {}  // Wait for ModeReady
    
    // Calculate remaining time for this symbol
    uint32_t elapsed = time_us_32() - start_time;
    if (elapsed < HORUS_SYMBOL_US) {
        sleep_us(HORUS_SYMBOL_US - elapsed);
    }
}

// Send complete Horus v2 telemetry frame

void send_horus_telemetry(void) {
    // STATIC buffers to avoid stack issues
    static uint8_t tx_packet[72];
    static uint8_t raw_copy[32];  // Copy of raw data before encoding
    static char log_buf[512];
    static const char hex[] = "0123456789ABCDEF";
    
    // Copy raw payload BEFORE encoding (in case encoder modifies it)
    memcpy(raw_copy, (uint8_t*)&horus_packet_data, 32);
    
    // Encode packet
    int tx_len = build_horus_v2_encoded(tx_packet);
    
    // Build debug log: R=raw, E=final encoded
    char* p = log_buf;
    
    // RAW (32 bytes = 64 hex chars)
    *p++ = 'R'; *p++ = ':';
    for (int i = 0; i < 32; i++) {
        *p++ = hex[raw_copy[i] >> 4];
        *p++ = hex[raw_copy[i] & 0xF];
    }
    *p++ = '\n';
    
    // ENC final - full encoded packet
    *p++ = 'E'; *p++ = ':';
    for (int i = 0; i < tx_len; i++) {
        *p++ = hex[tx_packet[i] >> 4];
        *p++ = hex[tx_packet[i] & 0xF];
    }
    *p++ = '\n';
    
    msc_disk_overwrite_file("HORUS.TXT", (const uint8_t*)log_buf, p - log_buf);
    
    // Start 4FSK transmission (direct carrier frequency control)
    horus_tx_start();
    
    // Preamble: 32 bytes of 0x1B for FSK modem sync (longer for reliable lock)
    // 0x1B = 00 01 10 11 = symbols 0,1,2,3
    // MSB-first: send bits 7-6, then 5-4, then 3-2, then 1-0
    for (int i = 0; i < 32; i++) {
        // Unrolled loop - send all 4 symbols explicitly
        horus_tx_symbol(0);  // bits 7-6 = 00
        horus_tx_symbol(1);  // bits 5-4 = 01
        horus_tx_symbol(2);  // bits 3-2 = 10
        horus_tx_symbol(3);  // bits 1-0 = 11
    }
    
    // Transmit encoded packet (includes $$ unique word)
    // MSB-first: send bits 7-6, then 5-4, then 3-2, then 1-0
    for (int i = 0; i < tx_len; i++) {
        uint8_t byte = tx_packet[i];
        horus_tx_symbol((byte >> 6) & 0x03);
        horus_tx_symbol((byte >> 4) & 0x03);
        horus_tx_symbol((byte >> 2) & 0x03);
        horus_tx_symbol(byte & 0x03);
    }
    
    // Stop 4FSK transmission
    horus_tx_stop();
    horus_packet_count++;
}

// Update telemetry data (call this with real GPS/sensor data)
void horus_update_telemetry(uint8_t h, uint8_t m, uint8_t s,
                            float lat, float lon, uint16_t alt,
                            uint8_t spd, uint8_t sats,
                            int8_t temp, uint8_t batt) {
    horus_packet_data.hours = h;
    horus_packet_data.minutes = m;
    horus_packet_data.seconds = s;
    horus_packet_data.latitude = lat;
    horus_packet_data.longitude = lon;
    horus_packet_data.altitude = alt;
    horus_packet_data.speed = spd;
    horus_packet_data.sats = sats;
    horus_packet_data.temperature = temp;
    horus_packet_data.battery = batt;
}

// ============================================================================
// SX1276 functions
// ============================================================================
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
    
    // PA config - standard power for RA-02 module
    // 0x8F = PA_BOOST on, OutputPower=15 (max for PA_BOOST without +20dBm mode)
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
    // Note: GP2,GP3,GP4 are used for GPS GND control
    gpio_init(5);
    gpio_set_dir(5, GPIO_IN);
    gpio_pull_up(5);
    sleep_ms(10);
    
    // USB MSC init
    tusb_init();
    msc_disk_init();
    
    // If GP5 is grounded, force format the flash
    if (!gpio_get(5)) {
        printf("GP5 grounded - forcing flash format...\n");
        msc_disk_format();
        const char* cfg_name_fmt = "config.txt";
        const char* default_cfg_fmt = "freq=434.500\nppm=0.0\ninterval=3\nmode=robot36\nhorus=0\nhorus_id=256\nhorus_count=1\n";
        msc_disk_create_file_if_missing(cfg_name_fmt, (const uint8_t*)default_cfg_fmt, strlen(default_cfg_fmt));
        printf("Format complete. Remove GP5 jumper and reset.\n");
    }
    
    // Initialize GPS (ATGM336H)
    gps_init();

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
    const char* default_cfg = "freq=434.500\nppm=0.0\ninterval=3\nmode=robot36\nhorus=0\nhorus_id=256\nhorus_count=1\n";
    if (msc_disk_create_file_if_missing(cfg_name, (const uint8_t*)default_cfg, strlen(default_cfg))) {
    
    // Create HORUS.TXT for debug output (512 bytes for full packet dump)
    char horus_init[512];
    memset(horus_init, '.', 511);
    horus_init[511] = '\n';
    msc_disk_create_file_if_missing("HORUS.TXT", (const uint8_t*)horus_init, 512);
    
        uint32_t cfg_size;
        const uint8_t* cfg_data = msc_disk_find_file(cfg_name, &cfg_size);
        if (cfg_data && cfg_size > 0) {
            // Parse config: freq=xxx.xxx, ppm=x.x, interval=x, mode=robot36|pd120, horus=0|1, horus_id=N, horus_count=N
            bool config_valid = false;
            float freq_mhz = 0.0f;
            float ppm_val = 0.0f;
            uint32_t interval_val = 3;
            sstv_mode_t mode_val = SSTV_ROBOT36;
            bool horus_val = false;
            uint16_t horus_id_val = 256;
            uint32_t horus_count_val = 1;
            
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
                // Check for horus= (0 or 1)
                else if (strncmp(p, "horus=", 6) == 0) {
                    p += 6;
                    if (*p == '1') horus_val = true;
                    else horus_val = false;
                    while (p < end && *p != '\n' && *p != '\r') p++;
                }
                // Check for horus_id=
                else if (strncmp(p, "horus_id=", 9) == 0) {
                    p += 9;
                    horus_id_val = 0;
                    while (p < end && *p != '\n' && *p != '\r') {
                        if (*p >= '0' && *p <= '9') {
                            horus_id_val = horus_id_val * 10 + (*p - '0');
                        }
                        p++;
                    }
                }
                // Check for horus_count=
                else if (strncmp(p, "horus_count=", 12) == 0) {
                    p += 12;
                    horus_count_val = 0;
                    while (p < end && *p != '\n' && *p != '\r') {
                        if (*p >= '0' && *p <= '9') {
                            horus_count_val = horus_count_val * 10 + (*p - '0');
                        }
                        p++;
                    }
                    if (horus_count_val < 1) horus_count_val = 1;
                    if (horus_count_val > 10) horus_count_val = 10;
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
                horus_enabled = horus_val;
                horus_payload_id = horus_id_val;
                horus_tx_count = horus_count_val;
                printf("Config OK: freq=%.3f MHz, ppm=%.1f, interval=%lu s, mode=%s\n", 
                       freq_mhz, ppm_correction, tx_interval_sec,
                       sstv_mode == SSTV_PD120 ? "PD120" : "Robot36");
                if (horus_enabled) {
                    printf("Horus v2 ENABLED: ID=%u, %lu packets/cycle\n", 
                           horus_payload_id, horus_tx_count);
                }
            } else {
                // invalid config -> overwrite with default
                printf("Config invalid, overwriting with default\n");
                msc_disk_overwrite_file(cfg_name, (const uint8_t*)default_cfg, strlen(default_cfg));
                base_freq_hz = BASE_FREQ_HZ;
                ppm_correction = 0.0f;
                tx_interval_sec = 3;
                sstv_mode = SSTV_ROBOT36;
                horus_enabled = false;
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
    
    // ========================================
    // HORUS TEST MODE - nadaj 5 pakietów na start
    // ========================================
    printf("\n*** HORUS TEST MODE ***\n");
    printf("Sending 5 Horus packets for testing...\n");
    for (int test_pkt = 0; test_pkt < 5; test_pkt++) {
        printf("\n--- Test packet %d/5 ---\n", test_pkt + 1);
        
        // Process GPS data before sending
        gps_process();
        
        send_horus_telemetry();
        
        // 3 second pause between packets
        printf("Waiting 3s...\n");
        for (int d = 0; d < 300; d++) {
            tud_task();
            gps_process();
            sleep_ms(10);
        }
    }
    printf("*** HORUS TEST COMPLETE ***\n\n");
    // ========================================
    
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
        gps_process();  // Process GPS NMEA data
        
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
                        const char* default_cfg2 = "freq=434.500\nppm=0.0\ninterval=3\nmode=robot36\nhorus=0\nhorus_id=256\nhorus_count=1\n";
                        msc_disk_create_file_if_missing(cfg_name2, (const uint8_t*)default_cfg2, strlen(default_cfg2));
                        base_freq_hz = BASE_FREQ_HZ;
                        ppm_correction = 0.0f;
                        tx_interval_sec = 3;
                        sstv_mode = SSTV_ROBOT36;
                        horus_enabled = false;
                        printf("Format complete; default config restored.\n");
                    }
                }
            }
        }
        
        // Transmit if image loaded, then wait
        if (image_loaded) {
            // Send Horus telemetry BEFORE SSTV image (if enabled)
            if (horus_enabled) {
                printf("\n--- Horus v2 telemetry (%lu packets) ---\n", horus_tx_count);
                for (uint32_t h = 0; h < horus_tx_count; h++) {
                    // Process GPS data before each packet
                    gps_process();
                    
                    send_horus_telemetry();
                    
                    // Small delay between packets
                    if (h + 1 < horus_tx_count) {
                        for (int d = 0; d < 100; d++) {
                            tud_task();
                            gps_process();
                            sleep_ms(10);
                        }
                    }
                }
                printf("--- Horus done, 2s pause before SSTV ---\n");
                
                // 2 second delay between Horus and SSTV
                for (int d = 0; d < 200; d++) {
                    tud_task();
                    gps_process();
                    sleep_ms(10);
                }
            }
            
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

            // Wait tx_interval_sec seconds, but keep USB and GPS alive
            for (uint32_t i = 0; i < tx_interval_sec * 100; i++) {
                tud_task();
                gps_process();
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
