// MSC Flash Disk for SSTV images
// Uses last 256KB of Pico's 2MB flash as FAT12 filesystem

#include "tusb.h"
#include "msc_disk.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"
#include <ctype.h>
#include <string.h>
#include <stdio.h>

// Flash storage: last 256KB of 2MB flash
#define DISK_BLOCK_SIZE   512
#define DISK_BLOCK_NUM    512  // 256KB total

// Flash: 2MB starting at XIP_BASE (0x10000000)
// Program uses first ~100KB, we use last 256KB
// Offset: 2MB - 256KB = 1792KB = 0x1C0000
#define FLASH_STORAGE_OFFSET  (1792 * 1024)

// Direct read pointer to flash
static const uint8_t* flash_storage = (const uint8_t*)(XIP_BASE + FLASH_STORAGE_OFFSET);

static volatile bool disk_changed = false;

// Write buffer - flash must be written in 4KB sectors
static uint8_t sector_buf[FLASH_SECTOR_SIZE];

// Flush pending writes (write whole 4KB sector to flash)
static void flush_sector(uint32_t flash_sector_offset) {
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_STORAGE_OFFSET + flash_sector_offset, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_STORAGE_OFFSET + flash_sector_offset, sector_buf, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}

// Check if filesystem needs formatting
static bool needs_format(void) {
    // Check for valid boot sector signature
    return (flash_storage[510] != 0x55 || flash_storage[511] != 0xAA);
}

// Format the flash filesystem
void msc_disk_format(void) {
    printf("Formatting flash disk...\n");
    
    // Prepare boot sector
    memset(sector_buf, 0, FLASH_SECTOR_SIZE);
    
    uint8_t* boot = sector_buf;
    boot[0] = 0xEB; boot[1] = 0x3C; boot[2] = 0x90;  // Jump
    memcpy(&boot[3], "MSDOS5.0", 8);                  // OEM
    boot[11] = 0x00; boot[12] = 0x02;                 // Bytes per sector: 512
    boot[13] = 1;                                      // Sectors per cluster
    boot[14] = 1; boot[15] = 0;                       // Reserved sectors
    boot[16] = 2;                                      // Number of FATs
    boot[17] = 0x10; boot[18] = 0x00;                 // Root entries: 16
    boot[19] = 0x00; boot[20] = 0x02;                 // Total sectors: 512
    boot[21] = 0xF8;                                   // Media type: fixed disk
    boot[22] = 4; boot[23] = 0;                       // Sectors per FAT: 4
    boot[24] = 1; boot[25] = 0;                       // Sectors per track
    boot[26] = 1; boot[27] = 0;                       // Heads
    boot[510] = 0x55; boot[511] = 0xAA;               // Signature
    
    // FAT1 at sector 1
    uint8_t* fat1 = sector_buf + 512;
    fat1[0] = 0xF8;
    fat1[1] = 0xFF;
    fat1[2] = 0xFF;
    
    // Write first 4KB sector (boot + FAT1 start)
    flush_sector(0);
    
    // FAT2 at sector 5 + root directory at sector 9
    memset(sector_buf, 0, FLASH_SECTOR_SIZE);
    
    // FAT2 at offset 0 in this sector (sector 4-7 in second 4KB)
    uint8_t* fat2 = sector_buf + 512;  // sector 5
    fat2[0] = 0xF8;
    fat2[1] = 0xFF;
    fat2[2] = 0xFF;
    
    // Write second 4KB sector
    flush_sector(FLASH_SECTOR_SIZE);
    
    // Root directory at sector 9
    memset(sector_buf, 0, FLASH_SECTOR_SIZE);
    uint8_t* root = sector_buf + 512;  // sector 9 in third 4KB (sectors 8-11)
    memcpy(root, "SSTV       ", 11);
    root[11] = 0x08;  // Volume label
    
    // Write third 4KB sector
    flush_sector(2 * FLASH_SECTOR_SIZE);
    
    printf("Format complete. 256KB disk ready.\n");
}

void msc_disk_init(void) {
    // Do NOT format on startup anymore - keep existing contents.
    // The user requested that flash is not wiped automatically.
    disk_changed = false;
}

// Helper: copy 8.3 name into a C string with dot (NUL terminated)
static void make_name(const uint8_t* entry, char* out, int out_len) {
    // Name: entry[0..7], ext: entry[8..10]
    int p = 0;
    for (int i = 0; i < 8 && p < out_len-1; i++) {
        char c = entry[i];
        if (c == ' ') break;
        out[p++] = c;
    }
    // extension
    if (entry[8] != ' ') {
        if (p < out_len-1) out[p++] = '.';
        for (int i = 8; i < 11 && p < out_len-1; i++) {
            char c = entry[i];
            if (c == ' ') break;
            out[p++] = c;
        }
    }
    out[p] = '\0';
}

// Case-insensitive ASCII compare (simple)
static int ascii_casecmp(const char* a, const char* b) {
    while (*a && *b) {
        char ca = *a; if (ca >= 'A' && ca <= 'Z') ca = ca - 'A' + 'a';
        char cb = *b; if (cb >= 'A' && cb <= 'Z') cb = cb - 'A' + 'a';
        if (ca != cb) return (ca < cb) ? -1 : 1;
        a++; b++;
    }
    if (*a) return 1;
    if (*b) return -1;
    return 0;
}

int msc_disk_list_jpegs(ImageEntry* out_entries, int max_entries) {
    const uint8_t* root = flash_storage + (9 * DISK_BLOCK_SIZE);
    int count = 0;
    for (int i = 0; i < 16 && count < max_entries; i++) {
        const uint8_t* entry = root + (i * 32);
        if (entry[0] == 0x00 || entry[0] == 0xE5) continue;
        if (entry[11] & 0x18) continue; // skip dirs/volume

        char a = entry[8]; char b = entry[9]; char c = entry[10];
        bool is_jpg = false;
        if ((a=='J' || a=='j') && (b=='P' || b=='p') && (c=='G' || c=='g')) is_jpg = true;
        if ((a=='J' || a=='j') && (b=='P' || b=='p') && (c=='E' || c=='e')) is_jpg = true; // .JPE
        if (!is_jpg) continue;

        // Fill entry
        ImageEntry ie;
        make_name(entry, ie.name, sizeof(ie.name));
        ie.cluster = entry[26] | (entry[27] << 8);
        ie.size = entry[28] | (entry[29] << 8) | (entry[30] << 16) | (entry[31] << 24);
        if (ie.cluster < 2) continue;
        out_entries[count++] = ie;
    }

    // Sort alphabetically (case-insensitive)
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            if (ascii_casecmp(out_entries[i].name, out_entries[j].name) > 0) {
                ImageEntry tmp = out_entries[i];
                out_entries[i] = out_entries[j];
                out_entries[j] = tmp;
            }
        }
    }

    return count;
}

const uint8_t* msc_disk_get_jpeg_by_index(int index, uint32_t* size_out) {
    ImageEntry entries[16];
    int n = msc_disk_list_jpegs(entries, 16);
    if (index < 0 || index >= n) return NULL;
    uint16_t cluster = entries[index].cluster;
    if (size_out) *size_out = entries[index].size;
    uint32_t sector = 10 + (cluster - 2);
    return flash_storage + (sector * DISK_BLOCK_SIZE);
}

// Helper: case-insensitive compare of generated 8.3 name and provided name
static bool name_matches_entry(const char* want, const uint8_t* entry) {
    char gen[13];
    make_name(entry, gen, sizeof(gen));
    return (ascii_casecmp(gen, want) == 0);
}

const uint8_t* msc_disk_find_file(const char* name, uint32_t* size_out) {
    const uint8_t* root = flash_storage + (9 * DISK_BLOCK_SIZE);
    for (int i = 0; i < 16; i++) {
        const uint8_t* entry = root + (i * 32);
        if (entry[0] == 0x00 || entry[0] == 0xE5) continue;
        if (entry[11] & 0x18) continue;
        if (name_matches_entry(name, entry)) {
            uint16_t cluster = entry[26] | (entry[27] << 8);
            uint32_t filesize = entry[28] | (entry[29] << 8) |
                               (entry[30] << 16) | (entry[31] << 24);
            if (size_out) *size_out = filesize;
            if (cluster >= 2) {
                uint32_t sector = 10 + (cluster - 2);
                return flash_storage + (sector * DISK_BLOCK_SIZE);
            }
        }
    }
    return NULL;
}

// Create a simple file with given name and data in the last 4KB sector if missing.
bool msc_disk_create_file_if_missing(const char* name, const uint8_t* data, uint32_t len) {
    // Return if exists
    uint32_t existing_size;
    if (msc_disk_find_file(name, &existing_size)) return true;

    // Find a free root entry in the root sector (LBA 9)
    uint32_t root_lba = 9;
    uint32_t flash_sector_index = (root_lba * DISK_BLOCK_SIZE) / FLASH_SECTOR_SIZE; // should be 1
    uint32_t flash_sector_offset = flash_sector_index * FLASH_SECTOR_SIZE;

    // Copy sector into sector_buf
    memcpy(sector_buf, flash_storage + flash_sector_offset, FLASH_SECTOR_SIZE);

    uint8_t* root = sector_buf + ((root_lba - flash_sector_index * (FLASH_SECTOR_SIZE / DISK_BLOCK_SIZE)) * DISK_BLOCK_SIZE);
    int free_idx = -1;
    for (int i = 0; i < 16; i++) {
        uint8_t* entry = root + (i * 32);
        if (entry[0] == 0x00 || entry[0] == 0xE5) { free_idx = i; break; }
    }
    if (free_idx < 0) return false;

    // Prepare filename in 8.3 format (uppercase, padded)
    char upname[12];
    int p = 0;
    // Split input name at dot
    const char* dot = strchr(name, '.');
    int namelen = dot ? (dot - name) : strlen(name);
    int extlen = dot ? (int)strlen(dot+1) : 0;
    memset(upname, ' ', sizeof(upname));
    for (int i = 0; i < namelen && i < 8; i++) upname[i] = toupper((unsigned char)name[i]);
    for (int i = 0; i < extlen && i < 3; i++) upname[8 + i] = toupper((unsigned char)dot[1 + i]);

    // Prepare directory entry
    uint8_t* entry = root + (free_idx * 32);
    memcpy(entry, upname, 11);
    entry[11] = 0x20; // archive

    // Choose last available data sector: use last flash sector
    int last_flash_sector = (DISK_BLOCK_NUM * DISK_BLOCK_SIZE) / FLASH_SECTOR_SIZE - 1; // 63
    uint32_t data_lba = last_flash_sector * (FLASH_SECTOR_SIZE / DISK_BLOCK_SIZE);
    // data_lba now is starting LBA for that flash sector (e.g., 504)
    uint16_t cluster = 2 + (data_lba - 10);
    entry[26] = cluster & 0xFF;
    entry[27] = (cluster >> 8) & 0xFF;
    entry[28] = (len & 0xFF);
    entry[29] = (len >> 8) & 0xFF;
    entry[30] = (len >> 16) & 0xFF;
    entry[31] = (len >> 24) & 0xFF;

    // Write data to the chosen flash sector
    uint32_t data_flash_offset = last_flash_sector * FLASH_SECTOR_SIZE;
    uint8_t data_sector[FLASH_SECTOR_SIZE];
    // Fill with 0xFF
    memset(data_sector, 0xFF, FLASH_SECTOR_SIZE);
    if (len > FLASH_SECTOR_SIZE) return false; // too big
    memcpy(data_sector, data, len);

    // Program data sector
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_STORAGE_OFFSET + data_flash_offset, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_STORAGE_OFFSET + data_flash_offset, data_sector, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);

    // Program root sector back
    flush_sector(flash_sector_offset);

    return true;
}

bool msc_disk_overwrite_file(const char* name, const uint8_t* data, uint32_t len) {
    // Find existing entry
    const uint8_t* root_flash = flash_storage + (9 * DISK_BLOCK_SIZE);
    int found = -1;
    uint16_t cluster = 0;
    for (int i = 0; i < 16; i++) {
        const uint8_t* entry = root_flash + (i * 32);
        if (entry[0] == 0x00 || entry[0] == 0xE5) continue;
        if (entry[11] & 0x18) continue;
        if (name_matches_entry(name, entry)) {
            found = i;
            cluster = entry[26] | (entry[27] << 8);
            break;
        }
    }
    if (found < 0) return false;
    if (len > FLASH_SECTOR_SIZE) return false;

    // Calculate flash sector offset to write
    uint32_t sector = 10 + (cluster - 2);
    uint32_t flash_sector_index = (sector * DISK_BLOCK_SIZE) / FLASH_SECTOR_SIZE;
    uint32_t flash_sector_offset = flash_sector_index * FLASH_SECTOR_SIZE;

    // Prepare data sector buffer
    uint8_t data_sector[FLASH_SECTOR_SIZE];
    memset(data_sector, 0xFF, FLASH_SECTOR_SIZE);
    memcpy(data_sector, data, len);

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(FLASH_STORAGE_OFFSET + flash_sector_offset, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_STORAGE_OFFSET + flash_sector_offset, data_sector, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);

    return true;
}

bool msc_disk_has_new_image(void) {
    bool ret = disk_changed;
    disk_changed = false;
    return ret;
}

// Find BMP in root directory, return flash pointer
const uint8_t* msc_disk_find_bmp(uint32_t* size_out) {
    // Root directory at sector 9
    const uint8_t* root = flash_storage + (9 * DISK_BLOCK_SIZE);
    
    for (int i = 0; i < 16; i++) {
        const uint8_t* entry = root + (i * 32);
        
        if (entry[0] == 0x00 || entry[0] == 0xE5) continue;
        if (entry[11] & 0x18) continue;  // Skip dirs/volume labels
        
        // Check .BMP extension
        if (entry[8] == 'B' && entry[9] == 'M' && entry[10] == 'P') {
            uint16_t cluster = entry[26] | (entry[27] << 8);
            uint32_t filesize = entry[28] | (entry[29] << 8) | 
                               (entry[30] << 16) | (entry[31] << 24);
            
            if (size_out) *size_out = filesize;
            
            // Data area starts at sector 10, cluster 2 = sector 10
            if (cluster >= 2) {
                uint32_t sector = 10 + (cluster - 2);
                return flash_storage + (sector * DISK_BLOCK_SIZE);
            }
        }
    }
    return NULL;
}

// Find JPG/JPEG in root directory and return pointer to data in flash
const uint8_t* msc_disk_find_jpeg(uint32_t* size_out) {
    const uint8_t* root = flash_storage + (9 * DISK_BLOCK_SIZE);
    for (int i = 0; i < 16; i++) {
        const uint8_t* entry = root + (i * 32);
        if (entry[0] == 0x00 || entry[0] == 0xE5) continue;
        if (entry[11] & 0x18) continue;
        // check extension for JPG/JPE (case-insensitive)
        char a = entry[8]; char b = entry[9]; char c = entry[10];
        if ((a=='J' || a=='j') && (b=='P' || b=='p') && (c=='G' || c=='g')) {
            uint16_t cluster = entry[26] | (entry[27] << 8);
            uint32_t filesize = entry[28] | (entry[29] << 8) | 
                               (entry[30] << 16) | (entry[31] << 24);
            if (size_out) *size_out = filesize;
            if (cluster >= 2) {
                uint32_t sector = 10 + (cluster - 2);
                return flash_storage + (sector * DISK_BLOCK_SIZE);
            }
        }
        if ((a=='J' || a=='j') && (b=='P' || b=='p') && (c=='E' || c=='e')) {
            // .JPE extension
            uint16_t cluster = entry[26] | (entry[27] << 8);
            uint32_t filesize = entry[28] | (entry[29] << 8) | 
                               (entry[30] << 16) | (entry[31] << 24);
            if (size_out) *size_out = filesize;
            if (cluster >= 2) {
                uint32_t sector = 10 + (cluster - 2);
                return flash_storage + (sector * DISK_BLOCK_SIZE);
            }
        }
    }
    return NULL;
}

const uint8_t* msc_disk_get_raw(void) {
    return flash_storage;
}

//--------------------------------------------------------------------+
// TinyUSB MSC Callbacks
//--------------------------------------------------------------------+

void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8], 
                        uint8_t product_id[16], uint8_t product_rev[4]) {
    (void)lun;
    memcpy(vendor_id, "SP8ESA  ", 8);
    memcpy(product_id, "SSTV Flash      ", 16);
    memcpy(product_rev, "1.0 ", 4);
}

bool tud_msc_test_unit_ready_cb(uint8_t lun) {
    (void)lun;
    return true;
}

void tud_msc_capacity_cb(uint8_t lun, uint32_t* block_count, uint16_t* block_size) {
    (void)lun;
    *block_count = DISK_BLOCK_NUM;
    *block_size = DISK_BLOCK_SIZE;
}

bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition, 
                           bool start, bool load_eject) {
    (void)lun; (void)power_condition; (void)start; (void)load_eject;
    return true;
}

// Read from flash - direct memory access
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                          void* buffer, uint32_t bufsize) {
    (void)lun;
    if (lba >= DISK_BLOCK_NUM) return -1;
    
    const uint8_t* addr = flash_storage + (lba * DISK_BLOCK_SIZE) + offset;
    memcpy(buffer, addr, bufsize);
    return bufsize;
}

// Write to flash - need to buffer and write whole sectors
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                           uint8_t* buffer, uint32_t bufsize) {
    (void)lun;
    if (lba >= DISK_BLOCK_NUM) return -1;
    
    // Calculate which 4KB flash sector this belongs to
    uint32_t flash_sector = (lba * DISK_BLOCK_SIZE) / FLASH_SECTOR_SIZE;
    uint32_t flash_offset = flash_sector * FLASH_SECTOR_SIZE;
    
    // Read current sector content
    memcpy(sector_buf, flash_storage + flash_offset, FLASH_SECTOR_SIZE);
    
    // Modify the part being written
    uint32_t buf_offset = (lba * DISK_BLOCK_SIZE) - flash_offset + offset;
    memcpy(sector_buf + buf_offset, buffer, bufsize);
    
    // Write back to flash
    flush_sector(flash_offset);
    
    disk_changed = true;
    return bufsize;
}

int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16],
                        void* buffer, uint16_t bufsize) {
    (void)lun; (void)buffer; (void)bufsize;
    
    switch (scsi_cmd[0]) {
        default:
            tud_msc_set_sense(lun, SCSI_SENSE_ILLEGAL_REQUEST, 0x20, 0x00);
            return -1;
    }
}
