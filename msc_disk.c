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
// FAT12 layout for 512 sectors (256KB):
// - Sector 0: Boot sector
// - Sectors 1-4: FAT1 (4 sectors)
// - Sectors 5-8: FAT2 (4 sectors)
// - Sector 9: Root directory (16 entries = 512 bytes)
// - Sectors 10-511: Data area (502 clusters, cluster 2 = sector 10)
void msc_disk_format(void) {
    printf("Formatting flash disk...\n");
    
    // === FLASH SECTOR 0 (LBA 0-7) ===
    memset(sector_buf, 0, FLASH_SECTOR_SIZE);
    
    // Boot sector at LBA 0
    uint8_t* boot = sector_buf;
    boot[0] = 0xEB; boot[1] = 0x3C; boot[2] = 0x90;  // Jump instruction
    memcpy(&boot[3], "MSDOS5.0", 8);                  // OEM name
    boot[11] = 0x00; boot[12] = 0x02;                 // Bytes per sector: 512
    boot[13] = 1;                                      // Sectors per cluster: 1
    boot[14] = 1; boot[15] = 0;                       // Reserved sectors: 1
    boot[16] = 2;                                      // Number of FATs: 2
    boot[17] = 0x10; boot[18] = 0x00;                 // Root entries: 16
    boot[19] = 0x00; boot[20] = 0x02;                 // Total sectors: 512
    boot[21] = 0xF8;                                   // Media type: fixed disk
    boot[22] = 4; boot[23] = 0;                       // Sectors per FAT: 4
    boot[24] = 1; boot[25] = 0;                       // Sectors per track: 1
    boot[26] = 1; boot[27] = 0;                       // Heads: 1
    boot[28] = 0; boot[29] = 0; boot[30] = 0; boot[31] = 0; // Hidden sectors: 0
    boot[510] = 0x55; boot[511] = 0xAA;               // Boot signature
    
    // FAT1 starts at LBA 1 (offset 512 in this 4KB sector)
    // FAT12 initial entries: F8 FF FF (media descriptor + cluster 0/1 reserved)
    uint8_t* fat1 = sector_buf + (1 * 512);  // LBA 1
    fat1[0] = 0xF8;  // Media descriptor
    fat1[1] = 0xFF;  // Cluster 0 reserved (low nibble) + cluster 1 reserved (high nibble)
    fat1[2] = 0xFF;  // Cluster 1 reserved cont.
    // Rest of FAT1 is 0x00 (free clusters)
    
    // FAT1 continues in LBA 2,3,4 (all zeros = free)
    // FAT2 starts at LBA 5 (offset 2560 in this 4KB sector)
    uint8_t* fat2 = sector_buf + (5 * 512);  // LBA 5
    fat2[0] = 0xF8;
    fat2[1] = 0xFF;
    fat2[2] = 0xFF;
    // FAT2 continues in LBA 6,7 (remaining in this sector, all zeros)
    
    // Write first 4KB sector (LBA 0-7: boot + FAT1 + FAT2 start)
    flush_sector(0);
    
    // === FLASH SECTOR 1 (LBA 8-15) ===
    memset(sector_buf, 0, FLASH_SECTOR_SIZE);
    
    // LBA 8 is last sector of FAT2 (zeros = free)
    // LBA 9 is root directory
    uint8_t* root = sector_buf + (1 * 512);  // LBA 9 = offset 512 in this sector
    memcpy(root, "SSTV       ", 11);  // Volume label
    root[11] = 0x08;  // Attribute: volume label
    
    // Create IMAGES subdirectory entry (second entry in root)
    // Directory will be stored in cluster 2 (LBA 10)
    uint8_t* images_entry = root + 32;  // Second directory entry
    memcpy(images_entry, "IMAGES     ", 11);  // Directory name (8.3 format, no extension)
    images_entry[11] = 0x10;  // Attribute: directory
    images_entry[26] = 2;     // Start cluster: 2 (low byte)
    images_entry[27] = 0;     // Start cluster: 2 (high byte)
    // Size is 0 for directories
    
    // LBA 10 is cluster 2 - will contain IMAGES directory entries
    // Initialize with . and .. entries
    uint8_t* images_dir = sector_buf + (2 * 512);  // LBA 10 = offset 1024 in this sector
    
    // "." entry (self-reference)
    memcpy(images_dir, ".          ", 11);
    images_dir[11] = 0x10;  // Directory
    images_dir[26] = 2;     // Cluster 2
    images_dir[27] = 0;
    
    // ".." entry (parent = root, cluster 0)
    uint8_t* dotdot = images_dir + 32;
    memcpy(dotdot, "..         ", 11);
    dotdot[11] = 0x10;  // Directory
    dotdot[26] = 0;     // Root directory = cluster 0
    dotdot[27] = 0;
    
    // Rest of IMAGES directory is empty (zeros)
    
    // LBA 11-15 are data sectors (cluster 3-7)
    
    // Write second 4KB sector (LBA 8-15: FAT2 end + root + IMAGES dir + data)
    flush_sector(FLASH_SECTOR_SIZE);
    
    // Now update FAT to mark cluster 2 as end-of-chain (0xFFF) for IMAGES directory
    // Read back first flash sector (contains FAT1 and FAT2)
    memcpy(sector_buf, flash_storage, FLASH_SECTOR_SIZE);
    
    // FAT1 at offset 512, cluster 2 is at byte offset (2*3/2) = 3
    // Cluster 2 is even, so: byte[3] = low 8 bits, byte[4] low nibble = high 4 bits
    uint8_t* fat1_c2 = sector_buf + 512 + 3;
    fat1_c2[0] = 0xFF;  // Low 8 bits of 0xFFF
    fat1_c2[1] = (fat1_c2[1] & 0xF0) | 0x0F;  // High 4 bits of 0xFFF
    
    // FAT2 at offset 2560
    uint8_t* fat2_c2 = sector_buf + 2560 + 3;
    fat2_c2[0] = 0xFF;
    fat2_c2[1] = (fat2_c2[1] & 0xF0) | 0x0F;
    
    // Write back FAT sector
    flush_sector(0);
    
    printf("Format complete. 256KB disk ready.\n");
    printf("  FAT12: 512 sectors, 501 data clusters available\n");
    printf("  Created /IMAGES/ directory for JPEG files\n");
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

// Helper: Find IMAGES directory cluster from root directory
static uint16_t find_images_dir_cluster(void) {
    const uint8_t* root = flash_storage + (9 * DISK_BLOCK_SIZE);
    for (int i = 0; i < 16; i++) {
        const uint8_t* entry = root + (i * 32);
        if (entry[0] == 0x00 || entry[0] == 0xE5) continue;
        if ((entry[11] & 0x10) == 0) continue;  // Not a directory
        
        // Check if name is "IMAGES     "
        if (memcmp(entry, "IMAGES     ", 11) == 0) {
            return entry[26] | (entry[27] << 8);
        }
    }
    return 0;  // Not found
}

// List JPEG files from /IMAGES/ subdirectory
// Subdirectory can have many more entries than root (limited only by cluster chain)
int msc_disk_list_jpegs(ImageEntry* out_entries, int max_entries) {
    // Find IMAGES directory
    uint16_t images_cluster = find_images_dir_cluster();
    
    // Fallback: if no IMAGES dir, search root directory (backward compatibility)
    if (images_cluster == 0) {
        const uint8_t* root = flash_storage + (9 * DISK_BLOCK_SIZE);
        int count = 0;
        for (int i = 0; i < 16 && count < max_entries; i++) {
            const uint8_t* entry = root + (i * 32);
            if (entry[0] == 0x00 || entry[0] == 0xE5) continue;
            if (entry[11] & 0x18) continue;

            char a = entry[8]; char b = entry[9]; char c = entry[10];
            bool is_jpg = false;
            if ((a=='J' || a=='j') && (b=='P' || b=='p') && (c=='G' || c=='g')) is_jpg = true;
            if ((a=='J' || a=='j') && (b=='P' || b=='p') && (c=='E' || c=='e')) is_jpg = true;
            if (!is_jpg) continue;

            ImageEntry ie;
            make_name(entry, ie.name, sizeof(ie.name));
            ie.cluster = entry[26] | (entry[27] << 8);
            ie.size = entry[28] | (entry[29] << 8) | (entry[30] << 16) | (entry[31] << 24);
            if (ie.cluster < 2) continue;
            out_entries[count++] = ie;
        }
        // Sort
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
    
    // Read IMAGES directory from its cluster(s)
    // For simplicity, assume directory fits in one cluster (16 entries per 512-byte cluster)
    // But we follow the FAT chain to support larger directories
    int count = 0;
    uint16_t cluster = images_cluster;
    
    while (cluster >= 2 && cluster < 0xFF8 && count < max_entries) {
        // Calculate sector for this cluster
        uint32_t sector = 10 + (cluster - 2);
        const uint8_t* dir_data = flash_storage + (sector * DISK_BLOCK_SIZE);
        
        // Each cluster has 16 directory entries (512 bytes / 32 bytes per entry)
        for (int i = 0; i < 16 && count < max_entries; i++) {
            const uint8_t* entry = dir_data + (i * 32);
            
            // End of directory
            if (entry[0] == 0x00) {
                cluster = 0xFFF;  // Stop scanning
                break;
            }
            
            // Deleted or special entry
            if (entry[0] == 0xE5) continue;
            if (entry[0] == '.') continue;  // Skip . and ..
            if (entry[11] & 0x18) continue; // Skip subdirs and volume labels

            // Check for JPEG extension
            char a = entry[8]; char b = entry[9]; char c = entry[10];
            bool is_jpg = false;
            if ((a=='J' || a=='j') && (b=='P' || b=='p') && (c=='G' || c=='g')) is_jpg = true;
            if ((a=='J' || a=='j') && (b=='P' || b=='p') && (c=='E' || c=='e')) is_jpg = true;
            if (!is_jpg) continue;

            // Fill entry
            ImageEntry ie;
            make_name(entry, ie.name, sizeof(ie.name));
            ie.cluster = entry[26] | (entry[27] << 8);
            ie.size = entry[28] | (entry[29] << 8) | (entry[30] << 16) | (entry[31] << 24);
            if (ie.cluster < 2) continue;
            out_entries[count++] = ie;
        }
        
        // Follow FAT chain to next cluster
        if (cluster < 0xFF8) {
            const uint8_t* fat1 = flash_storage + (1 * 512);
            uint32_t fat_offset = (cluster * 3) / 2;
            uint16_t next;
            if (cluster & 1) {
                next = ((fat1[fat_offset] >> 4) & 0x0F) | (fat1[fat_offset + 1] << 4);
            } else {
                next = fat1[fat_offset] | ((fat1[fat_offset + 1] & 0x0F) << 8);
            }
            cluster = next;
        }
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

// Helper: Read FAT12 entry for a cluster
static uint16_t get_fat12_entry(uint16_t cluster) {
    if (cluster < 2) return 0xFFF;  // Reserved
    
    const uint8_t* fat1 = flash_storage + (1 * 512);  // FAT1 at LBA 1
    uint32_t fat_byte_offset = (cluster * 3) / 2;
    
    if (cluster & 1) {
        // Odd cluster
        return ((fat1[fat_byte_offset] >> 4) & 0x0F) | 
               (fat1[fat_byte_offset + 1] << 4);
    } else {
        // Even cluster
        return fat1[fat_byte_offset] | 
               ((fat1[fat_byte_offset + 1] & 0x0F) << 8);
    }
}

// Check if file clusters are contiguous (for files spanning multiple clusters)
static bool clusters_contiguous(uint16_t start_cluster, uint32_t file_size) {
    uint32_t clusters_needed = (file_size + 511) / 512;
    if (clusters_needed <= 1) return true;
    
    uint16_t cluster = start_cluster;
    for (uint32_t i = 1; i < clusters_needed; i++) {
        uint16_t next = get_fat12_entry(cluster);
        if (next != cluster + 1) {
            // Not contiguous or end of chain
            return false;
        }
        cluster = next;
    }
    return true;
}

const uint8_t* msc_disk_get_jpeg_by_index(int index, uint32_t* size_out) {
    ImageEntry entries[16];
    int n = msc_disk_list_jpegs(entries, 16);
    if (index < 0 || index >= n) return NULL;
    
    uint16_t cluster = entries[index].cluster;
    uint32_t file_size = entries[index].size;
    
    if (size_out) *size_out = file_size;
    
    // Check if file is contiguous - if not, we can't return a simple pointer
    if (!clusters_contiguous(cluster, file_size)) {
        printf("Warning: File '%s' has fragmented clusters, may not load correctly\n", 
               entries[index].name);
    }
    
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

// Helper: Set FAT12 entry for a cluster in both FAT1 and FAT2
// FAT12 entries are 12 bits packed: cluster N at offset (N*3/2)
// Even cluster: low byte + low nibble of next byte
// Odd cluster: high nibble of byte + next byte
static void set_fat12_entry(uint16_t cluster, uint16_t value) {
    if (cluster < 2) return;  // Clusters 0,1 are reserved
    
    // FAT1 starts at LBA 1 (byte offset 512)
    // FAT2 starts at LBA 5 (byte offset 2560)
    uint32_t fat1_offset = 1 * 512;
    uint32_t fat2_offset = 5 * 512;
    
    // FAT12 byte offset for this cluster
    uint32_t fat_byte_offset = (cluster * 3) / 2;
    
    // Read the 4KB sector containing FAT1 and FAT2 (sector 0)
    memcpy(sector_buf, flash_storage, FLASH_SECTOR_SIZE);
    
    // Update FAT1
    uint8_t* fat1 = sector_buf + fat1_offset + fat_byte_offset;
    if (cluster & 1) {
        // Odd cluster: high nibble of byte[0], all of byte[1]
        fat1[0] = (fat1[0] & 0x0F) | ((value & 0x0F) << 4);
        fat1[1] = (value >> 4) & 0xFF;
    } else {
        // Even cluster: all of byte[0], low nibble of byte[1]
        fat1[0] = value & 0xFF;
        fat1[1] = (fat1[1] & 0xF0) | ((value >> 8) & 0x0F);
    }
    
    // Update FAT2 (same structure, different offset)
    uint8_t* fat2 = sector_buf + fat2_offset + fat_byte_offset;
    if (cluster & 1) {
        fat2[0] = (fat2[0] & 0x0F) | ((value & 0x0F) << 4);
        fat2[1] = (value >> 4) & 0xFF;
    } else {
        fat2[0] = value & 0xFF;
        fat2[1] = (fat2[1] & 0xF0) | ((value >> 8) & 0x0F);
    }
    
    // Write back the FAT sector
    flush_sector(0);
}

// Helper: Find first free cluster (FAT entry == 0x000)
static uint16_t find_free_cluster(void) {
    const uint8_t* fat1 = flash_storage + (1 * 512);  // FAT1 at LBA 1
    
    // Scan clusters 2 to 503 (502 data clusters available)
    for (uint16_t cluster = 2; cluster < 504; cluster++) {
        uint32_t fat_byte_offset = (cluster * 3) / 2;
        uint16_t entry;
        
        if (cluster & 1) {
            // Odd cluster
            entry = ((fat1[fat_byte_offset] >> 4) & 0x0F) | 
                    (fat1[fat_byte_offset + 1] << 4);
        } else {
            // Even cluster
            entry = fat1[fat_byte_offset] | 
                    ((fat1[fat_byte_offset + 1] & 0x0F) << 8);
        }
        
        if (entry == 0x000) {
            return cluster;  // Free cluster found
        }
    }
    return 0;  // No free cluster
}

// Create a simple file with given name and data if missing.
// Properly updates FAT12 to mark cluster as end-of-chain (0xFFF).
bool msc_disk_create_file_if_missing(const char* name, const uint8_t* data, uint32_t len) {
    // Return if exists
    uint32_t existing_size;
    if (msc_disk_find_file(name, &existing_size)) return true;

    // Limit file size to one cluster (512 bytes) for simplicity
    if (len > 512) {
        printf("File too large for simple creation: %lu > 512\n", len);
        return false;
    }

    // Find a free cluster
    uint16_t cluster = find_free_cluster();
    if (cluster == 0) {
        printf("No free cluster available\n");
        return false;
    }

    // Mark cluster as end-of-chain in FAT (0xFFF)
    set_fat12_entry(cluster, 0xFFF);

    // Find a free root entry in the root sector (LBA 9)
    uint32_t root_lba = 9;
    uint32_t flash_sector_index = (root_lba * DISK_BLOCK_SIZE) / FLASH_SECTOR_SIZE;  // = 1
    uint32_t flash_sector_offset = flash_sector_index * FLASH_SECTOR_SIZE;

    // Copy sector into sector_buf
    memcpy(sector_buf, flash_storage + flash_sector_offset, FLASH_SECTOR_SIZE);

    // Root directory is at offset 512 within this 4KB sector (LBA 9 = sector 1 of flash sector 1)
    uint8_t* root = sector_buf + ((root_lba % 8) * DISK_BLOCK_SIZE);  // LBA 9 % 8 = 1, offset 512
    int free_idx = -1;
    for (int i = 0; i < 16; i++) {
        uint8_t* entry = root + (i * 32);
        if (entry[0] == 0x00 || entry[0] == 0xE5) { free_idx = i; break; }
    }
    if (free_idx < 0) {
        printf("No free directory entry\n");
        return false;
    }

    // Prepare filename in 8.3 format (uppercase, padded)
    char upname[12];
    const char* dot = strchr(name, '.');
    int namelen = dot ? (int)(dot - name) : (int)strlen(name);
    int extlen = dot ? (int)strlen(dot + 1) : 0;
    memset(upname, ' ', 11);
    for (int i = 0; i < namelen && i < 8; i++) upname[i] = toupper((unsigned char)name[i]);
    for (int i = 0; i < extlen && i < 3; i++) upname[8 + i] = toupper((unsigned char)dot[1 + i]);

    // Prepare directory entry
    uint8_t* entry = root + (free_idx * 32);
    memcpy(entry, upname, 11);
    entry[11] = 0x20;  // Archive attribute
    // Clear reserved/time fields
    memset(entry + 12, 0, 14);
    entry[26] = cluster & 0xFF;
    entry[27] = (cluster >> 8) & 0xFF;
    entry[28] = (len & 0xFF);
    entry[29] = (len >> 8) & 0xFF;
    entry[30] = (len >> 16) & 0xFF;
    entry[31] = (len >> 24) & 0xFF;

    // Write root directory sector back
    flush_sector(flash_sector_offset);

    // Write file data to the cluster
    // Data area: cluster 2 starts at LBA 10
    uint32_t data_lba = 10 + (cluster - 2);
    uint32_t data_flash_sector = (data_lba * DISK_BLOCK_SIZE) / FLASH_SECTOR_SIZE;
    uint32_t data_flash_offset = data_flash_sector * FLASH_SECTOR_SIZE;
    uint32_t data_sector_offset = (data_lba * DISK_BLOCK_SIZE) % FLASH_SECTOR_SIZE;

    // Read the 4KB flash sector containing this cluster
    memcpy(sector_buf, flash_storage + data_flash_offset, FLASH_SECTOR_SIZE);
    
    // Clear and write file data
    memset(sector_buf + data_sector_offset, 0, 512);
    memcpy(sector_buf + data_sector_offset, data, len);
    
    // Write back
    flush_sector(data_flash_offset);

    printf("Created file '%s' in cluster %u\n", name, cluster);
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
    if (len > 512) return false;  // Single cluster limit
    if (cluster < 2) return false;

    // Calculate data location
    uint32_t data_lba = 10 + (cluster - 2);
    uint32_t flash_sector_index = (data_lba * DISK_BLOCK_SIZE) / FLASH_SECTOR_SIZE;
    uint32_t flash_sector_offset = flash_sector_index * FLASH_SECTOR_SIZE;
    uint32_t sector_internal_offset = (data_lba * DISK_BLOCK_SIZE) % FLASH_SECTOR_SIZE;

    // Read the entire 4KB flash sector, modify only our 512-byte cluster
    memcpy(sector_buf, flash_storage + flash_sector_offset, FLASH_SECTOR_SIZE);
    memset(sector_buf + sector_internal_offset, 0, 512);  // Clear cluster
    memcpy(sector_buf + sector_internal_offset, data, len);

    // Write back the entire flash sector
    flush_sector(flash_sector_offset);

    // Update file size in directory entry if changed
    uint32_t root_flash_sector = (9 * DISK_BLOCK_SIZE) / FLASH_SECTOR_SIZE;  // = 1
    uint32_t root_flash_offset = root_flash_sector * FLASH_SECTOR_SIZE;
    memcpy(sector_buf, flash_storage + root_flash_offset, FLASH_SECTOR_SIZE);
    
    uint8_t* root = sector_buf + ((9 % 8) * DISK_BLOCK_SIZE);  // LBA 9 offset in sector
    uint8_t* entry = root + (found * 32);
    entry[28] = (len & 0xFF);
    entry[29] = (len >> 8) & 0xFF;
    entry[30] = (len >> 16) & 0xFF;
    entry[31] = (len >> 24) & 0xFF;
    
    flush_sector(root_flash_offset);

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
