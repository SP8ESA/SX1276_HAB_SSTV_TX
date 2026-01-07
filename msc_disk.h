#ifndef MSC_DISK_H
#define MSC_DISK_H

#include <stdint.h>
#include <stdbool.h>

void msc_disk_init(void);
void msc_disk_format(void);
bool msc_disk_has_new_image(void);
const uint8_t* msc_disk_find_bmp(uint32_t* size_out);
const uint8_t* msc_disk_find_jpeg(uint32_t* size_out);
const uint8_t* msc_disk_get_raw(void);

// Image listing API
typedef struct {
	char name[13]; // e.g. "IMG00123.JPG" (null terminated)
	uint16_t cluster; // starting cluster
	uint32_t size; // file size in bytes
} ImageEntry;

// Fill up to max_entries entries with JPEG files found in the root dir;
// returns number of entries found (0..max_entries). Entries are sorted
// alphabetically by name (case-insensitive by ASCII ordering).
int msc_disk_list_jpegs(ImageEntry* out_entries, int max_entries);

// Return pointer to JPEG data for the given index (as returned by list_jpegs)
const uint8_t* msc_disk_get_jpeg_by_index(int index, uint32_t* size_out);

// Find a file by name (case-insensitive, accepts e.g. "config.txt"). Returns pointer to data in flash or NULL.
const uint8_t* msc_disk_find_file(const char* name, uint32_t* size_out);

// Create a file with given name and content if it does not exist. Returns true on success.
bool msc_disk_create_file_if_missing(const char* name, const uint8_t* data, uint32_t len);

// Overwrite an existing file's content (must be <= 4KB). Returns true on success.
bool msc_disk_overwrite_file(const char* name, const uint8_t* data, uint32_t len);

#endif
