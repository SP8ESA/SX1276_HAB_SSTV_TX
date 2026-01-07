#include <stdio.h>
#include <string.h>
#include "tjpgd.h"
#include "msc_disk.h"

#define IMG_WIDTH  320
#define IMG_HEIGHT 240

// Forward declare image buffers from main.c (full resolution)
extern uint8_t image_Y[IMG_HEIGHT][IMG_WIDTH];
extern uint8_t image_RY[IMG_HEIGHT][IMG_WIDTH];
extern uint8_t image_BY[IMG_HEIGHT][IMG_WIDTH];

// Device for input stream with scaling info
typedef struct {
    const uint8_t* data;
    size_t size;
    size_t off;
    uint16_t src_width;
    uint16_t src_height;
} JDEV;

static size_t tjpgd_infunc(JDEC* jd, uint8_t* buff, size_t nbyte) {
    JDEV* dev = (JDEV*)jd->device;
    if (!buff) {
        // skip
        size_t remain = dev->size - dev->off;
        size_t r = (nbyte < remain) ? nbyte : remain;
        dev->off += r;
        return r;
    }
    size_t remain = dev->size - dev->off;
    size_t r = (nbyte < remain) ? nbyte : remain;
    memcpy(buff, dev->data + dev->off, r);
    dev->off += r;
    return r;
}

// RGB -> YCbCr converter
static void rgb_to_ycbcr_local(uint8_t r, uint8_t g, uint8_t b, uint8_t* y, uint8_t* cb, uint8_t* cr) {
    int Y  = ((66 * r + 129 * g + 25 * b + 128) >> 8) + 16;
    int Cb = ((-38 * r - 74 * g + 112 * b + 128) >> 8) + 128;
    int Cr = ((112 * r - 94 * g - 18 * b + 128) >> 8) + 128;
    *y  = (Y  < 0) ? 0 : (Y  > 255) ? 255 : Y;
    *cb = (Cb < 0) ? 0 : (Cb > 255) ? 255 : Cb;
    *cr = (Cr < 0) ? 0 : (Cr > 255) ? 255 : Cr;
}

static int tjpgd_outfunc(JDEC* jd, void* bitmap, JRECT* rect) {
    JDEV* dev = (JDEV*)jd->device;
    uint8_t* pix = (uint8_t*)bitmap; // RGB888
    int left = rect->left;
    int top = rect->top;
    int right = rect->right;
    int bottom = rect->bottom;
    
    uint16_t src_w = dev->src_width;
    uint16_t src_h = dev->src_height;
    
    for (int sy = top; sy <= bottom; sy++) {
        for (int sx = left; sx <= right; sx++) {
            uint8_t r = *pix++;
            uint8_t g = *pix++;
            uint8_t b = *pix++;
            
            // Scale source coordinates to destination 320x240
            int dx = (sx * IMG_WIDTH) / src_w;
            int dy = (sy * IMG_HEIGHT) / src_h;
            
            if (dy >= 0 && dy < IMG_HEIGHT && dx >= 0 && dx < IMG_WIDTH) {
                uint8_t Y, Cb, Cr;
                rgb_to_ycbcr_local(r, g, b, &Y, &Cb, &Cr);
                
                image_Y[dy][dx] = Y;
                image_RY[dy][dx] = Cr;
                image_BY[dy][dx] = Cb;
            }
        }
    }
    return 1; // continue
}

bool load_jpeg_image(const uint8_t* jpeg_data, uint32_t size) {
    if (!jpeg_data || size < 64) return false;

    JDEC jd;
    JDEV dev = { jpeg_data, size, 0, 0, 0 };
    // TJpgDec requires a separate working pool (several KB).
    static uint8_t work_pool[4096];

    JRESULT r = jd_prepare(&jd, tjpgd_infunc, work_pool, sizeof(work_pool), &dev);
    if (r != JDR_OK) {
        printf("jd_prepare failed: %d\n", r);
        return false;
    }
    
    // Store source dimensions for scaling in output callback
    dev.src_width = jd.width;
    dev.src_height = jd.height;
    
    printf("JPEG: %dx%d", jd.width, jd.height);
    if (jd.width != IMG_WIDTH || jd.height != IMG_HEIGHT) {
        printf(" -> scaling to %dx%d", IMG_WIDTH, IMG_HEIGHT);
    }
    printf("\n");

    // Decompress (scale = 0 => full size from decoder, we scale in output)
    r = jd_decomp(&jd, tjpgd_outfunc, 0);
    if (r != JDR_OK) {
        printf("jd_decomp failed: %d\n", r);
        return false;
    }
    printf("JPEG decoded OK\n");
    return true;
}
