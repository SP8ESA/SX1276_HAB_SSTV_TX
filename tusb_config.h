#ifndef TUSB_CONFIG_H
#define TUSB_CONFIG_H

#define CFG_TUSB_RHPORT0_MODE   OPT_MODE_DEVICE
#define CFG_TUD_ENDPOINT0_SIZE  64

// Mass Storage Class only
#define CFG_TUD_MSC     1
#define CFG_TUD_CDC     0
#define CFG_TUD_HID     0
#define CFG_TUD_MIDI    0
#define CFG_TUD_VENDOR  0

// MSC buffer size
#define CFG_TUD_MSC_EP_BUFSIZE  512

#endif
