// Board and hardware specific configuration
#define MICROPY_HW_BOARD_NAME                   "DP Pico V1.0"
//#define PICO_FLASH_SIZE_BYTES (10 * 1024 * 1024)
//increase size in order to alocate the ram memory dump
#define MICROPY_HW_FLASH_STORAGE_BYTES          (((8*1024)+1408) * 1024)
//#define MICROPY_HW_FLASH_STORAGE_BYTES          (1408 * 1024)

// Enable USB Mass Storage with FatFS filesystem.
#define MICROPY_HW_USB_MSC  (1)
