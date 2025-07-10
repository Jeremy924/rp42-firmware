/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_storage_if.c
  * @version        : v1.0_Cube
  * @brief          : Memory management layer for QSPI Flash via USB MSC.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics. // Or your copyright
  * All rights reserved.
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  * www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_msc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "main.h"       // Include for HAL types (HAL_OK, HAL_StatusTypeDef) if needed by CSP_QSPI
#include "quadspi.h"   // Include your QSPI driver header
#include "ffconf.h"     // For _MIN_SS if used, otherwise define FATFS_SECTOR_SIZE directly
#include <stdbool.h>
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @defgroup USBD_STORAGE
  * @brief Usb mass storage device module
  * @{
  */

/** @defgroup USBD_STORAGE_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */
volatile uint8_t g_msc_is_active = 1; // 1 for active, 0 for inactive

// Define DWORD and BYTE if not available from included headers (like integer.h or ff.h)
#ifndef DWORD
typedef uint32_t DWORD;
#endif
#ifndef BYTE
typedef uint8_t BYTE;
#endif
#ifndef UINT
typedef unsigned int UINT;
#endif
#ifndef WORD
typedef uint16_t WORD;
#endif

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Defines
  * @brief Private defines.
  * @{
  */

// --- QSPI Flash & Filesystem Configuration ---

// 8MB - 256KB for copying firmware and 1MB for temporari
#define QSPI_FLASH_TOTAL_SIZE_BYTES  0x6C0000

// Define the start address offset in the QSPI flash where the filesystem partition begins.
#define FILESYSTEM_OFFSET            0x100000UL // 1MB offset

// Define the physical erase block size of your QSPI flash chip.
// Common sizes are 4KB, 32KB, or 64KB. Check your datasheet!
#define QSPI_BLOCK_SIZE              4096UL     // 4KB erase block size

// Define the logical sector size used by FatFs and reported to the USB host.
// MUST match FF_MIN_SS/FF_MAX_SS in ffconf.h if FatFs is also used locally.
#define FATFS_SECTOR_SIZE            512UL      // 512 bytes logical sector size

// --- USB MSC Configuration ---

// Calculate the total number of usable sectors for the filesystem partition
#define STORAGE_BLK_NBR              (QSPI_FLASH_TOTAL_SIZE_BYTES / FATFS_SECTOR_SIZE)

// Define the block size reported to the USB host (must match FATFS_SECTOR_SIZE)
#define STORAGE_BLK_SIZ              FATFS_SECTOR_SIZE

// Number of Logical Units (LUNs). Usually 1 for a single partition.
#define STORAGE_LUN_NBR              1

/* USER CODE BEGIN PRIVATE_DEFINES */
// Sanity check: Ensure calculated size is positive
#if (QSPI_FLASH_TOTAL_SIZE_BYTES <= FILESYSTEM_OFFSET)
#error "FILESYSTEM_OFFSET is larger than or equal to QSPI_FLASH_TOTAL_SIZE_BYTES!"
#endif
/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_Variables
  * @brief Private variables.
  * @{
  */

/* USER CODE BEGIN INQUIRY_DATA */
/** USB Mass storage Standard Inquiry Data. */
const int8_t STORAGE_Inquirydata[STANDARD_INQUIRY_DATA_LEN] = { /* 36 bytes */

  /* LUN 0 */
  0x00,		/* Direct Access Device */
  0x80,		/* RMB = 1: Removable Medium Bit */
  0x02,		/* Version: ANSI X3.131: 1994 */
  0x02,		/* Response Data Format = 2 */
  (STANDARD_INQUIRY_DATA_LEN - 5), /* Additional Length */
  0x00,		/* SCCS = 0: No Storage Controller Component */
  0x00,		/* Reserved */
  0x00,		/* Reserved */
  'J', 'M', 'L', ' ', ' ', ' ', ' ', ' ', /* Manufacturer: 8 bytes */
  'R', 'P', '-', '4', '2', ' ', ' ', ' ', /* Product ID: 16 bytes */
  ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
  '0', '.', '2', ' '                     /* Product Revision Level: 4 bytes */
};
/* USER CODE END INQUIRY_DATA */

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDevice; // Assume this is defined in usb_device.c

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_STORAGE_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t STORAGE_Init(uint8_t lun);
static int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size);
static int8_t STORAGE_IsReady(uint8_t lun);
static int8_t STORAGE_IsWriteProtected(uint8_t lun);
static int8_t STORAGE_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
static int8_t STORAGE_GetMaxLun(void);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_StorageTypeDef USBD_Storage_Interface_fops =
{
  STORAGE_Init,
  STORAGE_GetCapacity,
  STORAGE_IsReady,
  STORAGE_IsWriteProtected,
  STORAGE_Read,
  STORAGE_Write,
  STORAGE_GetMaxLun,
  (int8_t *)STORAGE_Inquirydata // Cast is appropriate here
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the storage media (QSPI Flash).
  * @param  lun: Logical unit number (should be 0)
  * @retval USBD_OK if initialization is OK, USBD_FAIL otherwise.
  */
int8_t STORAGE_Init(uint8_t lun)
{
  /* USER CODE BEGIN 2 */
  // You might need to call your QSPI initialization function here
  // if it's not already called elsewhere before USB starts.
  // Example:
  // if (CSP_QSPI_Init() != HAL_OK) {
  //     return USBD_FAIL;
  // }
  return (USBD_OK);
  /* USER CODE END 2 */
}

/**
  * @brief  Returns the medium capacity.
  * @param  lun: Logical unit number (should be 0)
  * @param  block_num: Pointer to store the total number of sectors.
  * @param  block_size: Pointer to store the sector size in bytes.
  * @retval USBD_OK.
  */
int8_t STORAGE_GetCapacity(uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  /* USER CODE BEGIN 3 */
  if (lun != 0) {
      return USBD_FAIL; // Only support LUN 0
  }
  *block_num  = STORAGE_BLK_NBR;  // Use calculated total sectors
  *block_size = STORAGE_BLK_SIZ;  // Use defined sector size (512)
  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  Checks whether the medium is ready.
  * @param  lun: Logical unit number (should be 0)
  * @retval USBD_OK if the medium is ready, USBD_FAIL otherwise.
  */
int8_t STORAGE_IsReady(uint8_t lun)
{
  /* USER CODE BEGIN 4 */
  // Add checks here if the QSPI flash requires time to initialize
  // or has a status register indicating readiness.
  // Example:
  // uint8_t status = CSP_QSPI_GetStatus();
  // if (status == QSPI_READY) {
  //     return USBD_OK;
  // } else {
  //     return USBD_FAIL;
  // }
	if (!g_msc_is_active) return (USBD_BUSY);
  return (USBD_OK); // Assume always ready for simplicity
  /* USER CODE END 4 */
}

/**
  * @brief  Checks whether the medium is write protected.
  * @param  lun: Logical unit number (should be 0)
  * @retval USBD_OK if medium is not write protected, USBD_FAIL otherwise.
  */
int8_t STORAGE_IsWriteProtected(uint8_t lun)
{
  /* USER CODE BEGIN 5 */
  // Add checks here if your hardware has write protection features.
  return (USBD_OK); // Assume not write protected
  /* USER CODE END 5 */
}

/**
  * @brief  Reads data from the medium.
  * @param  lun: Logical unit number (should be 0)
  * @param  buf: Pointer to data buffer to store read data.
  * @param  blk_addr: Start block address (LBA).
  * @param  blk_len: Number of blocks (sectors) to read.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Read(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
	if (g_msc_is_active == 0) return USBD_BUSY;

  /* USER CODE BEGIN 6 */
    HAL_StatusTypeDef status;

    if (lun != 0) {
        return USBD_FAIL;
    }

    // Basic bounds check (more robust check might be needed depending on QSPI driver)
    if ((blk_addr + blk_len) > STORAGE_BLK_NBR) {
        return USBD_FAIL; // Attempting to read beyond defined capacity
    }

    // Calculate the physical QSPI flash address
    DWORD flashAddress = (blk_addr * FATFS_SECTOR_SIZE) + FILESYSTEM_OFFSET;
    DWORD totalSize = blk_len * FATFS_SECTOR_SIZE;

    // Call the low-level QSPI read function
    status = CSP_QSPI_Read(buf, flashAddress, totalSize);

    if (status != HAL_OK) {
    	HAL_StatusTypeDef status = HAL_QSPI_Abort(&hqspi);
    	if (status != HAL_OK)
    		Error_Handler();

        status = CSP_QSPI_Read(buf, flashAddress, totalSize);
        if (status != HAL_OK)
        	Error_Handler();

        status = CSP_QSPI_EnableMemoryMappedMode();
        if (status != HAL_OK)
        	Error_Handler();
    }

    return USBD_OK;
  /* USER CODE END 6 */
}

/**
  * @brief  Writes data to the medium.
  * @param  lun: Logical unit number (should be 0)
  * @param  buf: Pointer to data buffer containing data to write.
  * @param  blk_addr: Start block address (LBA).
  * @param  blk_len: Number of blocks (sectors) to write.
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
int8_t STORAGE_Write(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len)
{
  /* USER CODE BEGIN 7 */
    // This implementation uses the Read-Modify-Erase-Write approach
    // suitable for NOR flash like QSPI.

	if (g_msc_is_active == 0) return USBD_BUSY;

    HAL_StatusTypeDef status = HAL_OK;
    // Allocate buffer on stack - ensure stack size is sufficient!
    // If QSPI_BLOCK_SIZE is large (e.g., 64KB), consider static or dynamic allocation.
    BYTE* flash_buffer = sector_copy_buffer;

    if (lun != 0) {
        return USBD_FAIL;
    }

    // --- Parameter Validation ---
    if (blk_len == 0) {
        return USBD_OK; // Nothing to write
    }
    // Check if write starts beyond the disk or extends beyond the disk
    if (blk_addr >= STORAGE_BLK_NBR || (blk_addr + blk_len) > STORAGE_BLK_NBR) {
        // Attempting to write outside the defined capacity
        // Returning FAIL as the host likely sent invalid parameters.
        // USBD_EMEM might also be considered, but FAIL is clearer.
        return USBD_FAIL;
    }
    // Check if the device is write-protected (if applicable)
    // if (STORAGE_IsWriteProtected(lun) != USBD_OK) {
    //     return USBD_FAIL; // Indicate write protection error
    // }

    // --- Calculate Address Range ---
    DWORD startWriteAddr = (blk_addr * FATFS_SECTOR_SIZE) + FILESYSTEM_OFFSET;
    // Calculate non-inclusive end address carefully to avoid overflow if blk_len is large
    DWORD endWriteAddr   = startWriteAddr + ((DWORD)blk_len * FATFS_SECTOR_SIZE);

    // Align the start address down to the nearest block boundary
    DWORD firstBlockToProcess = startWriteAddr & ~(QSPI_BLOCK_SIZE - 1);

    bool hasDisconnectedMM = false;
    // --- Iterate through each affected Erase Block ---
    for (DWORD currentBlockAddr = firstBlockToProcess; currentBlockAddr < endWriteAddr; currentBlockAddr += QSPI_BLOCK_SIZE)
    {
        // --- 1. Read the entire block into the buffer ---
        status = CSP_QSPI_Read(flash_buffer, currentBlockAddr, QSPI_BLOCK_SIZE);
        if (status == HAL_BUSY) {
        	hasDisconnectedMM = true;
        	HAL_QSPI_Abort(&hqspi);

            status = CSP_QSPI_Read(flash_buffer, currentBlockAddr, QSPI_BLOCK_SIZE);
        }
        if (status != HAL_OK) {
            // Consider adding specific error logging here
        	if (hasDisconnectedMM) CSP_QSPI_EnableMemoryMappedMode();
            return USBD_FAIL; // Read error
        }

        // --- 2. Erase the current block ---
        // Ensure QSPI peripheral is ready/not busy before erasing if needed by your driver
        status = CSP_QSPI_Erase_Block(currentBlockAddr);
        if (status != HAL_OK) {
            // Erase error - data in flash_buffer might still be valid, but block is likely bad
        	if (hasDisconnectedMM) CSP_QSPI_EnableMemoryMappedMode();
            return USBD_FAIL;
        }

        // --- 3. Modify the buffer and Write back sector-by-sector within the block ---
        // Iterate through each logical sector *within* the current erase block
        for (DWORD currentSectorFlashAddr = currentBlockAddr; currentSectorFlashAddr < (currentBlockAddr + QSPI_BLOCK_SIZE); currentSectorFlashAddr += FATFS_SECTOR_SIZE)
        {
            // Check if this logical sector's starting address falls within the requested write range
            if (currentSectorFlashAddr >= startWriteAddr && currentSectorFlashAddr < endWriteAddr)
            {
                // This sector needs new data from the input buffer 'buf'

                // Calculate the logical sector number (LBA) corresponding to this flash address
                DWORD currentLogicalSector = (currentSectorFlashAddr - FILESYSTEM_OFFSET) / FATFS_SECTOR_SIZE;

                // Calculate the offset into the *original* input buffer 'buf'
                // (currentLogicalSector - blk_addr) gives the index (0, 1, 2...)
                // of the current sector relative to the start of the USB write request.
                uint32_t offset_in_buffer = (currentLogicalSector - blk_addr) * FATFS_SECTOR_SIZE;

                // Write the new data for this sector
                // Ensure QSPI peripheral is ready/not busy before writing if needed by your driver
                status = CSP_QSPI_WriteMemory((BYTE*)buf + offset_in_buffer, currentSectorFlashAddr, FATFS_SECTOR_SIZE);
                if (status != HAL_OK) {
                    // Write error after erase - data is lost for this block!
                	if (hasDisconnectedMM) CSP_QSPI_EnableMemoryMappedMode();
                    return USBD_FAIL;
                }
            }
            else
            {
                // This sector is within the erased block but *outside* the requested write range.
                // It needs the original data (already in flash_buffer) to be written back.

                // Calculate the offset within the flash_buffer for this sector's data
                // Use modulo operation (masking is faster for power-of-2 block sizes)
                uint32_t offset_in_flash_buffer = currentSectorFlashAddr & (QSPI_BLOCK_SIZE - 1);

                // Write the old data back for this sector
                // Ensure QSPI peripheral is ready/not busy before writing if needed by your driver
                status = CSP_QSPI_WriteMemory(flash_buffer + offset_in_flash_buffer, currentSectorFlashAddr, FATFS_SECTOR_SIZE);
                 if (status != HAL_OK) {
                    // Write error after erase - data is lost for this block!
                	 if (hasDisconnectedMM) CSP_QSPI_EnableMemoryMappedMode();
                    return USBD_FAIL;
                }
            }
        } // End loop through sectors within a block
    } // End loop through affected blocks

    if (hasDisconnectedMM) CSP_QSPI_EnableMemoryMappedMode();
    return USBD_OK; // Success
  /* USER CODE END 7 */
}

/**
  * @brief  Returns the number of supported logical units.
  * @param  None
  * @retval Number of logical units (LUN) - 1. (0 for 1 LUN).
  */
int8_t STORAGE_GetMaxLun(void)
{
  /* USER CODE BEGIN 8 */
  return (STORAGE_LUN_NBR - 1);
  /* USER CODE END 8 */
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */
