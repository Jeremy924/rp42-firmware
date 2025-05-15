/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/*
 * Warning: the user section 0 is no more in use (starting from CubeMx version 4.16.0)
 * To be suppressed in the future.
 * Kept to ensure backward compatibility with previous CubeMx versions when
 * migrating projects.
 * User code previously added there should be copied in the new user sections before
 * the section contents can be deleted.
 */
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "ff_gen_drv.h"
#include <stdbool.h>
#include "quadspi.h"
#include "stm32l475xx.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Disk status */
static volatile DSTATUS Stat = STA_NOINIT;

#define TOTAL_CAPACITY 0x700000
#define SECTOR_SIZE    _MIN_SS
#define TOTAL_SECTORS  TOTAL_CAPACITY / SECTOR_SIZE

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

// Define TOTAL_SECTORS based on the size of your QSPI flash partition
// Example: 15MB partition = (15 * 1024 * 1024) / 512 = 30720 sectors
// IMPORTANT: Ensure this matches the actual usable size AFTER your offset.
// If total QSPI is 16MB and you offset 1MB, usable is 15MB.

// Define the start address offset in the QSPI flash where the filesystem partition begins.
#define FILESYSTEM_OFFSET   0x100000UL // 1MB offset as used in the original code

// Define the physical erase block size of your QSPI flash chip.
// Common sizes are 4KB, 32KB, or 64KB. Check your datasheet!
#define QSPI_BLOCK_SIZE     4096UL     // 4KB erase block size

// Define the logical sector size used by FatFs (MUST match FF_MIN_SS/FF_MAX_SS in ffconf.h)
#define FATFS_SECTOR_SIZE   512UL      // Usually 512 for FatFs

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
	if (pdrv != 0)
		Stat = STA_NODISK;
	Stat = 0;
    return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
    Stat = 0;
    return Stat;
  /* USER CODE END STATUS */
}


/**
 * @brief Checks if the QSPI peripheral is currently enabled and configured for memory-mapped mode.
 * @param hqspi: pointer to a QSPI_HandleTypeDef structure that contains
 * the configuration information for QSPI module.
 * @retval bool: true if in memory-mapped mode, false otherwise.
 */
bool QSPI_IsMemoryMapped(QSPI_HandleTypeDef *hqspi) {
    // 1. Check if the QSPI peripheral is enabled
    if ((hqspi->Instance->CR & QUADSPI_CR_EN) == 0) {
        return false; // QSPI is not enabled, so not actively memory-mapped
    }

    // 2. Check the FMODE bits in QUADSPI_CCR
    // QSPI_FUNCTIONAL_MODE_MEMORY_MAPPED is typically (0x3UL << QUADSPI_CCR_FMODE_Pos)
    if ((hqspi->Instance->CCR & QUADSPI_CCR_FMODE) == 0b1100000000000000000000000000) {
        return true;
    }

    return false;
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
	if (pdrv != 0) return RES_ERROR;
	if (sector >= TOTAL_SECTORS) return RES_ERROR;

	DWORD totalSize = count * SECTOR_SIZE;

	// offset 1MB for free42
	DWORD flashAddress = sector * SECTOR_SIZE + 0x100000;

	if (QSPI_IsMemoryMapped(&hqspi)) {
		memcpy(buff, (void*) (0x90000000 + flashAddress), totalSize);
		return RES_OK;
	}

	HAL_StatusTypeDef status = CSP_QSPI_Read(buff, flashAddress, totalSize);

	if (status != HAL_OK) return RES_ERROR;

    return RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
    BYTE pdrv,          /* Physical drive number (0) */
    const BYTE *buff,   /* Data to be written */
    DWORD sector,       /* Start sector address (LBA) */
    UINT count          /* Number of sectors to write (1..128) */
)
{
    HAL_StatusTypeDef status = HAL_OK;
    BYTE* flash_buffer = sector_copy_buffer; // Local buffer for one erase block (ensure stack space!)

    // --- Parameter Validation ---
    if (pdrv != 0) {
        return RES_PARERR; // Invalid drive number
    }
    if (count == 0) {
        return RES_PARERR; // Nothing to write
    }
    // Check if write starts beyond the disk or extends beyond the disk
    if (sector >= TOTAL_SECTORS || (sector + count) > TOTAL_SECTORS) {
        return RES_PARERR; // Parameter error (invalid sector range)
    }
    // Check if the device is write-protected (if applicable)
    // if (USER_CheckWriteProtection()) { // Implement this if needed
    //     return RES_WRPRT;
    // }

    // --- Calculate Address Range ---
    DWORD startWriteAddr = (sector * FATFS_SECTOR_SIZE) + FILESYSTEM_OFFSET;
    DWORD endWriteAddr   = startWriteAddr + (count * FATFS_SECTOR_SIZE); // Non-inclusive end address

    // Align the start address down to the nearest block boundary
    DWORD firstBlockToProcess = startWriteAddr & ~(QSPI_BLOCK_SIZE - 1);

    bool hasDisconnectedMM = false;

    // --- Iterate through each affected Erase Block ---
    for (DWORD currentBlockAddr = firstBlockToProcess; currentBlockAddr < endWriteAddr; currentBlockAddr += QSPI_BLOCK_SIZE)
    {
        // --- 1. Read the entire block into the buffer ---
        status = CSP_QSPI_Read(flash_buffer, currentBlockAddr, QSPI_BLOCK_SIZE);
        if (status == HAL_BUSY) {
        	HAL_QSPI_Abort(&hqspi);
        	hasDisconnectedMM = true;

            status = CSP_QSPI_Read(flash_buffer, currentBlockAddr, QSPI_BLOCK_SIZE);
        }
        if (status != HAL_OK) {
            // Consider adding specific error logging here
        	if (hasDisconnectedMM) CSP_QSPI_EnableMemoryMappedMode();
            return RES_ERROR; // Read error
        }

        // --- 2. Erase the current block ---
        // Ensure QSPI peripheral is ready/not busy before erasing if needed
        status = CSP_QSPI_Erase_Block(currentBlockAddr);
        if (status != HAL_OK) {
            // Erase error - data in flash_buffer might still be valid, but block is likely bad
            return RES_ERROR;
        }

        // --- 3. Modify the buffer and Write back sector-by-sector within the block ---
        // Iterate through each FatFs sector *within* the current erase block
        for (DWORD currentSectorAddr = currentBlockAddr; currentSectorAddr < (currentBlockAddr + QSPI_BLOCK_SIZE); currentSectorAddr += FATFS_SECTOR_SIZE)
        {
            // Check if this sector address falls within the requested write range
            if (currentSectorAddr >= startWriteAddr && currentSectorAddr < endWriteAddr)
            {
                // This sector needs new data from the input buffer 'buff'

                // Calculate the logical sector number corresponding to this flash address
                DWORD currentLogicalSector = (currentSectorAddr - FILESYSTEM_OFFSET) / FATFS_SECTOR_SIZE;

                // Calculate the offset into the *original* input buffer 'buff'
                // Note: (currentLogicalSector - sector) gives the index (0, 1, 2...)
                // of the current sector relative to the start of the write request.
                uint32_t offset_in_buffer = (currentLogicalSector - sector) * FATFS_SECTOR_SIZE;

                // Write the new data for this sector
                // Ensure QSPI peripheral is ready/not busy before writing if needed
                status = CSP_QSPI_WriteMemory((BYTE*)buff + offset_in_buffer, currentSectorAddr, FATFS_SECTOR_SIZE);

                if (status != HAL_OK) {
                    // Write error after erase - data is lost for this block!
                	if (hasDisconnectedMM) CSP_QSPI_EnableMemoryMappedMode();
                    return RES_ERROR;
                }
            }
            else
            {
                // This sector is within the erased block but *outside* the requested write range.
                // It needs the original data (already in flash_buffer) to be written back.

                // Calculate the offset within the flash_buffer for this sector's data
                uint32_t offset_in_flash_buffer = currentSectorAddr & (QSPI_BLOCK_SIZE - 1); // Faster modulo for power-of-2

                // Write the old data back for this sector
                // Ensure QSPI peripheral is ready/not busy before writing if needed
                status = CSP_QSPI_WriteMemory(flash_buffer + offset_in_flash_buffer, currentSectorAddr, FATFS_SECTOR_SIZE);
                 if (status != HAL_OK) {
                    // Write error after erase - data is lost for this block!
                	 if (hasDisconnectedMM) CSP_QSPI_EnableMemoryMappedMode();
                    return RES_ERROR;
                }
            }
        } // End loop through sectors within a block
    } // End loop through affected blocks

    if (hasDisconnectedMM) CSP_QSPI_EnableMemoryMappedMode();
    return RES_OK; // Success
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
    DRESULT res = RES_ERROR;

    if (pdrv != 0) return res;

    switch (cmd) {
    case CTRL_SYNC:
    	return RES_OK;
    case GET_SECTOR_COUNT:
    	*(DWORD*) buff = TOTAL_SECTORS;
    	return RES_OK;
    case GET_SECTOR_SIZE:
    	*(WORD*) buff = (WORD) SECTOR_SIZE;
    	return RES_OK;
    case GET_BLOCK_SIZE:
    	*(DWORD*) buff = (DWORD) 0x1000;
    	return RES_OK;
    }

    return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */

