/******************************************************************************
*
* @file    $Name$
* @version $Revision$
* @date    $LastChangedDate$
*
* @Copyright 2015 - JIM.
*  This material is protected by copyright law. It is unlawful
*  to copy it.
*
* This document contains confidential information. It is not to be
* disclosed or used except in accordance with applicable contracts
* or agreements. This document must be rendered illegible when
* being discarded.
* 
* @remarks
* Modification History
* -------------------------
* See Version Control System History.
*
* @file This file contains the template class.
*
******************************************************************************/

#ifndef __USER_DEF_H__
#define __USER_DEF_H__

#ifdef __cplusplus
extern "C" {
#endif

/*****************************************************************************/
/*                              Include File                                 */
/*****************************************************************************/


/*****************************************************************************/
/*                      Global Macro or Enum Definition	                     */
/*****************************************************************************/
#define BYTE0_Msk              (0x000000FFUL)                   ///< Mask to get bit0~bit7 from a 32 bit integer
#define BYTE1_Msk              (0x0000FF00UL)                   ///< Mask to get bit8~bit15 from a 32 bit integer
#define BYTE2_Msk              (0x00FF0000UL)                   ///< Mask to get bit16~bit23 from a 32 bit integer
#define BYTE3_Msk              (0xFF000000UL)                   ///< Mask to get bit24~bit31 from a 32 bit integer

#define GET_BYTE0(u32Param)    (((u32Param) & BYTE0_Msk)      ) /*!< Extract Byte 0 (Bit  0~ 7) from parameter u32Param */
#define GET_BYTE1(u32Param)    (((u32Param) & BYTE1_Msk) >>  8) /*!< Extract Byte 1 (Bit  8~15) from parameter u32Param */
#define GET_BYTE2(u32Param)    (((u32Param) & BYTE2_Msk) >> 16) /*!< Extract Byte 2 (Bit 16~23) from parameter u32Param */
#define GET_BYTE3(u32Param)    (((u32Param) & BYTE3_Msk) >> 24) /*!< Extract Byte 3 (Bit 24~31) from parameter u32Param */

/*
    F24 : 256K flash , 256 block * 1024 = 0x40000
    F23 : 128K flash , 128 block * 1024 = 0x20000
    one block = 1K    
*/
#define FLASH_END_BLOCK			256ul

#define	FLASH_AREA				0x5000  // app code addresss
#define FLASH_TABLE				FLASH_AREA
#define FLASH_BYTE_PER_BLOCK	1024ul

// F24 : 256K
#define CRC_ADDRESS				((FLASH_END_BLOCK*FLASH_BYTE_PER_BLOCK)-4)
#define FLASH_END_ADDRESS		((FLASH_END_BLOCK*FLASH_BYTE_PER_BLOCK)-1)
// #define CRC_ADDRESS				0x3FFFC
// #define FLASH_END_ADDRESS		0x3FFFF

//RL78 F24 : 256K - boot code size - 4 (last 4 bytes:CRC address)
//RL78 F23 : 128K - boot code size - 4 (last 4 bytes:CRC address)
#define FLASH_AP_SIZE		    (FLASH_END_BLOCK*FLASH_BYTE_PER_BLOCK-FLASH_AREA-4)
// #define FLASH_AP_SIZE		    (0x40000-FLASH_AREA-4)  

#define SW						P13_bit.no7


#define TARGET_DEVICE_NOA                       ( 1 )       // Number of recordable areas (user area in code flash, data flash)

#define TARGET_DEVICE_TYPE                      ( 0x02 )    // 0x02(RA2/RA4), 0x03(RA6)

#define BOOT_FIRMWARE_MAJOR_VERSION             ( 1 )

#define BOOT_FIRMWARE_MINOR_VERSION             ( 0 )

#define MAX_CODE_FLASH_WRITE_DATA_SIZE          (256)

#define MAX_FLASH_READ_DATA_SIZE                (256)

/*****************************************************************************/
/*                   Global Structure or Union Definition     		         */
/*****************************************************************************/


/*****************************************************************************/
/*                       Global Variable Declaration                         */
/*****************************************************************************/


/*****************************************************************************/
/*                         Configuration Checking                            */
/*****************************************************************************/


/*****************************************************************************/
/*                       Global Function Declaration                         */
/*****************************************************************************/


#endif /* __TEMPLATE_H */
/* EOF */
