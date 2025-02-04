/**********************************************************************************************************************
 * DISCLAIMER
 * This software is supplied by Renesas Electronics Corporation and is only intended for use with Renesas products. No
 * other uses are authorized. This software is owned by Renesas Electronics Corporation and is protected under all
 * applicable laws, including copyright laws.
 * THIS SOFTWARE IS PROVIDED "AS IS" AND RENESAS MAKES NO WARRANTIES REGARDING
 * THIS SOFTWARE, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. ALL SUCH WARRANTIES ARE EXPRESSLY DISCLAIMED. TO THE MAXIMUM
 * EXTENT PERMITTED NOT PROHIBITED BY LAW, NEITHER RENESAS ELECTRONICS CORPORATION NOR ANY OF ITS AFFILIATED COMPANIES
 * SHALL BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES FOR ANY REASON RELATED TO
 * THIS SOFTWARE, EVEN IF RENESAS OR ITS AFFILIATES HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES.
 * Renesas reserves the right, without notice, to make changes to this software and to discontinue the availability of
 * this software. By using this software, you agree to the additional terms and conditions found by accessing the
 * following link:
 * http://www.renesas.com/disclaimer
 *
 * Copyright (C) 2020-2022 Renesas Electronics Corporation. All rights reserved.
 *********************************************************************************************************************/
/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :                                                      */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/

#include <stdio.h>
#include <string.h>

#include "sample_control_code_flash.h"
#include "sample_control_common.h"
#include "sample_defines.h"
#include "sample_types.h"
// #include "sample_config.h"

#include "r_rfd_common_api.h"
#include "r_rfd_common_control_api.h"
#include "r_rfd_common_userown.h"
#include "r_rfd_code_flash_api.h"

#include "iodefine.h"
#include "r_smc_entry.h"
#include "platform.h"

#include "r_typedefs.h"
// #include "crc16.h"
#include "user_def.h"
#include "i2c_download_protocol.h"

#include "xmodem.h"

#include "sw_crc32.h"

#define ENABLE_UART_PRINTF                              (1)
#define ENABLE_UART_BL                                  (1)
#define ENALBE_IICA0_BL                                 (0)


/**** CPU frequency (MHz) ****/
/* It must be rounded up digits after the decimal point to form an integer (MHz). */
#define CPU_FREQUENCY                                   (40u)


/*
    RL78/F23  12 Kbytes (FCF00H to FFEFFH) 
    RL78/F24  24 Kbytes (F9F00H to FFEFFH) 

    ram flag : LAST ram address - 2K (0x800)
    RL78/F24:FFF00H - 0x800 = FF700
    RL78/F23:FFF00H - 0x800 = FF700
*/

#define RESET_TO_BOOT_SIGN 0xAA55AA55
#pragma address (reset_to_bootloader = 0x000FF700)  
volatile uint32_t reset_to_bootloader;


//programming relative data
uint32_t volatile   flash_memory_start_address = 0;
uint32_t volatile   flash_memory_end_address = 0;
uint32_t volatile   erase_area_start_address = 0;
uint32_t volatile   erase_area_end_address = 0;
// uint32_t volatile   read_area_start_address = 0;
// uint32_t volatile   read_area_end_address = 0;
uint32_t volatile   flash_target_memory_size = 0;
uint32_t volatile   flash_memory_destination_address = 0;
volatile bool g_flash_operation_progressing = false;
volatile bool g_app_required_update = false;

#if ENALBE_IICA0_BL
//I2C relative data
uint8_t DataBuf[MAXIMUM_RECV_PACKET_SIZE] = {0};
uint8_t ResponseBuf[MAXIMUM_RESP_PACKET_SIZE] = {0};
i2c_downloader_ctrl_t i2c_downloader_ctrl;
volatile uint8_t 	downloader_iica0_slave_status_flag;     /* iica0 slave flag */
volatile uint16_t 	downloader_iica0_rx_cnt;                /* iica0 receive data count */
volatile uint16_t 	downloader_iica0_tx_cnt;                /* iica0 send data count */

const uint8_t device_auth_status_ok[7] = {DATA_PACKET_START, 0x00, 0x02, ID_AUTHENTICATION_CMD_BYTE, STATUS_CODE_OK, 0xCE, END_OF_PACKET};
const uint8_t device_inquiry_data_status_ok[7] = {DATA_PACKET_START, 0x00, 0x02, INQUIRY_CMD_BYTE, STATUS_CODE_OK, 0xFE, END_OF_PACKET};
const uint8_t device_memory_erase_status_ok[7] = {DATA_PACKET_START, 0x00, 0x02, ERASE_CMD_BYTE, STATUS_CODE_OK, 0xEC, END_OF_PACKET};
const uint8_t device_memory_write_status_ok[7] = {DATA_PACKET_START, 0x00, 0x02, WRITE_CMD_BYTE, STATUS_CODE_OK, 0xEB, END_OF_PACKET};
const uint8_t device_memory_read_status_ok[7] = {DATA_PACKET_START, 0x00, 0x02, READ_CMD_BYTE, STATUS_CODE_OK, 0xE9, END_OF_PACKET};
const uint8_t device_memory_read_packet_error[7] = {DATA_PACKET_START, 0x00, 0x02, 0x95, STATUS_CODE_PACKET_ERROR, 0xA8, END_OF_PACKET};
const uint8_t device_memory_read_checksum_error[7] = {DATA_PACKET_START, 0x00, 0x02, 0x95, STATUS_CODE_CHECKSUM_ERROR, 0xA7, END_OF_PACKET};
const uint8_t device_memory_switch_app_ok[7] = {DATA_PACKET_START, 0x00, 0x02, SWTICH_APP_CMD_BYTE, STATUS_CODE_OK, 0xC2, END_OF_PACKET};

const uint8_t device_part_number[13] = "RL78_F24_FPB";


void ErrorHandler(uint8_t);
// uint8_t WriteCodeFlash(uint32_t u32_start_addr, uint8_t * u8_write_data, uint32_t u32_write_data_len);


void I2C_Downloader_routine(void);
static void transmit_handler(i2c_downloader_ctrl_t * p_i2c_downloader_ctrl);
static void received_handler(i2c_downloader_ctrl_t * p_i2c_downloader_ctrl);
static bool checksum_verification(uint8_t const * p_input, uint16_t len);
static uint8_t checksum_calculate(uint8_t const * p_input, uint16_t data_len);
static void assemble_error_data_packet(uint8_t * txdata, uint8_t associated_cmd, uint8_t error_code);
static void encode16bit(uint32_t input_data, uint8_t * output_array);
static uint16_t decode16bit(uint8_t * data);
static uint32_t decode32bit(uint8_t * data);
static bool Is_target_address_valid(uint8_t *data);
#endif

bool condition_check(void);

// void boot_R_Config_UART0_Send_Byte(int8_t data);
// void boot_R_Config_UART0_Send_String(__far int8_t buffer[]);

void boot_R_Config_TAU0_0_Stop(void);
void boot_R_Config_TAU0_0_Start(void);

/*******************************************************************************
* Function Name : ErrorHandler
* Description	: This function handles the error behavior.
* Parameters	: case_num -
*                    error case number
* Return value  : none
*******************************************************************************/
void ErrorHandler(uint8_t case_num)
{
#if ENABLE_UART_PRINTF
	switch (case_num)
	{
		case SAMPLE_ENUM_RET_ERR_PARAMETER:
			printf_tiny("[Error] R_RFSP_Init failed\r\n");
			break;
		case SAMPLE_ENUM_RET_ERR_CONFIGURATION:
			printf_tiny("[Error] HOCO is not started\r\n");
			break;
		case SAMPLE_ENUM_RET_ERR_MODE_MISMATCHED:
			printf_tiny("[Error] R_RFSP_SetFlashMemoryMode failed\r\n");
			break;
		case SAMPLE_ENUM_RET_ERR_ACT_ERASE:
		case SAMPLE_ENUM_RET_ERR_CMD_ERASE:
			printf_tiny("[Error] R_RFSP_EraseCodeFlashReq\r\n");
			break;
		case SAMPLE_ENUM_RET_ERR_ACT_WRITE:
		case SAMPLE_ENUM_RET_ERR_CMD_WRITE:
			printf_tiny("[Error] R_RFSP_WriteCodeFlashReq failed\r\n");
			break;
		case SAMPLE_ENUM_RET_ERR_CFDF_SEQUENCER:
			printf_tiny("[Error] CF/DF sequencer error\r\n");
			break;
		default:
			printf_tiny("[Error] ???\r\n");
			break;
	}
	printf_tiny("RFSP Operation failed\r\n");
// #else
// 	R_RFD_ForceReset();
#endif
}

#define  SAMPLE_START_SECTION_SMP_CF
#include "sample_memmap.h"

/********************************************************************************
 * Function Name : WriteFlash
 * Description	: This function initializes the TAU0 Channel 0 module.
 * Parameters	: addr -
 *					start address to write
 *				  data[] -
 *				  	data buffer to write
 *				  len -
 *				  	size of data buffer
 * Return value  : fsl_u08, status_code
 *                   = 0x00(FSL_OK), normal
 *                   = 0x05(FSL_ERR_PARAMETER), parameter error
 *                   = 0x10(FSL_ERR_PROTECTION), protection error
 *                   = 0x1C(FSL_ERR_WRITE), write error
 *                   = 0x1F(FSL_ERR_FLOW), last operation has not finished, yet.
 *                                      or violates the precondition.
 *                                      or FSL is suspending.
 *                   = 0xFF(FSL_BUSY), normal and means "process was started"
 ********************************************************************************/
uint8_t WriteCodeFlash(uint32_t u32_start_addr, uint8_t * u8_write_data, uint32_t u32_write_data_len)
 {
 	bool           l_e_sam_error_flag;
 	uint32_t       l_u32_count;

 	l_e_sam_error_flag = false;

    BSP_DI();

    if ((u32_start_addr & (FLASH_BYTE_PER_BLOCK - 1)) == 0)
    {      
        Sample_CodeFlashControl_Erase(u32_start_addr);
        #if 1
        printf_tiny("(boot)I2C transfer:ERASE_MEMORY(0x%04X," , u32_start_addr>>16);
        printf_tiny("0x%04X)\r\n" , u32_start_addr&0xFFFF);     
        #endif
    }   


    for (l_u32_count = 0u; (l_u32_count < u32_write_data_len) && (false == l_e_sam_error_flag);
        l_u32_count += 4u)
    {
        #if 1
        Sample_CodeFlashControl_SingleWrite(u32_start_addr + l_u32_count + 0, &u8_write_data[l_u32_count]);
        #else
        // printf_tiny("(boot)I2C transfer:Write(0x%02X)" , l_u32_count);
        // printf_tiny("0x%02X," , l_u32_count);
        // printf_tiny("0x%04X," ,u32_start_addr>>16);
        // printf_tiny("0x%04X:" ,u32_start_addr&0xFFFF);
        // printf_tiny("0x%02X," ,i2c_downloader_ctrl.write_data_length);

        // printf_tiny("0x%02X," , u8_write_data[l_u32_count + 0]);         
        // printf_tiny("0x%02X," , u8_write_data[l_u32_count + 1]);         
        // printf_tiny("0x%02X," , u8_write_data[l_u32_count + 2]);         
        // printf_tiny("0x%02X," , u8_write_data[l_u32_count + 3]);   

        // if ((l_u32_count+1)%16 ==0)
        // {
        //     printf("\r\n");
        // }  

        #endif

    }

 	return SAMPLE_ENUM_RET_STS_OK;
 }

 void EraseCodeFlash(void)
 {    
    BSP_DI();
    for (uint16_t i = FLASH_AREA/FLASH_BYTE_PER_BLOCK; i < FLASH_END_BLOCK; i++)	//0x1000/0x400=4, 32
    {
        Sample_CodeFlashControl_Erase(i*FLASH_BYTE_PER_BLOCK);
    }
 }

 void InitCodeFlash(void)
 {
    e_rfd_ret_t    l_e_rfd_status_flag;
    // e_sample_ret_t l_e_sam_ret_value;

    BSP_DI();
    /* Check whether HOCO is already started */
    if (SAMPLE_VALUE_U01_MASK0_1BIT == HIOSTOP)
    {
        /* Initialize RFD */
        l_e_rfd_status_flag = R_RFD_Init(CPU_FREQUENCY);
        printf_tiny("R_RFD_Init\r\n");
        
        if (R_RFD_ENUM_RET_STS_OK == l_e_rfd_status_flag)
        {
            /* Code flash blankcheck, erase, write, and iverify */
        }
        else
        {
            /* Failed to initialize RFD */
            // l_e_sam_ret_value = SAMPLE_ENUM_RET_ERR_PARAMETER;
            printf_tiny("SAMPLE_ENUM_RET_ERR_PARAMETER\r\n");
        }
    }
    else
    {
        /* HOCO is not started */
        // l_e_sam_ret_value = SAMPLE_ENUM_RET_ERR_CONFIGURATION;
        printf_tiny("SAMPLE_ENUM_RET_ERR_CONFIGURATION\r\n");
    }
    // BSP_EI();


    /******************************************************************************************************************
     * Set the code flash programming mode
     *****************************************************************************************************************/
    #if 0
    uint16_t       l_u16_count;
    uint8_t        l_au08_write_data[128];
    
    /* Create writing data */
    for (l_u16_count = 0u; l_u16_count < 128; l_u16_count++)
    {
        l_au08_write_data[l_u16_count] = (uint8_t)(l_u16_count & SAMPLE_VALUE_U08_MASK1_8BIT) + 10;
    }

    #if ENABLE_UART_PRINTF
    printf_tiny("(boot)before R_RFD_SetFlashMemoryMode\r\n");
    #endif   
    BSP_DI();
    Sample_CodeFlashControl(0x00020000uL, 128, l_au08_write_data);
    BSP_EI();
    #if ENABLE_UART_PRINTF
    printf_tiny("(boot)R_RFD_SetFlashMemoryMode rdy\r\n");
    #endif  
    
    // R_RFD_HOOK_EnterCriticalSection();

    // #if ENABLE_UART_PRINTF
    // printf_tiny("(boot)before R_RFD_SetFlashMemoryMode\r\n");
    // #endif   

    // l_e_rfd_status_flag = R_RFD_SetFlashMemoryMode(R_RFD_ENUM_FLASH_MODE_CODE_PROGRAMMING);


    // if (R_RFD_ENUM_RET_STS_OK != l_e_rfd_status_flag)
    // {
    //     // ErrorHandler(SAMPLE_ENUM_RET_ERR_MODE_MISMATCHED);
    // }
    // else
    // {
    //     /* No operation */
    //     #if ENABLE_UART_PRINTF
    //     printf_tiny("(boot)R_RFD_SetFlashMemoryMode rdy\r\n");
    //     #endif   
    // }

    #endif
 }

#define  SAMPLE_END_SECTION_SMP_CF
#include "sample_memmap.h"

#if ENALBE_IICA0_BL
static int target_recv_data_size_set(i2c_downloader_ctrl_t * p_i2c_downloader_ctrl)
{
	int 	status = 0;
	uint8_t start_byte = p_i2c_downloader_ctrl->p_rxbuff[0]; //First byte is Generic Code or the header byte

    switch(p_i2c_downloader_ctrl->recv_cmd)
    {
    	case ID_AUTHENTICATION_CMD_BYTE:
    	{
    		p_i2c_downloader_ctrl->rxbyteCnt = ID_AUTH_CMD_SIZE;
    		break;
    	}

		case INQUIRY_CMD_BYTE:
		{
			p_i2c_downloader_ctrl->rxbyteCnt = DEVICE_INQUIRY_CMD_SIZE;
			break;
		}

        case ERASE_CMD_BYTE:
        {
        	p_i2c_downloader_ctrl->rxbyteCnt = ERASE_CMD_SIZE;
            break;
        }

        case SIGNATURE_CMD_BYTE:
        {
        	p_i2c_downloader_ctrl->rxbyteCnt = SIGNATURE_REQUEST_CMD_SIZE;
            break;
        }

        case MEMORY_AREA_INFO_CMD_BYTE:
        {
        	p_i2c_downloader_ctrl->rxbyteCnt = MEMORY_AREA_INFO_CMD_SIZE;
            break;
        }

        case SWTICH_APP_CMD_BYTE:
        {
        	p_i2c_downloader_ctrl->rxbyteCnt = JUMP_APP_CMD_SIZE;
            break;
        }

        case WRITE_CMD_BYTE:
        {
            if(start_byte == COMMAND_PACKET_START)
            {
            	p_i2c_downloader_ctrl->rxbyteCnt = WRITE_CMD_SIZE;
            }
            else if(start_byte == DATA_PACKET_START)
            {
            	p_i2c_downloader_ctrl->write_data_length = (decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[1]) - 1); // Length of "RES+DATA"
            	p_i2c_downloader_ctrl->rxbyteCnt = p_i2c_downloader_ctrl->write_data_length + WRITE_DATA_HEADER_SIZE + 2;
            }
            break;
        }

        case READ_CMD_BYTE:
        {
            if(start_byte == COMMAND_PACKET_START)
            {
            	p_i2c_downloader_ctrl->rxbyteCnt = READ_CMD_SIZE;
            }
            else if(start_byte == DATA_PACKET_START)
            {
            	p_i2c_downloader_ctrl->rxbyteCnt = READ_DATA_STATUS_RESP_SIZE;
            }
            break;
        }

        default:
        	status = -1;
        	break;
    }

    return status;
}

static void received_handler(i2c_downloader_ctrl_t * p_i2c_downloader_ctrl)
{
    uint8_t     start_byte = p_i2c_downloader_ctrl->p_rxbuff[0]; //First byte is Generic Code or the header byte
    uint8_t     end_byte = p_i2c_downloader_ctrl->p_rxbuff[p_i2c_downloader_ctrl->rxbyteCnt - 1]; //End of packet
    uint8_t     i = 0;

    p_i2c_downloader_ctrl->recv_cmd = p_i2c_downloader_ctrl->p_rxbuff[3];

    /* check the ETX to make sure we receive a complete frame */
    if(end_byte != END_OF_PACKET)
    {
        p_i2c_downloader_ctrl->resp_type = PACKET_ERROR;
        return;
    }

    /* check the SUM to make sure the integrity of received frame */
    if(false == checksum_verification(&p_i2c_downloader_ctrl->p_rxbuff[0], p_i2c_downloader_ctrl->rxbyteCnt))
    {
        p_i2c_downloader_ctrl->resp_type = CHECKSUM_ERROR;
        return;
    }

    if(AUTHENTICATION_PHASE == p_i2c_downloader_ctrl->downloader_operation_phase)
    {
        if((start_byte == COMMAND_PACKET_START) && (p_i2c_downloader_ctrl->recv_cmd == INQUIRY_CMD_BYTE) &&
           (DEVICE_INQUIRY_CMD_SIZE == p_i2c_downloader_ctrl->rxbyteCnt))
        {
            p_i2c_downloader_ctrl->resp_type = INQUIRY_RESPOND;
            p_i2c_downloader_ctrl->downloader_operation_phase = COMMAND_ACCEPTABLE_PHASE;
        }
        else
        {
            p_i2c_downloader_ctrl->resp_type = FLOW_ERROR;
        }
        return;
    }

    /* check whether this is a Command packet or a Data packet */
    switch(p_i2c_downloader_ctrl->recv_cmd)
    {
        case INQUIRY_CMD_BYTE:
        {
            p_i2c_downloader_ctrl->resp_type = INQUIRY_RESPOND;
            break;
        }

        case ERASE_CMD_BYTE:
        {
            // if(false == Is_target_address_valid(&p_i2c_downloader_ctrl->p_rxbuff[4]))
            // {
            //     p_i2c_downloader_ctrl->resp_type = ADDRESS_ERROR;
            //     return;
            // }

            // erase_area_start_address = decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[6]);
            // erase_area_end_address = decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[10]);

            // erase_area_start_address = decode32bit(&p_i2c_downloader_ctrl->p_rxbuff[4]);
            // erase_area_end_address = decode32bit(&p_i2c_downloader_ctrl->p_rxbuff[8]);

            erase_area_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[7]& 0xFF;
            erase_area_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[6]<< 8;
            erase_area_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[5]<< 16;
            erase_area_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[4]<< 24;

            erase_area_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[11]& 0xFF;
            erase_area_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[10]<< 8;
            erase_area_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[9]<< 16;
            erase_area_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[8]<< 24;               

            // printf_tiny("(boot)received_handler erase_area_start:0x%04X," ,erase_area_start_address>>16);
            // printf_tiny("0x%04X\r\n" ,erase_area_start_address&0xFFFF);
            // printf_tiny("(boot)received_handler erase_area_end:0x%04X," ,erase_area_end_address>>16);
            // printf_tiny("0x%04X\r\n" ,erase_area_end_address&0xFFFF);
            // for( i = 0 ; i < 15; i++)
            // {
            //     printf_tiny("(boot)received_handler erase_area_end_address:(%d)" ,i);
            //     printf_tiny("0x%02X\r\n" ,p_i2c_downloader_ctrl->p_rxbuff[i]);
            // }

            p_i2c_downloader_ctrl->resp_type = ERASE_MEMORY;
            g_flash_operation_progressing = true;
            break;
        }

        case WRITE_CMD_BYTE:
        {
            if(start_byte == COMMAND_PACKET_START)
            {
                /* receive command packet of write transfer */
                // if(false == Is_target_address_valid(&p_i2c_downloader_ctrl->p_rxbuff[4]))
                // {
                //     p_i2c_downloader_ctrl->resp_type = ADDRESS_ERROR;
                //     return;
                // }

                // flash_memory_start_address = decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[6]);
                // flash_memory_end_address = decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[10]);

                // flash_memory_start_address = decode32bit(&p_i2c_downloader_ctrl->p_rxbuff[4]);
                // flash_memory_end_address = decode32bit(&p_i2c_downloader_ctrl->p_rxbuff[8]);

                flash_memory_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[7]& 0xFF;
                flash_memory_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[6]<< 8;
                flash_memory_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[5]<< 16;
                flash_memory_start_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[4]<< 24;

                flash_memory_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[11]& 0xFF;
                flash_memory_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[10]<< 8;
                flash_memory_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[9]<< 16;
                flash_memory_end_address |= (unsigned long) p_i2c_downloader_ctrl->p_rxbuff[8]<< 24;

                // printf_tiny("(boot)received_handler flash_memory_start:0x%04X," ,flash_memory_start_address>>16);
                // printf_tiny("0x%04X\r\n" ,flash_memory_start_address&0xFFFF);
                
                // printf_tiny("(boot)received_handler flash_memory_end:0x%04X," ,flash_memory_end_address>>16);
                // printf_tiny("0x%04X\r\n" ,flash_memory_end_address&0xFFFF);

                flash_memory_destination_address = flash_memory_start_address;
                flash_target_memory_size = flash_memory_end_address - flash_memory_start_address + 1;
                p_i2c_downloader_ctrl->resp_type = WRITE_MEMORY_START;
                g_flash_operation_progressing = true;
            }
            else if(start_byte == DATA_PACKET_START)
            {
                if(!g_flash_operation_progressing)
                {
                    p_i2c_downloader_ctrl->resp_type = SEQUENCER_ERROR;
                    return;
                }

                /* receive data packet of write transfer */
                p_i2c_downloader_ctrl->write_data_length = (decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[1]) - 1); // Length of "RES+DATA"
                p_i2c_downloader_ctrl->resp_type = WRITE_MEMORY_NEXT;
            }
            break;
        }

        case SIGNATURE_CMD_BYTE:
        {
            p_i2c_downloader_ctrl->resp_type = SIGNATURE_RESPOND;
            break;
        }

        case MEMORY_AREA_INFO_CMD_BYTE:
        {
            p_i2c_downloader_ctrl->area_num = p_i2c_downloader_ctrl->p_rxbuff[4];
            if(p_i2c_downloader_ctrl->area_num > TARGET_DEVICE_NOA)
            {
                p_i2c_downloader_ctrl->resp_type = ADDRESS_ERROR;
            }
            else
            {
                p_i2c_downloader_ctrl->resp_type = AREA_INFO_RESPOND;
            }
            break;
        }

        case SWTICH_APP_CMD_BYTE:
        {
            if(TWO_BYTE != (decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[1]) - 1))
            {
                p_i2c_downloader_ctrl->resp_type = PACKET_ERROR;
            }
            else
            {
                p_i2c_downloader_ctrl->image_version = decode16bit(&p_i2c_downloader_ctrl->p_rxbuff[4]);
                p_i2c_downloader_ctrl->resp_type = SWITCH_APP;
            }
            break;
        }

        default:
            p_i2c_downloader_ctrl->resp_type = UNSUPPORT_ERROR;
            break;
    }
}

static void transmit_handler(i2c_downloader_ctrl_t * p_i2c_downloader_ctrl)
{
    switch((uint8_t) p_i2c_downloader_ctrl->resp_type)
    {
        case BOOT_MODE_ACK_RESPOND:
        {
            p_i2c_downloader_ctrl->p_txbuff[0] = BOOT_MODE_ACK;
            p_i2c_downloader_ctrl->txbyteCnt = 0x01;
            break;
        }

        case AUTH_RESPOND:
        {
            memcpy(&p_i2c_downloader_ctrl->p_txbuff[0], device_auth_status_ok, DATA_STATUS_RESPONSE_BYTES);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case INQUIRY_RESPOND:
        {
            memcpy(&p_i2c_downloader_ctrl->p_txbuff[0], device_inquiry_data_status_ok, DATA_STATUS_RESPONSE_BYTES);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case ERASE_MEMORY:
        {
            if(!g_flash_operation_progressing)
            {
                /* flash erase operation is completed */
                memcpy(&p_i2c_downloader_ctrl->p_txbuff[0], device_memory_erase_status_ok, DATA_STATUS_RESPONSE_BYTES);
                p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            }
            else
            {
                /* flash erase operation is not completed */
                assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_ERASE_ERROR);
                p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
                /* cancel the flash operation */
                g_flash_operation_progressing = false;
            }
            break;
        }

        case WRITE_MEMORY_START:
        {
            if(g_flash_operation_progressing)
            {
                /* flash programming setup is ready */
                memcpy(&p_i2c_downloader_ctrl->p_txbuff[0], device_memory_write_status_ok, DATA_STATUS_RESPONSE_BYTES);
                p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            }
            else
            {
                /* flash programming operation does not start properly */
                assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_PACKET_ERROR);
                p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            }
            break;
        }

        case WRITE_MEMORY_NEXT:
        {
            memcpy(&p_i2c_downloader_ctrl->p_txbuff[0], device_memory_write_status_ok, DATA_STATUS_RESPONSE_BYTES);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case SIGNATURE_RESPOND:
        {
            p_i2c_downloader_ctrl->p_txbuff[0] = DATA_PACKET_START;                 //SOD
            p_i2c_downloader_ctrl->p_txbuff[1] = 0x00;                              //LNH
            p_i2c_downloader_ctrl->p_txbuff[2] = 0x14;                              //LNL
            p_i2c_downloader_ctrl->p_txbuff[3] = p_i2c_downloader_ctrl->recv_cmd;   //RES
            p_i2c_downloader_ctrl->p_txbuff[4] = TARGET_DEVICE_NOA;                 //NOA, number of recordable areas
            p_i2c_downloader_ctrl->p_txbuff[5] = TARGET_DEVICE_TYPE;               //TYP
            memcpy(&p_i2c_downloader_ctrl->p_txbuff[6], &device_part_number[0], 13);
            p_i2c_downloader_ctrl->p_txbuff[19] = BOOT_FIRMWARE_MAJOR_VERSION;      //BFV Major
            p_i2c_downloader_ctrl->p_txbuff[20] = BOOT_FIRMWARE_MINOR_VERSION;      //BFV Minor
            p_i2c_downloader_ctrl->p_txbuff[21] = 1;   //AFV Major
            p_i2c_downloader_ctrl->p_txbuff[22] = 0;   //AFV Minor
            p_i2c_downloader_ctrl->p_txbuff[23] = checksum_calculate(p_i2c_downloader_ctrl->p_txbuff, 25);//SUM
            p_i2c_downloader_ctrl->p_txbuff[24] = END_OF_PACKET;
            p_i2c_downloader_ctrl->txbyteCnt = 25;
            break;
        }

        case AREA_INFO_RESPOND:
        {
            p_i2c_downloader_ctrl->p_txbuff[0] = DATA_PACKET_START;                 //SOD
            p_i2c_downloader_ctrl->p_txbuff[1] = 0x00;                              //LNH
            p_i2c_downloader_ctrl->p_txbuff[2] = 0x16;                              //LNL
            p_i2c_downloader_ctrl->p_txbuff[3] = p_i2c_downloader_ctrl->recv_cmd;   //RES
            p_i2c_downloader_ctrl->p_txbuff[4] = p_i2c_downloader_ctrl->area_num;   //KOA
            //SAD (Start Address) => 0x5000 only needs 2 bytes
			p_i2c_downloader_ctrl->p_txbuff[5] = 0;
			p_i2c_downloader_ctrl->p_txbuff[6] = 0;
			// p_i2c_downloader_ctrl->p_txbuff[7] = (uint8_t)(FLASH_AREA>>8);
			// p_i2c_downloader_ctrl->p_txbuff[8] = (uint8_t)(FLASH_AREA&0xFF);
			encode16bit(FLASH_AREA, &p_i2c_downloader_ctrl->p_txbuff[7]);
			//EAD (End Address) => 0x3FFFF only needs 3 bytes
            #if 1
			p_i2c_downloader_ctrl->p_txbuff[9] = 0;
			// p_i2c_downloader_ctrl->p_txbuff[10] = (uint8_t)(FLASH_END_ADDRESS>>16);
			// p_i2c_downloader_ctrl->p_txbuff[11] = (uint8_t)(FLASH_END_ADDRESS>>8);
			// p_i2c_downloader_ctrl->p_txbuff[12] = (uint8_t)(FLASH_END_ADDRESS&0xFF);
			encode16bit(FLASH_END_ADDRESS, &p_i2c_downloader_ctrl->p_txbuff[10]);
            #else
			p_i2c_downloader_ctrl->p_txbuff[9] = 0;
			p_i2c_downloader_ctrl->p_txbuff[10] = 0;
			encode16bit(FLASH_END_ADDRESS, &p_i2c_downloader_ctrl->p_txbuff[11]);
            #endif
			//EAU (Erase Access Unit) => 0x400 only needs 2 bytes
			p_i2c_downloader_ctrl->p_txbuff[13] = 0;
			p_i2c_downloader_ctrl->p_txbuff[14] = 0;
			encode16bit(FLASH_BYTE_PER_BLOCK, &p_i2c_downloader_ctrl->p_txbuff[15]);
			//MWAU (Max Write Access Unit) => 256 only needs 2 bytes
			p_i2c_downloader_ctrl->p_txbuff[17] = 0;
			p_i2c_downloader_ctrl->p_txbuff[18] = 0;
			encode16bit(MAX_CODE_FLASH_WRITE_DATA_SIZE, &p_i2c_downloader_ctrl->p_txbuff[19]);
			//MRAU (Max Read Access Unit) => 256 only needs 2 bytes
			p_i2c_downloader_ctrl->p_txbuff[21] = 0;
			p_i2c_downloader_ctrl->p_txbuff[22] = 0;
            encode16bit(MAX_FLASH_READ_DATA_SIZE, &p_i2c_downloader_ctrl->p_txbuff[23]);
            p_i2c_downloader_ctrl->p_txbuff[25] = checksum_calculate(p_i2c_downloader_ctrl->p_txbuff, 27);//SUM
            p_i2c_downloader_ctrl->p_txbuff[26] = END_OF_PACKET;
            p_i2c_downloader_ctrl->txbyteCnt = 27;
            break;
        }

        case SWITCH_APP:
        {
            memcpy(&p_i2c_downloader_ctrl->p_txbuff[0], device_memory_switch_app_ok, DATA_STATUS_RESPONSE_BYTES);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            g_app_required_update = false;
            break;
        }

        case WRITE_ERROR:
        {
            assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_WRITE_ERROR);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case PACKET_ERROR:
        {
            assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_PACKET_ERROR);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case CHECKSUM_ERROR:
        {
            assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_CHECKSUM_ERROR);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case ADDRESS_ERROR:
        {
            assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_ADDRESS_ERROR);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case FLOW_ERROR:
        {
            assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_FLOW_ERROR);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case SEQUENCER_ERROR:
        {
            assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_SEQUENCER_ERROR);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }

        case UNSUPPORT_ERROR:
        {
            assemble_error_data_packet(p_i2c_downloader_ctrl->p_txbuff,(uint8_t)p_i2c_downloader_ctrl->recv_cmd, STATUS_CODE_UNSUPPORTED_CMD);
            p_i2c_downloader_ctrl->txbyteCnt = DATA_STATUS_RESPONSE_BYTES;
            break;
        }
    }
}
void I2C_Downloader_routine(void)
{
	downloader_iica0_tx_cnt = 0U;
	downloader_iica0_slave_status_flag = 0U;

	//Start I2C operation
	IICE0 = 1U;

	while(1)
	{
		//wait for an I2C intterupt flag set
		while(IICAIF0 == 0U){};
		//clear the interrupt flag
		IICAIF0 = 0;

		//if STOP condition is received
		if (1U == SPD0)
		{
			//Disable STOP condition detection/interrupt generation
			SPIE0 = 0U;
			WREL0 = 1U;
			downloader_iica0_slave_status_flag = 0U;
			break;
		}
		else
		{
			//for I2C repeated start (restart)
			if(1U == STD0){
				downloader_iica0_slave_status_flag = 0;
			}

			if (0U == (downloader_iica0_slave_status_flag & _80_IICA_ADDRESS_COMPLETE))
			{
				//Address match
				if (1U == COI0)
				{
					//Enable STOP condition detection/interrupt generation
					SPIE0 = 1U;
					downloader_iica0_slave_status_flag |= _80_IICA_ADDRESS_COMPLETE;

					//I2C slave transmit requested
					if (1U == TRC0)
					{
						//Change interrupt generation from 8 th to 9 th falling edge
						WTIM0 = 1U;

						transmit_handler(&i2c_downloader_ctrl);
						i2c_downloader_ctrl.resp_type = NONE_RESP;
						//if there is something to send
						if (i2c_downloader_ctrl.txbyteCnt != 0U)
						{
							IICA0 = i2c_downloader_ctrl.p_txbuff[downloader_iica0_tx_cnt];
							downloader_iica0_tx_cnt++;
							i2c_downloader_ctrl.txbyteCnt -= downloader_iica0_tx_cnt;
						}
						else
						{
							#if 1 //Send dummy data?
							IICA0 = 0;
							#endif
							//release clock stretching
							WREL0 = 1U;
						}
					}
					//I2C slave receive requested
					else
					{
						// receive data portion of Master Write Slave Read is started from here
						downloader_iica0_rx_cnt = 0;
						ACKE0 = 1U; //Enable ACK
						WTIM0 = 0U; //Change interrupt generation from 9 th to 8 th falling edge for slave read data
						WREL0 = 1U; //release clock stretching
					}
				}
			}
			else
			{
				//I2C slave transmit requested
				if (1U == TRC0)
				{
					//ACK was not detected
					if (0U == ACKD0)
					{
						//release clock stretching
						WREL0 = 1U;
					}
					else
					{
						if (i2c_downloader_ctrl.txbyteCnt != 0U)
						{
							IICA0 = i2c_downloader_ctrl.p_txbuff[downloader_iica0_tx_cnt];
							downloader_iica0_tx_cnt++;
							i2c_downloader_ctrl.txbyteCnt -= downloader_iica0_tx_cnt;
						}
						else
						{
							//exit from communication mode
							LREL0 = 1U;
						}
					}
				}
				//I2C slave receive requested
				else
				{
					i2c_downloader_ctrl.p_rxbuff[downloader_iica0_rx_cnt] = IICA0;
					if(COMMUNICATION_SETTING_PHASE == i2c_downloader_ctrl.downloader_operation_phase)
					{
	                    if(i2c_downloader_ctrl.p_rxbuff[0] == BOOT_MODE_CHECK_CMD)
	                    {
	                    	i2c_downloader_ctrl.resp_type = BOOT_MODE_ACK_RESPOND;
	                    	i2c_downloader_ctrl.downloader_operation_phase = AUTHENTICATION_PHASE;
	                    }

	                    WTIM0 = 1U; //Change interrupt generation from 8 th to 9 th falling edge for slave write data
	                    WREL0 = 1U; //release clock stretching
					}
					else
					{
						downloader_iica0_rx_cnt++;
						if(downloader_iica0_rx_cnt > 3)
						{
							i2c_downloader_ctrl.recv_cmd = i2c_downloader_ctrl.p_rxbuff[3];
				            target_recv_data_size_set(&i2c_downloader_ctrl);
						}
						else if(downloader_iica0_rx_cnt > MAXIMUM_RECV_PACKET_SIZE)
						{
							downloader_iica0_rx_cnt = 0;
						}

						if((INVALID_CMD_BYTE != i2c_downloader_ctrl.recv_cmd) &&
						   (downloader_iica0_rx_cnt == i2c_downloader_ctrl.rxbyteCnt))
						{
							received_handler(&i2c_downloader_ctrl);
							WTIM0 = 1U; //Change interrupt generation from 8 th to 9 th falling edge for slave write data
						}

						WREL0 = 1U; //release clock stretching
					}
				}
			}
		}
	}
}

static void encode16bit(uint32_t input_data, uint8_t * output_array)
{
    if (input_data > 0xFFFF)
    {
        output_array[2] = (uint8_t)(input_data & 0x00ff);
        output_array[1] = (uint8_t)((input_data>>8) & 0x00ff);
        output_array[0] = (uint8_t)((input_data>>16) & 0x00ff);
    }
    else
    {
        output_array[1] = (uint8_t)(input_data & 0x00ff);
        output_array[0] = (uint8_t)((input_data>>8) & 0x00ff);
    }
}

static uint16_t decode16bit(uint8_t * data)
{
    return (uint16_t)( (data[1]) | (data[0] << 8) );
}

static uint32_t decode32bit(uint8_t * data)
{
    return (uint32_t)( (data[3]) | (data[2] << 8) | (data[1] << 16) | (data[0] << 24) );
}

static bool checksum_verification(uint8_t const * p_input, uint16_t frame_len)
{
    uint8_t wCRC = 0;
    uint16_t index = 1;
    uint16_t computed_length = frame_len - 2; // remove SOH, ETX bytes

    while (computed_length--)
    {
        wCRC += p_input[index];
        index++;
    }

    if (wCRC == 0)
    {
        return true;
    }

    return  false;
}

static uint8_t checksum_calculate(uint8_t const * p_input, uint16_t frame_len)
{
    uint8_t sum = 0;
    uint16_t index = 1;
    uint16_t computed_length = frame_len - 3; // remove SOD, SUM, ETX bytes

    // SUM = 0x00 - (LNH + LNL + RES + Data)
    while (computed_length--)
    {
        sum += p_input[index];
        index++;
    }

    return (uint8_t)(0x00 - sum);
}

static void assemble_error_data_packet(uint8_t * txdata, uint8_t associated_cmd, uint8_t error_code)
{
    txdata[0] = DATA_PACKET_START;          //SOD
    txdata[1] = 0x00;                       //LNH
    txdata[2] = 0x02;                       //LNL
    txdata[3] = (associated_cmd | 0x80);    //RES
    txdata[4] = error_code;                 //STS
    txdata[5] = checksum_calculate(txdata, DATA_STATUS_RESPONSE_BYTES); //SUM
    txdata[6] = END_OF_PACKET;
}

static bool Is_target_address_valid(uint8_t *data)
{
	uint16_t target_sad = (uint16_t)( (data[3]) | (data[2] << 8) );
	uint16_t target_ead = (uint16_t)( (data[7]) | (data[6] << 8) );

    if(target_sad > target_ead)
    {
        /* The start address is bigger than the end address */
        return false;
    }

    if ((target_sad >= FLASH_AREA) && (target_sad < FLASH_END_ADDRESS)
            && (target_ead > FLASH_END_ADDRESS))
    {
        /* The start address is in user code flash area, but end address is not */
        return false;
    }

    if ((target_sad >= FLASH_AREA) && (target_sad < FLASH_END_ADDRESS)
            && (target_ead > FLASH_END_ADDRESS))
    {
        /* The start address is in user data flash area, but end address is not */
        return false;
    }

    return true;
}

void boot_R_Config_IICA0_Create(void)
{
    IICA0EN = 1U;    /* enables input clock supply */
    IICE0 = 0U;
    IICAMK0 = 1U;    /* disable INTIICA0 interrupt */
    IICAIF0 = 0U;    /* clear INTIICA0 interrupt flag */
    /* Set INTIICA0 low priority */
    IICAPR10 = 1U;
    IICAPR00 = 1U;
    /* Set SCLA0, SDAA0 pin */
    POM6 |= 0x0CU;
    P6 &= 0xF3U;
    PM6 |= 0x0CU;
    IICWL0 = _0A_IICA0_IICWL_VALUE;
    IICWH0 = _0A_IICA0_IICWH_VALUE;
    IICRSV0 = 1U;
    SMC0 = 1U;
    DFC0 = 0U;    /* digital filter off */
    IICCTL01 |= _01_IICA_FCLK_HALF;
    SVA0 = _12_IICA0_SLAVEADDRESS;
    STCEN0 = 1U;
    SPIE0 = 0U;
    WTIM0 = 1U;
    ACKE0 = 1U;
    IICAMK0 = 0U;
    IICE0 = 1U;
    LREL0 = 1U;
    /* Set SCLA0, SDAA0 pin */
    PM6 &= 0xF3U;

    // R_Config_IICA0_Create_UserInit();
}

/***********************************************************************************************************************
* Function Name: R_Config_IICA0_Stop
* Description  : This function stops IICA0 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void boot_R_Config_IICA0_Stop(void)
{
    IICE0 = 0U;    /* disable IICA0 operation */
}

#endif


uint8_t verify_application_chksum(void)
{
	bool status = true;
    uint32_t CRC_rom = 0;
    uint32_t CRC_ccitt = 0;
    
    #if 1   // remove if need to pass checksum
    printf_tiny("(boot)verify checksum\r\n");

    // prevent app data corrupt , stay in boot loader
    CRC_ccitt = *(__far unsigned long *)CRC_ADDRESS;				    
    // CRC_rom = calcrc(FLASH_AREA, (CRC_ADDRESS - FLASH_AREA));	    //0x1000, 0x7FFC-0x1000=0x6FFC
    CRC_rom = caculate_crc32_checksum(FLASH_AREA,FLASH_AP_SIZE);
    
    #if ENABLE_UART_PRINTF
    printf_tiny("(boot)In app code:0x%X",CRC_ccitt>>16);
    printf_tiny("%X\r\n",CRC_ccitt&0xFFFF);
    printf_tiny("(boot)caculated:0x%X",CRC_rom>>16);
    printf_tiny("%X\r\n",CRC_rom&0xFFFF);
    #endif
    
    if( CRC_ccitt != CRC_rom )
    {
		status = false;
    }
    #endif

	return status;
}

bool condition_check(void)
{
	uint32_t* p_flash_empty;
	bool status = false;

	//if boot sign is there, then stay in bootloader
	if(reset_to_bootloader == RESET_TO_BOOT_SIGN)
    {
		status = true;
        
        #if ENABLE_UART_PRINTF
        printf_tiny("(boot)reset_to_bootloader:0x%X",reset_to_bootloader>>16);
        printf_tiny("%X\r\n",reset_to_bootloader&0xFFFF);
        #endif
	}
	//maybe just once after reset, then clear the sign
	reset_to_bootloader = 0;

	//if FLASH are all erased, then stay in bootloader
	p_flash_empty = (uint32_t*)FLASH_AREA;
	if(*p_flash_empty == 0xFFFFFFFF)
    {
		status = true;
        
        #if ENABLE_UART_PRINTF
        printf_tiny("(boot)app code flash empty\r\n");
        #endif
	}

	//if CRC error, then stay in bootloader

	return status;
}

// void _BusyWaitDelayMs(void)
// {
//     volatile uint32_t i;
//     BSP_DI();
//     #if 0   // wait delay , 1.0562ms
//     for (i = 0; i < 2000; i++) 
//     {
//         __nop();
//     }
//     #else   // timer delay , 1.007ms @ clock src 2500KHz , fCLK/2^4 , 1ms
//     boot_R_Config_TAU0_0_Start();
//     while(!(TCR00 == 0));
//     boot_R_Config_TAU0_0_Stop();
//     #endif
//     BSP_EI();
// }


// void delayMearsurement_in_polling(void)
// {
//     unsigned char i = 0;
//     for ( i = 0; i < 20 ; i++)
//     {
//         P7_bit.no3 = ~P7_bit.no3;// P73
//         _BusyWaitDelayMs();
//     }
// }

// 115200 : 1 bytes (10bit) = 0.08ms
int boot_UART1_Get_withTimeOut(unsigned long timeout, unsigned char *data)
{
	unsigned long tick_count;

	tick_count = timeout;

    #if 0
	while (SRIF1 == 0)
	{
	    _BusyWaitDelayMs();
        tick_count--;
        if (tick_count == 0)
        {
            break;
        }
	}
    SRIF1 = 0;

	if (tick_count == 0)
	{
		*data = '0';
		return FALSE;
	}

    *data = SDR11L;
    #else

	boot_R_Config_TAU0_0_Start();
	
	/* wait for a byte to arrive */
	while ((SRIF1 == 0) && (tick_count))
	{
		if (TMIF00 == 1)
		{
			TMIF00 = 0;
			boot_R_Config_TAU0_0_Stop();
			if (--tick_count)
			{
				boot_R_Config_TAU0_0_Start();
			}
		}
	}
	
	if (tick_count == 0)
	{
		*data = '0';
		return FALSE;
	}
		
	*data=(uint8_t) SDR11L;
	SRIF1 = 0;

    #endif
    
	return TRUE;
}

int boot_UART1_Get(void)
{
    unsigned char c = 0;
    // BSP_DI();

    // SRIF1 = 0;
    while (SRIF1 == 0);
    SRIF1 = 0;
    c = SDR11L;

    // BSP_EI();
    return c;
}

void boot_UART1_SendByte(int c)
{
    // BSP_DI();

    #if 0
    while (STIF1 == 0);
    STIF1 = 0;
    SDR10L = (unsigned char)c;
    #else
    STMK1 = 1U;    /* disable interrupt */
    SDR10L = (unsigned char)c;
    // STIF1 = 0U;    /* clear interrupt flag */  
    while(STIF1 == 0)
    {

    }
    STIF1 = 0U;    /* clear interrupt flag */  
    #endif

    // BSP_EI();
}


// void boot_UART1_sendString(unsigned char __near * str)
// {
//     while (*str) {
//         boot_UART1_SendByte(*str++);   // Send each character in the string
//     }
// }

#if ENABLE_UART_BL

void boot_R_Config_UART1_Create(void)
{
    SPS1 &= _00F0_SAU_CKM0_CLEAR;
    SPS1 |= _0001_SAU_CKM0_FCLK_1;
    ST1 |= (_0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON);
    STMK1 = 1U;    /* disable INTST1 interrupt */
    STIF1 = 0U;    /* clear INTST1 interrupt flag */
    SRMK1 = 1U;    /* disable INTSR1 interrupt */
    SRIF1 = 0U;    /* clear INTSR1 interrupt flag */
    /* Set INTST1 low priority */
    STPR11 = 1U;
    STPR01 = 1U;
    /* Set INTSR1 low priority */
    SRPR11 = 1U;
    SRPR01 = 1U;
    SMR10 = _0020_SAU_SMRMN_INITIALVALUE | _0000_SAU_CLOCK_SELECT_CKM0 | _0000_SAU_TRIGGER_SOFTWARE | 
            _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
    SCR10 = _8000_SAU_TRANSMISSION | _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0007_SAU_LENGTH_8;
    SDR10 = _AC00_SAU1_CH0_TRANSMIT_DIVISOR;
    NFEN0 |= _04_SAU_RXD1_FILTER_ON;
    SIR11 = _0004_SAU_FEF_CLEARED | _0002_SAU_PEF_CLEARED | _0001_SAU_OVF_CLEARED;    /* clear error flag */
    SMR11 = _0020_SAU_SMRMN_INITIALVALUE | _0000_SAU_CLOCK_SELECT_CKM0 | _0100_SAU_TRIGGER_RXD | 
            _0000_SAU_EDGE_FALLING | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
    SCR11 = _4000_SAU_RECEPTION | _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0007_SAU_LENGTH_8;
    SDR11 = _AC00_SAU1_CH1_RECEIVE_DIVISOR;
    SO1 |= _0001_SAU_CH0_DATA_OUTPUT_1;
    SOL1 |= _0000_SAU_CHANNEL0_NORMAL;    /* output level normal */
    SOE1 |= _0001_SAU_CH0_OUTPUT_ENABLE;    /* enable UART1 output */
    /* Set TXD1 pin */
    PMC7 &= 0xEFU;
    P7 |= 0x10U;
    PM7 &= 0xEFU;
    /* Set RXD1 pin */
    PM7 |= 0x20U;

    // R_Config_UART1_Create_UserInit();
}

/***********************************************************************************************************************
* Function Name: R_Config_UART1_Start
* Description  : This function starts UART1 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void boot_R_Config_UART1_Start(void)
{
    SO1 |= _0001_SAU_CH0_DATA_OUTPUT_1;    /* output level normal */
    SOE1 |= _0001_SAU_CH0_OUTPUT_ENABLE;    /* enable UART1 output */
    SS1 |= (_0002_SAU_CH1_START_TRG_ON | _0001_SAU_CH0_START_TRG_ON);    /* enable UART1 receive and transmit */
    STIF1 = 0U;    /* clear INTST1 interrupt flag */
    SRIF1 = 0U;    /* clear INTSR1 interrupt flag */
    // STMK1 = 0U;    /* enable INTST1 interrupt */
    // SRMK1 = 0U;    /* enable INTSR1 interrupt */
}

/***********************************************************************************************************************
* Function Name: R_Config_UART1_Stop
* Description  : This function stops UART1 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void boot_R_Config_UART1_Stop(void)
{
    STMK1 = 1U;    /* disable INTST1 interrupt */
    SRMK1 = 1U;    /* disable INTSR1 interrupt */
    ST1 |= (_0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON);
    SOE1 &= (uint16_t)~_0001_SAU_CH0_OUTPUT_ENABLE;    /* disable UART1 output */
    STIF1 = 0U;    /* clear INTST1 interrupt flag */
    SRIF1 = 0U;    /* clear INTSR1 interrupt flag */
}


void boot_R_SAU1_Create(void)
{
    SAU1EN = 1U;    /* supplies input clock */
    boot_R_Config_UART1_Create();
}

#endif


#if ENABLE_UART_PRINTF
void boot_R_Config_UART0_Create(void)
{
    SPS0 &= _00F0_SAU_CKM0_CLEAR;
    SPS0 |= _0001_SAU_CKM0_FCLK_1;
    ST0 |= (_0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON);
    STMK0 = 1U;    /* disable INTST0 interrupt */
    STIF0 = 0U;    /* clear INTST0 interrupt flag */
    SRMK0 = 1U;    /* disable INTSR0 interrupt */
    SRIF0 = 0U;    /* clear INTSR0 interrupt flag */
    /* Set INTST0 low priority */
    STPR10 = 1U;
    STPR00 = 1U;
    /* Set INTSR0 low priority */
    SRPR10 = 1U;
    SRPR00 = 1U;
    SMR00 = _0020_SAU_SMRMN_INITIALVALUE | _0000_SAU_CLOCK_SELECT_CKM0 | _0000_SAU_TRIGGER_SOFTWARE | 
            _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
    SCR00 = _8000_SAU_TRANSMISSION | _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0007_SAU_LENGTH_8;
    SDR00 = _AC00_SAU0_CH0_TRANSMIT_DIVISOR;
    NFEN0 |= _01_SAU_RXD0_FILTER_ON;
    SIR01 = _0004_SAU_FEF_CLEARED | _0002_SAU_PEF_CLEARED | _0001_SAU_OVF_CLEARED;    /* clear error flag */
    SMR01 = _0020_SAU_SMRMN_INITIALVALUE | _0000_SAU_CLOCK_SELECT_CKM0 | _0100_SAU_TRIGGER_RXD | 
            _0000_SAU_EDGE_FALLING | _0002_SAU_MODE_UART | _0000_SAU_TRANSFER_END;
    SCR01 = _4000_SAU_RECEPTION | _0000_SAU_PARITY_NONE | _0080_SAU_LSB | _0010_SAU_STOP_1 | _0007_SAU_LENGTH_8;
    SDR01 = _AC00_SAU0_CH1_RECEIVE_DIVISOR;
    SO0 |= _0001_SAU_CH0_DATA_OUTPUT_1;
    SOL0 |= _0000_SAU_CHANNEL0_NORMAL;    /* output level normal */
    SOE0 |= _0001_SAU_CH0_OUTPUT_ENABLE;    /* enable UART0 output */
    /* Set TXD0 pin */
    P1 |= 0x20U;
    PM1 &= 0xDFU;
    /* Set RXD0 pin */
    PM1 |= 0x40U;

    // R_Config_UART0_Create_UserInit();
}

void boot_R_Config_UART0_Start(void)
{
    SO0 |= _0001_SAU_CH0_DATA_OUTPUT_1;    /* output level normal */
    SOE0 |= _0001_SAU_CH0_OUTPUT_ENABLE;    /* enable UART0 output */
    SS0 |= (_0002_SAU_CH1_START_TRG_ON | _0001_SAU_CH0_START_TRG_ON);    /* enable UART0 receive and transmit */
    STIF0 = 0U;    /* clear INTST0 interrupt flag */
    SRIF0 = 0U;    /* clear INTSR0 interrupt flag */
    STMK0 = 0U;    /* enable INTST0 interrupt */
    SRMK0 = 0U;    /* enable INTSR0 interrupt */
}

void boot_R_Config_UART0_Stop(void)
{
    STMK0 = 1U;    /* disable INTST0 interrupt */
    SRMK0 = 1U;    /* disable INTSR0 interrupt */
    ST0 |= (_0002_SAU_CH1_STOP_TRG_ON | _0001_SAU_CH0_STOP_TRG_ON);
    SOE0 &= (uint16_t)~_0001_SAU_CH0_OUTPUT_ENABLE;    /* disable UART0 output */
    STIF0 = 0U;    /* clear INTST0 interrupt flag */
    SRIF0 = 0U;    /* clear INTSR0 interrupt flag */
}

// void boot_R_Config_UART0_Send_Byte(int8_t data)
// {
// 	if ((SRIF0) && (SSR00 != 0))        /* if any error in status register */
// 	{
// 		SRIF0 = 0;      				/* clear error interrupt request flag */
// 		SIR00 |= (SSR00&0x000F);
// 	}

// 	SDR00L = data;     					/* load character to transmit register */
// 	while (STIF0 == 0)					/* wait while transmission in progress */
// 	{
// 	}
// 	STIF0 = 0;       					/* clear transmission interrupt request flag */
// }

/********************************************************************************
* Function Name: R_Config_UART0_Send_String
* Description  : This function sends UART0 data string.
* Arguments    : data -
*					string buffer
* Return Value : None
********************************************************************************/
// void boot_R_Config_UART0_Send_String(__far int8_t buffer[])
// {
// 	int8_t i;

// 	for (i = 0; buffer[i] != 0; i++)
// 	{
// 		boot_R_Config_UART0_Send_Byte(buffer[i]);
// 	}
// }


void boot_R_SAU0_Create(void)
{
    SAU0EN = 1U;    /* supplies input clock */

    boot_R_Config_UART0_Create();

}
#endif

void boot_R_Config_TAU0_0_Stop(void)
{
    TT0 |= _0001_TAU_CH0_STOP_TRG_ON;
}

void boot_R_Config_TAU0_0_Start(void)
{
    TS0 |= _0001_TAU_CH0_START_TRG_ON;
}

void boot_R_Config_TAU0_0_Create(void)
{
    TPS0 &= _FFF0_TAU_CKM0_CLEAR;
    TPS0 |= _0004_TAU_CKM0_FCLK_4;
    /* Stop channel 0 */
    TT0 |= _0001_TAU_CH0_STOP_TRG_ON;
    /* TAU00 used as interval timer */
    TMR00 = _0000_TAU_CLOCK_SELECT_CKM0 | _0000_TAU_CLOCK_MODE_CKS | _0000_TAU_TRIGGER_SOFTWARE | 
            _0000_TAU_MODE_INTERVAL_TIMER | _0000_TAU_START_INT_UNUSED;
    TDR00 = _09C3_TAU_TDR00_VALUE;
    TO0 &= (uint16_t)~_0001_TAU_CH0_OUTPUT_VALUE_1;
    TOE0 &= (uint16_t)~_0001_TAU_CH0_OUTPUT_ENABLE;

    // R_Config_TAU0_0_Create_UserInit();
}


void boot_R_TAU0_Create(void)
{
    TAU0EN = 1U;    /* start TAU0 clock */
    /* Set TAU0 settings */
    boot_R_Config_TAU0_0_Create();
}

void boot_R_Config_PORT_Create(void)
{

    // R_Config_PORT_Create_UserInit();
}


void boot_mcu_clock_setup(void)
{

    uint8_t    cmc_tmp;

#if ((BSP_CFG_SUBCLK_OPERATION == 0) && (BSP_CFG_SUBCLK_SOURCE == 1)) || \
    (BSP_CFG_PLL_OPERATION == 1)
    volatile uint32_t    w_count;
#endif

#if (BSP_CFG_HISYSCLK_OPERATION == 0) && (BSP_CFG_HISYSCLK_SOURCE == 1)
    uint8_t    tmp_stab_wait;
    uint8_t    tmp_stab_set;
#endif

    cmc_tmp = 0x00U;

    /* High-speed on-chip oscillator(fIH) setting */
    /* High-Speed On-Chip Oscillator Frequency Select Register(HOCODIV) setting */
    HOCODIV = BSP_CFG_HOCO_DIVIDE;

    /* High-speed system clock(fMX) setting */
#if BSP_CFG_HISYSCLK_SOURCE == 0
    /* Input port mode */
    /* Not used.
     * When using high-speed on-chip oscillator,
     * when not using main system clock
     */
#elif BSP_CFG_HISYSCLK_SOURCE == 1
    /* X1 oscillation mode */
    /* High-speed system clock pin operation mode(EXCLK/OSCSEL) setting */
    /* Control of X1 clock oscillation frequency(AMPH) setting */
#if (BSP_CFG_FMX_HZ >= 2000000) && (BSP_CFG_FMX_HZ <= 10000000)
    /* 2 MHz <= fX <= 10 MHz */
    cmc_tmp |= 0x40U;
#else
    /* 2 MHz <= fX <= 20 MHz */
    cmc_tmp |= 0x41U;
#endif
#else
    /* External clock input mode */
    /* High-speed system clock pin operation mode(EXCLK/OSCSEL) setting */
    cmc_tmp |= 0xC0U;
#endif

    /* Subsystem clock oscillator clock(fSX) setting */
#if BSP_CFG_SUBCLK_SOURCE == 0
    /* Not used.
     * When using the low-speed on-chip oscillator or
     * when not using subsystem clock(fSUB)
     */
#elif BSP_CFG_SUBCLK_SOURCE == 1
#if BSP_CFG_MCU_PART_PIN_NUM < 1
    /* 32-pin device */
    /* Input port mode(low-speed on-chip oscillator operation mode) */
    /* EXCLKS/OSCSELS setting */
    cmc_tmp |= 0x10U;
#else
    /* XT1 oscillation mode */
#if BSP_CFG_XT1_OSCMODE == 0
    /* Low power consumption oscillation (default) Oscillation margin : Medium */
    /* EXCLKS/OSCSELS/AMPHS1/AMPHS0 setting */
    cmc_tmp |= 0x10U;
#elif BSP_CFG_XT1_OSCMODE == 1
    /* Normal oscillation Oscillation margin : High */
    /* EXCLKS/OSCSELS/AMPHS1/AMPHS0 setting */
    cmc_tmp |= 0x12U;
#else
    /* Ultra-low power consumption oscillation Oscillation margin : Low */
    /* EXCLKS/OSCSELS/AMPHS1/AMPHS0 setting */
    cmc_tmp |= 0x14U;
#endif
#endif
#else
#if BSP_CFG_MCU_PART_PIN_NUM < 1
    /* 32-pin device */
    /* Input port mode(low-speed on-chip oscillator operation mode) */
    /* EXCLKS/OSCSELS setting */
    cmc_tmp |= 0x30U;
#else
    /* External clock input mode */
    /* EXCLKS/OSCSELS setting */
    cmc_tmp |= 0x30U;
#endif
#endif /* BSP_CFG_SUBCLK_SOURCE == 0 */

    /* Clock Operation Mode Control Register(CMC) setting */
    CMC = cmc_tmp;

    /* When to use X1 clock oscillator(fX) */
#if BSP_CFG_HISYSCLK_SOURCE == 1
    /* Oscillation Stabilization Time Select Register(OSTS) setting */
    OSTS = BSP_CFG_X1_WAIT_TIME_SEL;
#endif

    /* High-speed system clock oscillation */
#if BSP_CFG_HISYSCLK_OPERATION == 0
    /* Start oscillation */
    MSTOP = 0U;

#if BSP_CFG_HISYSCLK_SOURCE == 1
    /* Wait for oscillation stabilization unless external clock input */
    tmp_stab_set = (uint8_t)~(0x7FU >> OSTS);

    /* WAIT_LOOP */
    do
    {
        tmp_stab_wait  = OSTC;
        tmp_stab_wait &= tmp_stab_set;
    }
    while (tmp_stab_wait != tmp_stab_set);
#endif
#else
    /* X1 oscillation stopped if high-speed system clock is not used */
    MSTOP = 1U;
#endif

    /* Main system clock(fMAIN) setting */
#if BSP_CFG_MAINCLK_SOURCE == 0
    /* High-speed on-chip oscillator clock(fIH) */
    MCM0 = 0U;

    /* WAIT_LOOP */
    /* Confirm that high-speed on-chip oscillator clock(fIH) is selected. */
    while (1U == MCS)
    {
        ;
    }
#else
    /* High-speed system clock(fMX) */
    MCM0 = 1U;

#if (BSP_CFG_HISYSCLK_OPERATION == 0) && (BSP_CFG_HISYSCLK_SOURCE != 0)
    /* WAIT_LOOP */
    /* Confirm that high-speed system clock(fMX) is selected. */
    while (0U == MCS)
    {
        ;
    }
#endif
#endif

    /* Subsystem clock oscillation */

    /* Operation Speed Mode Control Register(OSMC) setting */
#if (BSP_CFG_ALLOW_FSL_IN_STOPHALT == 0) && (BSP_CFG_FIL_OPERATION == 0)
    OSMC |= 0x00U;
#elif (BSP_CFG_ALLOW_FSL_IN_STOPHALT == 1) && (BSP_CFG_FIL_OPERATION == 0)
    OSMC |= 0x80U;
#elif (BSP_CFG_ALLOW_FSL_IN_STOPHALT == 0) && (BSP_CFG_FIL_OPERATION == 1)
    OSMC |= 0x10U;
#else
    OSMC |= 0x90U;
#endif

    /* When to use subsystem clock oscillator clock(fSX) */
    /* Subsystem clock oscillator clock oscillation */
#if BSP_CFG_SUBCLK_OPERATION == 0
    /* Start oscillation */
    XTSTOP = 0U;

#if BSP_CFG_SUBCLK_SOURCE == 1
    /* Wait for oscillation stabilization unless external clock input */
    /* WAIT_LOOP */
    for (w_count = 0U; w_count <= BSP_CFG_SUBWAITTIME; w_count++)
    {
#if BSP_CFG_WDT_REFRESH_ENABLE == 0
        BSP_NOP();
#elif BSP_CFG_WDT_REFRESH_ENABLE == 1
        WDTE = 0xACU;
#else
        BSP_CFG_USER_WDT_REFRESH_SETTING_FUNCTION();
#endif
    }
#endif

#else
    /* XT1 oscillator stopped if subsystem clock oscillator clock is not used */
    XTSTOP = 1U;
#endif

#if BSP_CFG_SUBSYSCLK_SOURCE == 1
    SELLOSC = 1U;
#endif

    /* fCLK selection */
#if BSP_CFG_FCLK_SOURCE == 0
    /* When using a high-speed system clock */
    CSS = 0U;

    /* WAIT_LOOP */
    /* Confirm that Main system/PLL select clock(fMP) is selected. */
    while (1U == CLS)
    {
        ;
    }
#else
    /* When using subsystem clock of low-speed on-chip oscillator */
    CSS = 1U;

#if ((BSP_CFG_SUBCLK_SOURCE != 0) && (BSP_CFG_SUBSYSCLK_SOURCE == 0) && (BSP_CFG_SUBCLK_OPERATION == 0)) || ((BSP_CFG_SUBCLK_SOURCE != 0) && (BSP_CFG_SUBSYSCLK_SOURCE == 1))
    /* WAIT_LOOP */
    /* Confirm that Subsystem/low-speed on-chip oscillator select clock(fSL) is selected. */
    while (0U == CLS)
    {
        ;
    }
#endif
#endif

    /* Control of PLL operation. */
#if BSP_CFG_PLL_OPERATION == 1
    /* Control of PLL output clock selection */
#if BSP_CFG_FPLL_HZ <= 40000000
    PLLDIV1 = 0U;
#else
    PLLDIV1 = 1U;
#endif

    /* Control of setting lock-up wait counter */
#if BSP_CFG_LOCKUP_WAIT_COUNT_SEL == 0
    LCKSEL1 = 0U;
    LCKSEL0 = 0U;
#elif BSP_CFG_LOCKUP_WAIT_COUNT_SEL == 1
    LCKSEL1 = 0U;
    LCKSEL0 = 1U;
#elif BSP_CFG_LOCKUP_WAIT_COUNT_SEL == 2
    LCKSEL1 = 1U;
    LCKSEL0 = 0U;
#else
    LCKSEL1 = 1U;
    LCKSEL0 = 1U;
#endif

    /* Control of PLL input clock (fPLLI) division selection */
#if BSP_CFG_FMAIN_DIVIDE == 0
    FMAINDIV1 = 0U;
    FMAINDIV0 = 0U;
#elif BSP_CFG_FMAIN_DIVIDE == 1
    FMAINDIV1 = 1U;
    FMAINDIV0 = 0U;
#else
    FMAINDIV1 = 1U;
    FMAINDIV0 = 1U;
#endif

    /* Control of PLL division selection */
#if BSP_CFG_PLL_DIVIDE == 0
    FPLLDIV = 1U;
#elif BSP_CFG_PLL_DIVIDE == 1
    FPLLDIV = 0U;
    PLLDIV0 = 0U;
#else
    FPLLDIV = 0U;
    PLLDIV0 = 1U;
#endif

    /* Control of PLL multiplication selection */
#if BSP_CFG_PLL_MULTI == 0
    PLLMULA = 0U;
    PLLMUL  = 0U;
#elif BSP_CFG_PLL_MULTI == 1
    PLLMULA = 0U;
    PLLMUL  = 1U;
#elif BSP_CFG_PLL_MULTI == 2
    PLLMULA = 1U;
    PLLMUL  = 0U;
#else
    PLLMULA = 1U;
    PLLMUL  = 1U;
#endif

    /* WAIT_LOOP */
    for (w_count = 0U; w_count <= BSP_CFG_PLLWAITTIME; w_count++)
    {
        BSP_NOP();
    }

    PLLON = 1U;

    /* WAIT_LOOP */
    /* Confirm that the PLL circuit is locked. */
    while (0U == LOCK)
    {
        ;
    }
#else /* BSP_CFG_PLL_OPERATION == 1 */
    PLLON = 0U;
#endif /* BSP_CFG_PLL_OPERATION == 1 */

    /* fMP clock division control */
    MDIV = BSP_CFG_FMP_DIVIDE;

    /* Control of clock mode selection */
#if BSP_CFG_PLL_MODE == 0
    SELPLL = 0U;

    /* WAIT_LOOP */
    /* Confirm that the Clock through mode is selected. */
    while (1U == SELPLLS)
    {
        ;
    }
#else
    SELPLL = 1U;

#if BSP_CFG_PLL_OPERATION == 1
    /* WAIT_LOOP */
    /* Confirm that the PLL-clock-selected mode is selected. */
    while (0U == SELPLLS)
    {
        ;
    }
#endif
#endif

    /* Starts high-speed on-chip oscillator */
    /* Only the high-speed on-chip oscillator is activated after reset,
     * so stop this clock last.
     */
#if BSP_CFG_FIH_START_ON_STARTUP == 1
    HIOSTOP = 0U;

    /* If main system/PLL select clock is stopped
     * when subsystem/low-speed on-chip oscillator select clock is supplied to fCLK,
     * put the analog block of A/D converter on standby state.
     */
#else
#if BSP_CFG_HISYSCLK_OPERATION == 1
    ADCEN = 1U;
    ADCSR &= 0x7FFFU;
    ADHVREFCNT |= 0x80U;
#endif
    HIOSTOP = 1U;
#endif

    /* Control of timer RDe clock selection */
    TRD_CKSEL = BSP_CFG_TRD_CLOCK_SOURCE;

    /* Control of supplying or stopping CAN X1 clock (fX) */
    CAN0MCKE = BSP_CFG_CAN_CLOCK_OPERATION;

    /* Control of selecting LIN1 communication clock source */
    LIN1MCK = BSP_CFG_LIN1_CLOCK_SOURCE;

    /* Control of supplying or stopping LIN1 communication clock source */
    LIN1MCKE = BSP_CFG_LIN1_CLOCK_OPERATION;

    /* Control of selecting LIN0 communication clock source */
    LIN0MCK = BSP_CFG_LIN0_CLOCK_SOURCE;

    /* Control of supplying or stopping LIN0 communication clock source */
    LIN0MCKE = BSP_CFG_LIN0_CLOCK_OPERATION;
} /* End of function mcu_clock_setup() */

void boot_bsp_init_system(void)
{
    /* Pin setting */
#if BSP_CFG_MCU_PART_PIN_NUM == 0
    /* The 32-pin products do not have the PIOR0 register. */
#else
    /* 100,80,64,48-pin */
    PIOR0 = BSP_CFG_PIOR00 | (BSP_CFG_PIOR01 << 1U) | (BSP_CFG_PIOR02 << 2U) |
            (BSP_CFG_PIOR03 << 3U) | (BSP_CFG_PIOR04 << 4U) | (BSP_CFG_PIOR05 << 5U) |
            (BSP_CFG_PIOR06 << 6U) | (BSP_CFG_PIOR07 << 7U);
#endif

    /* 100,80,64,48,32-pin */
    PIOR1 = BSP_CFG_PIOR10 | (BSP_CFG_PIOR11 << 1U) | (BSP_CFG_PIOR12 << 2U) |
            (BSP_CFG_PIOR13 << 3U) | (BSP_CFG_PIOR14 << 4U) | (BSP_CFG_PIOR15 << 5U) |
            (BSP_CFG_PIOR16 << 6U) | (BSP_CFG_PIOR17 << 7U);

#if BSP_CFG_MCU_PART_PIN_NUM < 3
    /* The 64-, 48-, and 32-pin products do not have the PIOR2 register. */
#else
    /* 100,80-pin */
    PIOR2 = BSP_CFG_PIOR20 | (BSP_CFG_PIOR21 << 1U) | (BSP_CFG_PIOR22 << 2U) |
            (BSP_CFG_PIOR23 << 3U) | (BSP_CFG_PIOR24 << 4U) | (BSP_CFG_PIOR25 << 5U) |
            (BSP_CFG_PIOR26 << 6U) | (BSP_CFG_PIOR27 << 7U);
#endif

#if BSP_CFG_MCU_PART_PIN_NUM < 3
    /* The 64-, 48-, and 32-pin products do not have the PIOR3 register. */
#else
    /* 100,80-pin */
    PIOR3 = BSP_CFG_PIOR30 | (BSP_CFG_PIOR31 << 1U) | (BSP_CFG_PIOR32 << 2U) |
            (BSP_CFG_PIOR33 << 3U) | (BSP_CFG_PIOR34 << 4U) | (BSP_CFG_PIOR35 << 5U) |
            (BSP_CFG_PIOR36 << 6U) | (BSP_CFG_PIOR37 << 7U);
#endif

#if BSP_CFG_MCU_PART_PIN_NUM == 0
    /* 32-pin */
    PIOR4 = BSP_CFG_PIOR40 | (BSP_CFG_PIOR41 << 1U) | (BSP_CFG_PIOR42 << 2U) |
            (BSP_CFG_PIOR44 << 4U) | (BSP_CFG_PIOR45 << 5U) | (BSP_CFG_PIOR46 << 6U);
#else
    /* 100,80,64,48-pin */
    PIOR4 = BSP_CFG_PIOR40 | (BSP_CFG_PIOR41 << 1U) | (BSP_CFG_PIOR42 << 2U) |
            (BSP_CFG_PIOR43 << 3U) | (BSP_CFG_PIOR44 << 4U) | (BSP_CFG_PIOR45 << 5U) |
            (BSP_CFG_PIOR46 << 6U);
#endif

    /* 100,80,64,48,32-pin */
    PIOR5 = BSP_CFG_PIOR50 | (BSP_CFG_PIOR52 << 2U) | (BSP_CFG_PIOR53 << 3U);

#if BSP_CFG_MCU_PART_PIN_NUM < 3
    /* The 64-, 48-, and 32-pin products do not have the PIOR6 register. */
#else
    /* 100,80-pin */
    PIOR6 = BSP_CFG_PIOR60 | (BSP_CFG_PIOR61 << 1U) | (BSP_CFG_PIOR62 << 2U) |
            (BSP_CFG_PIOR63 << 3U) | (BSP_CFG_PIOR64 << 4U) | (BSP_CFG_PIOR65 << 5U) |
            (BSP_CFG_PIOR66 << 6U) | (BSP_CFG_PIOR67 << 7U);
#endif

    /* 100,80,64,48,32-pin */
    PIOR7 = BSP_CFG_PIOR70 | (BSP_CFG_PIOR71 << 1U) | (BSP_CFG_PIOR73 << 3U);

#if BSP_CFG_MCU_PART_PIN_NUM < 4
    /* The 80-, 64-, 48-, and 32-pin products do not have the PIOR8 register. */
#else
    /* 100-pin */
    PIOR8 = BSP_CFG_PIOR80;
#endif

#if BSP_CFG_MCU_PART_PIN_NUM == 0
    /* 32-pin */
    PIOR9 = BSP_CFG_PIOR90 | (BSP_CFG_PIOR91 << 1U) | (BSP_CFG_PIOR93 << 3U);
#else
    /* 100,80,64,48-pin */
    PIOR9 = BSP_CFG_PIOR90 | (BSP_CFG_PIOR91 << 1U) | (BSP_CFG_PIOR92 << 2U) |
            (BSP_CFG_PIOR93 << 3U);
#endif

    /* If the window open period of watchdog timer is set to 50%, then call it. */
#if (BSP_CFG_WDT_REFRESH_ENABLE == 2) || (BSP_CFG_WDT_REFRESH_ENABLE == 3)
    BSP_CFG_USER_WDT_REFRESH_INIT_FUNCTION();
#endif

    /* Clock initialization */
#if BSP_CFG_STARTUP_DISABLE == 0
    boot_mcu_clock_setup();
#endif

    /* If the warm start Pre C runtime callback is enabled, then call it. */
#if BSP_CFG_USER_WARM_START_CALLBACK_PRE_INITC_ENABLED == 1
    BSP_CFG_USER_WARM_START_PRE_C_FUNCTION();
#endif
} /* End of function bsp_init_system() */

// retarget printf
int __far putchar(int c)
{
    // F24 , UART0
    STMK0 = 1U;    /* disable INTST0 interrupt */
    SDR00L = (unsigned char)c;
    while(STIF0 == 0)
    {

    }
    STIF0 = 0U;    /* clear INTST0 interrupt flag */
    return c;
}

void boot_hdwinit(void)
{
    BSP_DI();   // BSP_EI();

    // boot_bsp_init_system(); // need to initial clock before RFD driver
    boot_R_TAU0_Create();
    boot_R_Config_PORT_Create();

#if ENABLE_UART_PRINTF
    boot_R_SAU0_Create();
    boot_R_Config_UART0_Start();        // UART0 , P15/TX , P16/RX
#endif

#if ENABLE_UART_BL
    boot_R_SAU1_Create();               // UART1 , P74/TX , P75/RX
    boot_R_Config_UART1_Start();
#endif

#if ENALBE_IICA0_BL
    boot_R_Config_IICA0_Create();       // IICA0 , P62/SCL , P63/SDA
#endif
  
    // check_reset_source();    
    IAWCTL = 0x00U;
    
#if ENABLE_UART_PRINTF
    printf_tiny("\r\n(boot)boot_hdwinit finish\r\n");
#endif
}

#define  SAMPLE_START_SECTION_SMP_CF
#include "sample_memmap.h"
void boot_main(void)
{
    #if ENABLE_UART_BL
    signed long i32Err;
    #endif
    
    #if ENALBE_IICA0_BL
	uint32_t CRC_rom = 0;
	uint32_t CRC_ccitt = 0;
	e_rfd_ret_t	    l_e_rfd_status_flag;
	uint8_t			status;
    #endif

	/****************/
	/* Boot Program */
	/****************/

#if ENALBE_IICA0_BL
	i2c_downloader_ctrl.resp_type = NONE_RESP;
	i2c_downloader_ctrl.recv_cmd = INVALID_CMD_BYTE;
	i2c_downloader_ctrl.txbyteCnt = 0;
	i2c_downloader_ctrl.rxbyteCnt = 0;
	i2c_downloader_ctrl.p_txbuff = &ResponseBuf[0];
	i2c_downloader_ctrl.p_rxbuff = &DataBuf[0];
	i2c_downloader_ctrl.image_version = 0;
	i2c_downloader_ctrl.area_num = 0;
	i2c_downloader_ctrl.write_data_length = 0;
	i2c_downloader_ctrl.downloader_operation_phase = COMMUNICATION_SETTING_PHASE;
#endif

#if ENABLE_UART_PRINTF
    printf_tiny("(boot)wait for interface data\r\n");
#endif

	if((SW == 0) || (condition_check()==true))	//if SW button is pressed
	{
        #if ENABLE_UART_PRINTF
        if (SW == 0)
        {
            printf_tiny("(boot)button pressed\r\n");
        }
        else
        {
            printf_tiny("(boot)condition_check\r\n");
        }
        #endif

	    g_app_required_update = true;
	}

    if (g_app_required_update)
    {
        #if ENABLE_UART_PRINTF
        printf_tiny("(boot)stay in boot loader\r\n");
        #endif
    }
    else    // CRC compare OK
    {
        if (verify_application_chksum())
        {
            #if ENABLE_UART_PRINTF
            printf_tiny("(boot)jump to app\r\n");       
            #endif 
        }
        else
        {
	        g_app_required_update = true;
            printf_tiny("(boot)CRC compare fail\r\n"); 
        }
    }

    if (g_app_required_update)
    {        
        InitCodeFlash();
    }

	while(1)
	{
		if(g_app_required_update)
		{
            
            #if ENABLE_UART_BL

            printf_tiny("(boot)entry Xmodem transfer\r\n");
            i32Err = Xmodem(FLASH_AREA);

            if(i32Err < 0)
            {
                printf_tiny("(boot)Xmodem transfer fail! 0x%04X," ,i32Err>>16);
                printf_tiny("0x%04X\r\n" ,i32Err&0xFFFF);
            }
            else
            {
                printf_tiny("(boot)Xomdem transfer done!\r\n");
                printf_tiny("(boot)Total trnasfer size is 0x%04X,", i32Err>>16);
                printf_tiny("0x%04X\r\n", i32Err&0xFFFF);
                
                //
                // In order to verify the checksum in the application, 
                // do CHIP_RST to enter bootloader again.
                //                
                // set_TimeoutFlag(1);
                printf_tiny("(boot)XMODEM : Perform RST...\r\n");
                const unsigned char ILLEGAL_ACCESS_ON = 0x80;
                IAWCTL |= ILLEGAL_ACCESS_ON;            // switch IAWEN on (default off)
                *(__far volatile char *)0x00000 = 0x00; //write illegal address 0x00000(RESET VECTOR)

            }


            #endif
            
            #if ENALBE_IICA0_BL
			I2C_Downloader_routine();

			if(ERASE_MEMORY == i2c_downloader_ctrl.resp_type)
			{
                // EraseCodeFlash();
				g_flash_operation_progressing = false;                
                // printf_tiny("(boot)I2C transfer:ERASE_MEMORY bypass\r\n");
			}

			if(WRITE_MEMORY_NEXT == i2c_downloader_ctrl.resp_type)
			{   
                // printf_tiny("(boot)I2C transfer:WRITE_MEMORY_NEXT\r\n");

				if ((flash_memory_destination_address >= flash_memory_start_address) &&
					(flash_memory_destination_address < flash_memory_end_address))
				{
					status = WriteCodeFlash(flash_memory_destination_address,
							(uint8_t __near *)&DataBuf[4], i2c_downloader_ctrl.write_data_length);

                    #if 0   // debug
                    printf_tiny("(boot)I2C transfer write! 0x%04X," ,flash_memory_destination_address>>16);
                    printf_tiny("0x%04X," ,flash_memory_destination_address&0xFFFF);
                    // printf_tiny("0x%02X\r\n" ,i2c_downloader_ctrl.write_data_length);
                    printf_tiny("[0x%02X" ,DataBuf[4+i2c_downloader_ctrl.write_data_length-4]);
                    printf_tiny(",0x%02X" ,DataBuf[4+i2c_downloader_ctrl.write_data_length-3]);
                    printf_tiny(",0x%02X" ,DataBuf[4+i2c_downloader_ctrl.write_data_length-2]);
                    printf_tiny(",0x%02X]\r\n" ,DataBuf[4+i2c_downloader_ctrl.write_data_length-1]);
                    #endif

					if (status == SAMPLE_ENUM_RET_STS_OK)	//PFSP: SAMPLE_ENUM_RET_STS_OK
					{
						/* if programming routine passed, then increase the flash address */
						flash_memory_destination_address += i2c_downloader_ctrl.write_data_length;
						flash_target_memory_size -= i2c_downloader_ctrl.write_data_length;

						if(flash_memory_destination_address == (flash_memory_end_address+1))
						{
							/* end flash programming */
							g_flash_operation_progressing = false;

                            printf_tiny("(boot)I2C upgrade finish : Perform RST...\r\n");
                            const unsigned char ILLEGAL_ACCESS_ON = 0x80;
                            IAWCTL |= ILLEGAL_ACCESS_ON;            // switch IAWEN on (default off)
                            *(__far volatile char *)0x00000 = 0x00; //write illegal address 0x00000(RESET VECTOR)
						}
					}
					else
					{
						i2c_downloader_ctrl.resp_type = WRITE_ERROR;
                        printf_tiny("(boot)I2C transfer:WRITE_ERROR\r\n");

					}
				}
				else
				{
					i2c_downloader_ctrl.resp_type = PACKET_ERROR;
                    printf_tiny("(boot)I2C transfer:PACKET_ERROR\r\n");
				}
			}

			if(SWITCH_APP == i2c_downloader_ctrl.resp_type)
			{                
                printf_tiny("(boot)I2C transfer:SWITCH_APP\r\n");
				if((flash_target_memory_size != 0)||(g_flash_operation_progressing))
				{
					CRC_ccitt = *(__far unsigned long *)CRC_ADDRESS;				    //0x7FFC
					// CRC_rom = calcrc(FLASH_AREA, (CRC_ADDRESS - FLASH_AREA));	//0x1000, 0x7FFC-0x1000=0x6FFC
                    CRC_rom = caculate_crc32_checksum(FLASH_AREA,FLASH_AP_SIZE);
				}

				if( CRC_ccitt == CRC_rom )
				{
					/******************************************************************************************************************
				     * Set nonprogrammable mode
				     *****************************************************************************************************************/
					l_e_rfd_status_flag = R_RFD_SetFlashMemoryMode(R_RFD_ENUM_FLASH_MODE_CODE_TO_NONPROGRAMMABLE);

					if (R_RFD_ENUM_RET_STS_OK != l_e_rfd_status_flag)
					{
						ErrorHandler(SAMPLE_ENUM_RET_ERR_MODE_MISMATCHED);
					}
					else
					{
						/* No operation */
					}

				    // R_RFD_HOOK_ExitCriticalSection();
				}
                else
                {                    
                    g_app_required_update = true;

                    #if ENABLE_UART_PRINTF
                    printf_tiny("(boot)CRC compare fail after update\r\n");
                    #endif
                }
			}
            #endif
		}
		else
		{        
            #if ENABLE_UART_PRINTF
            printf_tiny("(boot)g_app_required_update = 0,break\r\n");
            #endif
			break;
		}
	}

    #if ENABLE_UART_PRINTF
    printf_tiny("(boot)leaving\r\n");
    #endif

	return;
}


#define  SAMPLE_END_SECTION_SMP_CF
#include "sample_memmap.h"

