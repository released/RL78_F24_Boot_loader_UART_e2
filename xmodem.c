#include <stdio.h>
#include <string.h>

#include "r_smc_entry.h"
#include "platform.h"

#include "xmodem.h"

#include "user_def.h"
#include "sample_control_code_flash.h"

#define XMD_MAX_TRANS_SIZE      (1024*1024ul)

extern void boot_UART1_SendByte(int c);
extern int boot_UART1_Get(void);
extern int boot_UART1_Get_withTimeOut(unsigned long timeout, unsigned char *data);

/* 1024 for XModem 1k + 3 head chars + 2 crc + nul */
unsigned char s_au8XmdBuf[1030] = {0};

/*
    To program data from Xmodem transfer
*/
signed long XMD_Write(unsigned long u32Addr, unsigned long u32Data)
{
    #if 1
    BSP_DI();

    if ((u32Addr & (FLASH_BYTE_PER_BLOCK - 1)) == 0)
    {
        Sample_CodeFlashControl_Erase(u32Addr);
        #if 1   //debug3
        printf_tiny("boot)XMD_Write-erase(0x%02X," , u32Addr>>16);
        printf_tiny("0x%04X)\r\n" , u32Addr&0xFFFF);
        #endif
    }    

    Sample_CodeFlashControl_SingleWrite(u32Addr, (uint8_t __near *) &u32Data);
    BSP_EI();
	#else
	
    #if 0   // debug3
    if ((u32Addr & (FLASH_BYTE_PER_BLOCK - 1)) == 0)
    {
        printf_tiny("boot)XMD_Write-erase(0x%02X," , u32Addr>>16);
        printf_tiny("0x%04X)-" , u32Addr&0xFFFF);
    }    

    printf_tiny("boot)XMD_Write:rcv(0x%02X," , u32Addr>>16);
    printf_tiny("0x%04X)-" , u32Addr&0xFFFF);
    printf_tiny("0x%02X," , u32Data>>16);
    printf_tiny("0x%04X\r\n" , u32Data&0xFFFF);

    #endif	
    #endif

    return 0;
}


void XMD_putc(unsigned char c)
{
    boot_UART1_SendByte(c);

}

signed long XMD_getc(void)
{
    unsigned char c = 0;
    boot_UART1_Get_withTimeOut(100,&c);
    // c = boot_UART1_Get();
    return c;
}

unsigned short crc16_ccitt(const unsigned char *pu8buf, signed long i32len)
{
    unsigned short crc = 0;

    while(i32len--)
    {
        signed long i;
        crc ^= *pu8buf++ << 8;
        for(i = 0; i < 8; ++i)
        {
            if(crc & 0x8000)
                crc = (unsigned short)(crc << 1) ^ (unsigned short)0x1021;
            else
                crc = (unsigned short)(crc << 1);
        }
    }

    return crc;
}

signed long check(signed long iscrc, const unsigned char *pu8buf, signed long i32Size)
{
    if(iscrc)
    {
        unsigned short crc = crc16_ccitt(pu8buf, i32Size);
        unsigned short tcrc = (unsigned short)(pu8buf[i32Size] << 8) + (unsigned short)pu8buf[i32Size + 1];
        if(crc == tcrc)
            return 1;
    }
    else
    {
        signed long i;
        unsigned char tsum = 0;
        for(i = 0; i < i32Size; ++i)
            tsum += pu8buf[i];
        if(tsum == pu8buf[i32Size])
            return 1;
    }

    return 0;
}




/**
  * @brief      Recive data from UART Xmodem transfer and program the data to flash.
  * @param[in]  u32DestAddr Destination address of flash to program.
  * @return     Recived data size if successful. Return -1 when error.
  *
  * @details    This function is used to recieve UART data through Xmodem transfer.
  *             The received data will be programmed to flash packet by packet.
  */
signed long Xmodem(unsigned long u32DestAddr)
{
    signed long i32Err = 0;
    // unsigned char *p;
    signed long bufsz, crc = 0;
    unsigned char trychar = 'C';
    unsigned char packetno = 1;
    signed long i, j;
    signed long retrans = MAXRETRANS;
    signed long i32TransBytes = 0;
    signed long ch;
    unsigned long u32StarAddr, u32Data;
    // static unsigned char timeout_flag = 1;

    for(;;)
    {
        for(i = 0; i < XMD_MAX_TIMEOUT; ++i) /* set timeout period */
        {
            // if (get_TimeoutFlag())
            // {
            //     check_Timeout();
            // }

            if(trychar)
                XMD_putc(trychar);

            ch = XMD_getc();
            if(ch >= 0)
            {
                switch(ch)
                {
                    case XMD_SOH:
                        bufsz = 128;
                        goto START_RECEIVE;

                    case XMD_STX:
                        bufsz = 1024;
                        goto START_RECEIVE;

                    case XMD_EOT:
                        XMD_putc(XMD_ACK);
                        return (i32Err == 0) ? i32TransBytes : i32Err; /* normal end */

                    case XMD_CAN:
                        XMD_putc(XMD_ACK);
                        return XMD_STS_USER_CANCEL; /* canceled by remote */
                    default:
                        // printf_tiny("boot)XMD_getc:0x%2X\r\n" , ch);
                        break;
                }
            }
        }

        if(trychar == 'C')
        {
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);
            return XMD_STS_TIMEOUT; /* too many retry error */
        }
        XMD_putc(XMD_CAN);
        XMD_putc(XMD_CAN);
        XMD_putc(XMD_CAN);
        return XMD_STS_NAK; /* sync error */

START_RECEIVE:
        if(trychar == 'C')
            crc = 1;
        trychar = 0;
        // p = s_au8XmdBuf;
        // *p++ = (unsigned char)ch;
        s_au8XmdBuf[0] = ch;
        for(i = 0; i < (bufsz + (crc ? 1 : 0) + 3); ++i)
        {
            ch = XMD_getc();

            if(ch < 0)
                goto REJECT_RECEIVE;
            s_au8XmdBuf[1+i] = ch;// *p++ = (char)ch;
        }

        if(s_au8XmdBuf[1] != packetno)
        {
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);
            return XMD_STS_PACKET_NUM_ERR;
        }
        else
        {
            if(((s_au8XmdBuf[1] + s_au8XmdBuf[2]) == 0xFF) && check(crc, &s_au8XmdBuf[3], bufsz))
            {
                if(s_au8XmdBuf[1] == packetno)
                {
                    volatile signed long count = XMD_MAX_TRANS_SIZE - i32TransBytes;
                    if(count > bufsz)
                        count = bufsz;
                    #if 0      // debug1
                    printf_tiny("bufsz:%d,",bufsz); 
                    printf_tiny("trans:0x%02X,",i32TransBytes>>16);  
                    printf_tiny("0x%04X,",i32TransBytes&0xFFFF);
                    printf_tiny("count:%d\r\n",count);  
                    #endif 

                    if(count > 0)
                    {

                        for(j = 0; j < (bufsz + 3) / 4; j++)
                        {
                            memcpy((unsigned char *)&u32Data, &s_au8XmdBuf[3 + (j * 0x4)], 4);

                            u32StarAddr = u32DestAddr + (unsigned long)i32TransBytes;

                            i32Err = XMD_Write(u32StarAddr + ((unsigned long)j * 0x4), u32Data);

                            if(i32Err < 0)
                                continue;
                        }
                        i32TransBytes += count;
                        
                        // set_TimeoutFlag(0);   // start to receive , no more counting 
                        
                        #if 0       // debug2
                        printf_tiny("boot)Xmodem addr:0x%02X," , u32StarAddr>>16);
                        printf_tiny("0x%04X," , u32StarAddr&0xFFFF);
                        printf_tiny("transfer:0x%02X," , i32TransBytes>>16);
                        printf_tiny("0x%04X\r\n" , i32TransBytes&0xFFFF);                        
                        #endif
                    }
                    ++packetno;
                    retrans = MAXRETRANS + 1;
                }
                if(--retrans <= 0)
                {
                    XMD_putc(XMD_CAN);
                    XMD_putc(XMD_CAN);
                    XMD_putc(XMD_CAN);
                    return XMD_STS_TIMEOUT; /* too many retry error */
                }
                XMD_putc(XMD_ACK);
                continue;
            }
        }

REJECT_RECEIVE:
        XMD_putc(XMD_NAK);
    }
}



/**
  * @brief      Send data by UART Xmodem transfer.
  * @param[in]  src     Address of the source data to transfer.
  * @param[in]  srcsz   Size of the total size to transfer.
  * @retval     Total transfer size when successfull
  * @retval     -1  Canceled by remote
  * @retval     -2  No sync chararcter received.
  * @retval     -4  Transmit error.
  * @retval     -5  Unknown error.
  * @details    This function is used to send UART data through Xmodem transfer.
  *
  */
signed long XmodemSend(unsigned char *src, signed long srcsz)
{
    int bufsz, crc = -1;
    unsigned char packetno = 1;
    int i, c, len = 0;
    int retry;

    for(;;)
    {
        for(retry = 0; retry < 160; ++retry)
        {
            if((c = XMD_getc()) >= 0)
            {
                switch(c)
                {
                    case 'C':
                        crc = 1;
                        goto start_trans;
                    case XMD_NAK:
                        crc = 0;
                        goto start_trans;
                    case XMD_CAN:
                        if((c = XMD_getc()) == XMD_CAN)
                        {
                            XMD_putc(XMD_ACK);

                            return -1; /* canceled by remote */
                        }
                        break;
                    default:
                        break;
                }
            }
        }

        if(retry >= 160)
        {
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);
            XMD_putc(XMD_CAN);

            return -2; /* no sync */
        }

        for(;;)
        {
start_trans:
            s_au8XmdBuf[0] = XMD_SOH;
            bufsz = 128;
            s_au8XmdBuf[1] = packetno;
            s_au8XmdBuf[2] = ~packetno;
            c = srcsz - len;
            if(c > bufsz) c = bufsz;
            if(c >= 0)
            {
                memset(&s_au8XmdBuf[3], 0, (unsigned long)bufsz);
                if(c == 0)
                {
                    s_au8XmdBuf[3] = XMD_CTRLZ;
                }
                else
                {

                    memcpy(&s_au8XmdBuf[3], src, (unsigned long)c);
                    src += c;

                    if(c < bufsz) s_au8XmdBuf[3 + c] = XMD_CTRLZ;
                }
                if(crc)
                {
                    unsigned short ccrc = crc16_ccitt(&s_au8XmdBuf[3], bufsz);
                    s_au8XmdBuf[bufsz + 3] = (ccrc >> 8) & 0xFF;
                    s_au8XmdBuf[bufsz + 4] = ccrc & 0xFF;
                }
                else
                {
                    unsigned char ccks = 0;
                    for(i = 3; i < bufsz + 3; ++i)
                    {
                        ccks += s_au8XmdBuf[i];
                    }
                    s_au8XmdBuf[bufsz + 3] = ccks;
                }
                for(retry = 0; retry < MAXRETRANS; ++retry)
                {
                    for(i = 0; i < bufsz + 4 + (crc ? 1 : 0); ++i)
                    {
                        XMD_putc(s_au8XmdBuf[i]);
                    }
                    if((c = XMD_getc()) >= 0)
                    {
                        switch(c)
                        {
                            case XMD_ACK:
                                ++packetno;
                                len += bufsz;
                                goto start_trans;
                            case XMD_CAN:
                                if((c = XMD_getc()) == XMD_CAN)
                                {
                                    XMD_putc(XMD_ACK);

                                    return -1; /* canceled by remote */
                                }
                                break;
                            case XMD_NAK:
                            default:
                                break;
                        }
                    }
                }
                XMD_putc(XMD_CAN);
                XMD_putc(XMD_CAN);
                XMD_putc(XMD_CAN);
                return -4; /* xmit error */
            }
            else
            {
                for(retry = 0; retry < 10; ++retry)
                {
                    XMD_putc(XMD_EOT);
                    if((c = XMD_getc()) == XMD_ACK) break;
                }

                return (c == XMD_ACK) ? len : -5;
            }
        }
    }
}

