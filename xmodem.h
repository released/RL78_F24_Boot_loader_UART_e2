
#ifndef _XMODEM_H
#define _XMODEM_H

/* Xmodem Standard Commands */
#define XMD_SOH             0x01
#define XMD_STX             0x02
#define XMD_EOT             0x04
#define XMD_ACK             0x06
#define XMD_NAK             0x15
#define XMD_CAN             0x18
#define XMD_CTRLZ           0x1A
#define XMD_MAX_TIMEOUT     0x600

/* Xmodem Status */
#define XMD_STS_SUCCESS         0
#define XMD_STS_USER_CANCEL     -1
#define XMD_STS_NAK             -2
#define XMD_STS_TIMEOUT         -3
#define XMD_STS_PACKET_NUM_ERR  -4
#define XMD_STS_WRITE_FAIL      -5

#define MAXRETRANS  25


signed long XMD_Write(unsigned long u32Addr, unsigned long u32Data);
void XMD_putc(unsigned char c);
signed long XMD_getc(void);

signed long Xmodem(unsigned long u32DestAddr);
signed long XmodemSend(unsigned char *pu8Src, signed long srcsz);

#endif
