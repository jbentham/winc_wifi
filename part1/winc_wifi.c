// ATWINC1500/1510 WiFi module interface for the Pi Pico
//
// Copyright (c) 2021 Jeremy P Bentham
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// v0.62 JPB 24/5/21 Simplified for first release

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "winc_wifi.h"
#include "winc_sock.h"

#define NEW_JOIN            0
#define U16_DATA(d, n, val) {d[n]=val>>8; d[n+1]=val;}
#define U24_DATA(d, n, val) {d[n]=val>>16; d[n+1]=val>>8; d[n+2]=val;}
#define U32_DATA(d, n, val) {d[n]=val>>24; d[n+1]=val>>16; d[n+2]=val>>8; d[n+3]=val;}
#define DATA_U32(d)         ((d[0]<<24) | (d[1]<<16) | (d[2]<<8) | d[3])
#define RSP_U32(d, n)       (d[n] | (uint32_t)(d[n+1])<<8 | (uint32_t)(d[n+2])<<16 | (uint32_t)(d[n+3])<<24)

uint8_t txbuff[SPI_BUFFLEN], rxbuff[SPI_BUFFLEN];
int verbose, spi_fd;
uint8_t tx_zeros[1024];
bool use_crc=1;
extern uint32_t spi_speed;

#define CLOCKLESS_ADDR      (1 << 15)

#define FINISH_BOOT_VAL 0x10add09e
#define DRIVER_VER_INFO 0x13521330    //19.5.2 - 19.3.0
#define CONF_VAL             0x102
#define START_FIRMWARE  0xef522f61
#define FINISH_INIT_VAL 0x02532636

typedef struct {uint8_t cmd, addr[3], zeros[7];} CMD_MSG_A;
typedef struct {uint8_t cmd, addr[3], count[3];} CMD_MSG_B;
typedef struct {uint8_t cmd, addr[2], data[4], zeros[2];} CMD_MSG_C;
typedef struct {uint8_t cmd, addr[3], data[4], zeros[2];} CMD_MSG_D;

// Connection header, 0x30 bytes
typedef struct {
    uint16_t cred_size;
    uint8_t flags, chan, ssid_len;
    char ssid[39];
    uint8_t auth, x[3];
} CONN_HDR;

// PSK data, 0x6c (108) bytes
typedef struct {
    uint8_t len;
    char phrase[0x63], x[8];
} PSK_DATA;

// Old PSK or open connection header, 104 bytes
typedef struct {
    char psk[65], typ, x1[2];
    uint16_t chan;
    char ssid[33];
    uint8_t nosave, x2[2];
} OLD_CONN_HDR;

// Structure to hold opcode names
typedef struct {
    int op;
    char *s;
} OP_STR;

OP_STR wifi_gop_resps[] = {{GOP_CONN_REQ_OLD, "Conn req"}, {GOP_STATE_CHANGE, "State change"},
    {GOP_DHCP_CONF, "DHCP conf"}, {GOP_CONN_REQ_NEW, "Conn_req"}, {GOP_BIND, "Bind"},
    {GOP_LISTEN, "Listen"}, {GOP_ACCEPT, "Accept"}, {GOP_SEND, "Send"}, {GOP_RECV, "Recv"},
    {GOP_SENDTO, "SendTo"}, {GOP_RECVFROM, "RecvFrom"}, {0,""}};

uint8_t remove_crc[11] = {0xC9, 0, 0xE8, 0x24, 0,  0,  0, 0x52, 0x5C, 0, 0};

// Return string for opcode
char *op_str(int gid, int op)
{
    OP_STR *ops=wifi_gop_resps;
    uint16_t gop=GIDOP(gid, op);

    while (ops->op && ops->op!=gop)
        ops++;
    return(ops->op ? ops->s : "");
}

// Return string for opcode
char *gop_str(uint16_t gop)
{
    OP_STR *ops=wifi_gop_resps;

    while (ops->op && ops->op!=gop)
        ops++;
    return(ops->op ? ops->s : "");
}

// Check for microsecond timeout
bool ustimeout(uint32_t *tp, uint32_t tout)
{
    bool ret=1;
    uint32_t t = usec();

    if (tout == 0)
        *tp = t;
    else if (t >= *tp + tout)
        *tp += tout;
    else
        ret = 0;
    return(ret);
}

// Delay given number of milliseconds
bool msdelay(int n)
{
    uint32_t tim;

    ustimeout(&tim, 0);
    while (!ustimeout(&tim, n*1000)) ;
    return(1);
}

// Delay given number of microseconds
bool usdelay(int n)
{
    uint32_t tim;

    ustimeout(&tim, 0);
    while (!ustimeout(&tim, n)) ;
    return(1);
}

// Display data in hex
void dump_hex(uint8_t *data, int dlen, int ncols, char *indent)
{
    int i;

    printf("%s", indent);
    for (i=0; i<dlen; i++)
    {
        if (ncols && (i && i%ncols==0))
            printf("\n%s", i==dlen-1 ? "" : indent);
        printf("%02X ", *data++);
    }
    printf("\n");
}

// Swap bytes of 16-bit value
uint16_t swap16(uint16_t val)
{
    return((val >> 8) | ((val&0xff)<<8));
}

// Disable SPI CRCs
void disable_crc(int fd)
{
    spi_xfer(fd, remove_crc, rxbuff, sizeof(remove_crc));
    use_crc = 0;
}

// Send SPI command, get response
int spi_cmd_resp(int fd, uint8_t *txd, uint8_t *rxd, int txlen, int rxlen)
{
    return spi_xfer(fd, txd, rxd, txlen+rxlen);
}

// Read register
int spi_read_reg(int fd, uint32_t addr, uint32_t *valp)
{
    CMD_MSG_A *mp=(CMD_MSG_A *)txbuff;
    int n, a, rxlen=sizeof(mp->zeros), txlen=sizeof(*mp)-rxlen;
    uint8_t *rsp = &rxbuff[txlen];

    mp->cmd = addr <= 0x30 ? CMD_INTERNAL_READ : CMD_SINGLE_READ;
    a = addr <= 0x30 ? (addr | CLOCKLESS_ADDR) << 8 : addr;
    U24_DATA(mp->addr, 0, a);
    memset(mp->zeros, 0, sizeof(mp->zeros));
    n = spi_cmd_resp(fd, txbuff, rxbuff, txlen, rxlen);
    if (n && rsp[0]==mp->cmd && rsp[1]==0 && (rsp[2] & 0xf0)==0xf0)
    {
        *valp = RSP_U32(rsp, 3);
        if (verbose > 1)
            printf("Rd reg %04x: %08x\n", addr, *valp);
    }
    return(rxlen);
}

// Read single data block
int spi_read_data(int fd, uint32_t addr, uint8_t *data, int dlen)
{
    CMD_MSG_B *mp=(CMD_MSG_B *)txbuff;
    int n, tries, txlen=sizeof(*mp);
    uint8_t b;

    mp->cmd = CMD_READ_DATA;
    U24_DATA(mp->addr, 0, addr);
    U24_DATA(mp->count, 0, dlen);
    n = spi_cmd_resp(fd, (uint8_t *)mp, rxbuff, txlen, 0);
    b = 0;
    tries = 10;
    while (n && !b && tries--)
        n = spi_xfer(fd, tx_zeros, &b, 1);
    if (n && b == CMD_READ_DATA)
    {
        n = spi_xfer(fd, tx_zeros, data, 2) &&
            spi_xfer(fd, tx_zeros, data, dlen);
        if (verbose > 1)
            printf("Rd data %04x: %u bytes\n", addr, dlen);
    }
    return(n);
}

// Write register
int spi_write_reg(int fd, uint32_t addr, uint32_t val)
{
    CMD_MSG_D *mp=(CMD_MSG_D *)txbuff;
    int n=0, rxlen=sizeof(mp->zeros), txlen=sizeof(*mp)-rxlen;
    uint8_t *rsp = &rxbuff[txlen];

    mp->cmd = CMD_SINGLE_WRITE;
    U24_DATA(mp->addr, 0, addr);
    U32_DATA(mp->data, 0, val);
    memset(mp->zeros, 0, sizeof(mp->zeros));
    n = spi_cmd_resp(fd, (uint8_t *)mp, rxbuff, txlen, rxlen);
    n = rsp[0]==mp->cmd && rsp[1]==0 ? n : 0;
    if (n && verbose > 1)
        printf("Wr reg %04x: %08x\n", addr, val);
    return(n);
}

// Write single data block
int spi_write_data(int fd, uint32_t addr, uint8_t *data, int dlen)
{
    CMD_MSG_B *mp=(CMD_MSG_B *)txbuff;
    int n, tries, txlen=sizeof(*mp);
    uint8_t b;

    mp->cmd = CMD_WRITE_DATA;
    U24_DATA(mp->addr, 0, addr);
    U24_DATA(mp->count, 0, dlen);
    txbuff[txlen] = txbuff[txlen+1] = 0;
    rxbuff[0] = 0;
    n = spi_cmd_resp(fd, (uint8_t *)mp, rxbuff, txlen, 2) && rxbuff[txlen]==CMD_WRITE_DATA;
    txbuff[0] = 0xf3;
    memcpy(&txbuff[1], data, dlen);
    n = n && spi_cmd_resp(fd, txbuff, rxbuff, dlen+1, 0);
    b = 0;
    tries = 10;
    while (n && b!=0xc3 && tries--)
        n = spi_xfer(fd, tx_zeros, &b, 1);
    n = n && spi_xfer(fd, tx_zeros, &b, 1);
    if (n && verbose > 1)
        printf("Wr data %04x: %u bytes\n", addr, dlen);
    return(n);
}

// Enable interrupt pin on chip
bool chip_interrupt_enable(int fd)
{
    uint32_t val;

    return(spi_read_reg(fd, PIN_MUX_REG0, &val) &&
           spi_write_reg(fd, PIN_MUX_REG0, val | 0x100) &&
           spi_read_reg(fd, NMI_EN_REG, &val) &&
           spi_write_reg(fd, NMI_EN_REG, val | 0x10000));
}

// Set module GPIO direction
bool set_gpio_dir(int fd, uint32_t dir)
{
    return(spi_write_reg(fd, 0x020108, dir));
}

// Set module GPIO value
bool set_gpio_val(int fd, uint32_t val)
{
    return(spi_write_reg(fd, 0x020100, val));
}

// Get chip ident number (1000 or 3000)
uint32_t chip_get_id(int fd)
{
    uint32_t ret=0, chip=0, rev=0;

    if (spi_read_reg(fd, CHIPID_REG, &chip) &&
        spi_read_reg(fd, REVID_REG, &rev))
        ret = chip;
    return(ret);
}

// Initialise WiFi chip
bool chip_init(int fd)
{
    uint32_t val;
    int tries, ok;

    // Wait until EFuse values have been loaded
    tries = 10;
    do {
        ok = spi_read_reg(fd, EFUSE_REG, &val) && (val & (1<<31));
    } while (!ok && tries-- && msdelay(1));
    // Wait for bootrom
    ok = ok && spi_read_reg(fd, HOST_WAIT_REG, &val);
    if (ok && (val&1)==0)
    {
        tries = 3;
        do {
            ok = spi_read_reg(fd, BOOTROM_REG, &val) && val==FINISH_BOOT_VAL;
        } while (!ok && tries-- && msdelay(1));
    }
    // Specify driver version
    ok = ok && spi_write_reg(fd, NMI_STATE_REG, DRIVER_VER_INFO);
    // Set configuration
    ok = ok && spi_write_reg(fd, NMI_GP_REG1, CONF_VAL);
    // Start firmware
    ok = ok && spi_write_reg(fd, BOOTROM_REG, START_FIRMWARE);
    // Wait until running
    tries = 20;
    if (ok) do {
        ok = spi_read_reg(fd, NMI_STATE_REG, &val) && val==FINISH_INIT_VAL;
    } while (!ok && tries-- && msdelay(10));
    ok = ok && spi_write_reg(fd, NMI_STATE_REG, 0);
    ok = ok && chip_interrupt_enable(fd);
    return(ok);
}

// Get firmware info
bool chip_get_info(int fd)
{
    uint32_t val;
    uint16_t data[4];
    uint8_t info[40], mac[6];
    bool ok;

    ok = spi_read_reg(fd, NMI_GP_REG2, &val);
    ok = ok && spi_read_data(fd, val|0x30000, (uint8_t *)data, sizeof(data));
    ok = ok && spi_read_data(fd, data[2]|0x30000, info, sizeof(info));
    ok = ok && spi_read_data(fd, data[1]|0x30000, mac, sizeof(mac));
    printf("Firmware %u.%u.%u, ", info[4], info[5], info[6]);
    printf("OTP MAC address %02X:%02X:%02X:%02X:%02X:%02X\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    return(ok);
}

// Start HIF transfer, interrupt WILC chip, wait until ack
bool hif_start(int fd, uint8_t gid, uint8_t op, int dlen)
{
    uint32_t val, tries=100, len=8+dlen;
    uint8_t hif[4] = {(uint8_t)(len>>8), (uint8_t)len, op, gid};
    bool ok;

    ok = spi_write_reg(fd, NMI_STATE_REG, DATA_U32(hif)) &&
         spi_write_reg(fd, RCV_CTRL_REG2, 2);
    if (ok) do {
        ok = spi_read_reg(fd, RCV_CTRL_REG2, &val) && (val&2)==0;
    } while (!ok && tries-- && usdelay(10));
    return(ok);
}

// Send 1 or 2 HIF data blocks
// (Send HIF hdr 4 bytes, then skip 4 bytes and send 1st data block
//  Send optional 2nd block, with offset measured from start of 1st block)
bool hif_put(int fd, uint16_t gop, void *dp1, int dlen1, void *dp2, int dlen2, int oset)
{
    uint32_t addr, a, dlen = HIF_HDR_SIZE + (dlen2 ? oset+dlen2 : dlen1);
    uint8_t gid = (uint8_t)(gop>>8), op=(uint8_t)gop;
    uint8_t hdr[8] = {gid, op&0x7f, (uint8_t)dlen, (uint8_t)(dlen>>8)};
    bool ok;

    ok = hif_start(fd, gid, op, dlen);                      // Start transfer
    ok = ok && spi_read_reg(fd, RCV_CTRL_REG4, &addr);      // Get DMA addr
    ok = ok && spi_write_data(fd, addr, hdr, sizeof(hdr));  // Write header
    a = addr + HIF_HDR_SIZE;
    ok = ok && spi_write_data(fd, a, dp1, dlen1);           // Write 1st block (e.g. SSID)
    if (dp2 && dlen2)                                       // Write 2nd block (e.g. passphrase)
        ok = ok && spi_write_data(fd, a+oset, dp2, dlen2);
    ok = ok && spi_write_reg(fd, RCV_CTRL_REG3, addr<<2|2); // Complete transfer
    if (verbose > 1)
    {
        printf("Send gid=%u op=%u len=%u,%u\n", gid, op, dlen1, dlen2);
        dump_hex(dp1, dlen1, 16, "  ");
        if (dp2)
            dump_hex(dp2, dlen2, 16, "  ");
    }
    return(ok);
}

// Receive Host Interface (HIF) header
int hif_hdr_get(int fd, uint32_t addr, HIF_HDR *hp)
{
    return(spi_read_data(fd, addr, (uint8_t *)hp, sizeof(HIF_HDR)));
}

// Fetch HIF data
int hif_get(int fd, uint32_t addr, void *buff, int len)
{
    return(spi_read_data(fd, addr, (uint8_t *)buff, len) ? len : 0);
}

// Complete HIF transfer
bool hif_rx_done(int fd)
{
    uint32_t val;

    return(spi_read_reg(fd, RCV_CTRL_REG0, &val) &&
           spi_write_reg(fd, RCV_CTRL_REG0, val|2));
}

// Join a WPA network, or open network if null password
bool join_net(int fd, char *ssid, char *pass)
{
#if NEW_JOIN
    CONN_HDR ch = {pass?0x98:0x2c, CRED_STORE, ANY_CHAN, strlen(ssid), "",
                   pass?AUTH_PSK:AUTH_OPEN, {0,0,0}};
    PSK_DATA pd;

    strcpy(ch.ssid, ssid);
    if (pass)
    {
        memset(&pd, 0, sizeof(PSK_DATA));
        strcpy(pd.phrase, pass);
        pd.len = strlen(pass);
        return(hif_put(fd, GOP_CONN_REQ_NEW|REQ_DATA, &ch, sizeof(CONN_HDR),
               &pd, sizeof(PSK_DATA), sizeof(CONN_HDR)));
    }
    return(hif_put(fd, GOP_CONN_REQ_NEW, &ch, sizeof(CONN_HDR), 0, 0, 0));
#else
    OLD_CONN_HDR och = {"", pass?AUTH_PSK:AUTH_OPEN, {0,0}, ANY_CHAN, "", 1, {0,0}};

    strcpy(och.ssid, ssid);
    strcpy(och.psk, pass ? pass : "");
    return(hif_put(fd, GOP_CONN_REQ_OLD, &och, sizeof(OLD_CONN_HDR), 0, 0, 0));
#endif
}

// EOF
