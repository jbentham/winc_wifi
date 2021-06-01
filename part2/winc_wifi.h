// ATWINC1500/1510 WiFi module socket definitions for the Pi Pico
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

#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif

#define IP_BYTES(x) x&255, x>>8&255, x>>16&255, x>>24&255

#define CHIPID_REG          0x1000
#define EFUSE_REG           0x1014
#define RCV_CTRL_REG3       0x106c
#define RCV_CTRL_REG0       0x1070
#define RCV_CTRL_REG2       0x1078
#define RCV_CTRL_REG1       0x1084
#define RCV_CTRL_REG5       0x1088
#define NMI_STATE_REG       0x108c
#define REVID_REG           0x13f4
#define PIN_MUX_REG0        0x1408
#define NMI_GP_REG1         0x14a0
#define NMI_EN_REG          0x1a00
#define SPI_CFG_REG         0xE824
#define HOST_WAIT_REG      0x207bc
#define NMI_GP_REG2        0xc0008
#define BOOTROM_REG        0xc000c
#define RCV_CTRL_REG4     0x150400

#define CMD_DMA_WRITE       0xc1
#define CMD_DMA_READ        0xc2
#define CMD_INTERNAL_WRITE  0xc3
#define CMD_INTERNAL_READ   0xc4
#define CMD_TERMINATE       0xc5
#define CMD_REPEAT          0xc6
#define CMD_WRITE_DATA      0xc7
#define CMD_READ_DATA       0xc8
#define CMD_SINGLE_WRITE    0xc9
#define CMD_SINGLE_READ     0xca
#define CMD_RESET           0xcf

// Host Interface (HIF) Group IDs
#define GID_MAIN        0
#define GID_WIFI        1
#define GID_IP          2
#define GID_HIF         3

// Host Interface operations with Group ID (GID)
#define GIDOP(gid, op) ((gid << 8) | op)
#define GOP_CONN_REQ_OLD    GIDOP(GID_WIFI, 40)
#define GOP_STATE_CHANGE    GIDOP(GID_WIFI, 44)
#define GOP_DHCP_CONF       GIDOP(GID_WIFI, 50)
#define GOP_CONN_REQ_NEW    GIDOP(GID_WIFI, 59)
#define GOP_BIND            GIDOP(GID_IP,   65)
#define GOP_LISTEN          GIDOP(GID_IP,   66)
#define GOP_ACCEPT          GIDOP(GID_IP,   67)
#define GOP_SEND            GIDOP(GID_IP,   69)
#define GOP_RECV            GIDOP(GID_IP,   70)
#define GOP_SENDTO          GIDOP(GID_IP,   71)
#define GOP_RECVFROM        GIDOP(GID_IP,   72)
#define GOP_CLOSE           GIDOP(GID_IP,   73)

// HIF header size (in bytes)
#define HIF_HDR_SIZE        8

#define ANY_CHAN        255
#define AUTH_OPEN       1
#define AUTH_PSK        2
#define CRED_NO_STORE   0
#define CRED_STORE      3
#define REQ_DATA        0x80

#define SPI_BUFFLEN     1600

// Header and status data for incoming HIF message
// Header is first 4 bytes, then dummy 4 bytes, then data starts
typedef struct {
    uint8_t gid, op;
    uint16_t len;
} HIF_HDR;

// Address field for socket, stored in network order (MSbyte first)
typedef struct {
    uint16_t family, port;
    uint32_t ip;
} SOCK_ADDR;

// Socket bind command, 12 bytes
typedef struct {
    SOCK_ADDR saddr;
    uint8_t sock, x;
    uint16_t session;
} BIND_CMD;

// Socket listen command, 4 bytes
typedef struct {
    uint8_t sock, backlog;
    uint16_t session;
} LISTEN_CMD;

// Recvfrom command, 8 bytes
typedef struct {
    uint32_t timeout;
    uint8_t sock, x;
    uint16_t session;
} RECVFROM_CMD;

// Recv command, 8 bytes
typedef struct {
    uint32_t timeout;
    uint8_t sock, x;
    uint16_t session;
} RECV_CMD;

// Sendto command, 16 bytes
typedef struct {
    uint8_t sock, x;
    uint16_t len;
    SOCK_ADDR saddr;
    uint16_t session, x2;
} SENDTO_CMD;

// Close command, 4 bytes
typedef struct {
    uint8_t sock, x;
    uint16_t session;
} CLOSE_CMD;

typedef void (* SOCK_HANDLER)(int fd, uint8_t sock, int rxlen);

char *op_str(int gid, int op);
bool ustimeout(uint32_t *tp, uint32_t tout);
bool msdelay(int n);
bool usdelay(int n);
uint16_t swap16(uint16_t val);
void dump_hex(uint8_t *data, int dlen, int ncols, char *indent);

void disable_crc(int fd);
int spi_cmd_resp(int fd, uint8_t *txd, uint8_t *rxd, int txlen, int rxlen);
int spi_read_reg(int fd, uint32_t addr, uint32_t *valp);
int spi_read_data(int fd, uint32_t addr, uint8_t *data, int dlen);
int spi_write_reg(int fd, uint32_t addr, uint32_t val);
int spi_write_data(int fd, uint32_t addr, uint8_t *data, int dlen);
bool chip_interrupt_enable(int fd);
bool set_gpio_dir(int fd, uint32_t dir);
bool set_gpio_val(int fd, uint32_t val);
uint32_t chip_get_id(int fd);
bool chip_init(int fd);
bool chip_get_info(int fd);
bool hif_start(int fd, uint8_t gid, uint8_t op, int dlen);
bool hif_put(int fd, uint16_t gop, void *dp1, int dlen1, void *dp2, int dlen2, int oset);
int hif_get(int fd, uint32_t addr, void *buff, int len);
bool sock_hdr_get(int fd, uint32_t addr, SOCK_ADDR *sap);
int hif_recv(int fd, uint32_t addr, uint8_t *gidp, uint8_t *opp, void *buff, int maxlen);
bool hif_rx_done(int fd);
bool join_net(int fd, char *ssid, char *pass);
bool connect_open(int fd);
bool connect_psk(int fd);
bool old_connect_open(int fd);
bool old_connect_psk(int fd);

uint32_t usec(void);
void spi_setup(int fd);
int read_irq(void);
void toggle_reset(void);
void release_reset(void);
void write_dev(char *dev, char *s);
int read_dev(char *dev);
int spi_xfer(int fd, uint8_t *txd, uint8_t *rxd, int len);
void err_exit(char *s);

// EOF
