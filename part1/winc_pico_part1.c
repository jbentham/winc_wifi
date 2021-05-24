// Raspberry Pi Pico interface for the ATWINC1500/1510 WiFi module
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

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "winc_wifi.h"

#define VERBOSE     1           // Diagnostic output level (0 to 3)
#define SPI_SPEED   11000000    // SPI clock (actually 10.42 MHz)
#define SPI_PORT    spi0        // SPI port number
#define NEW_PROTO   1           // Old or new Pico connections

#if !NEW_PROTO              // Old Pico prototype
#define SCK_PIN     2
#define MOSI_PIN    3
#define MISO_PIN    4
#define CS_PIN      5
#define RESET_PIN   18          // BCM pin 12
#define WAKE_PIN    12          // BCM pin 13
#define IRQ_PIN     17          // BCM pin 16
#else                       // New Pico prototype
#define SCK_PIN     18
#define MOSI_PIN    19
#define MISO_PIN    16
#define CS_PIN      17
#define WAKE_PIN    20
#define RESET_PIN   21
#define IRQ_PIN     22
#endif

#define PSK_SSID            "testnet"
#define PSK_PASSPHRASE      "testpass"

// DHCP response message
typedef struct {
    uint32_t self, gate, dns,
             mask, lease;
} DHCP_RESP_MSG;

// Response message union
typedef union {
    uint8_t data[16];
    int val;
    DHCP_RESP_MSG dhcp;
} RESP_MSG;

RESP_MSG resp_msg;
extern int verbose;

// Return microsecond time
uint32_t usec(void)
{
    return(time_us_32());
}

// Do SPI transfer
int spi_xfer(int fd, uint8_t *txd, uint8_t *rxd, int len)
{
    if (verbose > 2)
    {
        printf("  Tx:");
        for (int i=0; i<len; i++)
            printf(" %02X", txd[i]);
    }
    gpio_put(CS_PIN, 0);
    spi_write_read_blocking(SPI_PORT, txd, rxd, len);
    while (gpio_get(SCK_PIN)) ;
    gpio_put(CS_PIN, 1);
    if (verbose > 2)
    {
        printf("\n  Rx:");
        for (int i=0; i<len; i++)
            printf(" %02X", rxd[i]);
        printf("\n");
    }
    return(len);
}

// Read IRQ line
int read_irq(void)
{
    return(gpio_get(IRQ_PIN));
}

// Initialise SPI interface
void spi_setup(int fd)
{
    stdio_init_all();
    spi_init(SPI_PORT, SPI_SPEED);
    spi_set_format(SPI_PORT, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_init(MISO_PIN);
    gpio_set_function(MISO_PIN, GPIO_FUNC_SPI);
    gpio_set_function(CS_PIN,   GPIO_FUNC_SIO);
    gpio_set_function(SCK_PIN,  GPIO_FUNC_SPI);
    gpio_set_function(MOSI_PIN, GPIO_FUNC_SPI);
    gpio_init(CS_PIN);
    gpio_set_dir(CS_PIN, GPIO_OUT);
    gpio_put(CS_PIN, 1);
    gpio_init(WAKE_PIN);
    gpio_set_dir(WAKE_PIN, GPIO_OUT);
    gpio_put(WAKE_PIN, 1);
    gpio_init(IRQ_PIN);
    gpio_set_dir(IRQ_PIN, GPIO_IN);
    gpio_pull_up(IRQ_PIN);
    gpio_init(RESET_PIN);
    gpio_set_dir(RESET_PIN, GPIO_OUT);
    gpio_put(RESET_PIN, 0);
    sleep_ms(1);
    gpio_put(RESET_PIN, 1);
    sleep_ms(1);
}

void interrupt_handler(void)
{
    bool ok=1;
    int hlen, fd=0;
    uint16_t gop;
    uint32_t val, size, addr=0;
    HIF_HDR hh;
    RESP_MSG *rmp=&resp_msg;
    char temps[50]="";

    verbose = VERBOSE;
    if (verbose > 1)
        printf("Interrupt\n");
    ok = spi_read_reg(fd, RCV_CTRL_REG0, &val) &&
         (val&1) && (size = (val>>2) & 0xfff) != 0;
    ok = ok && spi_write_reg(fd, RCV_CTRL_REG0, val & ~1);
    ok = ok && spi_read_reg(fd, RCV_CTRL_REG1, &addr) && addr;

    // Read HIF header
    ok = ok && hif_get(fd, addr, &hh, sizeof(hh));
    gop = GIDOP((uint16_t)hh.gid, hh.op);
    hlen = MIN((hh.len - HIF_HDR_SIZE), sizeof(RESP_MSG));

    // Read response message
    ok = ok && hlen>0 && hif_get(fd, addr+HIF_HDR_SIZE, rmp, hlen);

    // Act on response
    if (gop==GOP_STATE_CHANGE && ok)
        sprintf(temps, rmp->val==0 ? "disconnected" : rmp->val==1 ? "connected" : "fail");
    else if (gop==GOP_DHCP_CONF && ok)
        sprintf(temps, "%u.%u.%u.%u gate %u.%u.%u.%u", IP_BYTES(rmp->dhcp.self), IP_BYTES(rmp->dhcp.gate));
    if (verbose)
        printf("Interrupt gid %u op %u len %u %s %s\n",
               hh.gid, hh.op, hh.len, op_str(hh.gid, hh.op), temps);
    ok = ok && hif_rx_done(fd);
    if (verbose > 1)
        printf("Interrupt complete %s\n", ok ? "OK":"error");
}

int main(int argc, char *argv[])
{
    int fd;
    uint32_t val=0;
    bool ok, irq=1;

    verbose = VERBOSE;
    spi_setup(fd);
    disable_crc(fd);
    ok = chip_init(fd);
    if (!ok)
        printf("Can't initialise chip\n");
    else
    {
        ok = chip_get_info(fd);
        ok = ok && set_gpio_val(fd, 0x58070) && set_gpio_dir(fd, 0x58070);

        ok = join_net(fd, PSK_SSID, PSK_PASSPHRASE);

        printf("Connecting");
        while (ok && (irq=read_irq()) && msdelay(100))
        {
            putchar('.');
            fflush(stdout);
        }
        printf("\n");
        while (ok)
        {
            if (read_irq() == 0)
                interrupt_handler();
        }
    }
	return(0);
}

// EOF
