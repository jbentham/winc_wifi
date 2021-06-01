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
#include "winc_sock.h"

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

int main(int argc, char *argv[])
{
    int fd;
    uint32_t val=0;
    bool ok, irq=1;
    int sock;

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

        sock = open_sock_server(TCP_PORTNUM, 1, tcp_echo_handler);
        printf("Socket %u TCP port %u %s\n", sock, TCP_PORTNUM, sock>=0 ? "ok" : "failed");
        sock = open_sock_server(UDP_PORTNUM, 0, udp_echo_handler);
        printf("Socket %u UDP port %u %s\n", sock, UDP_PORTNUM, sock>=0 ? "ok" : "failed");

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
