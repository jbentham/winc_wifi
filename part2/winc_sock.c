// ATWINC1500/1510 WiFi module socket functions for the Pi Pico
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
#include <stdbool.h>
#include "winc_wifi.h"
#include "winc_sock.h"

SOCKET sockets[MAX_SOCKETS];
RESP_MSG resp_msg;
uint8_t databuff[SPI_BUFFLEN];
extern int verbose, spi_fd;

// Socket errors, corresponding to negative length values
char *sock_errs[] = {"OK", "Invalid addr", "Addr already in use",
    "Too many TCP socks", "Too many UDP socks", "?", "Invalid arg",
    "Too many listening socks", "?", "Invalid operation", "?",
    "Addr required", "Client closed", "Sock timeout", "Sock buffer full"};

// Return a string corresponding to a socket error
char *sock_err_str(int err)
{
    err = err<0 ? -err: err;
    return(err < sizeof(sock_errs)/sizeof(char *) ? sock_errs[err] : "");
}

// Set up server socket, return socket number, -ve if error
int open_sock_server(int portnum, bool tcp, SOCK_HANDLER handler)
{
    int sock, smin=tcp?MIN_TCP_SOCK:MIN_UDP_SOCK, smax=tcp?MAX_TCP_SOCK:MAX_UDP_SOCK;
    static uint16_t session=1;

    for (sock=smin; sock<smax; sock++)
    {
        if (!sockets[sock].state)
        {
            sockets[sock].localport = portnum;
            sockets[sock].session = session++;
            sockets[sock].handler = handler;
            sock_state(sock, STATE_BINDING);
            return(sock);
        }
    }
    return(-1);
}

// Interrupt handler
void interrupt_handler(void)
{
    bool ok=1;
    int hlen, fd=spi_fd;
    uint16_t gop;
    uint32_t val, size, addr=0;
    HIF_HDR hh;
    RESP_MSG *rmp=&resp_msg;
    char temps[50]="";

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
    else if (gop==GOP_BIND && ok)
        sprintf(temps, "0x%X", rmp->val);
    else if (gop==GOP_ACCEPT && ok)
        sprintf(temps, "%u.%u.%u.%u:%u sock %u,%u",
            IP_BYTES(rmp->accept.addr.ip), rmp->accept.addr.port,
            rmp->accept.listen_sock, rmp->accept.conn_sock);
    else if (gop==GOP_RECVFROM && ok)
    {
        sprintf(temps, "%u.%u.%u.%u:%u sock %d dlen %d",
                IP_BYTES(rmp->recv.addr.ip), rmp->recv.addr.port, rmp->recv.sock, rmp->recv.dlen);
        if (rmp->recv.sock < MAX_SOCKETS)
            sockets[rmp->recv.sock].hif_data_addr = addr+HIF_HDR_SIZE+rmp->recv.oset;
    }
    else if (gop==GOP_RECV && ok)
    {
        sprintf(temps, "sock %d dlen %d", rmp->recv.sock, rmp->recv.dlen);
        if (rmp->recv.sock < MAX_SOCKETS)
            sockets[rmp->recv.sock].hif_data_addr = addr+HIF_HDR_SIZE+rmp->recv.oset;
    }
    if (verbose)
    {
        printf("Interrupt gid %u op %u len %u %s %s\n",
               hh.gid, hh.op, hh.len, op_str(hh.gid, hh.op), temps);
    }
    check_sock(fd, gop, rmp);
    ok = ok && hif_rx_done(fd);
    if (verbose > 1)
        printf("Interrupt complete %s\n", ok ? "OK":"error");
}

// Check for socket actions, given a received message
void check_sock(int fd, uint16_t gop, RESP_MSG *rmp)
{
    SOCKET *sp;
    uint8_t sock, sock2;

    if (gop==GOP_DHCP_CONF)
    {
        for (sock=MIN_SOCKET; sock<MAX_SOCKETS; sock++)
        {
            sp = &sockets[sock];
            if (sp->state==STATE_BINDING)
                put_sock_bind(fd, sock, sp->localport);
        }
    }
    else if (gop==GOP_BIND && (sock=rmp->bind.sock)<MAX_SOCKETS &&
             sockets[sock].state==STATE_BINDING)
    {
        sock_state(sock, STATE_BOUND);
        if (sock < MIN_UDP_SOCK)
            put_sock_listen(fd, sock);
        else
            put_sock_recvfrom(fd, sock);
    }
    else if (gop==GOP_RECVFROM && (sock=rmp->recv.sock)<MAX_SOCKETS &&
             (sp=&sockets[sock])->state==STATE_BOUND)
    {
        memcpy(&sp->addr, &rmp->recv.addr, sizeof(SOCK_ADDR));
        if (sp->handler)
            sp->handler(fd, sock, rmp->recv.dlen);
        put_sock_recvfrom(fd, sock);
    }
    else if (gop==GOP_ACCEPT &&
             (sock=rmp->accept.listen_sock)<MAX_SOCKETS &&
             (sock2=rmp->accept.conn_sock)<MAX_SOCKETS &&
             sockets[sock].state==STATE_BOUND)
    {
        memcpy(&sockets[sock2].addr, &rmp->recv.addr, sizeof(SOCK_ADDR));
        sockets[sock2].handler = sockets[sock].handler;
        sock_state(sock2, STATE_CONNECTED);
        put_sock_recv(fd, sock2);
    }
    else if (gop==GOP_RECV && (sock=rmp->recv.sock)<MAX_SOCKETS &&
            (sp=&sockets[sock])->state==STATE_CONNECTED)
    {
        if (sp->handler)
            sp->handler(fd, sock, rmp->recv.dlen);
        if (rmp->recv.dlen > 0)
            put_sock_recv(fd, sock);
    }
}

// Change state of socket
void sock_state(uint8_t sock, int news)
{
    if (sock < MAX_SOCKETS)
        sockets[sock].state = news;
}

// Request to bind a socket
bool put_sock_bind(int fd, uint8_t sock, uint16_t port)
{
    SOCKET *sp=&sockets[sock];
    BIND_CMD bc = {
        .saddr = {.family=IP_FAMILY, .port=swap16(port), .ip=-1},
        .sock=sock, .x=0, .session=sockets[sock].session};

    memcpy(&sp->addr, &bc.saddr, sizeof(SOCK_ADDR));
    return(hif_put(fd, GOP_BIND, &bc, sizeof(bc), 0, 0, 0));
}

// Request to enable socket listen
bool put_sock_listen(int fd, uint8_t sock)
{
    LISTEN_CMD lc = {sock, 0, sockets[sock].session};

    return(hif_put(fd, GOP_LISTEN, &lc, sizeof(lc), 0, 0, 0));
}

// Request TCP data from socket
bool put_sock_recv(int fd, uint8_t sock)
{
    RECV_CMD rc = {-1, sock, 0, sockets[sock].session};

    return(hif_put(fd, GOP_RECV, &rc, sizeof(rc), 0, 0, 0));
}

// Request UDP data from socket
bool put_sock_recvfrom(int fd, uint8_t sock)
{
    RECVFROM_CMD rc = {-1, sock, 0, sockets[sock].session};

    return(hif_put(fd, GOP_RECVFROM, &rc, sizeof(rc), 0, 0, 0));
}

// Send TCP data using socket
bool put_sock_send(int fd, uint8_t sock, void *data, int len)
{
    SOCKET *sp=&sockets[sock];
    SENDTO_CMD sc = {
        .saddr = {sp->addr.family, sp->addr.port, sp->addr.ip},
        .sock=sock, .len=len, .x=0, .session=sp->session, .x2=0};

    return(hif_put(fd, GOP_SEND|REQ_DATA, &sc, sizeof(sc), data, len, TCP_DATA_OSET));
}

// Send UDP data using socket
bool put_sock_sendto(int fd, uint8_t sock, void *data, int len)
{
    SOCKET *sp=&sockets[sock];
    SENDTO_CMD sc = {
        .saddr = {sp->addr.family, sp->addr.port, sp->addr.ip},
        .sock=sock, .len=len, .x=0, .session=sp->session, .x2=0};

    return(hif_put(fd, GOP_SENDTO|REQ_DATA, &sc, sizeof(sc), data, len, UDP_DATA_OSET));
}

// Close socket
bool put_sock_close(int fd, uint8_t sock)
{
    CLOSE_CMD cc = {sock, 0, sockets[sock].session};
    bool ok = hif_put(fd, GOP_CLOSE, &cc, sizeof(cc), 0, 0, 0);

    memset(&sockets[sock], 0, sizeof(SOCKET));
    return(ok);
}

// Get UDP or TCP data from socket
bool get_sock_data(int fd, uint8_t sock, void *data, int len)
{
    SOCKET *sp=&sockets[sock];
    bool ok=0;

    if (len > 0)
        ok = hif_get(fd, sp->hif_data_addr, data, len);
    return(ok);
}

// Handler for TCP echo
void tcp_echo_handler(int fd, uint8_t sock, int rxlen)
{
    printf("TCP Rx socket %u len %d %s\n", sock, rxlen,
           rxlen<=0 ? sock_err_str(rxlen) : "");
    if (rxlen < 0)
        put_sock_close(fd, sock);
    else if (rxlen>0 && get_sock_data(fd, sock, databuff, rxlen))
    {
        if (verbose > 1)
            dump_hex(databuff, rxlen, 16, "  ");
        put_sock_send(fd, sock, databuff, rxlen);
    }
}

// Handler for UDP echo
void udp_echo_handler(int fd, uint8_t sock, int rxlen)
{
    printf("UDP Rx socket %u len %d %s\n", sock, rxlen,
           rxlen<=0 ? sock_err_str(rxlen) : "");
    if (rxlen>0 && get_sock_data(fd, sock, databuff, rxlen))
    {
        if (verbose > 1)
            dump_hex(databuff, rxlen, 16, "  ");
        put_sock_sendto(fd, sock, databuff, rxlen);
    }
}

// EOF
