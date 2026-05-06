#include "morseApp.h"
#include "packetfunctions.h"
#include "scheduler.h"
#include <stdbool.h>

#ifndef OPENWSN_SOCK_H
#include "sock.h"
#endif

sock_udp_t  morse_sock;
static bool        sock_ready = false;

// buffer da mensagem recebida (acessível do morseChat.c)
morse_packet_t received_pkt;
uint8_t        new_message = 0;

void morse_display_task(void) {
    // por implementar
}

void morseApp_poll_task(void) {
    morse_packet_t pkt;
    sock_udp_ep_t  remote;

    int res = sock_udp_recv(&morse_sock, &pkt, sizeof(pkt), 0, &remote);
    if (res > 0) {
        if (pkt.receiver_id == My_ID || pkt.receiver_id == 0) {
            received_pkt = pkt;
            new_message  = true;
        }
    }

    // reagenda para continuar a fazer polling
    scheduler_push_task(morseApp_poll_task, TASKPRIO_COAP);
}

void morseApp_init(void) {
    sock_udp_ep_t local;
    local.family = AF_INET6;
    local.port   = MORSE_UDP_PORT;
    local.netif  = 0;

    if (sock_udp_create(&morse_sock, &local, NULL, 0) == 0) {
        sock_ready = true;
        scheduler_push_task(morseApp_poll_task, TASKPRIO_COAP);
    }
}