#include "nrf52833.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "scheduler.h"
#include "board.h"
#include "radio.h"

// ── Time ──────────────────────────────────────────────────────────────────────

volatile uint32_t ms_ticks = 0;

void systick_init(void) {
    SysTick->LOAD = 64000 - 1;
    SysTick->VAL  = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk
                  | SysTick_CTRL_TICKINT_Msk
                  | SysTick_CTRL_ENABLE_Msk;
}

void SysTick_Handler(void) {
    ms_ticks++;
}

static inline uint32_t now_ms(void) {
    return ms_ticks;
}

// ── Pins ──────────────────────────────────────────────────────────────────────

#define ROW1_DISCONNECT()   (NRF_P0->PIN_CNF[21] = 0x00000002)
#define ROW2_DISCONNECT()   (NRF_P0->PIN_CNF[22] = 0x00000002)
#define ROW3_DISCONNECT()   (NRF_P0->PIN_CNF[15] = 0x00000002)
#define ROW4_DISCONNECT()   (NRF_P0->PIN_CNF[24] = 0x00000002)
#define ROW5_DISCONNECT()   (NRF_P0->PIN_CNF[19] = 0x00000002)

#define ROW1_OUTPUT()       (NRF_P0->PIN_CNF[21] = 0x00000003)
#define ROW2_OUTPUT()       (NRF_P0->PIN_CNF[22] = 0x00000003)
#define ROW3_OUTPUT()       (NRF_P0->PIN_CNF[15] = 0x00000003)
#define ROW4_OUTPUT()       (NRF_P0->PIN_CNF[24] = 0x00000003)
#define ROW5_OUTPUT()       (NRF_P0->PIN_CNF[19] = 0x00000003)

#define ROW1_HIGH()         (NRF_P0->OUTSET = (1UL << 21))
#define ROW2_HIGH()         (NRF_P0->OUTSET = (1UL << 22))
#define ROW3_HIGH()         (NRF_P0->OUTSET = (1UL << 15))
#define ROW4_HIGH()         (NRF_P0->OUTSET = (1UL << 24))
#define ROW5_HIGH()         (NRF_P0->OUTSET = (1UL << 19))

#define COL1_DISCONNECT()   (NRF_P0->PIN_CNF[28] = 0x00000002)
#define COL2_DISCONNECT()   (NRF_P0->PIN_CNF[11] = 0x00000002)
#define COL3_DISCONNECT()   (NRF_P0->PIN_CNF[31] = 0x00000002)
#define COL4_DISCONNECT()   (NRF_P1->PIN_CNF[5]  = 0x00000002)
#define COL5_DISCONNECT()   (NRF_P0->PIN_CNF[30] = 0x00000002)

#define COL1_OUTPUT()       (NRF_P0->PIN_CNF[28] = 0x00000003)
#define COL2_OUTPUT()       (NRF_P0->PIN_CNF[11] = 0x00000003)
#define COL3_OUTPUT()       (NRF_P0->PIN_CNF[31] = 0x00000003)
#define COL4_OUTPUT()       (NRF_P1->PIN_CNF[5]  = 0x00000003)
#define COL5_OUTPUT()       (NRF_P0->PIN_CNF[30] = 0x00000003)

#define COL1_LOW()          (NRF_P0->OUTCLR = (1UL << 28))
#define COL2_LOW()          (NRF_P0->OUTCLR = (1UL << 11))
#define COL3_LOW()          (NRF_P0->OUTCLR = (1UL << 31))
#define COL4_LOW()          (NRF_P1->OUTCLR = (1UL <<  5))
#define COL5_LOW()          (NRF_P0->OUTCLR = (1UL << 30))

#define BUTTON_A_PIN 14
#define BUTTON_B_PIN 23

static void buttons_init(void) {
    NRF_P0->PIN_CNF[BUTTON_A_PIN] = (0 << 0) | (3 << 2);
    NRF_P0->PIN_CNF[BUTTON_B_PIN] = (0 << 0) | (3 << 2);
}

// ── LED ───────────────────────────────────────────────────────────────────────

typedef enum {
    LED11, LED12, LED13, LED14, LED15,
    LED21, LED22, LED23, LED24, LED25,
    LED31, LED32, LED33, LED34, LED35,
    LED41, LED42, LED43, LED44, LED45,
    LED51, LED52, LED53, LED54, LED55
} led_id_t;

static void display_clear(void) {
    NRF_P0->OUTCLR = (1UL<<21)|(1UL<<22)|(1UL<<15)|(1UL<<24)|(1UL<<19);
    NRF_P0->OUTSET = (1UL<<28)|(1UL<<11)|(1UL<<31)|(1UL<<30);
    NRF_P1->OUTSET = (1UL<<5);
    ROW1_DISCONNECT(); ROW2_DISCONNECT(); ROW3_DISCONNECT();
    ROW4_DISCONNECT(); ROW5_DISCONNECT();
    COL1_DISCONNECT(); COL2_DISCONNECT(); COL3_DISCONNECT();
    COL4_DISCONNECT(); COL5_DISCONNECT();
}

static void display_pixel(led_id_t id) {
    switch (id) {
        case LED11: ROW1_OUTPUT(); ROW1_HIGH(); COL1_OUTPUT(); COL1_LOW(); break;
        case LED12: ROW1_OUTPUT(); ROW1_HIGH(); COL2_OUTPUT(); COL2_LOW(); break;
        case LED13: ROW1_OUTPUT(); ROW1_HIGH(); COL3_OUTPUT(); COL3_LOW(); break;
        case LED14: ROW1_OUTPUT(); ROW1_HIGH(); COL4_OUTPUT(); COL4_LOW(); break;
        case LED15: ROW1_OUTPUT(); ROW1_HIGH(); COL5_OUTPUT(); COL5_LOW(); break;

        case LED21: ROW2_OUTPUT(); ROW2_HIGH(); COL1_OUTPUT(); COL1_LOW(); break;
        case LED22: ROW2_OUTPUT(); ROW2_HIGH(); COL2_OUTPUT(); COL2_LOW(); break;
        case LED23: ROW2_OUTPUT(); ROW2_HIGH(); COL3_OUTPUT(); COL3_LOW(); break;
        case LED24: ROW2_OUTPUT(); ROW2_HIGH(); COL4_OUTPUT(); COL4_LOW(); break;
        case LED25: ROW2_OUTPUT(); ROW2_HIGH(); COL5_OUTPUT(); COL5_LOW(); break;

        case LED31: ROW3_OUTPUT(); ROW3_HIGH(); COL1_OUTPUT(); COL1_LOW(); break;
        case LED32: ROW3_OUTPUT(); ROW3_HIGH(); COL2_OUTPUT(); COL2_LOW(); break;
        case LED33: ROW3_OUTPUT(); ROW3_HIGH(); COL3_OUTPUT(); COL3_LOW(); break;
        case LED34: ROW3_OUTPUT(); ROW3_HIGH(); COL4_OUTPUT(); COL4_LOW(); break;
        case LED35: ROW3_OUTPUT(); ROW3_HIGH(); COL5_OUTPUT(); COL5_LOW(); break;

        case LED41: ROW4_OUTPUT(); ROW4_HIGH(); COL1_OUTPUT(); COL1_LOW(); break;
        case LED42: ROW4_OUTPUT(); ROW4_HIGH(); COL2_OUTPUT(); COL2_LOW(); break;
        case LED43: ROW4_OUTPUT(); ROW4_HIGH(); COL3_OUTPUT(); COL3_LOW(); break;
        case LED44: ROW4_OUTPUT(); ROW4_HIGH(); COL4_OUTPUT(); COL4_LOW(); break;
        case LED45: ROW4_OUTPUT(); ROW4_HIGH(); COL5_OUTPUT(); COL5_LOW(); break;

        case LED51: ROW5_OUTPUT(); ROW5_HIGH(); COL1_OUTPUT(); COL1_LOW(); break;
        case LED52: ROW5_OUTPUT(); ROW5_HIGH(); COL2_OUTPUT(); COL2_LOW(); break;
        case LED53: ROW5_OUTPUT(); ROW5_HIGH(); COL3_OUTPUT(); COL3_LOW(); break;
        case LED54: ROW5_OUTPUT(); ROW5_HIGH(); COL4_OUTPUT(); COL4_LOW(); break;
        case LED55: ROW5_OUTPUT(); ROW5_HIGH(); COL5_OUTPUT(); COL5_LOW(); break;
    }
}

// ── Display multiplexing via TIMER0 ──────────────────────────────────────────

static const led_id_t* disp_pattern = NULL;
static int             disp_size    = 0;
static int             disp_idx     = 0;
static uint32_t        disp_until   = 0;

void display_timer_init(void) {
    NRF_TIMER0->TASKS_STOP  = 1;
    NRF_TIMER0->TASKS_CLEAR = 1;

    NRF_TIMER0->MODE      = 0;
    NRF_TIMER0->BITMODE   = 0;
    NRF_TIMER0->PRESCALER = 4;

    NRF_TIMER0->CC[0]  = 2000;
    NRF_TIMER0->SHORTS = 1;

    NRF_TIMER0->INTENSET = (1 << 16);

    NVIC_SetPriority(TIMER0_IRQn, 1);
    NVIC_EnableIRQ(TIMER0_IRQn);

    NRF_TIMER0->TASKS_START = 1;
}

void TIMER0_IRQHandler(void) {
    if (NRF_TIMER0->EVENTS_COMPARE[0]) {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;

        display_clear();

        if (disp_pattern != NULL && now_ms() < disp_until) {
            display_pixel(disp_pattern[disp_idx]);
            disp_idx = (disp_idx + 1) % disp_size;
        }
    }
}

void display_show_timed(const led_id_t* pattern, int size, uint32_t duration_ms) {
    disp_pattern = pattern;
    disp_size    = size;
    disp_idx     = 0;
    disp_until   = now_ms() + duration_ms;
}

// ── Patterns ──────────────────────────────────────────────────────────────────

const led_id_t DOT[]  = { LED33 };
const led_id_t DASH[] = { LED32, LED33, LED34 };
const led_id_t SAVE[] = { LED22, LED32, LED42, LED23, LED43, LED24, LED34, LED44 };
const led_id_t OK[]   = { LED32, LED43, LED34, LED25 };

// ── Numbers ───────────────────────────────────────────────────────────────────

const led_id_t Zero[] = {
    LED12, LED22, LED32, LED42, LED52, LED13, LED53, LED14, LED24, LED34, LED44, LED54
};
const led_id_t One[] = {
    LED22, LED13, LED23, LED33, LED43, LED52, LED53, LED54
};
const led_id_t Two[] = {
    LED12, LED13, LED14, LED25, LED33, LED34, LED42, LED52, LED53, LED54, LED55
};
const led_id_t Three[] = {
    LED12, LED13, LED14, LED25, LED33, LED34,
    LED45, LED52, LED53, LED54
};
const led_id_t Four[] = {
    LED14, LED23, LED24, LED32, LED33, LED34, LED35, LED44, LED54
};
const led_id_t Five[] = {
    LED11, LED12, LED13, LED14, LED15, LED21, LED31, LED32, LED33, LED34, LED45, LED51, LED52, LED53, LED54
};

const led_id_t * const Digits[]   = { Zero, One, Two, Three, Four, Five };
const uint8_t          Digits_size[] = {
    sizeof(Zero) /sizeof(led_id_t),
    sizeof(One)  /sizeof(led_id_t),
    sizeof(Two)  /sizeof(led_id_t),
    sizeof(Three)/sizeof(led_id_t),
    sizeof(Four) /sizeof(led_id_t),
    sizeof(Five) /sizeof(led_id_t)
};

// ── Letters ───────────────────────────────────────────────────────────────────

// Letter A
const led_id_t Letter_A[] = {
    LED13, LED22, LED24, LED32, LED33, LED34, LED42, LED44, LED52, LED54
};

// Letter B
const led_id_t Letter_B[] = {
    LED12, LED13, LED14, LED22, LED25, LED32, LED33, LED34, LED42, LED45, LED52, LED53, LED54
};

// Letter C
const led_id_t Letter_C[] = {
    LED13, LED14, LED15, LED22, LED32, LED42, LED53, LED54, LED55
};

// Letter D
const led_id_t Letter_D[] = {
    LED11, LED12, LED13, LED24, LED34, LED44, LED51, LED52, LED53, LED21, LED31, LED41
};

// Letter E
const led_id_t Letter_E[] = {
    LED12, LED13, LED14, LED15, LED22, LED32, LED33, LED34, LED42, LED52, LED53, LED54, LED55
};

// Letter F
const led_id_t Letter_F[] = {
    LED12, LED13, LED14, LED15, LED22, LED32, LED33, LED34, LED42, LED52
};

// Letter G
const led_id_t Letter_G[] = {
    LED13, LED14, LED15, LED22, LED32, LED34, LED35, LED42, LED45, LED53, LED54, LED55
};

// Letter H
const led_id_t Letter_H[] = {
    LED12, LED15, LED22, LED25, LED32, LED33, LED34, LED35, LED42, LED45, LED52, LED55
};

// Letter I
const led_id_t Letter_I[] = {
    LED12, LED13, LED14, LED23, LED33, LED43, LED52, LED53, LED54
};

// Letter J
const led_id_t Letter_J[] = {
    LED13, LED14, LED15, LED24, LED34, LED42, LED44, LED53
};

// Letter K
const led_id_t Letter_K[] = {
    LED12, LED15, LED22, LED24, LED32, LED33, LED42, LED44, LED52, LED55
};

// Letter L
const led_id_t Letter_L[] = {
    LED12, LED22, LED32, LED42, LED52, LED53, LED54, LED55
};

// Letter M
const led_id_t Letter_M[] = {
    LED11, LED21, LED31, LED41, LED51, LED22, LED33, LED24, LED15, LED25, LED35, LED45, LED55
};

// Letter N
const led_id_t Letter_N[] = {
    LED11, LED21, LED31, LED41, LED51, LED22, LED33, LED44, LED55, LED15, LED25, LED35, LED45
};

// Letter O
const led_id_t Letter_O[] = {
    LED13, LED14, LED22, LED25, LED32, LED35, LED42, LED45, LED53, LED54
};

// Letter P
const led_id_t Letter_P[] = {
    LED12, LED13, LED14, LED22, LED25, LED32, LED33, LED34, LED42, LED52
};

// Letter Q
const led_id_t Letter_Q[] = {
    LED13, LED14, LED22, LED25, LED32, LED35, LED42, LED44, LED53, LED54, LED55
};

// Letter R
const led_id_t Letter_R[] = {
    LED12, LED13, LED14, LED22, LED25, LED32, LED33, LED34, LED42, LED44, LED52, LED55
};

// Letter S
const led_id_t Letter_S[] = {
    LED13, LED14, LED15, LED22, LED33, LED34, LED45, LED52, LED53, LED54
};

// Letter T
const led_id_t Letter_T[] = {
    LED11, LED12, LED13, LED14, LED15, LED23, LED33, LED43, LED53
};

// Letter U
const led_id_t Letter_U[] = {
    LED11, LED21, LED31, LED41, LED52, LED53, LED54, LED15, LED25, LED35, LED45
};

// Letter V
const led_id_t Letter_V[] = {
    LED11, LED21, LED32, LED42, LED53, LED44, LED34, LED25, LED15
};

// Letter W
const led_id_t Letter_W[] = {
    LED11, LED21, LED31, LED41, LED51, LED42, LED33, LED44, LED15, LED25, LED35, LED45, LED55
};

// Letter X
const led_id_t Letter_X[] = {
    LED11, LED51, LED22, LED42, LED33, LED24, LED44, LED15, LED55
};

// Letter Y
const led_id_t Letter_Y[] = {
    LED11, LED21, LED32, LED43, LED53, LED34, LED25, LED15
};

// Letter Z
const led_id_t Letter_Z[] = {
    LED11, LED12, LED13, LED14, LED15, LED24, LED33, LED42, LED51, LED52, LED53, LED54, LED55
};

const led_id_t * const Letters[] = {
    Letter_A, Letter_B, Letter_C, Letter_D, Letter_E, Letter_F, Letter_G, 
    Letter_H, Letter_I, Letter_J, Letter_K, Letter_L, Letter_M, Letter_N, 
    Letter_O, Letter_P, Letter_Q, Letter_R, Letter_S, Letter_T, Letter_U, 
    Letter_V, Letter_W, Letter_X, Letter_Y, Letter_Z
};

const uint8_t Letters_size[] = {
    sizeof(Letter_A)/sizeof(led_id_t), sizeof(Letter_B)/sizeof(led_id_t),
    sizeof(Letter_C)/sizeof(led_id_t), sizeof(Letter_D)/sizeof(led_id_t),
    sizeof(Letter_E)/sizeof(led_id_t), sizeof(Letter_F)/sizeof(led_id_t),
    sizeof(Letter_G)/sizeof(led_id_t), sizeof(Letter_H)/sizeof(led_id_t),
    sizeof(Letter_I)/sizeof(led_id_t), sizeof(Letter_J)/sizeof(led_id_t),
    sizeof(Letter_K)/sizeof(led_id_t), sizeof(Letter_L)/sizeof(led_id_t),
    sizeof(Letter_M)/sizeof(led_id_t), sizeof(Letter_N)/sizeof(led_id_t),
    sizeof(Letter_O)/sizeof(led_id_t), sizeof(Letter_P)/sizeof(led_id_t),
    sizeof(Letter_Q)/sizeof(led_id_t), sizeof(Letter_R)/sizeof(led_id_t),
    sizeof(Letter_S)/sizeof(led_id_t), sizeof(Letter_T)/sizeof(led_id_t),
    sizeof(Letter_U)/sizeof(led_id_t), sizeof(Letter_V)/sizeof(led_id_t),
    sizeof(Letter_W)/sizeof(led_id_t), sizeof(Letter_X)/sizeof(led_id_t),
    sizeof(Letter_Y)/sizeof(led_id_t), sizeof(Letter_Z)/sizeof(led_id_t)
};

// ── Input ─────────────────────────────────────────────────────────────────────

typedef enum {
    EVT_NONE,
    EVT_DOT,
    EVT_DASH,
    EVT_SAVE,
    EVT_SEND,
    EVT_BOTH
} input_event_t;

typedef struct {
    uint32_t press_time;
    bool     is_pressed;
} button_t;

static button_t btnA = {0};
static button_t btnB = {0};

static input_event_t input_update(void) {
    static bool both_latched = false;

    bool a = ((NRF_P0->IN & (1 << BUTTON_A_PIN)) == 0);
    bool b = ((NRF_P0->IN & (1 << BUTTON_B_PIN)) == 0);

    uint32_t now = now_ms();

    if (a && !btnA.is_pressed) { btnA.is_pressed = true;  btnA.press_time = now; }
    if (b && !btnB.is_pressed) { btnB.is_pressed = true;  btnB.press_time = now; }

    if (a && b) { both_latched = true; }

    if (!a && btnA.is_pressed) {
        btnA.is_pressed = false;
        if (both_latched) {
            if (!b) both_latched = false;
            return b ? EVT_BOTH : EVT_NONE;
        }
        uint32_t dt = now - btnA.press_time;
        return (dt < 200) ? EVT_DOT : EVT_DASH;
    }

    if (!b && btnB.is_pressed) {
        btnB.is_pressed = false;
        if (both_latched) {
            if (!a) both_latched = false;
            return a ? EVT_BOTH : EVT_NONE;
        }
        uint32_t dt = now - btnB.press_time;
        return (dt < 200) ? EVT_SAVE : EVT_SEND;
    }

    return EVT_NONE;
}

// ── ID / Message configuration ────────────────────────────────────────────────

int Receiver_ID = 0;

#define MAX_SYMBOLS          32
#define N_Ids                6
#define My_ID                1

char message[MAX_SYMBOLS + 1];
int  message_len = 0;

#define MAX_MORSE_PER_LETTER 5

char morse_buf[MAX_MORSE_PER_LETTER + 1];
int  morse_len = 0;

// ── Morse table ───────────────────────────────────────────────────────────────

typedef struct { const char* code; char letter; } MorseEntry;

const MorseEntry morse_table[] = {
    {".-",   'A'}, {"-...", 'B'}, {"-.-.", 'C'}, {"-..",  'D'},
    {".",    'E'}, {"..-.", 'F'}, {"--.",  'G'}, {"....", 'H'},
    {"..",   'I'}, {".---", 'J'}, {"-.-",  'K'}, {".-..", 'L'},
    {"--",   'M'}, {"-.",   'N'}, {"---",  'O'}, {".--.", 'P'},
    {"--.-", 'Q'}, {".-.",  'R'}, {"...",  'S'}, {"-",    'T'},
    {"..-",  'U'}, {"...-", 'V'}, {".--",  'W'}, {"-..-", 'X'},
    {"-.--", 'Y'}, {"--..", 'Z'},
    {"-----",'0'}, {".----",'1'}, {"..---",'2'}, {"...--",'3'},
    {"....-",'4'}, {".....", '5'},{"-....", '6'}, {"--...", '7'},
    {"---..", '8'}, {"----.", '9'},
    {0, 0}
};

char morse_to_char(const char* code) {
    for (int i = 0; morse_table[i].code != 0; i++) {
        const char* a = morse_table[i].code;
        const char* b = code;
        while (*a && *b && *a == *b) { a++; b++; }
        if (*a == 0 && *b == 0) return morse_table[i].letter;
    }
    return '?';
}

// ── Letter display animation ──────────────────────────────────────────────────

static char     anim_msg[MAX_SYMBOLS + 1] = {0};
static int      anim_len   = 0;
static int      anim_idx   = 0;
static uint32_t anim_next  = 0;

#define LETTER_DISPLAY_MS  800   // tempo por letra
#define LETTER_GAP_MS      200   // pausa entre letras (display apagado)

static void display_message_task(void) {
    if (anim_idx >= anim_len) return;  // animação terminou, não re-agenda

    uint32_t now = now_ms();
    if (now < anim_next) {
        scheduler_push_task(display_message_task, TASKPRIO_COAP);
        return;
    }

    char c = anim_msg[anim_idx];

    if (c >= 'A' && c <= 'Z') {
        int i = c - 'A';
        display_show_timed(Letters[i], Letters_size[i], LETTER_DISPLAY_MS);
        anim_next = now + LETTER_DISPLAY_MS + LETTER_GAP_MS;
    } else if (c == ' ') {
        display_clear();
        anim_next = now + LETTER_GAP_MS;
    } else {
        anim_next = now;  // caractere desconhecido, pula
    }

    anim_idx++;
    scheduler_push_task(display_message_task, TASKPRIO_COAP);
}

void display_message(const char* msg, int len) {
    // copia e converte para maiúsculo
    int i;
    for (i = 0; i < len && i < MAX_SYMBOLS; i++) {
        char c = msg[i];
        anim_msg[i] = (c >= 'a' && c <= 'z') ? c - 32 : c;
    }
    anim_msg[i] = '\0';
    anim_len  = i;
    anim_idx  = 0;
    anim_next = now_ms();

    scheduler_push_task(display_message_task, TASKPRIO_COAP);
}

// ── Raw Radio ─────────────────────────────────────────────────────────────────

// Packet layout: [to_id][from_id][text...][CRC CRC]
// LENGTH_CRC (2) is appended automatically by the radio hardware
#define RADIO_CHANNEL        26
#define PKT_MAXLEN           (MAX_SYMBOLS + 2 + LENGTH_CRC)

static uint8_t rx_buf[PKT_MAXLEN] __attribute__((aligned(4)));
static volatile bool    pkt_received = false;
static volatile uint8_t pkt_data[MAX_SYMBOLS + 2];
static volatile uint8_t pkt_data_len = 0;

// Called by radio.c at start-of-frame — we don't need it but must provide it
static void radio_sof_cb(PORT_TIMER_WIDTH timestamp) {
    (void)timestamp;
}

static void radio_eof_cb(PORT_TIMER_WIDTH timestamp) {
    (void)timestamp;

    uint8_t len;
    int8_t  rssi;
    uint8_t lqi;
    bool    crc_ok;

    radio_getReceivedFrame(rx_buf, &len, sizeof(rx_buf), &rssi, &lqi, &crc_ok);

    if (!crc_ok || len < (2 + LENGTH_CRC)) {  // ← account for CRC in length
        radio_rxEnable();
        radio_rxNow();
        return;
    }

    uint8_t to_id = rx_buf[0];
    uint8_t actual_len = len - LENGTH_CRC;  // ← strip CRC count

    if (to_id == (uint8_t)My_ID || to_id == 0) {
        memcpy((void*)pkt_data, rx_buf, actual_len);
        pkt_data_len = actual_len;
        pkt_received = true;
    }

    radio_rxEnable();
    radio_rxNow();
}

static void raw_radio_init(void) {
    radio_init();
    radio_setStartFrameCb(radio_sof_cb);
    radio_setEndFrameCb(radio_eof_cb);
    radio_setFrequency(RADIO_CHANNEL, FREQ_RX);
    radio_rxEnable();
    radio_rxNow();
    printf("[RAW] Radio ready on channel %d, My_ID=%d\n", RADIO_CHANNEL, My_ID);
}

void raw_radio_send(void) {
    if (message_len == 0) return;

    uint8_t pkt[PKT_MAXLEN];
    pkt[0] = (uint8_t)Receiver_ID;
    pkt[1] = (uint8_t)My_ID;
    memcpy(&pkt[2], message, message_len);
    uint8_t pkt_len = message_len + 2;

    radio_rfOff();
    radio_setFrequency(RADIO_CHANNEL, FREQ_TX);
    radio_loadPacket(pkt, pkt_len + LENGTH_CRC);  // ← add LENGTH_CRC here
    radio_txEnable();
    radio_txNow();

    // Brief busy-wait for TX to complete (~1ms for 32 bytes at 250kbps)
    uint32_t t = now_ms();
    while (now_ms() - t < 5);

    // Switch back to RX
    radio_rfOff();
    radio_setFrequency(RADIO_CHANNEL, FREQ_RX);
    radio_rxEnable();
    radio_rxNow();

    printf("[RAW] Sent to=%d from=%d msg=%s\n", Receiver_ID, My_ID, message);
    printf("[RAW] Sending %d bytes: ", pkt_len);
    for (int i = 0; i < pkt_len; i++) {
        printf("0x%02X ", pkt[i]);
    }
    printf("\n");
}

static void check_incoming(void) {
    if (!pkt_received) return;
    pkt_received = false;

    uint8_t from_id  = pkt_data[1];
    uint8_t text_len = pkt_data_len - 2;
    pkt_data[pkt_data_len] = '\0';

    printf("[RAW] MSG from=%d: %s\n", from_id, (char*)&pkt_data[2]);

    display_message((const char*)&pkt_data[2], text_len);
}


// ── Task ──────────────────────────────────────────────────────────────────────

static void app_task(void) {
    check_incoming();

    input_event_t evt = input_update();

    switch (evt) {
        case EVT_DOT:
            printf("[ACTION] DOT\n");
            display_show_timed(DOT, sizeof(DOT)/sizeof(led_id_t), 800);
            if (morse_len < MAX_MORSE_PER_LETTER) {
                morse_buf[morse_len++] = '.';
                morse_buf[morse_len]   = '\0';
            }
            break;
        case EVT_DASH:
            printf("[ACTION] DASH\n");
            display_show_timed(DASH, sizeof(DASH)/sizeof(led_id_t), 800);
            if (morse_len < MAX_MORSE_PER_LETTER) {
                morse_buf[morse_len++] = '-';
                morse_buf[morse_len]   = '\0';
            }
            break;
        case EVT_SAVE:
            printf("[ACTION] SAVE\n");
            display_show_timed(SAVE, sizeof(SAVE)/sizeof(led_id_t), 800);
            if (morse_len > 0 && message_len < MAX_SYMBOLS) {
                message[message_len++] = morse_to_char(morse_buf);
                message[message_len]   = '\0';
                morse_len    = 0;
                morse_buf[0] = '\0';
            }
            break;
        case EVT_SEND:
            printf("[ACTION] SEND\n");
            display_show_timed(OK, sizeof(OK)/sizeof(led_id_t), 800);
            raw_radio_send();
            message_len  = 0;
            message[0]   = '\0';
            morse_len    = 0;
            morse_buf[0] = '\0';
            break;
        case EVT_BOTH:
            printf("[ACTION] BOTH\n");
            Receiver_ID = (Receiver_ID + 1) % N_Ids;
            display_show_timed(Digits[Receiver_ID], Digits_size[Receiver_ID], 1500);
            break;
        default:
            break;
    }

    scheduler_push_task(app_task, TASKPRIO_COAP);
}

// ── Entry ─────────────────────────────────────────────────────────────────────

void mote_main(void) {
    printf("START\n");

    systick_init();
    display_timer_init();
    buttons_init();
    display_clear();
    raw_radio_init();

    scheduler_push_task(app_task, TASKPRIO_COAP);
    scheduler_start();
}
