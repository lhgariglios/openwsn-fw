# micro:bit Morse Radio Communicator

A bare-metal C application for the **BBC micro:bit v2** (nRF52833) that lets multiple devices exchange text messages over a 2.4 GHz radio link. Messages are composed letter-by-letter using **Morse code** via the two onboard buttons, transmitted wirelessly, and displayed on the 5×5 LED matrix.

---

## Table of Contents

1. [Overview](#overview)
2. [Hardware](#hardware)
3. [Architecture](#architecture)
4. [Packet Format](#packet-format)
5. [How to Use](#how-to-use)
6. [Button Reference](#button-reference)
7. [LED Feedback](#led-feedback)
8. [Code Structure](#code-structure)
9. [Configuration](#configuration)
10. [Known Limitations](#known-limitations)

---

## Overview

Each micro:bit is assigned a unique numeric ID at compile time (`My_ID`). The user selects a destination ID (`Receiver_ID`), composes a message in Morse code, and sends it. The receiving device decodes the packet and animates each letter of the message on its LED matrix.

The entire application runs on bare-metal hardware — no RTOS, no Nordic SDK abstractions beyond the thin `radio.c` / `scheduler.c` helpers provided by the course framework.

---

## Hardware

| Component | Details |
|---|---|
| MCU | Nordic nRF52833 (ARM Cortex-M4, 64 MHz) |
| Board | BBC micro:bit v2 |
| Display | 5×5 LED matrix (row/column multiplexed) |
| Input | Button A (pin 14), Button B (pin 23) |
| Radio | 2.4 GHz proprietary (250 kbps) |

### LED Matrix Wiring

The matrix is driven by 5 row pins and 5 column pins. Rows are active-high, columns are active-low. Multiplexing is handled by TIMER0 at ~500 Hz (one pixel active per 2 ms slot), which gives a flicker-free display for multi-pixel patterns.

| Signal | nRF52833 Pin |
|---|---|
| ROW 1–5 | P0.21, P0.22, P0.15, P0.24, P0.19 |
| COL 1–3 | P0.28, P0.11, P0.31 |
| COL 4 | P1.05 |
| COL 5 | P0.30 |

---

## Architecture

```
┌─────────────────────────────────────────────┐
│                  mote_main()                │
│  systick_init  display_timer_init           │
│  buttons_init  raw_radio_init               │
│  scheduler_push_task(app_task)              │
│  scheduler_start()  ← cooperative loop      │
└─────────────────────┬───────────────────────┘
                      │ every tick
                      ▼
             ┌─────────────────┐
             │  app_task()     │  ← runs from scheduler
             │                 │
             │ check_incoming()|  reads pkt_received flag
             │ input_update()  |  polls buttons, returns event
             │ switch(evt)     |  DOT / DASH / SAVE / SEND / BOTH
             └─────────────────┘

Interrupts (async):
  SysTick_Handler   → ms_ticks++
  TIMER0_IRQHandler → LED multiplexing
  radio_eof_cb      → copies packet to pkt_data, sets pkt_received
```

The scheduler is **cooperative** — `app_task` always re-pushes itself at the end, creating a polling loop. All heavy work (radio send, display animation) is non-blocking; timing uses `ms_ticks` from SysTick.

---

## Packet Format

Packets are transmitted over the radio as raw bytes. The hardware automatically appends a 2-byte CRC.

```
Byte 0       Byte 1       Bytes 2..N     CRC (hw)
┌──────────┬───────────┬──────────────┬──────────┐
│  to_id   │  from_id  │  text (ASCII)│  2 bytes │
└──────────┴───────────┴──────────────┴──────────┘
```

- `to_id` — destination device ID (0 = broadcast, accepted by all devices; do not use 0 as a device ID)
- `from_id` — sender's `My_ID`
- `text` — raw ASCII, up to `MAX_SYMBOLS` (32) bytes
- CRC is verified by the radio hardware; packets with CRC errors are silently dropped

On reception, `LENGTH_CRC` (2) is subtracted from the reported length to obtain the true payload length.

---

## How to Use

### Step 1 — Select a Receiver

Press **A + B simultaneously** to cycle through device IDs (1–5). The current target ID is shown as a digit on the LED matrix. Repeat until the desired ID is displayed.

> **Note:** ID 0 is reserved for broadcast and is accepted by all devices.

### Step 2 — Compose a Letter in Morse

Use **Button A** to enter dots and dashes:
- Short press (< 200 ms) → **dot**
- Long press (≥ 200 ms) → **dash**

Each dot/dash is shown briefly on the display as feedback.

### Step 3 — Confirm the Letter

Press **Button B** shortly to **SAVE** the current Morse sequence as a letter. The decoded letter is appended to the message buffer (up to 32 characters). The display shows a save indicator.

Repeat steps 2–3 for each letter in your message.

### Step 4 — Send

Press **Button B** for a long press (≥ 200 ms) to **SEND** the full message. The packet is transmitted over the radio and the message buffer is cleared.

### Step 5 — Receiving

When a packet addressed to your device arrives, each letter of the message is displayed sequentially on the LED matrix (800 ms per letter, 200 ms gap between letters).

---

## Button Reference

| Action | Result |
|---|---|
| A short press | Enter a **dot** (.) |
| A long press | Enter a **dash** (-) |
| B short press | **Save** current Morse → letter |
| B long press | **Send** full message |
| A + B together | Cycle **Receiver ID** (0 → 1 → … → 5 → 0) |

### Morse Code Quick Reference

| Digit | Code |
|-------|------|
| 0 | ----- |
| 1 | .---- |
| 2 | ..--- |
| 3 | ...-- |
| 4 | ....- |
| 5 | ..... |
| 6 | -.... |
| 7 | --... |
| 8 | ---.. |
| 9 | ----. |

| Letter | Code |
|--------|------|
| A | .- |
| B | -... |
| C | -.-. |
| D | -.. |
| E | . |
| F | ..-. |
| G | --. |
| H | .... |
| I | .. |
| J | .--- |
| K | -.- |
| L | .-.. |
| M | -- |
| N | -. |
| O | --- |
| P | .--. |
| Q | --.- |
| R | .-. |
| S | ... |
| T | - |
| U | ..- |
| V | ...- |
| W | .-- |
| X | -..- |
| Y | -.-- |
| Z | --.. |

> **Note:** Pixel-art display patterns are only defined for digits 0–5. Digits 6–9 can be composed and transmitted in Morse but will not be rendered on the LED matrix when a received message is animated.

---

## LED Feedback

| Display | Meaning |
|---|---|
| Single centre dot | Dot entered |
| Three-pixel horizontal bar | Dash entered |
| Square ring pattern | Letter saved to buffer |
| Tick / checkmark pattern | Message sent or received |
| Digit (0–5) | Current Receiver ID |
| Letter animation | Each letter of a received message |

The letter animation cycles through each character in the received message, rendering a pixel-art version of each letter for 800 ms before moving to the next.

---

## Code Structure

```
main.c (or mote_main.c)
│
├── Time          SysTick at 64 MHz → 1 ms resolution
├── Pins          GPIO macros for rows, columns, buttons
├── LED           display_clear(), display_pixel(), display_show_timed()
├── Timer         TIMER0 ISR — LED multiplexing at ~500 Hz
├── Patterns      DOT, DASH, SAVE, OK pixel patterns
├── Digits        Pixel-art digits 0–5 for ID display
├── Letters       Pixel-art A–Z for received message animation
├── Animation     display_message_task() — scheduler-driven letter sequencer
├── Input         input_update() — button state machine, dot/dash/save/send/both
├── Morse table   morse_to_char() — 36-entry lookup (A–Z, 0–9)
├── Radio         radio_eof_cb() ISR, raw_radio_send(), check_incoming()
└── App task      app_task() — main cooperative loop
```

### Key Files from Framework

| File | Role |
|---|---|
| `radio.c / radio.h` | Low-level nRF52833 radio driver |
| `scheduler.c / scheduler.h` | Lightweight cooperative task scheduler |
| `board.h` | Board-level constants (e.g. `LENGTH_CRC`) |

---

## Configuration

All compile-time constants are at the top of the source file:

```c
#define My_ID           1       // This device's ID (change per device: 1–5; 0 is reserved for broadcast)
#define N_Ids           6       // Total number of IDs in the network (0–5)
int     Receiver_ID   = 0;      // Default target ID (changeable at runtime)

#define RADIO_CHANNEL   26      // 2.4 GHz channel (must match all devices)
#define MAX_SYMBOLS     32      // Maximum message length in characters
#define MAX_MORSE_PER_LETTER 5  // Maximum dots/dashes per letter

#define LETTER_DISPLAY_MS  800  // How long each letter is shown (ms)
#define LETTER_GAP_MS      200  // Pause between letters (ms)
```

To deploy to a new device, change only `My_ID` (use values 1–5; ID 0 is reserved for broadcast). All other settings must be identical across all devices on the network.

---

## Known Limitations

- **One message at a time** — if a second packet arrives while `pkt_received` is still set, it is silently dropped. The flag is cleared only when `check_incoming` runs in the main task.
- **No acknowledgement** — there is no confirmation that the receiver got the message. The sender only sees the local OK pattern.
- **No encryption** — packets are plain ASCII. Any device on the same channel and with CRC enabled can receive them.
- **TX busy-wait** — the send function uses a 5 ms busy-wait instead of polling `EVENTS_END`. On rare occasions this may be too short for the radio to finish; replacing it with a hardware event poll would be more robust.
- **Display conflict** — the letter animation uses the same `display_show_timed` infrastructure as button feedback. Pressing a button while a received message is animating will interrupt the animation.
- **Digit display limited to 0–5** — pixel-art patterns are only defined for digits 0–5. Digits 6–9 can be encoded and transmitted but will not animate correctly on the receiving device's LED matrix.
