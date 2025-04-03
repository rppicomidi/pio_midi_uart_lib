/**
 * @file midi_uart_lib.c
 * @brief this library provides functions for using a Raspberry Pi Pico
 * UART as a MIDI interface
 * 
 * MIT License

 * Copyright (c) 2022 rppicomidi

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/binary_info.h"
#include "pico/sync.h"
#include "pio_midi_uart_lib.h"

// You can override these to save space if need be
#ifndef MAX_PIO_MIDI_UARTS
#if NUM_PIOS == 2
#define MAX_PIO_MIDI_UARTS 4
#elif NUM_PIOS == 3
#define MAX_PIO_MIDI_UARTS 6
#else
#error "unsupported NUM_PIOS"
#endif
#endif
#ifndef MAX_PIO_MIDI_OUTS
#if NUM_PIOS == 2
#define MAX_PIO_MIDI_OUTS 8
#elif NUM_PIOS == 3
#define MAX_PIO_MIDI_OUTS 12
#else
#error "unsupported NUM_PIOS"
#endif
#endif
#ifndef MIDI_UART_RING_BUFFER_LENGTH
#define MIDI_UART_RING_BUFFER_LENGTH 128
#endif
#ifndef MIDI_BAUD_RATE
#define MIDI_BAUD_RATE 31250
#endif

#define PIO_PROG_INVALID_OFFSET 0xFFFF
#if NUM_PIOS == 3
static uint pio_prog_rx_offset[3] = {PIO_PROG_INVALID_OFFSET, PIO_PROG_INVALID_OFFSET, PIO_PROG_INVALID_OFFSET};
static uint pio_prog_tx_offset[3] = {PIO_PROG_INVALID_OFFSET, PIO_PROG_INVALID_OFFSET, PIO_PROG_INVALID_OFFSET};
#elif NUM_PIOS == 2
static uint pio_prog_rx_offset[2] = {PIO_PROG_INVALID_OFFSET, PIO_PROG_INVALID_OFFSET};
static uint pio_prog_tx_offset[2] = {PIO_PROG_INVALID_OFFSET, PIO_PROG_INVALID_OFFSET};
#else
#error "unsupported NUM_PIOS"
#endif
/**
 * @struct defines the properties of PIO MIDI UART
 */
typedef struct PIO_MIDI_UART_S {
    PIO pio;        // The PIO containing the RX and TX code
    uint rx_sm;     // The state machine executing the RX code
    uint tx_sm;     // The state machine executing the TX code
    uint tx_gpio;   // The TX pin
    uint rx_gpio;   // The RX pin
    uint irq;       // The PIO IRQ number
    uint32_t rx_mask; // The rx queue not empty interrupt mask
    uint32_t tx_mask; // The tx queue not empty interrupt mask
    io_ro_32* ints; // the PIO IRQ status register for this UART
    uint rx_offset; // The offset in PIO program RAM of the RX code
    uint tx_offset; // The offset in PIO program RAM of the TX code
    // PIO UART ring buffer info
    ring_buffer_t rx_rb, tx_rb;
    uint8_t rx_buf[MIDI_UART_RING_BUFFER_LENGTH];
    uint8_t tx_buf[MIDI_UART_RING_BUFFER_LENGTH];
} PIO_MIDI_UART_T;

/**
 * @brief List of PIO MIDI UARTs. For array index port_num,
 * the PIO number is (port_num / 2) and the IRQ for the PIO
 * is (port_num % 2). That is, port_num=0 is PIO0, IRQ0.
 * port_num=3 is PIO1, IRQ1, etc.
 */
static PIO_MIDI_UART_T pio_midi_uarts[MAX_PIO_MIDI_UARTS];

typedef struct PIO_MIDI_OUT_S {
    struct PIO_MIDI_OUT_S* out_shared_irq; // a pointer to the other state machine, that shares the same IRQ; NULL if not shared
    PIO pio;        // The PIO containing the RX and TX code
    uint tx_sm;     // The state machine executing the TX code
    uint tx_gpio;   // The TX pin
    uint irq;       // The PIO IRQ number
    uint32_t tx_mask; // The tx queue not empty interrupt mask
    io_ro_32* ints; // the PIO IRQ status register for this UART
    uint tx_offset; // The offset in PIO program RAM of the TX code
    // PIO MIDI OUT ring buffer info
    ring_buffer_t tx_rb;
    uint8_t tx_buf[MIDI_UART_RING_BUFFER_LENGTH];
} PIO_MIDI_OUT_T;

/**
 * @brief List of PIO MIDI OUTs. For array index port_num,
 * the PIO number is (port_num / 4), the state machine is
 * (port_num % 4), and the IRQ for the PIO
 * is (port_num % 2). That is, port_num=0 is PIO0, IRQ0.
 * port_num=3 is PIO0, IRQ1, etc.
 */
static PIO_MIDI_OUT_T pio_midi_outs[MAX_PIO_MIDI_OUTS];

/**
 * @brief enable or disable the FIFO not empty IRQ for a MIDI UART's RX FIFO
 *
 * @param pio the PIO the MIDI UART uses
 * @param sm the state machine the MIDI UART RX uses
 * @param enable true to enable the IRQ, false otherwise
 */
static inline void pio_midi_uart_set_rx_irq_enable(PIO pio, uint sm, bool enable)
{
    pio_set_irqn_source_enabled(pio, sm == 0 ? 0 : 1, sm == 0 ? pis_sm0_rx_fifo_not_empty : pis_sm2_rx_fifo_not_empty, enable);
}

/**
 * @brief enable or disable the FIFO not full IRQ for a MIDI UART's TX FIFO
 *
 * @param pio the PIO the MIDI UART uses
 * @param sm the state machine the MIDI UART TX uses
 * @param enable true to enable the IRQ, false otherwise
 */
static inline void pio_midi_uart_set_tx_irq_enable(PIO pio, uint sm, bool enable)
{
    pio_set_irqn_source_enabled(pio, sm == 1 ? 0 : 1, sm == 1 ? pis_sm1_tx_fifo_not_full : pis_sm3_tx_fifo_not_full, enable);
}

/**
 * @brief enable or disable the FIFO not full IRQ for a MIDI OUT's TX FIFO
 *
 * @param pio the PIO the MIDI UART uses
 * @param sm the state machine the MIDI UART TX uses
 * @param enable true to enable the IRQ, false otherwise
 * @note state machines 0 & 1 use IRQ0, 2 & 3 use IRQ1
 */
static inline void pio_midi_out_set_tx_irq_enable(PIO pio, uint sm, bool enable)
{
    pio_set_irqn_source_enabled(pio, sm < 2 ? 0 : 1, pis_sm0_tx_fifo_not_full+sm, enable);
}

/**
 * @brief
 *
 * @param pio the PIO the MIDI UART uses
 * @param sm the state machine the MIDI UART TX uses
 * @ return true if the MIDI UART's RX FIFO not empty IRQ is pending
 */
static inline bool pio_midi_uart_is_rx_irq_pending(PIO_MIDI_UART_T* midi_uart)
{
    return (*(midi_uart->ints) & midi_uart->rx_mask) != 0;
}

/**
 * @brief
 *
 * @param pio the PIO the MIDI UART uses
 * @param sm the state machine the MIDI UART TX uses
 * @ return true if the MIDI UART's TX FIFO not FULL IRQ is pending
 */
static inline bool pio_midi_uart_is_tx_irq_pending(PIO_MIDI_UART_T* midi_uart)
{
    return (*(midi_uart->ints) & midi_uart->tx_mask) != 0;
}

/**
 * @brief
 *
 * @param pio the PIO the MIDI OUT uses
 * @param sm the state machine the MIDI UART TX uses
 * @ return true if the MIDI UART's TX FIFO not FULL IRQ is pending
 */
static inline bool pio_midi_out_is_tx_irq_pending(PIO_MIDI_OUT_T* midi_out)
{
    return (*(midi_out->ints) & midi_out->tx_mask) != 0;
}

static void on_pio_midi_uart_irq(PIO_MIDI_UART_T *pio_midi_uart);

static void on_pio_midi_uart0_irq()
{
    on_pio_midi_uart_irq(pio_midi_uarts+0);
}

static void on_pio_midi_uart1_irq()
{
    on_pio_midi_uart_irq(pio_midi_uarts+1);
}

static void on_pio_midi_uart2_irq()
{
    on_pio_midi_uart_irq(pio_midi_uarts+2);
}

static void on_pio_midi_uart3_irq()
{
    on_pio_midi_uart_irq(pio_midi_uarts+3);
}

#if NUM_PIOS > 2
static void on_pio_midi_uart4_irq()
{
    on_pio_midi_uart_irq(pio_midi_uarts+4);
}

static void on_pio_midi_uart5_irq()
{
    on_pio_midi_uart_irq(pio_midi_uarts+5);
}
#endif

static void on_pio_midi_uart_irq(PIO_MIDI_UART_T *pio_midi_uart)
{
    if (pio_midi_uart_is_rx_irq_pending(pio_midi_uart)) {
        while (!pio_sm_is_rx_fifo_empty(pio_midi_uart->pio, pio_midi_uart->rx_sm) && 
                !ring_buffer_is_full_unsafe(&pio_midi_uart->rx_rb)) {
            uint8_t val = midi_rx_program_get(pio_midi_uart->pio, pio_midi_uart->rx_sm);
            ring_buffer_push_unsafe(&pio_midi_uart->rx_rb, &val, 1);
        }
    }
    if (pio_midi_uart_is_tx_irq_pending(pio_midi_uart)) {
        while (!ring_buffer_is_empty_unsafe(&pio_midi_uart->tx_rb) &&
                midi_tx_program_can_put(pio_midi_uart->pio, pio_midi_uart->tx_sm)) {
            uint8_t val;
            (void)ring_buffer_pop_unsafe(&pio_midi_uart->tx_rb, &val, 1);
            midi_tx_program_put(pio_midi_uart->pio, pio_midi_uart->tx_sm, val);
        }
        if (ring_buffer_is_empty_unsafe(&pio_midi_uart->tx_rb)) {
            pio_midi_uart_set_tx_irq_enable(pio_midi_uart->pio, pio_midi_uart->tx_sm, false);
        }
    }
}

static void on_pio_midi_out_irq(PIO_MIDI_OUT_T *pio_midi_out);

static void on_pio_midi_out01_irq()
{
    on_pio_midi_out_irq(pio_midi_outs+0);
    on_pio_midi_out_irq(pio_midi_outs->out_shared_irq);
}
static void on_pio_midi_out23_irq()
{
    on_pio_midi_out_irq(pio_midi_outs+2);
    on_pio_midi_out_irq((pio_midi_outs+2)->out_shared_irq);
}
static void on_pio_midi_out45_irq()
{
    on_pio_midi_out_irq(pio_midi_outs+4);
    on_pio_midi_out_irq((pio_midi_outs+4)->out_shared_irq);
}
static void on_pio_midi_out67_irq()
{
    on_pio_midi_out_irq(pio_midi_outs+6);
    on_pio_midi_out_irq((pio_midi_outs+6)->out_shared_irq);
}
#if NUM_PIOS > 2
static void on_pio_midi_out89_irq()
{
    on_pio_midi_out_irq(pio_midi_outs+8);
    on_pio_midi_out_irq((pio_midi_outs+8)->out_shared_irq);
}
static void on_pio_midi_out1011_irq()
{
    on_pio_midi_out_irq(pio_midi_outs+10);
    on_pio_midi_out_irq((pio_midi_outs+10)->out_shared_irq);
}
#endif
static void on_pio_midi_out_irq(PIO_MIDI_OUT_T *pio_midi_out)
{
    if (pio_midi_out && pio_midi_out_is_tx_irq_pending(pio_midi_out)) {
        while (!ring_buffer_is_empty_unsafe(&pio_midi_out->tx_rb) &&
                midi_tx_program_can_put(pio_midi_out->pio, pio_midi_out->tx_sm)) {
            uint8_t val;
            (void)ring_buffer_pop_unsafe(&pio_midi_out->tx_rb, &val, 1);
            midi_tx_program_put(pio_midi_out->pio, pio_midi_out->tx_sm, val);
        }
        if (ring_buffer_is_empty_unsafe(&pio_midi_out->tx_rb)) {
            pio_midi_uart_set_tx_irq_enable(pio_midi_out->pio, pio_midi_out->tx_sm, false);
        }
    }
}

void* pio_midi_uart_create(uint8_t txgpio, uint8_t rxgpio)
{
    PIO_MIDI_UART_T* midi_uart = NULL;
    // claim two state machines in the same PIO
    PIO pio = pio0;
    int rx_sm = 0;
    int tx_sm = 1;
    int irq = PIO0_IRQ_0;
    int idx = 0;
    int pio_idx = 0;
    while (pio != NULL) {
        if (pio_sm_is_claimed(pio, rx_sm) || pio_sm_is_claimed(pio, tx_sm)) {
            rx_sm +=2;
            tx_sm +=2;
            ++irq;
            ++idx;
            if (pio_sm_is_claimed(pio, rx_sm) || pio_sm_is_claimed(pio, tx_sm)) {
                if (pio == pio0) {
                    // try again with PIO1
                    pio = pio1;
                    pio_idx = 1;
                    rx_sm = 0;
                    tx_sm = 1;
                    irq = PIO1_IRQ_0;
                    idx = 2;
                }
#if NUM_PIOS > 2
                else if (pio == pio1) {
                    // try again with PIO2
                    pio = pio2;
                    pio_idx = 2;
                    rx_sm = 0;
                    tx_sm = 1;
                    irq = PIO2_IRQ_0;
                    idx = 4;
                }
#endif
                else {
                    // no PIO resources available
                    pio = NULL;
                }
            }
            else {
                // there are enough PIO state machines in the pio
                break;
            }
        }
        else {
            // there are enough PIO state machines in the pio
            break;
        }
    }
    if (pio == NULL)
        return NULL; // no state machines available

    midi_uart = pio_midi_uarts + idx;

    if (pio_prog_rx_offset[pio_idx] == PIO_PROG_INVALID_OFFSET) {
        if (pio_can_add_program(pio, &midi_rx_program)) {
            midi_uart->rx_offset = pio_add_program(pio, &midi_rx_program);
            pio_prog_rx_offset[pio_idx] = midi_uart->rx_offset;
        }
        else {
            // programs won't fit
            return NULL;
        }
    }
    else {
        midi_uart->rx_offset = pio_prog_rx_offset[pio_idx];
    }
    if (pio_prog_tx_offset[pio_idx] == PIO_PROG_INVALID_OFFSET) {
        if (pio_can_add_program(pio, &midi_tx_program)) {
            midi_uart->tx_offset = pio_add_program(pio, &midi_tx_program);
            pio_prog_tx_offset[pio_idx] = midi_uart->tx_offset;
        }
        else {
            // programs won't fit
            return NULL;
        }
    }
    else {
        midi_uart->tx_offset = pio_prog_tx_offset[pio_idx];
    }

    irq_set_enabled(irq, false);
    pio_sm_claim(pio, rx_sm);
    pio_sm_claim(pio, tx_sm);

    midi_uart->pio = pio;

    midi_uart->rx_sm = rx_sm;
    midi_uart->tx_sm = tx_sm;
    midi_uart->tx_gpio = txgpio;
    midi_uart->rx_gpio = rxgpio;
    midi_uart->irq = irq;
    if (rx_sm == 0) {
        midi_uart->rx_mask = 1ul << pis_sm0_rx_fifo_not_empty;
        midi_uart->tx_mask = 1ul << pis_sm1_tx_fifo_not_full;
        midi_uart->ints = &pio->ints0;
    }
    else {
        midi_uart->rx_mask = 1ul << pis_sm2_rx_fifo_not_empty;
        midi_uart->tx_mask = 1ul << pis_sm3_tx_fifo_not_full;
        midi_uart->ints = &pio->ints1;
    }

    midi_rx_program_init(pio, rx_sm, midi_uart->rx_offset, rxgpio, MIDI_BAUD_RATE);
    midi_tx_program_init(pio, tx_sm, midi_uart->tx_offset, txgpio, MIDI_BAUD_RATE);
    // Prepare the MIDI UART ring buffers and interrupt handler and enable interrupts
    ring_buffer_init(&midi_uart->rx_rb, midi_uart->rx_buf, MIDI_UART_RING_BUFFER_LENGTH, midi_uart->irq);
    ring_buffer_init(&midi_uart->tx_rb, midi_uart->tx_buf, MIDI_UART_RING_BUFFER_LENGTH, midi_uart->irq);

    // Install the interrupt handler
    if (idx == 0) {
        irq_set_exclusive_handler(midi_uart->irq, on_pio_midi_uart0_irq);
    }
    else if (idx == 1) {
        irq_set_exclusive_handler(midi_uart->irq, on_pio_midi_uart1_irq);
    }
    else if (idx == 2) {
        irq_set_exclusive_handler(midi_uart->irq, on_pio_midi_uart2_irq);
    }
    else if (idx == 3) {
        irq_set_exclusive_handler(midi_uart->irq, on_pio_midi_uart3_irq);
    }
#if NUM_PIOS > 2
    else if (idx == 4) {
        irq_set_exclusive_handler(midi_uart->irq, on_pio_midi_uart4_irq);
    }
    else if (idx == 5) {
        irq_set_exclusive_handler(midi_uart->irq, on_pio_midi_uart5_irq);
    }
#endif
    // enable the rx state machine IRQ
    pio_midi_uart_set_rx_irq_enable(pio, rx_sm, true);
    // disable the tx state machine IRQ (no data to send yet)
    pio_midi_uart_set_tx_irq_enable(pio, tx_sm, false);

    irq_set_enabled(midi_uart->irq, true);

    return midi_uart;
}


void* pio_midi_out_create(uint8_t txgpio)
{
    PIO_MIDI_OUT_T* midi_out = NULL;
    // claim a state machine in the first available PIO
    PIO pio = pio0;
    int tx_sm = 0;
    int irq = PIO0_IRQ_0;
    int idx = 0;
    int pio_idx = 0;
    while (true) {
        if (pio_sm_is_claimed(pio, tx_sm)) {
            ++tx_sm;
            ++idx;
            // PIOn_IRQ_0 is used for sm == 0 && sm == 1
            // PIOn_IRQ_1 is used for sm == 2 && sm == 3
            if (tx_sm == 2) {
                ++irq;
            }
            if (tx_sm > 3) {
                // no state machines available on the current PIO
                if (pio == pio0) {
                    // try again with PIO1
                    pio = pio1;
                    pio_idx = 1;
                    tx_sm = 0;
                    irq = PIO1_IRQ_0;
                }
#if NUM_PIOS > 2
                if (pio == pio1) {
                    // try again with PIO2
                    pio = pio2;
                    pio_idx = 1;
                    tx_sm = 0;
                    irq = PIO2_IRQ_0;
                }
#endif
                else {
                    // no PIO resources available
                    return NULL;
                }
            }
        }
        else {
            // there are enough PIO state machines in the pio
            break;
        }
    }

    midi_out = pio_midi_outs + idx;

    if (pio_prog_tx_offset[pio_idx] == PIO_PROG_INVALID_OFFSET) {
        if (pio_can_add_program(pio, &midi_tx_program)) {
            midi_out->tx_offset = pio_add_program(pio, &midi_tx_program);
            pio_prog_tx_offset[pio_idx] = midi_out->tx_offset;
        }
        else {
            // programs won't fit
            return NULL;
        }
    }
    else {
        midi_out->tx_offset = pio_prog_tx_offset[pio_idx];
    }

    irq_set_enabled(irq, false);
    pio_sm_claim(pio, tx_sm);

    midi_out->pio = pio;

    midi_out->tx_sm = tx_sm;
    midi_out->tx_gpio = txgpio;
    midi_out->irq = irq;
    midi_out->tx_mask = 1ul << (pis_sm0_tx_fifo_not_full+tx_sm);
    if (tx_sm < 2) {
        midi_out->ints = &pio->ints0;
    }
    else {
        midi_out->ints = &pio->ints1;
    }

    midi_tx_program_init(pio, tx_sm, midi_out->tx_offset, txgpio, MIDI_BAUD_RATE);
    // Prepare the MIDI UART ring buffers and interrupt handler and enable interrupts
    ring_buffer_init(&midi_out->tx_rb, midi_out->tx_buf, MIDI_UART_RING_BUFFER_LENGTH, midi_out->irq);

    irq_set_enabled(midi_out->irq, false);
    // Install the interrupt handler
    if (idx/2 == 0) {
        irq_set_exclusive_handler(midi_out->irq, on_pio_midi_out01_irq);
    }
    else if (idx/2 == 1) {
        irq_set_exclusive_handler(midi_out->irq, on_pio_midi_out23_irq);
    }
    else if (idx/2 == 2) {
        irq_set_exclusive_handler(midi_out->irq, on_pio_midi_out45_irq);
    }
    else if (idx/2 == 3) {
        irq_set_exclusive_handler(midi_out->irq, on_pio_midi_out67_irq);
    }
#if NUM_PIOS > 2
    else if (idx/2 == 4) {
        irq_set_exclusive_handler(midi_out->irq, on_pio_midi_out89_irq);
    }
    else if (idx/2 == 5) {
        irq_set_exclusive_handler(midi_out->irq, on_pio_midi_out1011_irq);
    }
#endif
    // disable the tx state machine IRQ (no data to send yet)
    pio_midi_out_set_tx_irq_enable(pio, tx_sm, false);

    irq_set_enabled(midi_out->irq, true);

    int jdx = idx & 0x6;

    // See if the IRQ is shared with another MIDI output
    if (pio_midi_outs[jdx].irq == pio_midi_outs[jdx+1].irq) {
        pio_midi_outs[jdx].out_shared_irq = pio_midi_outs + jdx + 1;
        pio_midi_outs[jdx+1].out_shared_irq = pio_midi_outs + jdx;
    }
    return midi_out;
}

uint8_t pio_midi_uart_poll_rx_buffer(void *instance, uint8_t* buffer, RING_BUFFER_SIZE_TYPE buflen)
{
    PIO_MIDI_UART_T *midi_uart = (PIO_MIDI_UART_T *)instance;
    return ring_buffer_pop(&midi_uart->rx_rb, buffer, buflen);
}

uint8_t pio_midi_uart_write_tx_buffer(void* instance, uint8_t* buffer, RING_BUFFER_SIZE_TYPE buflen)
{
    PIO_MIDI_UART_T *midi_uart = (PIO_MIDI_UART_T *)instance;
    return ring_buffer_push(&midi_uart->tx_rb, buffer, buflen);
}

uint8_t pio_midi_out_write_tx_buffer(void* instance, uint8_t* buffer, RING_BUFFER_SIZE_TYPE buflen)
{
    PIO_MIDI_OUT_T *midi_out = (PIO_MIDI_OUT_T *)instance;
    return ring_buffer_push(&midi_out->tx_rb, buffer, buflen);
}

void pio_midi_uart_drain_tx_buffer(void* instance)
{
    PIO_MIDI_UART_T *midi_uart = (PIO_MIDI_UART_T *)instance;
    // disable UART interrupts because checking if still transmitting from the buffer
    irq_set_enabled(midi_uart->irq, false);
    // can use the unsafe version of ring_buffer_is_empty_unsafe() because UART IRQ
    // is disabled
    if (!ring_buffer_is_empty_unsafe(&midi_uart->tx_rb)) {
        uint8_t val;
        if (pio_sm_is_tx_fifo_empty(midi_uart->pio, midi_uart->tx_sm)) {
            // then last transmission is complete. Kick start a new one
            RING_BUFFER_SIZE_TYPE result = ring_buffer_pop_unsafe(&midi_uart->tx_rb, &val, 1);
            assert(result == 1);
            pio_sm_put(midi_uart->pio, midi_uart->tx_sm, val);
            pio_midi_uart_set_tx_irq_enable(midi_uart->pio, midi_uart->tx_sm, true);
        }
    }
    irq_set_enabled(midi_uart->irq, true);
}

void pio_midi_out_drain_tx_buffer(void* instance)
{
    PIO_MIDI_OUT_T *midi_out = (PIO_MIDI_OUT_T *)instance;
    // disable UART interrupts because checking if still transmitting from the buffer
    irq_set_enabled(midi_out->irq, false);
    // can use the unsafe version of ring_buffer_is_empty_unsafe() because UART IRQ
    // is disabled
    if (!ring_buffer_is_empty_unsafe(&midi_out->tx_rb)) {
        uint8_t val;
        if (pio_sm_is_tx_fifo_empty(midi_out->pio, midi_out->tx_sm)) {
            // then last transmission is complete. Kick start a new one
            RING_BUFFER_SIZE_TYPE result = ring_buffer_pop_unsafe(&midi_out->tx_rb, &val, 1);
            assert(result == 1);
            pio_sm_put(midi_out->pio, midi_out->tx_sm, val);
            pio_midi_uart_set_tx_irq_enable(midi_out->pio, midi_out->tx_sm, true);
        }
    }
    irq_set_enabled(midi_out->irq, true);
}

void pio_midi_uart_show_pio_info(void* instance)
{
    if (instance == NULL) {
        printf("pio_midi_uart_show_pio_info: Error MIDI port instance is NULL\r\n");
        return;
    }
    PIO_MIDI_UART_T *midi_uart = (PIO_MIDI_UART_T *)instance;
    uint8_t idx = 0;
    for (;idx < MAX_PIO_MIDI_UARTS && &pio_midi_uarts[idx] != midi_uart; idx++) {
    }
    if (idx == MAX_PIO_MIDI_UARTS) {
        printf("pio_midi_uart_show_pio_info: Error MIDI UART port instance is not valid\r\n");
        return;
    }
    printf("MIDI UART Port %u using PIO%c rx_sm=%u tx_sm=%u irq=%u\r\n", idx, PIO_NUM(midi_uart->pio)+'0', midi_uart->rx_sm, midi_uart->tx_sm, midi_uart->irq);
}

void pio_midi_out_show_pio_info(void* instance)
{
    if (instance == NULL) {
        printf("pio_midi_out_show_pio_info: Error MIDI OUT port instance is NULL\r\n");
        return;
    }
    PIO_MIDI_OUT_T *midi_out = (PIO_MIDI_OUT_T *)instance;
    uint8_t idx = 0;
    for (;idx < MAX_PIO_MIDI_OUTS && &pio_midi_outs[idx] != midi_out; idx++) {
    }
    if (idx == MAX_PIO_MIDI_OUTS) {
        printf("pio_midi_out_show_pio_info: Error MIDI OUT port instance is not valid\r\n");
        return;
    }
    printf("MIDI OUT Port %u using PIO%c tx_sm=%u irq=%u ", idx, PIO_NUM(midi_out->pio)+'0', midi_out->tx_sm, midi_out->irq);
    if (midi_out->out_shared_irq) {
        uint8_t jdx = 0;
        for (;jdx < MAX_PIO_MIDI_OUTS && &pio_midi_outs[jdx] != midi_out->out_shared_irq; jdx++) {
        }
        if (idx == MAX_PIO_MIDI_OUTS) {
            printf("\r\npio_midi_out_show_pio_info: Error MIDI OUT port shared IRQ pointer not valid\r\n");
            return;
        }
        else {
            printf("IRQ share with MIDI OUT Port %u\r\n", jdx);
        }
    }
    else {
        printf("IRQ not shared\r\n");
    }
}
