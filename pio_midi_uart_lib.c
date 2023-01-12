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
#define MAX_PIO_MIDI_UARTS 4
#endif
#ifndef MIDI_UART_RING_BUFFER_LENGTH
#define MIDI_UART_RING_BUFFER_LENGTH 128
#endif
#ifndef MIDI_BAUD_RATE
#define MIDI_BAUD_RATE 31250
#endif
/**
 * @struct defines the properties of PIO MIDI UART
 */
typedef struct MIDI_UART_S {
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

void* pio_midi_uart_create(uint8_t txgpio, uint8_t rxgpio)
{
    PIO_MIDI_UART_T* midi_uart = NULL;
    // claim two state machines in the same PIO
    PIO pio = pio0;
    int rx_sm = 0;
    int tx_sm = 1;
    int irq = PIO0_IRQ_0;
    int idx = 0;
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
                    rx_sm = 0;
                    tx_sm = 1;
                    irq = PIO1_IRQ_0;
                    idx = 2;
                }
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

    if ((idx & 1) == 0) {
        // then the programs are not loaded in the PIO program memory yet
        if (pio_can_add_program(pio, &midi_rx_program) && pio_can_add_program(pio, &midi_tx_program)) {
            midi_uart->rx_offset = pio_add_program(pio, &midi_rx_program);
            midi_uart->tx_offset = pio_add_program(pio, &midi_tx_program);
        }
        else {
            // programs won't fit
            return NULL;
        }
    }
    else {
        midi_uart->rx_offset = pio_midi_uarts[idx-1].rx_offset;
        midi_uart->tx_offset = pio_midi_uarts[idx-1].tx_offset;
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
    // enable the rx state machine IRQ
    pio_midi_uart_set_rx_irq_enable(pio, rx_sm, true);
    // disable the tx state machine IRQ (no data to send yet)
    pio_midi_uart_set_tx_irq_enable(pio, tx_sm, false);

    irq_set_enabled(midi_uart->irq, true);

    return midi_uart;
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
        printf("pio_midi_uart_show_pio_info: Error MIDI port instance is not valid\r\n");
        return;
    }
    printf("MIDI Port %u using PIO%c rx_sm=%u tx_sm=%u irq=%u\r\n", idx, midi_uart->pio == pio0 ? '0':'1', midi_uart->rx_sm, midi_uart->tx_sm, midi_uart->irq);
}