/**
 * @file pio_midi_uart.h
 * @brief this library provides functions for using a Raspberry Pi Pico
 * PIOs as a UART  MIDI interface
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
#pragma once
#include <stdint.h>
#include "hardware/pio.h"
#include "pio_midi_uart.pio.h"
#include "ring_buffer_lib.h"
#ifdef __cplusplus
extern "C" {
#endif
/**
 * @brief Create a PIO MIDI port pair
 * 
 * @param txgpio the GPIO number of the MIDI OUT pin
 * @param rxgpio the GPIO number of the MIDI IN pin
 * @return a pointer to the MIDI port instance or NULL if
 * the new MIDI port could not be created
 */
void* pio_midi_uart_create(uint8_t txgpio, uint8_t rxgpio);

/**
 * @brief fetch up to buflen bytes from the MIDI UART RX buffer
 *
 * @param midi_port a pointer to a MIDI port created by pio_midi_uart_create()
 * @param buffer is a pointer to an array of bytes to receive the message
 * @param buflen is the the maximum number of bytes in the array
 * 
 * @return the number of bytes fetched
 */
uint8_t pio_midi_uart_poll_rx_buffer(void *midi_port, uint8_t *buffer, RING_BUFFER_SIZE_TYPE buflen);

/**
 * @brief put the bytes in buffer into the MIDI UART TX buffer
 * 
 * @param midi_port a pointer to a MIDI port created by pio_midi_uart_create()
 * @param buffer is a pointer to an array of bytes to receive the message
 * @param buflen is the the number of bytes in the array
 * 
 * @return the number of bytes loaded; may be less than buflen if the buffer is full
 * @note you must call midi_uart_drain_tx_buffer() to actually send the bytes
 */
uint8_t pio_midi_uart_write_tx_buffer(void *midi_port, uint8_t *buffer, RING_BUFFER_SIZE_TYPE buflen);

/**
 * @brief start transmitting bytes from the tx buffer if not already doing so
 * 
 * This function is necessary to kickstart data transmission either initially,
 * or after the buffer becomes completely empty.
 * @param midi_port a pointer to a MIDI port created by pio_midi_uart_create()
 */
void pio_midi_uart_drain_tx_buffer(void *midi_port);

/**
 * @brief print out PIO-related info about the MIDI port
 *
 * @param midi_port a pointer to a MIDI port created by pio_midi_uart_create()
 */
void pio_midi_uart_show_pio_info(void* midi_port);

#ifdef __cplusplus
}
#endif
