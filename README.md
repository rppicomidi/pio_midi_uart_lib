# pio\_midi\_uart\_lib

This library adds up 4 PIO-based MIDI serial ports to a Rasberry Pi Pico.
Each serial port has both MIDI IN and MIDI OUT. Data I/O is interrupt
driven and is designed to be read and written from a simple loop.

If you need more MIDI OUT than MIDI IN, you can create a PIO MIDI OUT port
without creating a MIDI IN port for up to 8 MIDI OUT and no MIDI IN.

Please report bugs and feature requests as github issues on this project.
Pull requests that add features are welcome.

# Library features:
- The MIDI TX signal and MIDI RX signal can be on arbitrary pins.
- The MIDI RX pin of each MIDI UART is configured as an input pin
and the MIDI TX pin of each MIDI UART is configured for open drain output.
- The create function automatically allocates a MIDI UART intialized
to the correct MIDI baud rate. It uses the first 2 available state 
machines on the first available PIO. If all PIO state machines are
used for MIDI UART, up to 4 MIDI UARTs can be created.
- The library uses a [ring buffer library](https://github.com/rppicomidi/ring_buffer_lib) so there are more than 8 bytes of FIFO between the MIDI UART and the application.

# Why not use the RP2040 hardware UARTs instead?
This library API is very similar to [midi_uart_lib](https://github.com/rppicomidi/midi_uart_lib) except it does not use the native hardware UARTs. The advantages of this library are:
- The MIDI TX output is open drain. This makes hardware interface easier.
- RP2040 has only 2 hardware UARTs, and UART0 is often hard-coded for use as the debug console. If you need more than one MIDI UART, this technique is likely your only practical option.
- RP2040 UARTs are always bidirectional. With this library, you may create
MIDI OUT ports without creating MIDI IN ports

# How to use
1. Create a subdirectory (e.g. `lib`) under your main application library.
2. Install [this library](https://github.com/rppicomidi/pio_midi_uart_lib)
and the [ring buffer library](https://github.com/rppicomidi/ring_buffer_lib) as git submodules under the lib directory. 
3. Add `pio_midi_uart_lib` to the `target_link_libraries` in your `CMakeLists.txt` file.
4. Include `pio_midi_uart_lib.h` in your application.
5. Call `pio_midi_uart_create()` in your application for each MIDI port
you wish to create.
6. In your application main loop, call `pio_midi_uart_poll_rx_buffer` to
see if the interrupt handler has loaded any characters to the RX ring buffer.
7. In your application main loop, add messages to send by calling
`pio_midi_uart_write_tx_buffer()` and then call `pio_midi_uart_drain_tx_buffer()` to kick off a new transmission.

# Sample program
The [midi-multistream2usbdev](https://github.com/rppicomidi/midi-multistream2usbdev) project demonstrates creating two bi-directional
MIDI ports and 4 MIDI OUT only ports.

# TODO and possible future features
- If there is a feature request for it, add an API to handle more MIDI IN
than MIDI OUT.
