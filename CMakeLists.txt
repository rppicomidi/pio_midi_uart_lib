cmake_minimum_required(VERSION 3.13)

add_library(pio_midi_uart_lib INTERFACE)
target_sources(pio_midi_uart_lib INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/pio_midi_uart_lib.c
)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../ring_buffer_lib lib/ring_buffer_lib)
target_include_directories(pio_midi_uart_lib INTERFACE ${CMAKE_CURRENT_LIST_DIR})
pico_generate_pio_header(pio_midi_uart_lib ${CMAKE_CURRENT_LIST_DIR}/pio_midi_uart.pio)
target_include_directories(pio_midi_uart_lib INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)
target_link_libraries(pio_midi_uart_lib INTERFACE ring_buffer_lib hardware_pio hardware_gpio)