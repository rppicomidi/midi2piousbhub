cmake_minimum_required(VERSION 3.13)

set(BOARD pico_sdk)
# Set the TinyUSB log level
# set(LOG 2)
include(pico_sdk_import.cmake)

set(target_proj midi2piousbhub)
project(midi2piousbhub C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPICO_USE_MALLOC_MUTEX=1")
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/ext_lib/littlefs-lib ext_lib/littlefs-lib)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/ext_lib/fatfs/source)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/midi_uart_lib)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/rp2040_rtc)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/usb_midi_host)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/cdc_stdio_lib)

set(EMBEDDED_CLI_PATH ${CMAKE_CURRENT_LIST_DIR}/ext_lib/embedded-cli/lib/)
add_executable(midi2piousbhub
    midi2piousbhub.cpp
    preset_manager.cpp
    pico_lfs_cli.cpp
    pico_fatfs_cli.cpp
    preset_manager_cli.cpp
    midi2piousbhub_cli.cpp
    usb_descriptors.c
    ${EMBEDDED_CLI_PATH}/src/embedded_cli.c
    ${CMAKE_CURRENT_LIST_DIR}/ext_lib/parson/parson.c
)

pico_enable_stdio_uart(midi2piousbhub 1)

target_include_directories(midi2piousbhub PRIVATE
 ${CMAKE_CURRENT_LIST_DIR}
 ${EMBEDDED_CLI_PATH}/include
 ${CMAKE_CURRENT_LIST_DIR}/ext_lib/parson
)

target_link_options(midi2piousbhub PRIVATE -Xlinker --print-memory-usage)
target_compile_options(midi2piousbhub PRIVATE -Wall -Wextra -DPICO_HEAP_SIZE=0x20000)
target_link_libraries(midi2piousbhub midi_uart_lib tinyusb_host tinyusb_device tinyusb_board usb_midi_host_app_driver ring_buffer_lib pico_stdlib
littlefs-lib msc_fatfs rp2040_rtc pico_multicore tinyusb_pico_pio_usb cdc_stdio_lib)
if(DEFINED PICO_BOARD)
if(${PICO_BOARD} MATCHES "pico_w")
message("board is pico_w")
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/lib/pico-w-ble-midi-lib)

# add additional compile and link options
target_compile_definitions(midi2piousbhub PRIVATE
  RPPICOMIDI_PICO_W=1
  #uncomment this to see the bluetooth trace on the serial port
  #WANT_HCI_DUMP=1
)
# suppress warning about ABI change in GCC 7.1. AFAIK, it does not matter in this context. Please see
# https://stackoverflow.com/questions/48149323/what-does-the-gcc-warning-project-parameter-passing-for-x-changed-in-gcc-7-1-m
target_compile_options( midi2piousbhub PRIVATE -Wno-psabi)
target_include_directories(midi2piousbhub PRIVATE
    ${CMAKE_CURRENT_LIST_DIR} # For btstack config, etc.
    )
target_link_libraries(midi2piousbhub
  pico_cyw43_arch_none
  ble_midi_server_lib
  ble_midi_client_lib
  pico_btstack_cyw43
)
target_sources(midi2piousbhub PRIVATE
  ble_midi_manager.cpp
  ble_midi_manager_cli.cpp
)
set(RPPICOMIDI_PICO_W 1)
pico_btstack_make_gatt_header(midi2piousbhub PRIVATE "${CMAKE_CURRENT_LIST_DIR}/ble_midi_profile.gatt" "${CMAKE_CURRENT_LIST_DIR}/lib/pico-w-ble-midi-lib")
else()
message("board is pico")
endif()
else()
message("board is not defined")
endif()

pico_add_extra_outputs(midi2piousbhub)

