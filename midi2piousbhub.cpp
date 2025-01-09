/**
 * @file Pico-USB-Host-MIDI-Adapter.c
 * @brief A USB Host to Serial Port MIDI adapter that runs on a Raspberry Pi
 * Pico board
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

/**
 * This file uses code from various BlueKitchen example files, which contain
 * the following copyright notice, included per the notice below.
 *
 * Copyright (C) 2018 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */
#ifdef NDEBUG
// Need to do this here for release builds or no CLI commands will be added
// All build variants except DEBUG define NDEBUG, which makes assert() macro generate
// no code at all, which prevents msc_demo_cli_init() from adding any CLI commands.
#undef NDEBUG
#endif

#include <cstdio>
#include <vector>
#include <cstdint>
#include <string>
#include "midi2piousbhub.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "midi_uart_lib.h"
#include "cdc_stdio_lib.h"
#include "bsp/board_api.h"
#include "preset_manager.h"
#include "diskio.h"

// Because the PIO USB code runs in core 1
// and USB MIDI OUT sends are triggered on core 0,
// need to synchronize core startup
static volatile bool core1_booting = true;
static volatile bool core0_booting = true;

// core1: handle host events
void core1_main()
{
    sleep_ms(10);
    multicore_lockout_victim_init(); // need to lockout core1 when core0 writes to flash
    pio_usb_configuration_t pio_cfg = PIO_USB_DEFAULT_CONFIG;
    // Use GP16 for USB D+ and GP17 for USB D-
    pio_cfg.pin_dp = 16;

    // Pico-PIO-USB 0.6.0 consumes all of PIO 0. The Pico W CYW43 SPI PIO
    // code uses some of PIO 1. So the PIO usages no longer conflicts. However,
    // there is still a chance that the DMA tx_ch will conflict with the
    // Pico W CYW43 SPI PIO code. However,
    // the CYW43 SPI driver code is not hard-wired to any particular
    // DMA channel, so as long as tuh_configure() and tuh_init()run
    // after board_init(), which also calls tuh_configure(), and before
    // cyw43_arch_init(), there should be no conflict.
    tuh_configure(BOARD_TUH_RHPORT, TUH_CFGID_RPI_PIO_USB_CONFIGURATION, &pio_cfg);

    // To run USB SOF interrupt in core1, init host stack for pio_usb (roothub
    // port1) on core1
    tuh_init(BOARD_TUH_RHPORT);
    core1_booting = false;
    while(core0_booting) {
    }
    printf("core1 has booted\r\n");
    while (true) {
        tuh_task(); // tinyusb host task
        rppicomidi::Midi2PioUsbhub::instance().flush_usb_tx();
    }
}

void rppicomidi::Midi2PioUsbhub::serialize(std::string &serialized_string)
{
    JSON_Value *root_value = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);
    JSON_Value *from_value = json_value_init_object();
    JSON_Object *from_object = json_value_get_object(from_value);
    JSON_Value *to_value = json_value_init_object();
    JSON_Object *to_object = json_value_get_object(to_value);
    for (auto &midi_in : midi_in_port_list)
    {
        std::string default_nickname;
        make_default_nickname(default_nickname, attached_devices[midi_in->devaddr].vid, attached_devices[midi_in->devaddr].pid, midi_in->cable, true);
        json_object_set_string(from_object, default_nickname.c_str(), midi_in->nickname.c_str());
    }
    for (auto &midi_out : midi_out_port_list)
    {
        std::string default_nickname;
        make_default_nickname(default_nickname, attached_devices[midi_out->devaddr].vid, attached_devices[midi_out->devaddr].pid, midi_out->cable, false);
        json_object_set_string(to_object, default_nickname.c_str(), midi_out->nickname.c_str());
    }
    json_object_set_value(root_object, "from", from_value);
    json_object_set_value(root_object, "to", to_value);

    JSON_Value *routing_value = json_value_init_object();
    JSON_Object *routing_object = json_value_get_object(routing_value);
    for (auto &midi_in : midi_in_port_list)
    {
        JSON_Value *routing = json_value_init_array();
        JSON_Array *routing_array = json_value_get_array(routing);
        for (auto &midi_out : midi_in->sends_data_to_list)
        {
            json_array_append_string(routing_array, midi_out->nickname.c_str());
        }
        json_object_set_value(routing_object, midi_in->nickname.c_str(), json_array_get_wrapping_value(routing_array));
    }
    json_object_set_value(root_object, "routing", routing_value);

#ifdef RPPICOMIDI_PICO_W
    JSON_Value *bluetooth_value = json_value_init_object();
    JSON_Object *bluetooth_object = json_value_get_object(bluetooth_value);
    uint8_t bdaddr[6];
    int adtyp = blem.get_last_connected(bdaddr);
    const char* bdaddr_str = bd_addr_to_str(bdaddr);
    json_object_set_number(bluetooth_object, "last_addr_type", adtyp);
    json_object_set_string(bluetooth_object, "last_addr", bdaddr_str);
    json_object_set_boolean(bluetooth_object, "is_client", blem.is_client_mode());
    json_object_set_value(root_object, "bluetooth", bluetooth_value);
#endif

    auto ser = json_serialize_to_string(root_value);
    serialized_string = std::string(ser);
    json_free_serialized_string(ser);
    json_value_free(root_value);
}

bool rppicomidi::Midi2PioUsbhub::deserialize(std::string &serialized_string)
{
    JSON_Value* root_value = json_parse_string(serialized_string.c_str());
    if (root_value == nullptr) {
        return false;
    }
    JSON_Object* root_object = json_value_get_object(root_value);
    JSON_Value* midi_in_nicknames_value = json_object_get_value(root_object, "from");
    if (midi_in_nicknames_value == nullptr) {
        json_value_free(root_value);
        return false;
    }
    JSON_Object* midi_in_nicknames_object = json_value_get_object(midi_in_nicknames_value);
    if (midi_in_nicknames_object) {
        // update the nicknames
        for (auto& midi_in: midi_in_port_list) {
            std::string def_nickname;
            auto info = &attached_devices[midi_in->devaddr];
            make_default_nickname(def_nickname, info->vid, info->pid, midi_in->cable, true);
            const char* nickname = json_object_get_string(midi_in_nicknames_object, def_nickname.c_str());
            if (nickname) {
                midi_in->nickname = std::string(nickname);
            }
            else {
                printf("could not find nickname %s\r\n", def_nickname.c_str());
            }
        }
    }
    else {
        json_value_free(root_value);
        return false;
    }
    JSON_Value* midi_out_nicknames_value = json_object_get_value(root_object, "to");
    if (midi_out_nicknames_value == nullptr) {
        json_value_free(root_value);
        return false;
    }
    JSON_Object* midi_out_nicknames_object = json_value_get_object(midi_out_nicknames_value);
    if (midi_out_nicknames_object) {
        // update the nicknames
        for (auto& midi_out: midi_out_port_list) {
            std::string def_nickname;
            auto info = &attached_devices[midi_out->devaddr];
            make_default_nickname(def_nickname, info->vid, info->pid, midi_out->cable, false);
            const char* nickname = json_object_get_string(midi_out_nicknames_object, def_nickname.c_str());
            if (nickname) {
                midi_out->nickname = std::string(nickname);
            }
            else {
                printf("could not find nickname %s\r\n", def_nickname.c_str());
            }
        }
    }
    else {
        json_value_free(root_value);
        return false;
    }
    JSON_Value* routing_value = json_object_get_value(root_object, "routing");
    if (routing_value == nullptr) {
        json_value_free(root_value);
        return false;
    }
    JSON_Object* routing_object = json_value_get_object(routing_value);
    if (routing_object) {
        for (auto& midi_in: midi_in_port_list) {
            JSON_Array* routes = json_object_get_array(routing_object, midi_in->nickname.c_str());
            if (routes) {
                midi_in->sends_data_to_list.clear();
                size_t count = json_array_get_count(routes);
                for (size_t idx = 0; idx < count; idx++) {
                    const char* to_nickname = json_array_get_string(routes, idx);
                    if (to_nickname) {
                        std::string nickname = std::string(to_nickname);
                        // Find to_nickname in the midi_out_port_list
                        for (auto& midi_out: midi_out_port_list ) {
                            if (nickname == midi_out->nickname) {
                                // it's connected, so route it
                                midi_in->sends_data_to_list.push_back(midi_out);
                                break;
                            }
                        }
                    }
                    else {
                        // poorly formatted JSON
                        json_value_free(root_value);
                        return false;
                    }
                }
            }
            else {
                // poorly formatted JSON
                //json_value_free(root_value);
                //return false;
                printf("%s is not routed\r\n", midi_in->nickname.c_str());
            }
        }
    }
    else {
        // poorly formatted JSON
        json_value_free(root_value);
        return false;
    }
    #ifdef RPPICOMIDI_PICO_W
    JSON_Value* bluetooth_value = json_object_get_value(root_object, "bluetooth");
    if (bluetooth_value != nullptr) {
        JSON_Object* bluetooth_object = json_value_get_object(bluetooth_value);
        int addr_typ;
        const char* addr_str;
        uint8_t bdaddr[6];
        uint8_t prev_bdaddr[6];
        int prev_addr_typ = blem.get_last_connected(prev_bdaddr);
        int is_client;
        if (json_object_has_value_of_type(bluetooth_object, "last_addr_type", JSONNumber)) {
            addr_typ = json_object_get_number(bluetooth_object, "last_addr_type");
        }
        else {
            // poorly formatted JSON
            json_value_free(root_value);
            return false;
        }
        addr_str = json_object_get_string(bluetooth_object, "last_addr");
        if (sscanf_bd_addr(addr_str, bdaddr) != 1) {
            // poorly formatted JSON
            json_value_free(root_value);
            return false;
        }
        is_client = json_object_get_boolean(bluetooth_object, "is_client");
        if (is_client == -1) {
             // poorly formatted JSON
            json_value_free(root_value);
            return false;
        }
        blem_is_client = (is_client != 0);
        if (blem_is_client != blem.is_client_mode()) {
            if (blem_is_client) {
                blem.set_last_connected(addr_typ, bdaddr);
                blem.reconnect();
            }
            else {
                blem_init(false);
            }
        }
        else if (blem_is_client && (addr_typ != prev_addr_typ || memcmp(bdaddr, prev_bdaddr, 6)) != 0) {
            blem.set_last_connected(addr_typ, bdaddr);
            blem.reconnect();
        }
    }
    // else it is OK if it it is nullptr. Might be a preset from a non-Bluetooth enabled device
    #endif
    json_value_free(root_value);
    return true;
}


int rppicomidi::Midi2PioUsbhub::connect(const std::string& from_nickname, const std::string& to_nickname)
{
    for (auto &in_port : midi_in_port_list) {
        if (in_port->nickname == from_nickname) {
            for (auto out_port : midi_out_port_list) {
                if (out_port->nickname == to_nickname) {
                    in_port->sends_data_to_list.push_back(out_port);
                    return 0;
                }
            }
           return -1;
        }
    }
    return -2;
}

int rppicomidi::Midi2PioUsbhub::disconnect(const std::string& from_nickname, const std::string& to_nickname)
{
    for (auto &in_port : midi_in_port_list) {
        if (in_port->nickname == from_nickname) {
            for (auto it = in_port->sends_data_to_list.begin(); it != in_port->sends_data_to_list.end();) {
                if ((*it)->nickname == to_nickname) {
                    in_port->sends_data_to_list.erase(it);
                    return 0;
                }
                else {
                    ++it;
                }
            }
            return -1;
        }
    }
    return -2;
}

void rppicomidi::Midi2PioUsbhub::reset()
{
    for (auto &in_port :midi_in_port_list) {
        in_port->sends_data_to_list.clear();
    }
}

int rppicomidi::Midi2PioUsbhub::rename(const std::string& old_nickname, const std::string& new_nickname)
{
    // make sure the new nickname is not already in use
    for (auto midi_in : midi_in_port_list) {
        if (midi_in->nickname == new_nickname) {
            return 0;
        }
    }
    for (auto midi_out : midi_out_port_list) {
        if (midi_out->nickname == new_nickname) {
            return 0;
        }
    }
    for (auto &midi_in : midi_in_port_list) {
        if (midi_in->nickname == old_nickname) {
            midi_in->nickname = new_nickname;
            return 1;
        }
    }
    for (auto &midi_out : midi_out_port_list) {
        if (midi_out->nickname == old_nickname) {
            midi_out->nickname = new_nickname;
            return 2;
        }
    }
    return -2;
}

void rppicomidi::Midi2PioUsbhub::blink_led()
{
    static absolute_time_t previous_timestamp = {0};

    static bool led_state = false;

    // This design has no on-board LED
    if (NO_LED_GPIO == LED_GPIO)
        return;
    absolute_time_t now = get_absolute_time();

    int64_t diff = absolute_time_diff_us(previous_timestamp, now);
    if (diff > 1000000)
    {
    	// Set the LED to the current led_state
        #ifndef RPPICOMIDI_PICO_W
        gpio_put(LED_GPIO, led_state);
        #else
        // TODO: cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_state);
        #endif
        // Toggle the led_state & update the timestamp
        led_state = !led_state;
        previous_timestamp = now;
    }
}

void rppicomidi::Midi2PioUsbhub::flush_usb_tx()
{
    uint32_t port_flushed_mask = 0;
    for (auto &out_port : midi_out_port_list)
    {
        // Call tuh_midi_stream_flush() once per output port device address
        uint32_t port_mask = 1 << out_port->devaddr;
        if (out_port->devaddr <= CFG_TUH_DEVICE_MAX &&
            (port_flushed_mask && port_mask) == 0 &&
            tuh_midi_configured(out_port->devaddr))
        {
            tuh_midi_stream_flush(out_port->devaddr);
            port_flushed_mask |= port_mask;
        }
    }
}

void rppicomidi::Midi2PioUsbhub::route_midi(Midi_out_port* out_port, const uint8_t* buffer, uint32_t bytes_read)
{
    if (out_port->devaddr != 0 && attached_devices[out_port->devaddr].configured)
    {
        if (out_port->devaddr < uart_devaddr)
        {
            uint32_t nwritten = tuh_midi_stream_write(out_port->devaddr, out_port->cable, buffer, bytes_read);
            if (nwritten != bytes_read) {
                TU_LOG1("Warning: Dropped %lu bytes sending to %s\r\n", bytes_read - nwritten, out_port->nickname.c_str());
            }
        }
        else if (out_port->devaddr == uart_devaddr)
        {
            uint8_t npushed = midi_uart_write_tx_buffer(midi_uart_instance, buffer, bytes_read);
            if (npushed != bytes_read)
            {
                TU_LOG1("Warning: Dropped %lu bytes sending to UART MIDI OUT\r\n", bytes_read - npushed);
            }
        }
        else if (out_port->devaddr == usbdev_devaddr)
        {
            uint32_t nwritten = tud_midi_stream_write(0, buffer, bytes_read);
            if (nwritten != bytes_read) {
                TU_LOG1("Warning: Dropped %lu bytes sending to USB DEV MIDI IN of host\r\n", bytes_read - nwritten);
            }
        }
#ifdef RPPICOMIDI_PICO_W
        else
        {
            uint8_t nwritten = blem.stream_write(buffer, bytes_read);
            if (nwritten != bytes_read) {
                TU_LOG1("Warning: Dropped %lu bytes sending to BT MIDI IN of remote client\r\n", bytes_read - nwritten);
            }
        }
#endif
    }
    else
    {
        TU_LOG1("skipping %s dev_addr=%u\r\n", out_port->nickname.c_str(), out_port->devaddr);
    }
}

void rppicomidi::Midi2PioUsbhub::poll_midi_uart_rx()
{
    uint8_t rx[48];
    // Pull any bytes received on the MIDI UART out of the receive buffer and
    // send them out via USB MIDI on other connected ports
    uint8_t nread = midi_uart_poll_rx_buffer(midi_uart_instance, rx, sizeof(rx));
    if (nread > 0)
    {
        // figure out where to send data from UART MIDI IN
        for (auto &out_port : uart_midi_in_port.sends_data_to_list)
        {
            route_midi(out_port, rx, nread);
        }
    }
}

void rppicomidi::Midi2PioUsbhub::poll_midi_usbdev_rx()
{
    if (attached_devices[usbdev_devaddr].configured)
    {
        uint8_t rx[48];
        // Pull any bytes received on the USB Device MIDI out of the receive buffer and
        // send them out via other connected ports
        uint8_t nread = tud_midi_stream_read(rx, sizeof(rx));
        if (nread > 0)
        {
            // figure out where to send data from USB Device MIDI IN
            for (auto &out_port : usbdev_midi_in_port.sends_data_to_list)
            {
                route_midi(out_port, rx, nread);
            }
        }
    }
}

#ifdef RPPICOMIDI_PICO_W
void rppicomidi::Midi2PioUsbhub::poll_ble_rx()
{
    if (blem.is_connected()) {
        uint8_t rx[3];
        uint8_t nread = blem.stream_read(rx, sizeof(rx));
        if (nread > 0)
        {
            for (auto &out_port: ble_midi_in_port.sends_data_to_list)
            {
                route_midi(out_port, rx, nread);
            }
        }
    }
}
#endif

#ifdef RPPICOMIDI_PICO_W
rppicomidi::Midi2PioUsbhub::Midi2PioUsbhub() : blem{"Pico W MIDI USB BLE Hub"}, blem_is_client{false}, cli{&preset_manager, &blem}
#else
rppicomidi::Midi2PioUsbhub::Midi2PioUsbhub() : cli{&preset_manager}
#endif
{
    bi_decl(bi_program_description("Provide a USB host interface for Serial Port MIDI."));
    bi_decl(bi_1pin_with_name(LED_GPIO, "On-board LED"));
    bi_decl(bi_2pins_with_names(MIDI_UART_TX_GPIO, "MIDI UART TX", MIDI_UART_RX_GPIO, "MIDI UART RX"));

    // board_init(); is called before this class is created in main();
    tud_init(BOARD_TUD_RHPORT);
    cdc_stdio_lib_init();

    // Map the pins to functions
    gpio_init(LED_GPIO);
    gpio_set_dir(LED_GPIO, GPIO_OUT);
    gpio_init(USBA_PWR_EN_GPIO);
    gpio_put(USBA_PWR_EN_GPIO, 0);
    gpio_set_dir(USBA_PWR_EN_GPIO, GPIO_OUT);
    gpio_put(USBA_PWR_EN_GPIO, 1);
    midi_uart_instance = midi_uart_configure(MIDI_UART_NUM, MIDI_UART_TX_GPIO, MIDI_UART_RX_GPIO);
    printf("Configured MIDI UART %u for 31250 baud\r\n", MIDI_UART_NUM);
    while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT)
    {
        // flush out the console input buffer
    }
    uart_midi_in_port.cable = 0;
    uart_midi_in_port.devaddr = uart_devaddr;
    uart_midi_in_port.sends_data_to_list.clear();
    uart_midi_in_port.nickname = "MIDI-IN-A";
    uart_midi_out_port.cable = 0;
    uart_midi_out_port.devaddr = uart_devaddr;
    uart_midi_out_port.nickname = "MIDI-OUT-A";
    usbdev_midi_in_port.cable = 0;
    usbdev_midi_in_port.devaddr = usbdev_devaddr;
    usbdev_midi_in_port.sends_data_to_list.clear();
    usbdev_midi_in_port.nickname = "PC-MIDI-OUT"; // it's named backwards because MIDI OUT from the PC goes to this device's USB MIDI IN
    usbdev_midi_out_port.cable = 0;
    usbdev_midi_out_port.devaddr = usbdev_devaddr;
    usbdev_midi_out_port.nickname = "PC-MIDI-IN"; // it's named backwards because MIDI IN to the PC comes from this device's USB MIDI OUT
    ble_midi_in_port.cable = 0;
    ble_midi_in_port.devaddr = ble_devaddr;
    ble_midi_in_port.nickname = "BT-MIDI-OUT"; // it's named backwards because MIDI OUT from the BT Client (PC, iPad, etc.) goes to this device's MIDI IN
    ble_midi_out_port.cable = 0;
    ble_midi_out_port.devaddr = ble_devaddr;
    ble_midi_out_port.nickname = "BT-MIDI-IN";  // it's named backwards because MIDI IN from the BT Client (PC, iPad, etc.) comes from this device's MIDI OUT
    attached_devices[uart_devaddr].vid = 0;
    attached_devices[uart_devaddr].pid = 0;
    attached_devices[uart_devaddr].product_name = "MIDI A";
    attached_devices[uart_devaddr].rx_cables = 1;
    attached_devices[uart_devaddr].tx_cables = 1;
    attached_devices[uart_devaddr].configured = true;
    attached_devices[usbdev_devaddr].vid = 0;
    attached_devices[usbdev_devaddr].pid = 1;
    attached_devices[usbdev_devaddr].product_name = "PC MIDI";
    attached_devices[usbdev_devaddr].rx_cables = 1;
    attached_devices[usbdev_devaddr].tx_cables = 1;
    attached_devices[usbdev_devaddr].configured = false;
    attached_devices[ble_devaddr].vid = 0;
    attached_devices[ble_devaddr].pid = 2;
    attached_devices[ble_devaddr].product_name = "BT MIDI";
    attached_devices[ble_devaddr].rx_cables = 1;
    attached_devices[ble_devaddr].tx_cables = 1;
    attached_devices[ble_devaddr].configured = false;
    midi_in_port_list.push_back(&uart_midi_in_port);
    midi_out_port_list.push_back(&uart_midi_out_port);
    midi_in_port_list.push_back(&usbdev_midi_in_port);
    midi_out_port_list.push_back(&usbdev_midi_out_port);
    midi_in_port_list.push_back(&ble_midi_in_port);
    midi_out_port_list.push_back(&ble_midi_out_port);
    preset_manager.init();

    cli.printWelcome();
}

#if 0
// the following utf conversion and print code comes from tinyusb example code, copyright Ha Thach 2019 (tinyusb.org)
//--------------------------------------------------------------------+
// String Descriptor Helper
//--------------------------------------------------------------------+

static void _convert_utf16le_to_utf8(const uint16_t *utf16, size_t utf16_len, uint8_t *utf8, size_t utf8_len) {
    // TODO: Check for runover.
    (void)utf8_len;
    // Get the UTF-16 length out of the data itself.

    for (size_t i = 0; i < utf16_len; i++) {
        uint16_t chr = utf16[i];
        if (chr < 0x80) {
            *utf8++ = chr & 0xffu;
        } else if (chr < 0x800) {
            *utf8++ = (uint8_t)(0xC0 | (chr >> 6 & 0x1F));
            *utf8++ = (uint8_t)(0x80 | (chr >> 0 & 0x3F));
        } else {
            // TODO: Verify surrogate.
            *utf8++ = (uint8_t)(0xE0 | (chr >> 12 & 0x0F));
            *utf8++ = (uint8_t)(0x80 | (chr >> 6 & 0x3F));
            *utf8++ = (uint8_t)(0x80 | (chr >> 0 & 0x3F));
        }
        // TODO: Handle UTF-16 code points that take two entries.
    }
}

// Count how many bytes a utf-16-le encoded string will take in utf-8.
static int _count_utf8_bytes(const uint16_t *buf, size_t len) {
    size_t total_bytes = 0;
    for (size_t i = 0; i < len; i++) {
        uint16_t chr = buf[i];
        if (chr < 0x80) {
            total_bytes += 1;
        } else if (chr < 0x800) {
            total_bytes += 2;
        } else {
            total_bytes += 3;
        }
        // TODO: Handle UTF-16 code points that take two entries.
    }
    return (int) total_bytes;
}

static void print_utf16(uint16_t *temp_buf, size_t buf_len) {
    size_t utf16_len = ((temp_buf[0] & 0xff) - 2) / sizeof(uint16_t);
    size_t utf8_len = (size_t) _count_utf8_bytes(temp_buf + 1, utf16_len);
    _convert_utf16le_to_utf8(temp_buf + 1, utf16_len, (uint8_t *) temp_buf, sizeof(uint16_t) * buf_len);
    ((uint8_t*) temp_buf)[utf8_len] = '\0';

    printf((char*)temp_buf);
}
#endif

void rppicomidi::Midi2PioUsbhub::task()
{
    static absolute_time_t previous_timestamp = {0};

    static bool cli_up_message_pending = false;

    tud_task();

    poll_midi_uart_rx();
    // TinyUSB provides no mounted callback for USB MIDI devices
    attached_devices[usbdev_devaddr].configured = tud_midi_mounted();
#ifdef RPPICOMIDI_PICO_W
    attached_devices[ble_devaddr].configured = blem.is_connected();
#endif
    poll_midi_usbdev_rx();
#ifdef RPPICOMIDI_PICO_W
    poll_ble_rx();
#endif

    midi_uart_drain_tx_buffer(midi_uart_instance);


    cli.task();
    if (cli_up_message_pending)
    {
        absolute_time_t now = get_absolute_time();

        int64_t diff = absolute_time_diff_us(previous_timestamp, now);
        if (diff > 1000000ll) {
            cli_up_message_pending = false;
            cli.printWelcome();
        }
    }
    if (cdc_state_has_changed) {
        // If the CDC CLI terminal is up now,
        // print the CLI running message after a delay
        // to allow the terminal program to stabilize
        cdc_state_has_changed = false;
        cli_up_message_pending = tud_cdc_connected();
        previous_timestamp = get_absolute_time();
    }
}


void rppicomidi::Midi2PioUsbhub::load_current_preset()
{
    std::string current;
    instance().preset_manager.get_current_preset_name(current);
    if (current.length() < 1 || !instance().preset_manager.load_preset(current)) {
        printf("current preset load failed.\r\n");
    }
}

// Main loop
int main()
{
    // default 125MHz is not appropriate. Sysclock should be multiple of 12MHz.
    set_sys_clock_khz(120000, true);

    sleep_ms(10);

    // direct printf to UART
    board_init();

    // all USB Host task run in core1
    multicore_reset_core1();    
    multicore_launch_core1(core1_main);
        // wait for core 1 to finish claiming PIO state machines and DMA
    while(core1_booting) {
    }
    rppicomidi::Midi2PioUsbhub &instance = rppicomidi::Midi2PioUsbhub::instance();
    instance.load_current_preset();
#if RPPICOMIDI_PICO_W
    if (!instance.blem_init()) {
        printf("Error starting up Bluetooth Module\r\nProgam stalled\r\n");
        for (;;) {
            tight_loop_contents();
        }
    }
#endif
    core0_booting = false;
    while (1) {
        instance.task();
    }
}

void rppicomidi::Midi2PioUsbhub::make_default_nickname(std::string &nickname, uint16_t vid, uint16_t pid, uint8_t cable, bool is_from)
{
    char default_nickname[17];
    snprintf(default_nickname, sizeof(default_nickname) - 1, "%04x-%04x%c%d",
             vid,
             pid,
             is_from ? 'F' : 'T',
             cable + 1);
    default_nickname[12] = '\0'; // limit to 12 characters.
    nickname = std::string(default_nickname);
}

void get_info_from_default_nickname(std::string nickname, uint16_t &vid, uint16_t &pid, uint8_t &cable, bool &is_from)
{
    vid = std::stoi(nickname.substr(0, 4), 0, 16);
    pid = std::stoi(nickname.substr(5, 4), 0, 16);
    cable = std::stoi(nickname.substr(10, std::string::npos));
    is_from = nickname.substr(9, 1) == "F";
}


//--------------------------------------------------------------------+
// TinyUSB Callbacks
//--------------------------------------------------------------------+
static uint16_t dev_string_buffer[128];
static uint16_t langid;
void rppicomidi::Midi2PioUsbhub::prod_str_cb(tuh_xfer_t *xfer)
{
    if (xfer->actual_len >= 4 /* long enough for at least one character*/)
    {
        size_t nchars = (xfer->actual_len - 2) / 2;
        char str[nchars + 1];
        uint16_t *utf16le = (uint16_t *)(xfer->buffer + 2);
        for (size_t idx = 0; idx < nchars; idx++)
        {
            str[idx] = (uint8_t)utf16le[idx];
        }
        str[nchars] = '\0';
        auto devinfo = reinterpret_cast<rppicomidi::Midi2PioUsbhub::Midi_device_info *>(xfer->user_data);
        devinfo->product_name = std::string(str);

        for (auto &midi_in : instance().midi_in_port_list)
        {
            if (midi_in->devaddr == xfer->daddr)
            {
                instance().make_default_nickname(midi_in->nickname, instance().attached_devices[xfer->daddr].vid,
                                                 instance().attached_devices[xfer->daddr].pid,
                                                 midi_in->cable, true);
            }
        }
        for (auto &midi_out : instance().midi_out_port_list)
        {
            if (midi_out->devaddr == xfer->daddr)
            {
                instance().make_default_nickname(midi_out->nickname, instance().attached_devices[xfer->daddr].vid,
                                                 instance().attached_devices[xfer->daddr].pid,
                                                 midi_out->cable, false);
            }
        }
        instance().load_current_preset();
        devinfo->configured = true;
    }
}

void rppicomidi::Midi2PioUsbhub::langid_cb(tuh_xfer_t *xfer)
{
    if (xfer->actual_len >= 4 /*length, type, and one lang ID*/)
    {
        langid = *((uint16_t *)(xfer->buffer + 2));
        tuh_descriptor_get_product_string(xfer->daddr, langid, dev_string_buffer, sizeof(dev_string_buffer), rppicomidi::Midi2PioUsbhub::prod_str_cb, xfer->user_data);
    }
}

void rppicomidi::Midi2PioUsbhub::tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep, uint8_t num_cables_rx, uint16_t num_cables_tx)
{
    (void)in_ep;
    (void)out_ep;
    TU_LOG2("MIDI device address = %u, IN endpoint %u has %u cables, OUT endpoint %u has %u cables\r\n",
            dev_addr, in_ep & 0xf, num_cables_rx, out_ep & 0xf, num_cables_tx);
    // As many MIDI IN ports and MIDI OUT ports as required
    for (uint8_t cable = 0; cable < num_cables_rx; cable++)
    {
        auto port = new Midi_in_port;
        port->cable = cable;
        port->devaddr = dev_addr;

        midi_in_port_list.push_back(port);
    }
    for (uint8_t cable = 0; cable < num_cables_tx; cable++)
    {
        auto port = new Midi_out_port;
        port->cable = cable;
        port->devaddr = dev_addr;

        midi_out_port_list.push_back(port);
    }
}

void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep, uint8_t num_cables_rx, uint16_t num_cables_tx)
{
    rppicomidi::Midi2PioUsbhub::instance().tuh_midi_mount_cb(dev_addr, in_ep, out_ep, num_cables_rx, num_cables_tx);
}

void rppicomidi::Midi2PioUsbhub::tuh_mount_cb(uint8_t dev_addr)
{
    // Don't need to fetch the product string if this is notification for MSC drive
    if (msc_fat_is_plugged_in(dev_addr-1))
        return;

    tuh_vid_pid_get(dev_addr, &attached_devices[dev_addr].vid, &attached_devices[dev_addr].pid);

    tuh_descriptor_get_string(dev_addr, 0, 0, dev_string_buffer, sizeof(dev_string_buffer), langid_cb, (uintptr_t)(attached_devices + dev_addr));
}

void tuh_mount_cb(uint8_t dev_addr)
{
    rppicomidi::Midi2PioUsbhub::instance().tuh_mount_cb(dev_addr);
}

// Invoked when device with MIDI interface is un-mounted
void rppicomidi::Midi2PioUsbhub::tuh_midi_unmount_cb(uint8_t dev_addr, uint8_t)
{
    for (std::vector<Midi_in_port *>::iterator it = midi_in_port_list.begin(); it != midi_in_port_list.end();)
    {
        if ((*it)->devaddr == dev_addr)
        {
            delete (*it);
            midi_in_port_list.erase(it);
        }
        else
        {
            // remove all reference to the device address in existing sends_data_to_list elements
            for (std::vector<Midi_out_port *>::iterator jt = (*it)->sends_data_to_list.begin(); jt != (*it)->sends_data_to_list.end();)
            {
                if ((*jt)->devaddr == dev_addr)
                {
                    (*it)->sends_data_to_list.erase(jt);
                }
                else
                {
                    ++jt;
                }
            }
            ++it;
        }
    }

    for (std::vector<Midi_out_port *>::iterator it = uart_midi_in_port.sends_data_to_list.begin(); it != uart_midi_in_port.sends_data_to_list.end();)
    {
        if ((*it)->devaddr == dev_addr)
        {
            uart_midi_in_port.sends_data_to_list.erase(it);
        }
        else
        {
            ++it;
        }
    }

    for (std::vector<Midi_out_port *>::iterator it = usbdev_midi_in_port.sends_data_to_list.begin(); it != usbdev_midi_in_port.sends_data_to_list.end();)
    {
        if ((*it)->devaddr == dev_addr)
        {
            usbdev_midi_in_port.sends_data_to_list.erase(it);
        }
        else
        {
            ++it;
        }
    }

    for (std::vector<Midi_out_port *>::iterator it = midi_out_port_list.begin(); it != midi_out_port_list.end();)
    {
        if ((*it)->devaddr == dev_addr)
        {
            delete (*it);
            midi_out_port_list.erase(it);
        }
        else
        {
            ++it;
        }
    }
    attached_devices[dev_addr].configured = false;
    attached_devices[dev_addr].product_name.clear();
    attached_devices[dev_addr].vid = 0;
    attached_devices[dev_addr].pid = 0;
    attached_devices[dev_addr].rx_cables = 0;
    attached_devices[dev_addr].tx_cables = 0;
}

void tuh_midi_umount_cb(uint8_t dev_addr, uint8_t instance)
{
    rppicomidi::Midi2PioUsbhub::instance().tuh_midi_unmount_cb(dev_addr, instance);
}

void rppicomidi::Midi2PioUsbhub::tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets)
{
    if (num_packets != 0)
    {
        uint8_t cable_num;
        uint8_t buffer[48];
        while (1)
        {
            uint32_t bytes_read = tuh_midi_stream_read(dev_addr, &cable_num, buffer, sizeof(buffer));
            if (bytes_read == 0)
                return;
            // Route the MIDI stream to the correct MIDI OUT port
            for (auto &in_port : midi_in_port_list)
            {
                if (in_port->devaddr == dev_addr && in_port->cable == cable_num)
                {
                    for (auto &out_port : in_port->sends_data_to_list)
                    {
                        route_midi(out_port, buffer, bytes_read);
                    }
                    break; // found the right in_port; don't need to stay in the loop
                }
            }
        }
    }
}

void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets)
{
    rppicomidi::Midi2PioUsbhub::instance().tuh_midi_rx_cb(dev_addr, num_packets);
}

void tuh_midi_tx_cb(uint8_t dev_addr)
{
    (void)dev_addr;
}
