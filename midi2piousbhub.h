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
#pragma once

#include <vector>
#include <cstdint>
#include <string>
#include "pio_usb.h"
#include "tusb.h"
#include "usb_midi_host.h"
#include "host/usbh_pvt.h"
#include "embedded_cli.h"
#include "parson.h"
#include "preset_manager.h"
#include "midi2piousbhub_cli.h"
#ifdef RPPICOMIDI_PICO_W
#include "ble_midi_manager.h"
#endif
namespace rppicomidi
{
    class Midi2PioUsbhub
    {
    public:
        // Singleton Pattern

        /**
         * @brief Get the Instance object
         *
         * @return the singleton instance
         */
        static Midi2PioUsbhub &instance()
        {
            static Midi2PioUsbhub _instance; // Guaranteed to be destroyed.
                                        // Instantiated on first use.
            return _instance;
        }
        Midi2PioUsbhub(Midi2PioUsbhub const &) = delete;
        void operator=(Midi2PioUsbhub const &) = delete;
        void blink_led();
        void flush_usb_tx();
        void poll_midi_uart_rx();
        void poll_midi_usbdev_rx();
#ifdef RPPICOMIDI_PICO_W
        void poll_ble_rx();
#endif
        /**
         * @brief construct a nickname string from the input parameters
         * 
         * @param nickname a reference to the std::string that will store the new nickname
         * @param vid the Connected MIDI Device's Vendor ID
         * @param pid the Connected MIDI Device's Product ID
         * @param cable the port's virtual cable number (0-15)
         * @param is_from is true if the hub terminal is from (connected to the Connected MIDI Device's MIDI OUT)
         */
        void make_default_nickname(std::string& nickname, uint16_t vid, uint16_t pid, uint8_t cable, bool is_from);
        void get_info_from_default_nickname(std::string nickname, uint16_t &vid, uint16_t &pid, uint8_t &cable, bool &is_from);
        struct Midi_device_info
        {
            uint16_t vid;
            uint16_t pid;
            uint8_t tx_cables;
            uint8_t rx_cables;
            std::string product_name;
            bool configured;
        };

        struct Midi_out_port
        {
            uint8_t devaddr;
            uint8_t cable;
            std::string nickname;
        };

        struct Midi_in_port
        {
            uint8_t devaddr;
            uint8_t cable;
            std::string nickname;
            std::vector<Midi_out_port *> sends_data_to_list;
        };
        void *midi_uart_instance;
        void tuh_mount_cb(uint8_t dev_addr);
        void tuh_midi_mount_cb(uint8_t dev_addr, uint8_t in_ep, uint8_t out_ep, uint8_t num_cables_rx, uint16_t num_cables_tx);
        void tuh_midi_unmount_cb(uint8_t dev_addr, uint8_t instance);
        void tuh_midi_rx_cb(uint8_t dev_addr, uint32_t num_packets);
        /**
         * @brief create JSON formatted string that represents the current settings
         *
         * The JSON format is
         * {
         *     "nicknames":{
         *         default_nickname: nickname,
         *         default_nickname: nickname,
         *               ...
         *         default_nickname: nickname
         *     },
         *     "routing": {
         *         from_nickname_string:[nickname_string, nickname_string,..., nickname_string],
         *         from_nickname_string:[nickname_string, nickname_string,..., nickname_string],
         *               ...
         *         from_nickname_string:[nickname_string, nickname_string,..., nickname_string]
         *     }
         * }
         * @param serialized_settings
         */
        void serialize(std::string &serialized_settings);
        bool deserialize(std::string &serialized_settings);
        /**
         * @brief connect the MIDI stream from the device and port from_nickname
         * to the device and port to_nickname
         *
         * @param from_nickname the nickname that represents the device an port
         * of the MIDI stream source
         * @param to_nickname the nickname that represents the device an port
         * of the MIDI stream sink
         * @return int 0 if successful, -1 if the to_nickname is invalid, -2
         * if the from_nickname is invalid
         */
        int connect(const std::string& from_nickname, const std::string& to_nickname);

        /**
         * @brief disconnect the MIDI stream from the device and port from_nickname
         * to the device and port to_nickname
         *
         * @param from_nickname the nickname that represents the device an port
         * of the MIDI stream source
         * @param to_nickname the nickname that represents the device an port
         * of the MIDI stream sink
         * @return int 0 if successful, -1 if the to_nickname is invalid, -2
         * if the from_nickname is invalid
         */
        int disconnect(const std::string& from_nickname, const std::string& to_nickname);

        /**
         * @brief clear all MIDI stream connections
         *
         */
        void reset();

        /**
         * @brief rename a device and port nickname
         *
         * @param old_nickname the previous nickname of the device and port
         * @param new_nickname the new nickname of the device and port
         * @return int -1 if the new_nickname is already in use, -2 if the old_nickname is not found,
         * 1 if FROM nickname renamed successfully, 2 if TO nickname renamed successfully
         */
        int rename(const std::string& old_nickname, const std::string& new_nickname);

        /**
         * @brief route the MIDI traffic
         *
         */
        void task();

        Midi_device_info* get_attached_device(size_t addr) { if (addr < 1 || addr > ble_devaddr) return nullptr; return &attached_devices[addr]; }
        bool is_device_configured(size_t addr) { Midi_device_info* dev = get_attached_device(addr); if (dev) return dev->configured; return false; }
        const std::vector<Midi_out_port *>& get_midi_out_port_list() {return midi_out_port_list; }
        const std::vector<Midi_in_port *>& get_midi_in_port_list() {return midi_in_port_list; }
        void notify_cdc_state_changed() {cdc_state_has_changed = true; }
#if RPPICOMIDI_PICO_W
        bool blem_init(bool is_client) { return blem.init(&blem, is_client);}
#endif
    private:
        Midi2PioUsbhub();
        Preset_manager preset_manager;
        void route_midi(Midi_out_port* out_port, const uint8_t* buffer, uint32_t bytes_read);

        static void langid_cb(tuh_xfer_t *xfer);
        static void prod_str_cb(tuh_xfer_t *xfer);
        // UART selection Pin mapping. You can move these for your design if you want to
        // Make sure all these values are consistent with your choice of midi_uart
        static const uint MIDI_UART_NUM = 1;
        static const uint MIDI_UART_TX_GPIO = 4;
        static const uint MIDI_UART_RX_GPIO = 5;
        // On-board LED mapping. If no LED, set to NO_LED_GPIO
        static const uint NO_LED_GPIO = 255;
        static const uint LED_GPIO = 25; // change to 7 for test8 hardware
        static const uint USBA_PWR_EN_GPIO=18;
        static const uint USBA_PWR_FLG_GPIO=19;

        static const uint8_t uart_devaddr = CFG_TUH_DEVICE_MAX + 1;
        static const uint8_t usbdev_devaddr = CFG_TUH_DEVICE_MAX + 2;
        static const uint8_t ble_devaddr = CFG_TUH_DEVICE_MAX + 3;
        // Indexed by dev_addr
        // device addresses start at 1. location 0 is unused
        // extra entries are for the UART MIDI Port and USB device port and the BLE server port
        Midi_device_info attached_devices[CFG_TUH_DEVICE_MAX + 4];

        std::vector<Midi_out_port *> midi_out_port_list;
        std::vector<Midi_in_port *> midi_in_port_list;

        Midi_in_port uart_midi_in_port;
        Midi_out_port uart_midi_out_port;
        Midi_in_port usbdev_midi_in_port;
        Midi_out_port usbdev_midi_out_port;
        Midi_in_port ble_midi_in_port;
        Midi_out_port ble_midi_out_port;
        #if RPPICOMIDI_PICO_W
        BLE_MIDI_Manager blem;
        #endif
        Midi2PioUsbhub_cli cli;
        bool cdc_state_has_changed;
    };
}