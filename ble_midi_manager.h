/**
 * MIT License
 *
 * Copyright (c) 2023 rppicomidi
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
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
 * To the extent this code is solely for use on the Rapsberry Pi Pico W or
 * Pico WH, the license file ${PICO_SDK_PATH}/src/rp2_common/pico_btstack/LICENSE.RP may
 * apply.
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
#pragma once

#include <string>
#include <map>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "midi_service_stream_handler.h"
#include "ble_midi_client.h"

namespace rppicomidi
{
class BLE_MIDI_Manager {
public:
    /**
     * @brief Construct a new ble midi manager object
     * 
     * @param local_name the local name; null terminated string (if more than 29 characters, will be truncated)
     */
    BLE_MIDI_Manager(const char* local_name);

    BLE_MIDI_Manager()=delete;
    ~BLE_MIDI_Manager()=default;
    BLE_MIDI_Manager(const BLE_MIDI_Manager&)=delete;
    void operator=(const BLE_MIDI_Manager&)=delete;

    enum LE_Advertising_Report_Event_Type {
        ADV_IND = 0,
        ADV_DIRECT_IND = 1,
        ADV_SCAN_IND = 2,
        ADV_NONCONN_IND = 3,
        SCAN_RSP = 4,
    };

    /**
     * @brief Initialize the Bluetooth system and start connection advertisements
     * 
     * @param instance_ a pointer to an instance of this class
     * @param is_client_ is true if initializing as a client; false otherwise
     * @return true if Bluetooth system successfully initialized, false otherwise
     */
    bool init(BLE_MIDI_Manager* instance_, bool is_client_);

    void deinit(bool is_client);
    /**
     * @brief see if the Bluetooth LE MIDI device is connected
     * 
     * @return true if connected, false otherwise.
     */
    bool is_connected() {return con_handle != HCI_CON_HANDLE_INVALID;}

    /**
     * @brief read a MIDI stream from the connected Bluetooth LE MIDI device
     * 
     * @param bytes the buffer to hold the bytes read
     * @param max_bytes the maximum number of bytes to read
     * @return 0 if not connected or no bytes available, otherwise
     * return as many bytes as there are to read.
     */
    uint8_t stream_read(uint8_t* bytes, uint8_t max_bytes);

    /**
     * @brief write a MIDI stream to the connected Bluetooth LED MIDI device
     * 
     * @param bytes the buffer that holds the bytes to write
     * @param num_bytes the number of bytes to write
     * @return the number of bytes actually written. Will be 0 if not connected
     */
    uint8_t stream_write(const uint8_t* bytes, uint8_t num_bytes);

    /**
     * @brief Force the Bluetooth connection to disconnect
     * 
     */
    void disconnect();

    /**
     * @brief list the bonded Bluetooth LE devices
     * 
     */
    void list_le_device_info();

    /**
     * @brief delete the bonding information at entry idx
     * 
     * @param idx the database index of the Bluetooth LE device
     */
    void delete_le_bonding_info(int idx);

    /**
     * @brief Start active scan. If necessary, exit server mode and switch to client mode first.
     * 
     */
    void scan_begin();

    /**
     * @brief Stop active scan
     * 
     */
    void scan_end();

    /**
     * @brief Write out a list of MIDI peripherals to the console
     * 
     */
    void dump_midi_peripherals();

    /**
     * @brief In client mode, connect to the specified device
     * @param idx 0: is the previously connected peripheral
     *            1 or greater: is the idx from the midi_peripherals
     *            list as displayed by dump_midi_peripherals
     * @return true if successful, false if idx==0 and no previously
     *         connected device is stored, or idx is greater than the
     *         number of peripherals discovered in the last scan.
     */
    bool client_request_connect(uint8_t idx);
private:
    /**
     * @brief called by static_packet_handler to handle BT Stack messages
     * 
     * @param packet_type the type of the BT Stack packet
     * @param channel the BT Stack channel
     * @param packet the packet data
     * @param size the size of the packet in bytes
     */
    void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
    // Handles connect, disconnect, and advertising report events,  
    // starts the GATT client, and sends the first query.
    void handle_hci_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
    static void static_handle_hci_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
    static void static_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
#if 0
    void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
    static void static_handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
    static void static_scan_timer_cb(btstack_timer_source_t* timer);
    /**
     * @brief 
     * 
     * @param bdaddr the 6 byte Bluetooth Address
     * @return a 64-bit version of the address
     */
    uint64_t bdaddr2uint64(uint8_t* bdaddr) {
        return (uint64_t)bdaddr[0] | ((uint64_t)bdaddr[1] << 8) |
        ((uint64_t)bdaddr[2] << 16) | ((uint64_t)bdaddr[3] << 24) |
        ((uint64_t)bdaddr[4] << 32) | ((uint64_t)bdaddr[5] << 40);
    }

    /**
     * @brief convert a bdaddr encoded in a uint64_t to a 6-byte bdaddr
     * 
     * @param bdaddr64 
     * @param bdaddr 
     */
    void uint64_2bdaddr(uint64_t bdaddr64, uint8_t* bdaddr) {
        for (int jdx = 0; jdx < 6; jdx++) {
            bdaddr[jdx] = (bdaddr64 >> (uint64_t)(8*jdx)) & 0xff;    
        }
    }

    void enter_client_mode();
    static uint32_t const scan_blink_timeout_ms = 500;
    static int32_t const scan_remove_timeout = 6; // when decremented to 0, remove entry from midi_peripherals (in units of scan_blink_timeout_ms)

    struct Advertised_MIDI_Peripheral {
        Advertised_MIDI_Peripheral() : type{0}, timeout{0} {}
        char name[32];
        uint8_t type; // BDADDR = 0; otherwise COMPLETE or SHORTENED
        uint8_t addr_type;
        int32_t timeout;
    };
    bool get_local_name_from_ad_data(uint8_t ad_len, const uint8_t* ad_data, Advertised_MIDI_Peripheral& peripheral);
    static const uint8_t APP_AD_FLAGS=0x06;
    static constexpr uint8_t adv_data[]{
        // Flags general discoverable
        0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
        // Service class list
        0x11, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS, 0x00, 0xc7, 0xc4, 0x4e, 0xe3, 0x6c, 0x51, 0xa7, 0x33, 0x4b, 0xe8, 0xed, 0x5a, 0x0e, 0xb8, 0x03,
    };
    static const uint8_t adv_data_len = sizeof(adv_data);
    uint8_t scan_resp_data[32];
    uint8_t scan_resp_data_len;
    hci_con_handle_t con_handle;
    uint16_t conn_interval;
    bool is_client;
    bool initialized;
    bool is_scan_mode;
    btstack_packet_callback_registration_t sm_event_callback_registration;
    btstack_packet_callback_registration_t hci_event_callback_registration;
    static BLE_MIDI_Manager* instance;
    btstack_timer_source_t scan_timer;
    bd_addr_type_t next_connect_bd_addr_type;
    uint8_t next_connect_bd_addr[6];
    /**
     * @brief a map of BLE MIDI peripheral bdaddr to local name strings
     */
    std::map<uint64_t, Advertised_MIDI_Peripheral> midi_peripherals;
#endif
    static const uint8_t APP_AD_FLAGS=0x06;
    static constexpr uint8_t adv_data[]{
        // Flags general discoverable
        0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
        // Service class list
        0x11, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS, 0x00, 0xc7, 0xc4, 0x4e, 0xe3, 0x6c, 0x51, 0xa7, 0x33, 0x4b, 0xe8, 0xed, 0x5a, 0x0e, 0xb8, 0x03,
    };
    static const uint8_t adv_data_len = sizeof(adv_data);
    uint8_t scan_resp_data[32];
    uint8_t scan_resp_data_len;
    hci_con_handle_t con_handle;
    bool is_client;
    bool initialized;
    bool is_scan_mode;
    btstack_packet_callback_registration_t sm_event_callback_registration;
    static BLE_MIDI_Manager* instance;
};
}