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
#include "ble_midi_server.h"
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

    /**
     * @brief De-initialize the Bluetooth system
     */
    void deinit();
    /**
     * @brief see if the Bluetooth LE MIDI device is connected
     * 
     * @return true if connected, false otherwise.
     */
    bool is_connected() {if (is_client) return ble_midi_client_is_ready(); else return ble_midi_server_is_connected(); }

    /**
     * @brief Get BDADDR and type of the last connected BLE MIDI server
     *
     * @param addr points to the 6 bytes to store the BDADDR (not modified if return value is BD_ADDR_TYPE_UNKNOWN)
     * @return BD_ADDR_TYPE_UNKNOWN if not client mode or if last connected was never set. Otherwise, return the address type
     */
    int get_last_connected(uint8_t* addr) {if (is_client) return ble_midi_client_get_last_conntected(addr); return BD_ADDR_TYPE_UNKNOWN; }

    /**
     * @brief Set the BDADDR and type of the last connected BLE MIDI server
     *
     * @param addr_type is the BDADDR type of the last connected
     * @param addr points to the 6-byte BDADDR of the lastt connected BLE MIDI server
     * @return true if in client mode, false in server mode.
     */
    void set_last_connected(int addr_type, uint8_t addr[6]) {ble_midi_client_set_last_connected(addr_type, addr); }

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

    bool is_server_mode() {return !is_client; }
    bool is_client_mode() {return is_client; }
    bool is_initialized() {return initialized; }
    bool reconnect();
    bool get_keep_client_connected();
    void set_keep_client_connected(bool keep_client_connected_);
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
    uint8_t scan_resp_data[32];
    uint8_t scan_resp_data_len;
    //hci_con_handle_t con_handle;
    bool is_client;
    bool initialized;
    bool is_scan_mode;
    btstack_packet_callback_registration_t sm_event_callback_registration;
    static BLE_MIDI_Manager* instance;
};
}