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

#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "midi_service_stream_handler.h"

namespace rppicomidi
{
class BLE_MIDI_Manager {
public:
    /**
     * @brief Construct a new ble midi manager object
     * 
     * @param local_name the local name (if more than 13 characters, will be truncated)
     * @param local_name_len the number of characters in the local name
     */
    BLE_MIDI_Manager(const char* local_name, size_t local_name_len);

    BLE_MIDI_Manager()=delete;
    ~BLE_MIDI_Manager()=default;
    BLE_MIDI_Manager(const BLE_MIDI_Manager&)=delete;
    void operator=(const BLE_MIDI_Manager&)=delete;

    /**
     * @brief Initialize the Bluetooth system and start connection advertisements
     * 
     * @param instance_ a pointer to an instance of this class
     * @return true if Bluetooth system successfully initialized, false otherwise
     */
    bool init(BLE_MIDI_Manager* instance_);

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
private:
    void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
    static void static_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
    static const uint8_t APP_AD_FLAGS=0x06;
    static constexpr uint8_t adv_data[]{
        // Flags general discoverable
        0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
        // Service class list
        0x11, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS, 0x00, 0xc7, 0xc4, 0x4e, 0xe3, 0x6c, 0x51, 0xa7, 0x33, 0x4b, 0xe8, 0xed, 0x5a, 0x0e, 0xb8, 0x03,
    };
    static const uint8_t adv_data_len = sizeof(adv_data);
    uint8_t scan_resp_data[16];
    uint8_t scan_resp_data_len;
    hci_con_handle_t con_handle;
    btstack_packet_callback_registration_t sm_event_callback_registration;
    static BLE_MIDI_Manager* instance;
};
}