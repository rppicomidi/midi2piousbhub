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
#ifdef RPPICOMIDI_PICO_W
#include <cinttypes>
#include <cstdio>
#include <cassert>
#include "pico/assert.h"
#include "pico/stdio.h"
#include "ble_midi_manager.h"
#include "ble_midi_profile.h"
rppicomidi::BLE_MIDI_Manager* rppicomidi::BLE_MIDI_Manager::instance=nullptr;

rppicomidi::BLE_MIDI_Manager::BLE_MIDI_Manager(const char* local_name) :
    is_client{false}, initialized{false}, is_scan_mode{false}
{
    size_t local_name_len = strlen(local_name);
    // The maximum local name length is 29 bytes because the maximum scan response is 31 bytes.
    scan_resp_data[1] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
    if (local_name_len > 29) {
        local_name_len = 29;
        scan_resp_data[1] = BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME;
    }
    scan_resp_data[0] = local_name_len+1;
    memcpy(scan_resp_data+2, local_name, local_name_len);
    scan_resp_data_len = local_name_len+2;
    uint8_t bdaddr[6] = {0,0,0,0,0,0};
    set_last_connected(BD_ADDR_TYPE_UNKNOWN, bdaddr);
    if (cyw43_arch_init()) {
        printf("ble-midi2usbhost: failed to initialize cyw43_arch\n");
        assert(false);
    }
}

bool rppicomidi::BLE_MIDI_Manager::init(BLE_MIDI_Manager* instance_, bool is_client_)
{
    if (initialized) {
        deinit();
    }
    instance = instance_;
    auto context = cyw43_arch_async_context();
    async_context_acquire_lock_blocking(context);
    // pairing request will display a numeric match value if the other end has a display too.
    if (is_client_) {
        const char client_name[]="Pico W MIDI USB BLE Hub";
        ble_midi_client_init(client_name, strlen(client_name),
            IO_CAPABILITY_DISPLAY_ONLY,
            SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_MITM_PROTECTION | SM_AUTHREQ_BONDING
        );
    }
    else
    {
        ble_midi_server_init(profile_data, scan_resp_data, scan_resp_data_len,
            IO_CAPABILITY_DISPLAY_ONLY,
            SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_MITM_PROTECTION | SM_AUTHREQ_BONDING
        );
    }
    is_client = is_client_;

    async_context_release_lock(context);
    initialized = true;
    return true;
}

void rppicomidi::BLE_MIDI_Manager::deinit()
{
    if (!initialized)
        return; // nothing to do
    auto context = cyw43_arch_async_context();
    async_context_acquire_lock_blocking(context);
    if (is_client)
        ble_midi_client_deinit();
    else
        ble_midi_server_deinit();
    async_context_release_lock(context);
    // Wait for deinit to complete
    int32_t timeout = 1000;
    if (is_client) {
        while(timeout >= 0 && !ble_midi_client_is_off()) {
            sleep_ms(10);
            timeout--;
        }
        if (timeout < 0)
            printf("timeout on client deinit()\r\n");
    }
    else {
        while(timeout >= 0 && ble_midi_server_is_initialized()) {
            sleep_ms(10);
            timeout--;
        }
        if (timeout < 0)
            printf("timeout on server deinit()\r\n");
    }
    initialized = false;
}

uint8_t rppicomidi::BLE_MIDI_Manager::stream_read(uint8_t* bytes, uint8_t max_bytes)
{
    uint16_t timestamp;
    uint8_t result = 0;
    if (is_connected()) {
        auto context = cyw43_arch_async_context();
        async_context_acquire_lock_blocking(context);
        if (is_client)
            result = ble_midi_client_stream_read(max_bytes, bytes, &timestamp);
        else
            result = ble_midi_server_stream_read(max_bytes, bytes, &timestamp);
        async_context_release_lock(context);
    }
    return result;
}

uint8_t rppicomidi::BLE_MIDI_Manager::stream_write(const uint8_t* bytes, uint8_t num_bytes)
{
    uint8_t result = 0;
    if (is_connected()) {
        auto context = cyw43_arch_async_context();
        async_context_acquire_lock_blocking(context);
        if (is_client) {
            result = ble_midi_client_stream_write(num_bytes, bytes);
        }
        else {
            result = ble_midi_server_stream_write(num_bytes, bytes);
        }
        async_context_release_lock(context);
    }
    return result;
}

void rppicomidi::BLE_MIDI_Manager::disconnect()
{
    //next_connect_bd_addr_type = BD_ADDR_TYPE_UNKNOWN;
    if (is_connected()) {
        auto context = cyw43_arch_async_context();
        async_context_acquire_lock_blocking(context);
        if (is_client) {
            ble_midi_client_request_disconnect();
        }
        else {
            //gap_disconnect(con_handle);
            ble_midi_server_request_disconnect();
        }
        async_context_release_lock(context);
    }
}

void rppicomidi::BLE_MIDI_Manager::list_le_device_info()
{
    int max_count = le_device_db_max_count();
    bd_addr_t entry_address;
    const char* addr_type_str[] = {"0 LE public",
                             "1 LE random",
                             "2 LE public identity",
                             "3 LE random identity"};
    int count = 0;
    printf("\r\nBonded Device List\r\n");
    printf("Entry Bluetooth Address Type\r\n");
    auto context = cyw43_arch_async_context();
    for (int idx=0; idx < max_count; idx++) {
        int entry_address_type = (int) BD_ADDR_TYPE_UNKNOWN;
        async_context_acquire_lock_blocking(context);
        le_device_db_info(idx, &entry_address_type, entry_address, NULL);
        async_context_release_lock(context);
        // skip non-LE and unused entries
        if (entry_address_type >= (int)BD_ADDR_TYPE_SCO)
            continue;
        printf("  %02d  %02x:%02x:%02x:%02x:%02x:%02x %s\r\n", idx,
            entry_address[0],entry_address[1],entry_address[2],entry_address[3],
            entry_address[4],entry_address[5], addr_type_str[entry_address_type]);
        ++count;
    }

    printf("\r\ntotal of %d bonded entries of %d maximum entries\r\n", count, max_count);
}

void rppicomidi::BLE_MIDI_Manager::delete_le_bonding_info(int idx)
{
    auto context = cyw43_arch_async_context();
    async_context_acquire_lock_blocking(context);
    int max_count = le_device_db_max_count();
    if (idx >= max_count) {
        printf("invalid device index %d\r\n", idx);
        async_context_release_lock(context);
        return;
    }
    bd_addr_t entry_address;
    int entry_address_type = (int) BD_ADDR_TYPE_UNKNOWN;
    le_device_db_info(idx, &entry_address_type, entry_address, NULL);
    async_context_release_lock(context);
    if (entry_address_type <= (int)BD_ADDR_TYPE_LE_RANDOM_IDENTITY) {
        printf("deleting entry=%d addr=%02x:%02x:%02x:%02x:%02x:%02x\r\n",
            idx, entry_address[0],entry_address[1],entry_address[2],entry_address[3],
            entry_address[4],entry_address[5]);
        async_context_acquire_lock_blocking(context);
        gap_delete_bonding(static_cast<bd_addr_type_t>(entry_address_type), entry_address);
        async_context_release_lock(context);
    }
    else {
        printf("invalid device index %d\r\n", idx);
    }
}

void rppicomidi::BLE_MIDI_Manager::scan_begin()
{    
    if (!is_client) {
        init(this, true);
    }
    auto context = cyw43_arch_async_context();
    async_context_acquire_lock_blocking(context);
    ble_midi_client_scan_begin();
    async_context_release_lock(context);
    is_scan_mode = true;
    is_client = true;
    initialized = true;
}

bool rppicomidi::BLE_MIDI_Manager::reconnect()
{
    auto context = cyw43_arch_async_context();
    async_context_acquire_lock_blocking(context);
    bool result = ble_midi_client_request_connect(0);
    async_context_release_lock(context);
    return result;
}

bool rppicomidi::BLE_MIDI_Manager::get_keep_client_connected()
{
    return ble_midi_client_get_keep_connected();
}

void rppicomidi::BLE_MIDI_Manager::set_keep_client_connected(bool keep_client_connected_)
{
    auto context = cyw43_arch_async_context();
    async_context_acquire_lock_blocking(context);
    ble_midi_client_set_keep_connected(keep_client_connected_);
    async_context_release_lock(context);
}
#endif // ifdef RPPICOMIDI_PICO_W
