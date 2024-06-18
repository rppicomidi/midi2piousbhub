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
rppicomidi::BLE_MIDI_Manager::BLE_MIDI_Manager(const char* local_name, size_t local_name_len) : con_handle{HCI_CON_HANDLE_INVALID}
{
    // The maximum local name length is 13 bytes because the maximum scan response is 15 bytes.
    if (local_name_len > 13)
        local_name_len = 13;
    scan_resp_data[0] = local_name_len+1;
    scan_resp_data[1] = BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME;
    memcpy(scan_resp_data+2, local_name, local_name_len);
    scan_resp_data_len = local_name_len+2;
}

void rppicomidi::BLE_MIDI_Manager::packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    UNUSED(size);
    UNUSED(channel);
    bd_addr_t local_addr;
    uint8_t event_type;
    bd_addr_t addr;
    bd_addr_type_t addr_type;
    uint8_t status;
    // setup advertisements
    uint16_t adv_int_min = 800;
    uint16_t adv_int_max = 800;
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    switch(packet_type) {
        case HCI_EVENT_PACKET:
            event_type = hci_event_packet_get_type(packet);
            switch(event_type){
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) {
                        return;
                    }
                    gap_local_bd_addr(local_addr);
                    printf("BTstack up and running on %s.\n", bd_addr_to_str(local_addr));

                    memset(null_addr, 0, 6);
                    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
                    assert(adv_data_len <= 31); // ble limitation
                    gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
                    assert(scan_resp_data_len <= 31); // ble limitation
                    gap_scan_response_set_data(scan_resp_data_len, (uint8_t*) scan_resp_data);
                    gap_advertisements_enable(1);

                    break;
                case HCI_EVENT_DISCONNECTION_COMPLETE:
                    printf("ble-midi2usbhost: HCI_EVENT_DISCONNECTION_COMPLETE event\r\n");
                    break;
                case HCI_EVENT_GATTSERVICE_META:
                    switch(hci_event_gattservice_meta_get_subevent_code(packet)) {
                        case GATTSERVICE_SUBEVENT_SPP_SERVICE_CONNECTED:
                            con_handle = gattservice_subevent_spp_service_connected_get_con_handle(packet);
                            break;
                        case GATTSERVICE_SUBEVENT_SPP_SERVICE_DISCONNECTED:
                            printf("ble-midi2usbhost: GATTSERVICE_SUBEVENT_SPP_SERVICE_DISCONNECTED event\r\n");
                            con_handle = HCI_CON_HANDLE_INVALID;
                            break;
                        default:
                            break;
                    }
                    break;
                case SM_EVENT_JUST_WORKS_REQUEST:
                    printf("ble-midi2usbhost: Just Works requested\n");
                    sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
                    break;
                case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
                    printf("ble-midi2usbhost: Confirming numeric comparison: %" PRIu32 "\n", sm_event_numeric_comparison_request_get_passkey(packet));
                    sm_numeric_comparison_confirm(sm_event_passkey_display_number_get_handle(packet));
                    break;
                case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
                    printf("ble-midi2usbhost: Display Passkey: %" PRIu32 "\n", sm_event_passkey_display_number_get_passkey(packet));
                    break;
                case SM_EVENT_IDENTITY_CREATED:
                    sm_event_identity_created_get_identity_address(packet, addr);
                    printf("ble-midi2usbhost: Identity created: type %u address %s\n", sm_event_identity_created_get_identity_addr_type(packet), bd_addr_to_str(addr));
                    break;
                case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
                    sm_event_identity_resolving_succeeded_get_identity_address(packet, addr);
                    printf("ble-midi2usbhost: Identity resolved: type %u address %s\n", sm_event_identity_resolving_succeeded_get_identity_addr_type(packet), bd_addr_to_str(addr));
                    break;
                case SM_EVENT_IDENTITY_RESOLVING_FAILED:
                    sm_event_identity_created_get_address(packet, addr);
                    printf("ble-midi2usbhost: Identity resolving failed\n");
                    break;
                case SM_EVENT_PAIRING_STARTED:
                    printf("Pairing started\n");
                    break;
                case SM_EVENT_PAIRING_COMPLETE:
                    switch (sm_event_pairing_complete_get_status(packet)){
                        case ERROR_CODE_SUCCESS:
                            printf("ble-midi2usbhost: Pairing complete, success\n");
                            break;
                        case ERROR_CODE_CONNECTION_TIMEOUT:
                            printf("ble-midi2usbhost: Pairing failed, timeout\n");
                            break;
                        case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                            printf("ble-midi2usbhost: Pairing failed, disconnected\n");
                            break;
                        case ERROR_CODE_AUTHENTICATION_FAILURE:
                            printf("ble-midi2usbhost: Pairing failed, authentication failure with reason = %u\n", sm_event_pairing_complete_get_reason(packet));
                            break;
                        default:
                            break;
                    }
                    break;
                case SM_EVENT_REENCRYPTION_STARTED:
                    sm_event_reencryption_complete_get_address(packet, addr);
                    printf("ble-midi2usbhost: Bonding information exists for addr type %u, identity addr %s -> re-encryption started\n",
                        sm_event_reencryption_started_get_addr_type(packet), bd_addr_to_str(addr));
                    break;
                case SM_EVENT_REENCRYPTION_COMPLETE:
                    switch (sm_event_reencryption_complete_get_status(packet)){
                        case ERROR_CODE_SUCCESS:
                            printf("ble-midi2usbhost: Re-encryption complete, success\n");
                            break;
                        case ERROR_CODE_CONNECTION_TIMEOUT:
                            printf("ble-midi2usbhost: Re-encryption failed, timeout\n");
                            break;
                        case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                            printf("ble-midi2usbhost: Re-encryption failed, disconnected\n");
                            break;
                        case ERROR_CODE_PIN_OR_KEY_MISSING:
                            printf("ble-midi2usbhost: Re-encryption failed, bonding information missing\n\n");
                            printf("Assuming remote lost bonding information\n");
                            printf("Deleting local bonding information to allow for new pairing...\n");
                            sm_event_reencryption_complete_get_address(packet, addr);
                            addr_type = static_cast<bd_addr_type_t>(sm_event_reencryption_started_get_addr_type(packet));
                            gap_delete_bonding(addr_type, addr);
                            break;
                        default:
                            break;
                    }
                    break;
                case GATT_EVENT_QUERY_COMPLETE:
                    status = gatt_event_query_complete_get_att_status(packet);
                    switch (status){
                        case ATT_ERROR_INSUFFICIENT_ENCRYPTION:
                            printf("ble-midi2usbhost: GATT Query failed, Insufficient Encryption\n");
                            break;
                        case ATT_ERROR_INSUFFICIENT_AUTHENTICATION:
                            printf("ble-midi2usbhost: GATT Query failed, Insufficient Authentication\n");
                            break;
                        case ATT_ERROR_BONDING_INFORMATION_MISSING:
                            printf("ble-midi2usbhost: GATT Query failed, Bonding Information Missing\n");
                            break;
                        case ATT_ERROR_SUCCESS:
                            printf("ble-midi2usbhost: GATT Query successful\n");
                            break;
                        default:
                            printf("ble-midi2usbhost: GATT Query failed, status 0x%02x\n", gatt_event_query_complete_get_att_status(packet));
                            break;
                    }
                    break;
                default:
                    break;
            } // event_type
            break;
        default:
            break;
    } // HCI_PACKET
}


void rppicomidi::BLE_MIDI_Manager::static_packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    instance->packet_handler(packet_type, channel, packet, size);
}

bool rppicomidi::BLE_MIDI_Manager::init(BLE_MIDI_Manager* instance_)
{
    instance = instance_;
    con_handle = HCI_CON_HANDLE_INVALID;
        // initialize CYW43 driver architecture (will enable BT if/because CYW43_ENABLE_BLUETOOTH == 1)
    if (cyw43_arch_init()) {
        printf("ble-midi2usbhost: failed to initialize cyw43_arch\n");
        return false;
    }
    l2cap_init();

    sm_init();

    att_server_init(profile_data, NULL, NULL);
    // just works, legacy pairing, with bonding
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_BONDING);
    // register for SM events
    sm_event_callback_registration.callback = &static_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);
    midi_service_stream_init(sm_event_callback_registration.callback);

    // turn on bluetooth
    hci_power_control(HCI_POWER_ON);
    return true;
}

uint8_t rppicomidi::BLE_MIDI_Manager::stream_read(uint8_t* bytes, uint8_t max_bytes)
{
    uint16_t timestamp;
    if (is_connected())
        return midi_service_stream_read(con_handle, max_bytes, bytes, &timestamp);
    return 0;
}

uint8_t rppicomidi::BLE_MIDI_Manager::stream_write(const uint8_t* bytes, uint8_t num_bytes)
{
    if (is_connected()) {
        return midi_service_stream_write(con_handle, num_bytes, bytes);
    }
    return 0;
}

#endif
