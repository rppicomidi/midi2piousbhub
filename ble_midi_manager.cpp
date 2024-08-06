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

rppicomidi::BLE_MIDI_Manager::BLE_MIDI_Manager(const char* local_name) : con_handle{HCI_CON_HANDLE_INVALID},
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
                    printf("blem: HCI_EVENT_DISCONNECTION_COMPLETE event\r\n");
                    con_handle = HCI_CON_HANDLE_INVALID;
#if 0
                    if (is_client && next_connect_bd_addr_type <= BD_ADDR_TYPE_LE_RANDOM_IDENTITY) {
                        // honor pending connection request
                        gap_connect(next_connect_bd_addr, next_connect_bd_addr_type);
                    }
#endif
                    break;
                case HCI_EVENT_GATTSERVICE_META:
                    switch(hci_event_gattservice_meta_get_subevent_code(packet)) {
                        case GATTSERVICE_SUBEVENT_SPP_SERVICE_CONNECTED:
                            con_handle = gattservice_subevent_spp_service_connected_get_con_handle(packet);
                            printf("blem: GATTSERVICE_SUBEVENT_SPP_SERVICE_CONNECTED event handle = %u\r\n", con_handle);
                            break;
                        case GATTSERVICE_SUBEVENT_SPP_SERVICE_DISCONNECTED:
                            printf("blem: GATTSERVICE_SUBEVENT_SPP_SERVICE_DISCONNECTED event\r\n");
                            con_handle = HCI_CON_HANDLE_INVALID;
                            break;
                        default:
                            break;
                    }
                    break;
                case SM_EVENT_JUST_WORKS_REQUEST:
                    printf("blem: Just Works requested\n");
                    sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
                    break;
                case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
                    printf("blem: Confirming numeric comparison: %" PRIu32 "\n", sm_event_numeric_comparison_request_get_passkey(packet));
                    sm_numeric_comparison_confirm(sm_event_passkey_display_number_get_handle(packet));
                    break;
                case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
                    printf("blem: Display Passkey: %" PRIu32 "\n", sm_event_passkey_display_number_get_passkey(packet));
                    break;
                case SM_EVENT_IDENTITY_CREATED:
                    sm_event_identity_created_get_identity_address(packet, addr);
                    printf("blem: Identity created: type %u address %s\n", sm_event_identity_created_get_identity_addr_type(packet), bd_addr_to_str(addr));
                    break;
                case SM_EVENT_IDENTITY_RESOLVING_SUCCEEDED:
                    sm_event_identity_resolving_succeeded_get_identity_address(packet, addr);
                    printf("blem: Identity resolved: type %u address %s\n", sm_event_identity_resolving_succeeded_get_identity_addr_type(packet), bd_addr_to_str(addr));
                    break;
                case SM_EVENT_IDENTITY_RESOLVING_FAILED:
                    sm_event_identity_created_get_address(packet, addr);
                    printf("blem: Identity resolving failed\n");
                    break;
                case SM_EVENT_PAIRING_STARTED:
                    printf("blem: Pairing started\n");
                    break;
                case SM_EVENT_PAIRING_COMPLETE:
                    switch (sm_event_pairing_complete_get_status(packet)){
                        case ERROR_CODE_SUCCESS:
                            printf("blem: Pairing complete, success\n");
                            break;
                        case ERROR_CODE_CONNECTION_TIMEOUT:
                            printf("blem: Pairing failed, timeout\n");
                            break;
                        case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                            printf("blem: Pairing failed, disconnected\n");
                            break;
                        case ERROR_CODE_AUTHENTICATION_FAILURE:
                            printf("blem: Pairing failed, authentication failure with reason = %u\n", sm_event_pairing_complete_get_reason(packet));
                            break;
                        default:
                            break;
                    }
                    break;
                case SM_EVENT_REENCRYPTION_STARTED:
                    sm_event_reencryption_complete_get_address(packet, addr);
                    printf("blem: Bonding information exists for addr type %u, identity addr %s -> re-encryption started\n",
                        sm_event_reencryption_started_get_addr_type(packet), bd_addr_to_str(addr));
                    break;
                case SM_EVENT_REENCRYPTION_COMPLETE:
                    switch (sm_event_reencryption_complete_get_status(packet)){
                        case ERROR_CODE_SUCCESS:
                            printf("blem: Re-encryption complete, success\n");
                            break;
                        case ERROR_CODE_CONNECTION_TIMEOUT:
                            printf("blem: Re-encryption failed, timeout\n");
                            break;
                        case ERROR_CODE_REMOTE_USER_TERMINATED_CONNECTION:
                            printf("blem: Re-encryption failed, disconnected\n");
                            break;
                        case ERROR_CODE_PIN_OR_KEY_MISSING:
                            printf("blem: Re-encryption failed, bonding information missing\n\n");
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
                            printf("blem: GATT Query failed, Insufficient Encryption\n");
                            break;
                        case ATT_ERROR_INSUFFICIENT_AUTHENTICATION:
                            printf("blem: GATT Query failed, Insufficient Authentication\n");
                            break;
                        case ATT_ERROR_BONDING_INFORMATION_MISSING:
                            printf("blem: GATT Query failed, Bonding Information Missing\n");
                            break;
                        case ATT_ERROR_SUCCESS:
                            printf("blem: GATT Query successful\n");
                            break;
                        default:
                            printf("blem: GATT Query failed, status 0x%02x\n", gatt_event_query_complete_get_att_status(packet));
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

bool rppicomidi::BLE_MIDI_Manager::init(BLE_MIDI_Manager* instance_, bool is_client_)
{
    if (initialized) {
        deinit(is_client);
        initialized = false;
    }
    is_client = is_client_;
    instance = instance_;
    if (is_client) {
        const char client_name[]="Pico W MIDI USB BLE Hub";
        ble_midi_client_init(&static_packet_handler, client_name, strlen(client_name));
        return true;
    }

    con_handle = HCI_CON_HANDLE_INVALID;
    printf("con_handle initialized\r\n");
    // initialize CYW43 driver architecture (will enable BT if/because CYW43_ENABLE_BLUETOOTH == 1)
    if (cyw43_arch_init()) {
        printf("ble-midi2usbhost: failed to initialize cyw43_arch\n");
        return false;
    }
    l2cap_init();

    sm_init();

    // just works, legacy pairing, with bonding
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_BONDING);
    // register for SM events
    sm_event_callback_registration.callback = &static_packet_handler;
    sm_add_event_handler(&sm_event_callback_registration);
    if (is_client) {

    }
    else {
        att_server_init(profile_data, NULL, NULL);
        midi_service_stream_init(sm_event_callback_registration.callback);
    }

    // turn on bluetooth
    hci_power_control(HCI_POWER_ON);
    initialized = true;
    return true;
}

void rppicomidi::BLE_MIDI_Manager::deinit(bool is_client)
{
    if (!initialized)
        return; // nothing to do

    hci_power_control(HCI_POWER_OFF);
    sm_remove_event_handler(&sm_event_callback_registration);
    if (is_client) {
        ble_midi_client_scan_end();
    }
    else {
        att_server_deinit();
    }

    sm_deinit();
    l2cap_deinit();
    cyw43_arch_deinit();
    initialized = false;
    con_handle = HCI_CON_HANDLE_INVALID;
    printf("con_handle deinit\r\n");
}

uint8_t rppicomidi::BLE_MIDI_Manager::stream_read(uint8_t* bytes, uint8_t max_bytes)
{
    uint16_t timestamp;
    if (is_connected()) {
        if (is_client)
            return ble_midi_client_stream_read(con_handle, max_bytes, bytes, &timestamp);
        else
            return midi_service_stream_read(con_handle, max_bytes, bytes, &timestamp);
    }
    return 0;
}

uint8_t rppicomidi::BLE_MIDI_Manager::stream_write(const uint8_t* bytes, uint8_t num_bytes)
{
    if (is_connected()) {
        if (is_client) {
            return ble_midi_client_stream_write(con_handle, num_bytes, bytes);
        }
        else {
            return midi_service_stream_write(con_handle, num_bytes, bytes);
        }
    }
    return 0;
}

void rppicomidi::BLE_MIDI_Manager::disconnect()
{
    //next_connect_bd_addr_type = BD_ADDR_TYPE_UNKNOWN;
    if (is_connected()) {
        if (is_client) {
            ble_midi_client_request_disconnect(con_handle);
        }
        else {
            gap_disconnect(con_handle);
        }
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
    for (int idx=0; idx < max_count; idx++) {
        int entry_address_type = (int) BD_ADDR_TYPE_UNKNOWN;
        le_device_db_info(idx, &entry_address_type, entry_address, NULL);
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
    int max_count = le_device_db_max_count();
    if (idx >= max_count) {
        printf("invalid device index %d\r\n", idx);
        return;
    }
    bd_addr_t entry_address;
    int entry_address_type = (int) BD_ADDR_TYPE_UNKNOWN;
    le_device_db_info(idx, &entry_address_type, entry_address, NULL);
    if (entry_address_type <= (int)BD_ADDR_TYPE_LE_RANDOM_IDENTITY) {
        printf("deleting entry=%d addr=%02x:%02x:%02x:%02x:%02x:%02x\r\n",
            idx, entry_address[0],entry_address[1],entry_address[2],entry_address[3],
            entry_address[4],entry_address[5]);
        gap_delete_bonding(static_cast<bd_addr_type_t>(entry_address_type), entry_address);
    }
    else {
        printf("invalid device index %d\r\n", idx);
        return;
    }
}

void rppicomidi::BLE_MIDI_Manager::scan_begin()
{    
    if (!is_client) {
        deinit(is_client);
        const char client_name[]="Pico W MIDI USB BLE Hub";
        ble_midi_client_init(&static_packet_handler, client_name, strlen(client_name));
    }
    ble_midi_client_scan_begin();
    is_scan_mode = true;
    is_client = true;
    initialized = true;
}

#if 0
void rppicomidi::BLE_MIDI_Manager::static_handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    instance->handle_gatt_client_event(packet_type, channel, packet, size);
}

static void printUUID(uint8_t * uuid128, uint16_t uuid16){
    if (uuid16){
        printf("%04x",uuid16);
    } else {
        printf("%s", uuid128_to_str(uuid128));
    }
}
static void dump_characteristic(gatt_client_characteristic_t * characteristic){
    printf("    * characteristic: [0x%04x-0x%04x-0x%04x], properties 0x%02x, uuid ",
                            characteristic->start_handle, characteristic->value_handle, characteristic->end_handle, characteristic->properties);
    printUUID(characteristic->uuid128, characteristic->uuid16);
    printf("\n");
}

static void dump_service(gatt_client_service_t * service){
    printf("    * service: [0x%04x-0x%04x], uuid ", service->start_group_handle, service->end_group_handle);
    printUUID(service->uuid128, service->uuid16);
    printf("\n");
}

static int service_count = 0;
static int service_index = 0;
static gatt_client_service_t services[40];
void rppicomidi::BLE_MIDI_Manager::handle_gatt_client_event(uint8_t , uint16_t , uint8_t *packet, uint16_t )
{
    gatt_client_service_t service;
    gatt_client_characteristic_t characteristic;
    switch(hci_event_packet_get_type(packet)){
        case GATT_EVENT_SERVICE_QUERY_RESULT:
            gatt_event_service_query_result_get_service(packet, &service);
            dump_service(&service);
            services[service_count++] = service;
            break;
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
            gatt_event_characteristic_query_result_get_characteristic(packet, &characteristic);
            dump_characteristic(&characteristic);
            break;
        case GATT_EVENT_QUERY_COMPLETE:
            // GATT_EVENT_QUERY_COMPLETE of search characteristics
            if (service_index < service_count) {
                service = services[service_index++];
                printf("\nCHARACTERISTIC for SERVICE %s, [0x%04x-0x%04x]\n",
                    uuid128_to_str(service.uuid128), service.start_group_handle, service.end_group_handle);
                gatt_client_discover_characteristics_for_service(static_handle_gatt_client_event, con_handle, &service);
                break;
            }
            service_index = 0;
            break;
        default:
            printf("unhandled packet type %u", hci_event_packet_get_type(packet));
            break;
    }
}

bool rppicomidi::BLE_MIDI_Manager::get_local_name_from_ad_data(uint8_t ad_len, const uint8_t* ad_data, Advertised_MIDI_Peripheral& peripheral)
{
    bool success = false;
    ad_context_t context;
    if (peripheral.type == BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME)
        return false; // don't copy over the complete local name
    
    ad_iterator_init(&context, ad_len, ad_data);
    while ( ad_iterator_has_more(&context) ){
        uint8_t data_type = ad_iterator_get_data_type(&context);
        uint8_t data_len  = ad_iterator_get_data_len(&context);
        const uint8_t * data = ad_iterator_get_data(&context);
        if (data_len >= sizeof(peripheral.name))
            data_len = sizeof(peripheral.name) - 1;
        switch (data_type){
            case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
            case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:                
                memcpy(peripheral.name, data, data_len);
                peripheral.name[data_len] = '\0';
                peripheral.type = data_type;
                success = true;
                break;
            default:
                break;
        }  
        ad_iterator_next(&context);
    }
    return success;
}

void rppicomidi::BLE_MIDI_Manager::handle_hci_event(uint8_t packet_type, uint16_t, uint8_t *packet, uint16_t)
{
    if (packet_type != HCI_EVENT_PACKET) return;
    
    uint8_t event = hci_event_packet_get_type(packet);
    const uint8_t* ad_data;
    uint8_t ad_data_len;
    bd_addr_t bdaddr;
    const uint8_t midi_profile_uuid128[] = { 0x03, 0xB8, 0x0E, 0x5A, 0xED, 0xE8, 0x4B, 0x33, 0xA7, 0x51, 0x6C, 0xE3, 0x4E, 0xC4, 0xC7, 0x00 };
    uint64_t idx; // The BD_ADDR converted to a 64-bit index into the map
    bool mapped; // true if the advertising report is from a BD_ADDR already mapped
    int err;
    Advertised_MIDI_Peripheral peripheral;
    switch (event) {
        case BTSTACK_EVENT_STATE:
            // BTstack activated, get started
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING)
                break;
            if (is_client) {
                if (is_scan_mode) {
                    midi_peripherals.clear();
                    printf("BTstack activated, start active scanning\n");
                    gap_set_scan_params(1,0x0030, 0x0030,0);
                    gap_start_scan();
                }
                else {
                    if (next_connect_bd_addr_type != BD_ADDR_TYPE_UNKNOWN) {
                        gap_connect(next_connect_bd_addr, next_connect_bd_addr_type);
                    }
                }
            }
            else {

            }
            break;
        case GAP_EVENT_ADVERTISING_REPORT:
            ad_data_len = gap_event_advertising_report_get_data_length(packet);
            ad_data = gap_event_advertising_report_get_data(packet);
            gap_event_advertising_report_get_address(packet, bdaddr);
            idx = bdaddr2uint64(bdaddr);
            mapped = midi_peripherals.find(idx) != midi_peripherals.end();
            if (ad_data_contains_uuid128(ad_data_len, ad_data, midi_profile_uuid128) || mapped) {
                if (mapped) {
                    if (midi_peripherals[idx].type == BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME) {
                        midi_peripherals[idx].timeout = scan_remove_timeout;
                        break; // no need to repeat ourselves.
                    }
                    peripheral = midi_peripherals[idx];
                }
                else {
                    // initialize the map with the BD_ADDR as the name and record the address type
                    peripheral.type = 0;
                    // Need the address type to connect
                    peripheral.addr_type = gap_event_advertising_report_get_address_type(packet);
                    memcpy(peripheral.name, bd_addr_to_str(bdaddr), strlen(bd_addr_to_str(bdaddr)));
                    midi_peripherals[idx] = peripheral;
                }
                midi_peripherals[idx].timeout = scan_remove_timeout; // do not time out this entry
                if (get_local_name_from_ad_data(ad_data_len, ad_data, peripheral)) {
                    midi_peripherals[idx] = peripheral;
                }
                //printf("    * adv. event: addr=%s[%s]\r\n", bd_addr_to_str(bdaddr), peripheral.name);
            }
            break;
        case HCI_EVENT_LE_META:
            // wait for connection complete
            if (hci_event_le_meta_get_subevent_code(packet) !=  HCI_SUBEVENT_LE_CONNECTION_COMPLETE) {
                break;
            }
            printf("\nCONNECTED\n");
            con_handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
            // print connection parameters (without using float operations)
            conn_interval = hci_subevent_le_connection_complete_get_conn_interval(packet);
            printf("Connection Interval: %u.%02u ms\n", conn_interval * 125 / 100, 25 * (conn_interval & 3));
            printf("Connection Latency: %u\n", hci_subevent_le_connection_complete_get_conn_latency(packet));
            // initialize gatt client context with handle, and add it to the list of active clients
            // query primary services
            printf("Search for MIDI service.\n");
            err = gatt_client_discover_primary_services_by_uuid128(static_handle_gatt_client_event, con_handle, midi_profile_uuid128);
            if (err != ERROR_CODE_SUCCESS)
                printf("Error(%d): Failed to discover primary services by uuid128\r\n", err);

            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("\nDISCONNECTED\n");
            service_count = 0;
            service_index = 0;
            // disconnected from previous and connected to new
            if (is_client && !is_scan_mode && BD_ADDR_TYPE_UNKNOWN != next_connect_bd_addr_type)
                gap_connect(next_connect_bd_addr, next_connect_bd_addr_type);
            break;
        default:
            break;
    }
}

void rppicomidi::BLE_MIDI_Manager::static_handle_hci_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size)
{
    instance->handle_hci_event(packet_type, channel, packet, size);
}

void rppicomidi::BLE_MIDI_Manager::static_scan_timer_cb(btstack_timer_source_t* timer_)
{
    static bool led_on = true;

    led_on = !led_on;
    auto mp = reinterpret_cast<BLE_MIDI_Manager*>(timer_->context);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
    // update midi_peripherals timeouts
    for (auto &it: mp->midi_peripherals) {
        --it.second.timeout;
    }
    // Remove items with expired timeouts
    for (auto it = mp->midi_peripherals.cbegin(); it != mp->midi_peripherals.cend(); /* no increment*/)
    {
        if (it->second.timeout <= 0) {
            it = mp->midi_peripherals.erase(it);
        }
        else {
            ++it;
        }
    }
    // Restart timer
    btstack_run_loop_set_timer(timer_, scan_blink_timeout_ms);
    btstack_run_loop_add_timer(timer_);
}

void rppicomidi::BLE_MIDI_Manager::enter_client_mode()
{
    // TODO change function to know whether to connect to a server
    // once client mode comes up.
    if (initialized)
        deinit(is_client);
    is_client = true;
    if (cyw43_arch_init() != 0) {
        printf("error initializing CYW43_ARCH\r\n");
        return;
    }

    // Initialize L2CAP and register HCI event handler
    l2cap_init();

    // Initialize GATT client 
    gatt_client_init();

    // Optionally, Setup security manager
    sm_init();
    sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
    sm_set_authentication_requirements(SM_AUTHREQ_SECURE_CONNECTION | SM_AUTHREQ_BONDING);
    // register for HCI events
    hci_event_callback_registration.callback = &static_handle_hci_event;
    hci_add_event_handler(&hci_event_callback_registration);
}

void rppicomidi::BLE_MIDI_Manager::scan_end()
{
    if (!is_scan_mode)
        return;
    gap_stop_scan();
    btstack_run_loop_remove_timer(&scan_timer);
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, false);
    is_scan_mode = false;
}

void rppicomidi::BLE_MIDI_Manager::dump_midi_peripherals()
{
    uint16_t idx = 1;
    printf("Index Bluetooth Address Name\r\n");
    for (auto it: midi_peripherals) {
        uint8_t bdaddr[6];
        uint64_2bdaddr(it.first, bdaddr);

        printf("%-5u %s %s\r\n", idx, bd_addr_to_str(bdaddr), it.second.name );
    }
}

bool rppicomidi::BLE_MIDI_Manager::client_request_connect(uint8_t idx)
{
    if (idx == 0) {
        // TODO: retrieve the last connected device and connect
        // if it exists. For now, just return false
        return false;
    }
    int jdx = 1;
    for (auto it: midi_peripherals) {
        if (jdx == idx) {
            uint64_2bdaddr(it.first, next_connect_bd_addr);
            next_connect_bd_addr_type = static_cast<bd_addr_type_t>(it.second.addr_type);
            if (is_scan_mode) {
                scan_end();
                gap_connect(next_connect_bd_addr, next_connect_bd_addr_type);
            }
            else if (is_client) {
                if (is_connected()) {
                    gap_disconnect(con_handle);
                }
                else {
                    gap_connect(next_connect_bd_addr, next_connect_bd_addr_type);
                }
            }
            else {
                enter_client_mode();
                hci_power_control(HCI_POWER_ON);
                initialized = true;
            }
            return true;
        }
    }
    return false;
}
#endif
#endif
