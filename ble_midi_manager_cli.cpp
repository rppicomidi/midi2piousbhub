/**
 * MIT License
 *
 * Copyright (c) 2024 rppicomidi
 *
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
#ifdef RPPICOMIDI_PICO_W
#include "ble_midi_manager_cli.h"
#include "pico/assert.h"
rppicomidi::BLE_MIDI_Manager_cli::BLE_MIDI_Manager_cli(EmbeddedCli* cli_, BLE_MIDI_Manager* blem_) : cli{cli_}
{
    volatile bool result = embeddedCliAddBinding(cli, {
        "btmidi-disconnect",
        "disconnect an active connection",
        false,
        blem_,
        static_disconnect
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "btmidi-list",
        "list bonded devices",
        false,
        blem_,
        static_list_bonded_devices
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "btmidi-rm",
        "remove the device specified by the index from the bonded database",
        false,
        blem_,
        static_delete_bonded_device_by_index
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "btmidi-scan-begin",
        "start a scan for MIDI devices",
        false,
        blem_,
        static_scan_begin
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "btmidi-scan-list",
        "list all found MIDI devices",
        false,
        blem_,
        static_scan_list
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "btmidi-scan-end",
        "end scan for MIDI devices and list all found",
        false,
        blem_,
        static_scan_end
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "btmidi-client-connect",
        "blmidi-client-connect <0 for last connected or index number from scan list>",
        true,
        blem_,
        static_client_connect
    });
    assert(result);

    (void)result;
}

void rppicomidi::BLE_MIDI_Manager_cli::static_disconnect(EmbeddedCli *, char *, void *context)
{
    auto blem = reinterpret_cast<BLE_MIDI_Manager*>(context);
    if (blem->is_connected())
        blem->disconnect();
    else
        printf("Already disconnected\r\n");
}

void rppicomidi::BLE_MIDI_Manager_cli::static_list_bonded_devices(EmbeddedCli *, char *, void *context)
{
    auto blem = reinterpret_cast<BLE_MIDI_Manager*>(context);
    blem->list_le_device_info();
}

void rppicomidi::BLE_MIDI_Manager_cli::static_delete_bonded_device_by_index(EmbeddedCli *, char *args, void *context)
{
    if (embeddedCliGetTokenCount(args) != 1) {
        printf("usage: btmidi-rm <entry number>\r\n");
        return;
    }
    int idx = atoi(embeddedCliGetToken(args, 1));
    auto blem = reinterpret_cast<BLE_MIDI_Manager*>(context);
    blem->delete_le_bonding_info(idx);
}

void rppicomidi::BLE_MIDI_Manager_cli::static_scan_begin(EmbeddedCli *, char *, void *context)
{
    auto blem = reinterpret_cast<BLE_MIDI_Manager*>(context);
    blem->scan_begin();
}

void rppicomidi::BLE_MIDI_Manager_cli::static_scan_end(EmbeddedCli *, char *, void *context)
{
    auto blem = reinterpret_cast<BLE_MIDI_Manager*>(context);
    blem->scan_end();
    blem->dump_midi_peripherals();
}

void rppicomidi::BLE_MIDI_Manager_cli::static_scan_list(EmbeddedCli *, char *, void *context)
{
    auto blem = reinterpret_cast<BLE_MIDI_Manager*>(context);
    blem->dump_midi_peripherals();
}

void rppicomidi::BLE_MIDI_Manager_cli::static_client_connect(EmbeddedCli *, char *args, void *context)
{
    auto blem = reinterpret_cast<BLE_MIDI_Manager*>(context);
    if (embeddedCliGetTokenCount(args) != 1) {
        printf("blmidi-client-connect <0 for last connected or index number from scan list>\r\n");
        return;
    }
    uint8_t idx = std::atoi(embeddedCliGetToken(args, 1));
    if (!blem->client_request_connect(idx)) {
        printf("could not connect to index==%u\r\n", idx);
    }
}
#endif