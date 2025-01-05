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
#include "midi2piousbhub.h"
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
        "btmidi-client-scan-begin",
        "start a scan for MIDI devices",
        false,
        blem_,
        static_scan_begin
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "btmidi-client-scan-list",
        "list all found MIDI devices",
        false,
        blem_,
        static_scan_list
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "btmidi-client-scan-end",
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
    result = embeddedCliAddBinding(cli, {
        "btmidi-server-start",
        "leave client mode and enter server mode",
        true,
        blem_,
        static_start_server
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "btmidi-client-auto-connect",
        "blmidi-client-auto-connect <0 to disable , 1 to set auto-connect to current server>",
        true,
        blem,
        static_client_auto_connect
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

void rppicomidi::BLE_MIDI_Manager_cli::static_scan_begin(EmbeddedCli *, char *, void * context)
{
    auto blem = reinterpret_cast<BLE_MIDI_Manager*>(context);
    blem->scan_begin();
}

void rppicomidi::BLE_MIDI_Manager_cli::static_scan_end(EmbeddedCli *, char *, void *)
{
    //auto blem = reinterpret_cast<BLE_MIDI_Manager*>(context);
    //blem->scan_end();
    //blem->dump_midi_peripherals();
    ble_midi_client_scan_end();
    ble_midi_client_dump_midi_peripherals();
}

void rppicomidi::BLE_MIDI_Manager_cli::static_scan_list(EmbeddedCli *, char *, void *)
{
    //auto blem = reinterpret_cast<BLE_MIDI_Manager*>(context);
    //blem->dump_midi_peripherals();
    ble_midi_client_dump_midi_peripherals();
}

void rppicomidi::BLE_MIDI_Manager_cli::static_client_connect(EmbeddedCli *, char *args, void *context)
{
    auto blem = reinterpret_cast<BLE_MIDI_Manager*>(context);
    if (embeddedCliGetTokenCount(args) != 1) {
        printf("blmidi-client-connect <0 for last connected or index number from scan list>\r\n");
        return;
    }
    if (blem->is_server_mode())
        blem->init(blem, true);
    uint8_t idx = std::atoi(embeddedCliGetToken(args, 1));
    if (!ble_midi_client_request_connect(idx)) {
        printf("could not connect to index==%u\r\n", idx);
    }
}

void rppicomidi::BLE_MIDI_Manager_cli::static_client_auto_connect(EmbeddedCli *, char *args, void *context)
{
    auto blem = reinterpret_cast<BLE_MIDI_Manager*>(context);
    if (embeddedCliGetTokenCount(args) != 1) {
        printf("blmidi-client-auto-connect <0 to disable , 1 to set auto-connect to current server>\r\n");
        return;
    }
    uint8_t set = std::atoi(embeddedCliGetToken(args, 1));
    if (set > 1) {
        printf("invalid argument %u\r\n", set);
        return;
    }
    if (blem->is_connected()) {
      uint8_t addr[6];
      int result = blem->get_last_connected(addr);
      if (result == BD_ADDR_TYPE_UNKNOWN) {
          printf("couldn't get the address of the last device\r\n");
          return;
      }
      // Convert addr to a string of hex digits
      char addr_str[13];
      for (int i = 0; i < 6; i++) {
          sprintf(&addr_str[i*2], "%02x", addr[i]);
      }
      Midi2PioUsbhub::instance().set_auto_connect(std::string(addr_str));
        printf("TODO: implement static_client_auto_connect");

    } else {
        printf("You need to connect first\r\n");
    }
}

void rppicomidi::BLE_MIDI_Manager_cli::static_start_server(EmbeddedCli *, char *args, void *context)
{
     if (embeddedCliGetTokenCount(args) != 0) {
        printf("blmidi-start-server\r\n");
        return;
    }
    auto blem = reinterpret_cast<BLE_MIDI_Manager*>(context);
    if (blem->is_server_mode()) {
        printf("already in server mode\r\n");
    }
    blem->init(blem, false);
}
#endif