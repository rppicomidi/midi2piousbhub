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
#pragma once
#include <cstdint>
#include "embedded_cli.h"
#include "ble_midi_manager.h"
#include "tusb.h"
namespace rppicomidi
{
class BLE_MIDI_Manager_cli
{
public:
    BLE_MIDI_Manager_cli() = delete;
    BLE_MIDI_Manager_cli(BLE_MIDI_Manager_cli const&) = delete;
    void operator=(BLE_MIDI_Manager_cli const&) = delete;
    ~BLE_MIDI_Manager_cli()=default;
    BLE_MIDI_Manager_cli(EmbeddedCli* cli_, BLE_MIDI_Manager* blem_);
    static uint16_t get_num_commands() { return 11; }
private:
    // The following are CLI functions
    static void static_disconnect(EmbeddedCli *, char *, void *);
    static void static_list_bonded_devices(EmbeddedCli *, char *, void *);
    static void static_delete_bonded_device_by_index(EmbeddedCli *, char *, void *);
    static void static_scan_begin(EmbeddedCli *, char *, void *);
    static void static_scan_end(EmbeddedCli *, char *, void *);
    static void static_scan_list(EmbeddedCli *, char *, void *);
    static void static_client_connect(EmbeddedCli *, char *, void *);
    static void static_client_cancel_connect(EmbeddedCli *, char *, void *);
    static void static_client_keep_connected(EmbeddedCli *, char *, void *);
    static void static_start_server(EmbeddedCli *, char *, void *);
    static void static_get_state(EmbeddedCli *, char *, void *);
    // data
    EmbeddedCli* cli;
    BLE_MIDI_Manager* blem;
};
}