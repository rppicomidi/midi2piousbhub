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
    assert(embeddedCliAddBinding(cli, {
        "btmidi-disconnect",
        "disconnect an active connection",
        false,
        blem_,
        static_disconnect
    }));
}

void rppicomidi::BLE_MIDI_Manager_cli::static_disconnect(EmbeddedCli *, char *, void *context)
{
    auto blem = reinterpret_cast<BLE_MIDI_Manager*>(context);
    if (blem->is_connected())
        blem->disconnect();
    else
        printf("Already disconnected\r\n");
}
#endif