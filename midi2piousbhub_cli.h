/**
 * MIT License
 *
 * Copyright (c) 2022 rppicomidi
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
#include "preset_manager.h"
#include "tusb.h"
#ifdef RPPICOMIDI_PICO_W
#include "ble_midi_manager_cli.h"
#endif
namespace rppicomidi
{
class Midi2PioUsbhub_cli
{
public:
    Midi2PioUsbhub_cli() = delete;
    Midi2PioUsbhub_cli(Midi2PioUsbhub_cli const&) = delete;
    void operator=(Midi2PioUsbhub_cli const&) = delete;
    ~Midi2PioUsbhub_cli()=default;
#ifdef RPPICOMIDI_PICO_W
    Midi2PioUsbhub_cli(Preset_manager* pm, BLE_MIDI_Manager* blem);
#else
    Midi2PioUsbhub_cli(Preset_manager* pm);
#endif
    void task();
    void printWelcome();
    static uint16_t get_num_commands() { return 6; }
private:
    /*
     * The following 3 functions are required by the EmbeddedCli library
     */
    static void onCommand(const char *name, char *tokens)
    {
        printf("Received command: %s\r\n", name);

        for (int i = 0; i < embeddedCliGetTokenCount(tokens); ++i)
        {
            printf("Arg %d : %s\r\n", i, embeddedCliGetToken(tokens, i + 1));
        }
    }

    static void onCommandFn(EmbeddedCli *embeddedCli, CliCommand *command)
    {
        (void)embeddedCli;
        embeddedCliTokenizeArgs(command->args);
        onCommand(command->name == NULL ? "" : command->name, command->args);
    }

    static void writeCharFn(EmbeddedCli *embeddedCli, char c)
    {
        (void)embeddedCli;
        putchar(c);
    }

    // The following are CLI functions
    static void static_list(EmbeddedCli *, char *, void *);
    static void static_connect(EmbeddedCli *, char *, void *);
    static void static_disconnect(EmbeddedCli *, char *, void *);
    static void static_show(EmbeddedCli *, char *, void *);
    static void static_reset(EmbeddedCli *, char *, void *);
    static void static_rename(EmbeddedCli *, char *, void *);
    // data
    EmbeddedCli* cli;
};
}