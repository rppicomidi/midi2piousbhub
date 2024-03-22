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
namespace rppicomidi
{
class Preset_manager_cli
{
public:
    Preset_manager_cli() = delete;
    Preset_manager_cli(Preset_manager_cli const&) = delete;
    void operator=(Preset_manager_cli const&) = delete;
    ~Preset_manager_cli()=default;
    Preset_manager_cli(EmbeddedCli* cli, Preset_manager* pm);
    static uint16_t get_num_commands() { return 4; }
private:
    // CLI Commands
    static void static_save_current_preset(EmbeddedCli* cli, char* args, void*);
    static void static_load_preset(EmbeddedCli* cli, char* args, void*);
    static void static_fatfs_backup(EmbeddedCli *cli, char *args, void *context);
    static void static_fatfs_restore(EmbeddedCli* cli, char* args, void*);

    // data
    EmbeddedCli* cli;
};
}