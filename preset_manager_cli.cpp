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
#ifdef NDEBUG
// Need to do this here for release builds or no CLI commands will be added
// All build variants except DEBUG define NDEBUG, which makes assert() macro generate
// no code at all, which prevents msc_demo_cli_init() from adding any CLI commands.
#undef NDEBUG
#endif

#include "preset_manager_cli.h"
#include <string>

rppicomidi::Preset_manager_cli::Preset_manager_cli(EmbeddedCli* cli, Preset_manager* pm)
{
    assert(embeddedCliAddBinding(cli, {
        "load",
        "load the current preset from the preset file name. usage: load <preset name>",
        true,
        pm,
        static_load_preset
    }));
    assert(embeddedCliAddBinding(cli, {
        "save",
        "save the current configuration as a preset. usage: save <preset name>",
        true,
        pm,
        static_save_current_preset
    }));
    assert(embeddedCliAddBinding(cli, {
        "backup",
        "backup the preset to USB drive, or all presets if none specified. usage: backup [preset name]",
        true,
        pm,
        static_fatfs_backup
    }));
    assert(embeddedCliAddBinding(cli, {
        "restore",
        "restore the preset from the USB drive. usage: restore <preset name>",
        true,
        pm,
        static_fatfs_restore
    }));
}

void rppicomidi::Preset_manager_cli::static_save_current_preset(EmbeddedCli* cli, char* args, void* context)
{
    (void)cli;
    if (embeddedCliGetTokenCount(args) != 1) {
        printf("usage: save <filename>\r\n");
        return;
    }
    if (reinterpret_cast<Preset_manager*>(context)->save_current_preset(std::string(embeddedCliGetToken(args, 1)))) {
        printf("Saved preset %s as current preset\r\n",embeddedCliGetToken(args, 1));
    }
}

void rppicomidi::Preset_manager_cli::static_load_preset(EmbeddedCli* cli, char* args, void* context)
{
    (void)cli;
    if (embeddedCliGetTokenCount(args) != 1) {
        printf("usage: load <filename>\r\n");
        return;
    }
    if (reinterpret_cast<Preset_manager*>(context)->load_preset(std::string(embeddedCliGetToken(args, 1)))) {
        printf("Loaded preset %s as current preset\r\n",embeddedCliGetToken(args, 1));
    }
    else {
        printf("Failed to load preset %s\r\n", embeddedCliGetToken(args, 1));
    }
}


void rppicomidi::Preset_manager_cli::static_fatfs_backup(EmbeddedCli *cli, char *args, void *context)
{
    (void)cli;
    uint16_t argc = embeddedCliGetTokenCount(args);
    FRESULT res;
    if (argc == 0) {
        res = reinterpret_cast<Preset_manager*>(context)->backup_all_presets();
        if (res != FR_OK) {
            printf("Error %u backing up files on drive\r\n", res);
        }
    }
    else if (argc == 1) {
        const char* preset_name = embeddedCliGetToken(args, 1);
        res = reinterpret_cast<Preset_manager*>(context)->backup_preset(preset_name);
        if (res != FR_OK) {
            printf("error %d backing up preset %s\r\n", res, preset_name);
        }
    }
    else {
        printf("usage: backup <preset name>\r\n");
    }
}

void rppicomidi::Preset_manager_cli::static_fatfs_restore(EmbeddedCli* cli, char* args, void* context)
{
    (void)cli;
    uint16_t argc = embeddedCliGetTokenCount(args);
    FRESULT res;
    if (argc == 1) {
        res = reinterpret_cast<Preset_manager*>(context)->restore_preset(embeddedCliGetToken(args, 1));
    }
    else {
        printf("usage: restore <preset name>\r\n");
        return;
    }
    if (res != FR_OK) {
        printf("error %u restoring preset from %s\r\n", res, embeddedCliGetToken(args, 1));
    }
}
