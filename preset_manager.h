/**
 * @file preset_manager.h
 * @brief this file supports loading and storing presets in
 * non-volatile storage
 *
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
#include <string>
#include "pico_hal.h"
#include "embedded_cli.h"
#include "ff.h"
namespace rppicomidi
{
class Preset_manager
{
public:
    Preset_manager();

    Preset_manager(Preset_manager const &) = delete;
    void operator=(Preset_manager const &) = delete;
    ~Preset_manager()=default;
    void init();
    /**
     * @brief Save the current settings to an LFS file called preset_name and
     * change the current preset to preset name
     * 
     * @param preset_name the new preset name
     * @return true if successful, false otherwise
     */
    bool save_current_preset(std::string preset_name);

    /**
     * @brief load the preset file named preset_name and update the
     * current settings as much as possible based on what devices
     * are currently plugged in, then change the current preset
     * to preset_name
     * 
     * @param preset_name the name of the preset to load
     */
#ifdef RPPICOMIDI_PICO_W
    bool load_preset(std::string preset_name, bool skip_bluetooth);
#else
    bool load_preset(std::string preset_name);
#endif

    /**
     * @brief Get the current preset name
     * 
     * @param preset_name is set to the current preset name
     */
    void get_current_preset_name(std::string& preset_name) { preset_name = current_preset_name; }

    /**
     * @brief Copy all presets to USB flash drive
     * 
     * @return FRESULT 
     */
    FRESULT backup_all_presets();

    /**
     * @brief Copy one preset to external flash drive
     * 
     * @param preset_name the preset name
     * @param mount is true to mount the LFS file system on entry and unmount it on ext
     * @return FRESULT FR_ERR_OK if successful, a FatFs error code if not
     */
    FRESULT backup_preset(const char* preset_name, bool mount=true);

    /**
     * @brief Restore the preset from external flash drive to the local LFS filesystem
     * 
     * @param preset_name the preset name
     * @return FRESULT FR_ERR_OK if successful, a FatFs error code if not
     */
    FRESULT restore_preset(const char* preset_name);
private:
    /**
     * @brief set raw_settings_ptr to point to the data contained in the settings
     * file fn.
     *
     * The caller of this function must delete the memory allocated to raw_settings_ptr
     * using delete[];
     * @param fn the file name in lfs flash that contains the preset settings
     * @param raw_settings_ptr will point to a new block of memory initialized to
     * contain the preset settings
     * @param mount is true if the lfs filesystem needs to be mounted on entry
     * and unmounted on exit
     * @return int the number of bytes in the settings string, or a negative
     * LFS error code if there was an error.
     */
    int load_settings_string(const char* fn, char** raw_settings_ptr, bool mount=true);

    bool update_current_preset(std::string& preset_name, bool mount = true);

    std::string current_preset_name;
    static constexpr const char* preset_dir_name = "/rppicomidi-midi2usbhub";
    static constexpr const char* current_preset_filename = "current-preset";
};
}