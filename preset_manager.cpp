/**
 * @file preset_manager.cpp
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
#ifdef NDEBUG
// Need to do this here for release builds or no CLI commands will be added
// All build variants except DEBUG define NDEBUG, which makes assert() macro generate
// no code at all, which prevents msc_demo_cli_init() from adding any CLI commands.
#undef NDEBUG
#endif

#include "preset_manager.h"
#include "midi2piousbhub.h"
#include "diskio.h"
#include "rp2040_rtc.h"
rppicomidi::Preset_manager::Preset_manager()
{

}

void rppicomidi::Preset_manager::init()
{
    // Make sure the flash filesystem is working
    int error_code = pico_mount(false);
    if (error_code != 0) {
        printf("Error %s mounting the flash file system\r\nFormatting...", pico_errmsg(error_code));
        error_code = pico_mount(true);
        if (error_code != 0) {
            printf("Error %s mounting the flash file system\r\nFormatting...", pico_errmsg(error_code));
            exit(-1);
        }
        else {
            printf("completed\r\n");
        }
    }
    printf("successfully mounted flash file system\r\n");

    lfs_file_t file;
    error_code = lfs_file_open(&file, current_preset_filename, LFS_O_RDONLY);
    if (error_code == LFS_ERR_OK) {
        char curr_preset[LFS_NAME_MAX+1];
        lfs_ssize_t nread = lfs_file_read(&file, curr_preset, LFS_NAME_MAX);
        if (nread > 0) {
            curr_preset[nread] = '\0'; // just in case
            current_preset_name = std::string(curr_preset);
        }
    }
    else {
        printf("Error %s opening file %s for reading\r\n", pico_errmsg(error_code), current_preset_filename);
    }
    if (pico_unmount() != 0) {
        printf("Failed to unmount the flash file system\r\n");
    }
    msc_fat_init();
}
bool rppicomidi::Preset_manager::save_current_preset(std::string preset_name)
{
    // mount the lfs
    int error_code = pico_mount(false);
    if (error_code != 0) {
        printf("Error %s mounting the flash file system\r\n", pico_errmsg(error_code));
        return false;
    }

    std::string ser;
    Midi2PioUsbhub::instance().serialize(ser);
    lfs_file_t file;
    std::string filename = preset_name;
    error_code = lfs_file_open(&file, filename.c_str(),
                                LFS_O_WRONLY | LFS_O_TRUNC | LFS_O_CREAT); // open for write, truncate if it exists and create if it doesn't
    if (error_code != LFS_ERR_OK) {
        pico_unmount();
        printf("error %s opening file %s\r\n", pico_errmsg(error_code), filename.c_str());
        return false;
    }
    lfs_ssize_t size = lfs_file_write(&file, ser.c_str(), ser.length());
    error_code = lfs_file_close(&file);
    if (size < 0 || size != static_cast<lfs_ssize_t>(ser.length())) {
        printf("error %s writing preset data to file %s\r\n", pico_errmsg(size), filename.c_str());
        if (error_code != LFS_ERR_OK) {
            printf("error %s closing to file %s\r\n", pico_errmsg(error_code), filename.c_str());
        }
        pico_unmount();
        return false;
    }
    // preset data is written. Now need to update the current preset file
    bool result = update_current_preset(preset_name, false);
    pico_unmount();
    return result;
}

bool rppicomidi::Preset_manager::update_current_preset(std::string& preset_name, bool mount)
{
    bool result = true;
    // See if the current preset file is present
    // If it isn't, clear current_preset_name. This
    // can happen if someone deletes the file
    // or reformats the the littlefs file system
    // outside of the control of Preset_manager.
    if (mount) {
        int err = pico_mount(false);
        if (err != 0) {
            printf("Error %s mounting the flash file system\r\n", pico_errmsg(err));
            return false;
        }
    }

    lfs_file_t file;
    int error_code = lfs_file_open(&file, current_preset_filename, LFS_O_RDONLY); // open for read (see if file exists)
    if (error_code == LFS_ERR_OK) {
        char curr_preset[LFS_NAME_MAX+1];
        lfs_ssize_t nread = lfs_file_read(&file, curr_preset, LFS_NAME_MAX);
        if (nread > 0) {
            curr_preset[nread] = '\0'; // just in case
            current_preset_name = std::string(curr_preset);
        }
        error_code = lfs_file_close(&file);
        if (error_code != LFS_ERR_OK) {
            if (mount)
                pico_unmount();
            return false;
        }
    }
    else {
        current_preset_name = "";
    }

    // only need to do stuff if the preset_name is not the current_preset_name
    if (preset_name != current_preset_name) {
        int error_code = lfs_file_open(&file, current_preset_filename,
                                    LFS_O_WRONLY | LFS_O_TRUNC | LFS_O_CREAT); // open for write, truncate if it exists and create if it doesn't
        if (error_code != LFS_ERR_OK) {
            if (mount)
                pico_unmount();
            printf("error %s opening file %s\r\n", pico_errmsg(error_code), current_preset_filename);
            return false;
        }
        auto size = lfs_file_write(&file, preset_name.c_str(), preset_name.length());
        error_code = lfs_file_close(&file);
        if (error_code != LFS_ERR_OK) {
            result = false;
            printf("error %s closing to file %s\r\n", pico_errmsg(error_code), current_preset_filename);
        }
        if (size < 0 || size != static_cast<lfs_ssize_t>(preset_name.length())) {
            printf("error %s writing preset name to file %s\r\n", pico_errmsg(size), current_preset_filename);
            result = false;
        }
        if (result) { 
            current_preset_name = preset_name;
        }
        else {
            current_preset_name = "";
        }
        if (mount)
            pico_unmount();
    }
    else if (mount)
        pico_unmount();
    return result;
}

int rppicomidi::Preset_manager::load_settings_string(const char* settings_filename, char** raw_settings_ptr, bool mount)
{
    int error_code = 0;
    if (mount)
        error_code = pico_mount(false);
    if (error_code != 0) {
        printf("unexpected error %s mounting flash\r\n", pico_errmsg(error_code));
        return error_code;
    }
    int file = pico_open(settings_filename, LFS_O_RDONLY);
    if (file < 0) {
        // file isn't there
        if (mount)
            pico_unmount();
        return file; // the error code
    }
    else {
        // file is open. Read the whole contents to a string
        auto flen = pico_size(file);
        if (flen < 0) {
            // Something went wrong
            if (mount)
                pico_unmount();
            return flen;
        }
        // create a string long enough
        *raw_settings_ptr = new char[flen+1];
        if (!raw_settings_ptr) {
            pico_close(file);
            if (mount)
                pico_unmount();
            return LFS_ERR_NOMEM; // new failed
        }
        auto nread = pico_read(file, *raw_settings_ptr, flen);
        pico_close(file);
        if (mount)
            pico_unmount();
        if (nread == (lfs_size_t)flen) {
            // Success!
            (*raw_settings_ptr)[flen]='\0'; // just in case, add null termination
            return nread;
        }
        if (nread > 0) {
            delete[] *raw_settings_ptr;
            *raw_settings_ptr = nullptr;
            nread = LFS_ERR_IO;
        }
        return nread;
    }
}

#ifdef RPPICOMIDI_PICO_W
bool rppicomidi::Preset_manager::load_preset(std::string preset_name, bool skip_bluetooth)
#else
bool rppicomidi::Preset_manager::load_preset(std::string preset_name)
#endif
{
    char* raw_preset_string;
    int error_code = load_settings_string(preset_name.c_str(), &raw_preset_string);
    bool result = false;
    if (error_code > 0) {
        std::string settings = std::string(raw_preset_string);
        delete[] raw_preset_string;
#ifdef RPPICOMIDI_PICO_W
        if (Midi2PioUsbhub::instance().deserialize(settings, skip_bluetooth)) {
#else
        if (Midi2PioUsbhub::instance().deserialize(settings)) {
#endif
            if (update_current_preset(preset_name)) {
                printf("load preset %s successful\r\n", preset_name.c_str());
                result = true;
            }
        }
        else {
            printf("error deserilizating the preset\r\n");
        }        
    }
    else {
        printf("error %s loading settings %s\r\n", pico_errmsg(error_code), preset_name.c_str());
    }
    return result;
}

FRESULT rppicomidi::Preset_manager::backup_preset(const char* preset_name, bool mount)
{
    int error_code = LFS_ERR_OK;
    if (mount) {
        error_code = pico_mount(false);
        if (error_code != 0) {
            printf("unexpected error %s mounting flash\r\n", pico_errmsg(error_code));
            return FR_INT_ERR;
        }
    }
    lfs_file_t file;
    error_code = lfs_file_open(&file, preset_name, LFS_O_RDONLY);
    if (error_code != LFS_ERR_OK) {
        printf("error %s opening preset file %s\r\n", pico_errmsg(error_code), preset_name);
        if (mount)
            pico_unmount();
        return FR_INT_ERR;
    }
    lfs_soff_t sz_src = lfs_file_size(&file);
    if (sz_src < 0) {
        printf("error %s determining preset file %s size\r\n", pico_errmsg(error_code), preset_name);
        lfs_file_close(&file);
        if (mount)
            pico_unmount();
        return FR_INT_ERR;
    }
    uint8_t buffer[sz_src];
    lfs_ssize_t nread = lfs_file_read(&file, buffer, sz_src);
    if (nread < 0) {
        printf("error %s reading preset file %s\r\n", pico_errmsg(error_code), preset_name);
        return FR_INT_ERR;
    }
    lfs_file_close(&file);
    if (mount)
        pico_unmount();
    
    DIR dir;
    FRESULT res = f_opendir(&dir, preset_dir_name);
    if (res == FR_NO_PATH) {
        res = f_mkdir(preset_dir_name);
        if (res != FR_OK) {
            printf("error %d creating directory %s\r\n", res, preset_dir_name);
            return res;
        }
        else {
            res = f_opendir(&dir, preset_dir_name);
            if (res != FR_OK) {
                printf("error %d probing for directory %s\r\n", res, preset_dir_name);
                return res;            
            }
        }
    }
    else if (res != FR_OK) {
        printf("error %d probing for directory %s\r\n", res, preset_dir_name);
        return res;
    }
    res = f_closedir(&dir);
    if (res != FR_OK) {
        printf("error %d closing directory %s\r\n", res, preset_dir_name);
        return res;
    }
    std::string path = std::string(preset_dir_name) + "/" + std::string(preset_name);
    FIL fil;
    res = f_open(&fil, path.c_str(), FA_WRITE | FA_CREATE_ALWAYS);
    if (res == FR_OK) {
        UINT nwritten;
        res = f_write(&fil, buffer, nread, &nwritten);
        f_close(&fil);
        if (res != FR_OK) {
            printf("error %d writing file %s\r\n", res, path.c_str());
        }
        else if (nwritten != (UINT)nread) {
            printf("error writing %s: nread=%ld nwritten=%u\r\n", path.c_str(), nread, nwritten);
        }
    }
    return res;
}

FRESULT rppicomidi::Preset_manager::backup_all_presets()
{
    int error_code = pico_mount(false);
    if (error_code != LFS_ERR_OK) {
        printf("unexpected error %s mounting flash\r\n", pico_errmsg(error_code));
        return FR_INT_ERR;
    }
    lfs_dir_t dir;
    error_code = lfs_dir_open(&dir, "/");
    if (error_code != LFS_ERR_OK) {
        printf("unexpected error %s opening backup directory /\r\n", pico_errmsg(error_code));
        pico_unmount();
        return FR_INT_ERR;
    }
    struct lfs_info info;
    while (true) {
        lfs_ssize_t nread = lfs_dir_read(&dir, &info);
        if (nread < 0) {
            printf("errror %s reading preset root directory\r\n", pico_errmsg(nread));
            return FR_INT_ERR;
        }

        if (nread == 0) {
            break;
        }
        if (info.type == LFS_TYPE_REG) {
            backup_preset(info.name, false);
        }
    }

    error_code = lfs_dir_close(&dir);
    if (error_code) {
        return FR_INT_ERR;
    }
    return FR_OK;
}


FRESULT rppicomidi::Preset_manager::restore_preset(const char* preset_name)
{
    FIL fil;
    std::string path = std::string(preset_dir_name)+ "/" + std::string(preset_name);
    FRESULT res = f_open(&fil,path.c_str(), FA_READ);
    if (res != FR_OK) {
        printf("error %d opening file %s\r\n", res, path.c_str());
        return res;
    }
    size_t flen = f_size(&fil);
    uint8_t buffer[flen];
    UINT nread = 0;
    res = f_read(&fil, buffer, flen, &nread);
    if (res != FR_OK) {
        printf("error %d reading file %s\r\n", res, path.c_str());
        f_close(&fil);
        return res;
    }
    f_close(&fil);
    int error_code = pico_mount(false);
    if (error_code != 0) {
        printf("unexpected error %s mounting flash\r\n", pico_errmsg(error_code));
        return FR_INT_ERR;
    }
    lfs_file_t file;
    error_code = lfs_file_open(&file, preset_name, LFS_O_WRONLY | LFS_O_CREAT | LFS_O_TRUNC);
    if (error_code != LFS_ERR_OK) {
        printf("error %s opening preset file %s for write\r\n", pico_errmsg(error_code), preset_name);
        pico_unmount();
        return FR_INT_ERR;
    }
    lfs_ssize_t nwritten = lfs_file_write(&file, buffer, flen);
    error_code = lfs_file_close(&file);
    pico_unmount();
    if (nwritten < 0) {
        printf("error %s opening preset file %s for write\r\n", pico_errmsg(error_code), preset_name);
    }
    else if (nwritten != (lfs_ssize_t)flen) {
        printf("File %s nwritten=%ld nread=%u\r\n", preset_name, nwritten, flen);
    }
    else {
        printf("preset %s restored\r\n", preset_name);
    }
    return FR_OK;
}

//-------------MSC/FATFS IMPLEMENTATION -------------//
static scsi_inquiry_resp_t inquiry_resp;
static FATFS fatfs[FF_VOLUMES];
static_assert(FF_VOLUMES == CFG_TUH_DEVICE_MAX);

bool inquiry_complete_cb(uint8_t dev_addr, tuh_msc_complete_data_t const* cb_data)
{
    if (cb_data->csw->status != 0) {
        printf("Inquiry failed\r\n");
        return false;
    }

    // Print out Vendor ID, Product ID and Rev
    printf("%.8s %.16s rev %.4s\r\n", inquiry_resp.vendor_id, inquiry_resp.product_id, inquiry_resp.product_rev);

    // Get capacity of device
    uint32_t const block_count = tuh_msc_get_block_count(dev_addr, cb_data->cbw->lun);
    uint32_t const block_size = tuh_msc_get_block_size(dev_addr, cb_data->cbw->lun);

    printf("Disk Size: %lu MB\r\n", block_count / ((1024*1024)/block_size));
    printf("Block Count = %lu, Block Size: %lu\r\n", block_count, block_size);

    return true;
}

void tuh_msc_mount_cb(uint8_t dev_addr)
{
    uint8_t pdrv = mmc_map_next_pdrv(dev_addr);

    assert(pdrv < FF_VOLUMES);
    msc_fat_plug_in(pdrv);
    uint8_t const lun = 0;
    tuh_msc_inquiry(dev_addr, lun, &inquiry_resp, inquiry_complete_cb, 0);
    char path[3] = "0:";
    path[0] += pdrv;
    if ( f_mount(&fatfs[pdrv],path, 0) != FR_OK ) {
        printf("mount failed\r\n");
        return;
        if (f_chdrive(path) != FR_OK) {
            printf("f_chdrive(%s) failed\r\n", path);
        }
    }
    printf("\r\nMass Storage drive %u is mounted\r\n", pdrv);
    printf("Run the set-date and set-time commands so file timestamps are correct\r\n\r\n");
}

void tuh_msc_umount_cb(uint8_t dev_addr)
{
    uint8_t pdrv = mmc_unmap_pdrv(dev_addr);
    char path[3] = "0:";
    path[0] += pdrv;

    f_mount(NULL, path, 0); // unmount disk
    msc_fat_unplug(pdrv);
    printf("Mass Storage drive %u is unmounted\r\n", pdrv);
}

void main_loop_task()
{
    rppicomidi::Midi2PioUsbhub::instance().task();
}