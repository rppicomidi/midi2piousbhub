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
#include "pico_lfs_cli.h"
#include "pico_hal.h"
rppicomidi::Pico_lfs_cli::Pico_lfs_cli(EmbeddedCli* cli_) : cli{cli_}
{
    volatile bool result;
    result = embeddedCliAddBinding(cli, {
        "cat",
        "print file contents. usage: cat <fn>",
        true,
        this,
        static_print_file
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "format",
        "format settings file system",
        false,
        this,
        static_file_system_format
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "fsstat",
        "display settings file system status",
        false,
        this,
        static_file_system_status
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "ls",
        "list all settings files",
        true,
        this,
        static_list_files
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "rm",
        "delete. usage: rm <fn>",
        true,
        this,
        static_delete_file
    });
    assert(result);
    (void)result;
}

int rppicomidi::Pico_lfs_cli::lfs_ls(const char *path)
{
    lfs_dir_t dir;
    int err = lfs_dir_open(&dir, path);
    if (err) {
        return err;
    }

    struct lfs_info info;
    while (true) {
        int res = lfs_dir_read(&dir, &info);
        if (res < 0) {
            return res;
        }

        if (res == 0) {
            break;
        }

        switch (info.type) {
            case LFS_TYPE_REG: printf("reg "); break;
            case LFS_TYPE_DIR: printf("dir "); break;
            default:           printf("?   "); break;
        }

        static const char *prefixes[] = {"", "K", "M", "G"};
        for (int i = sizeof(prefixes)/sizeof(prefixes[0])-1; i >= 0; i--) {
            if (info.size >= static_cast<size_t>((1 << 10*i)-1)) {
                printf("%*lu%sB ", 4-(i != 0), info.size >> 10*i, prefixes[i]);
                break;
            }
        }

        printf("%s\n", info.name);
    }

    err = lfs_dir_close(&dir);
    if (err) {
        return err;
    }

    return 0;
}

void rppicomidi::Pico_lfs_cli::static_file_system_format(EmbeddedCli*, char*, void*)
{
    printf("formatting settings file system then mounting it\r\n");
    int error_code = pico_mount(true);
    if (error_code != LFS_ERR_OK) {

    }
    else {
        error_code = pico_unmount();
        if (error_code != LFS_ERR_OK) {
            printf("unexpected error %s unmounting settings file system\r\n", pico_errmsg(error_code));
        }
        else {
            printf("File system successfully formated and unmounted\r\n");
        }
    }
}

void rppicomidi::Pico_lfs_cli::static_file_system_status(EmbeddedCli*, char*, void*)
{
    int error_code = pico_mount(false);
    if (error_code != LFS_ERR_OK) {
        printf("can't mount settings file system\r\n");
        return;
    }
    struct pico_fsstat_t stat;
    if (pico_fsstat(&stat) == LFS_ERR_OK) {
        printf("FS: blocks %d, block size %d, used %d\n", (int)stat.block_count, (int)stat.block_size,
                (int)stat.blocks_used);
    }
    else {
        printf("could not read file system status\r\n");
    }
    error_code = pico_unmount();
    if (error_code != LFS_ERR_OK) {
        printf("can't unmount settings file system\r\n");
        return;
    }
}


void rppicomidi::Pico_lfs_cli::static_list_files(EmbeddedCli* cli, char* args, void* context)
{
    (void)cli;
    (void)args;
    char path[256] = {'/','\0'};
    uint16_t argc = embeddedCliGetTokenCount(args);
    if (argc == 1) {
        strncpy(path, embeddedCliGetToken(args, 1), sizeof(path)-1);
        path[sizeof(path)-1] = '\0';
    }
    else {
        printf("usage: ls [path]\r\n");
    }
    auto me = reinterpret_cast<Pico_lfs_cli*>(context);
    int error_code = pico_mount(false);
    if (error_code != LFS_ERR_OK) {
        printf("can't mount settings file system\r\n");
        return;
    }
 
    error_code = me->lfs_ls(path);
    if (error_code != LFS_ERR_OK) {
        printf("error listing path \"/\"\r\n");
    }
    error_code = pico_unmount();
    if (error_code != LFS_ERR_OK) {
        printf("can't unmount settings file system\r\n");
        return;
    }
}

void rppicomidi::Pico_lfs_cli::static_delete_file(EmbeddedCli* cli, char* args, void* context)
{
    (void)cli;
    if (embeddedCliGetTokenCount(args) != 1) {
        printf("usage: rm <filename>\r\n");
        return;
    }
    auto me = reinterpret_cast<Pico_lfs_cli*>(context);
    const char* fn=embeddedCliGetToken(args, 1);
    me->delete_file(fn);
}

int rppicomidi::Pico_lfs_cli::delete_file(const char* filename, bool mount)
{
    int error_code = LFS_ERR_OK;
    if (mount)
        error_code = pico_mount(false);
    if (error_code == LFS_ERR_OK) {
        error_code = pico_remove(filename);
        printf("error %s deleting file %s\r\n", pico_errmsg(error_code), filename);
        if (mount)
            error_code = pico_unmount();
        if (error_code != LFS_ERR_OK) {
            printf("Unexpected Error %s unmounting settings file system\r\n", pico_errmsg(error_code));
        }
    }
    return error_code;
}

void rppicomidi::Pico_lfs_cli::static_print_file(EmbeddedCli* cli, char* args, void*)
{
    (void)cli;
    if (embeddedCliGetTokenCount(args) != 1) {
        printf("usage: cat <filename>\r\n");
        return;
    }
    const char* fn=embeddedCliGetToken(args, 1);
    int error_code = pico_mount(false);
    if (error_code != LFS_ERR_OK) {
        printf("could not mount LFS filesystem\r\n");
        return;
    }
    lfs_file_t file;
    error_code = lfs_file_open(&file, fn, LFS_O_RDONLY);
    if (error_code != LFS_ERR_OK) {
        printf("could not open file %s for reading\r\n", fn);
        pico_unmount();
        return;
    }
    auto sz = lfs_file_size(&file);
    if (sz < 0) {
        printf("error determining %s file size\r\n", fn);
        lfs_file_close(&file);
        pico_unmount();
        return;
    }
    char* file_contesnts = new char[sz+1];
    error_code = lfs_file_read(&file, file_contesnts, sz);
    lfs_file_close(&file);
    pico_unmount();
    file_contesnts[sz] = '\0';

    if (error_code > 0) {
        printf("File: %s\r\n%s\r\n", fn, file_contesnts);
    }
    else {
        switch(error_code) {
        case LFS_ERR_NOENT:
            printf("File %s does not exist\r\n", fn);
            break;
        case LFS_ERR_ISDIR:
            printf("%s is a directory\r\n",fn);
            break;
        case LFS_ERR_CORRUPT:
            printf("%s is corrupt\r\n",fn);
            break;
        case LFS_ERR_IO:
            printf("IO Error reading %s\r\n", fn);
            break;
        default:
            printf("Unexpected Error %s reading %s\r\n", pico_errmsg(error_code), fn);
            break;
        }
    }
    delete[] file_contesnts;
}
