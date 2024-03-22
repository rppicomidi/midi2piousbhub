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

namespace rppicomidi
{
class Pico_lfs_cli
{
public:
    Pico_lfs_cli() = delete;
    Pico_lfs_cli(Pico_lfs_cli const&) = delete;
    void operator=(Pico_lfs_cli const&) = delete;
    ~Pico_lfs_cli()=default;
    Pico_lfs_cli(EmbeddedCli* cli_);
    static uint16_t get_num_commands() { return 5; }
private:
    // CLI Commands
    static void static_file_system_format(EmbeddedCli *, char *, void *);
    static void static_file_system_status(EmbeddedCli *, char *, void *);
    static void static_list_files(EmbeddedCli *cli, char *args, void *context);
    static void static_print_file(EmbeddedCli *cli, char *args, void *context);
    static void static_delete_file(EmbeddedCli *cli, char *args, void *);

    // Helper methods
    int lfs_ls(const char *path);
    /**
     * @brief remove a file from the lfs filesystem
     *
     * @param filename the full path to the file
     * @param mount is true to mount the filesystem on entry and unmount it on exit
     * @return int LFS_ERR_OK if successful, a negative error code if not
     */
    int delete_file(const char *filename, bool mount = true);

    // data
    EmbeddedCli* cli;
};
}