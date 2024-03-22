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
#include "ff.h"

namespace rppicomidi
{
class Pico_fatfs_cli
{
public:
    Pico_fatfs_cli() = delete;
    Pico_fatfs_cli(Pico_fatfs_cli const&) = delete;
    void operator=(Pico_fatfs_cli const&) = delete;
    ~Pico_fatfs_cli()=default;
    Pico_fatfs_cli(EmbeddedCli* cli_);
    static uint16_t get_num_commands() { return 7; }
private:
    FRESULT scan_files(const char* path);
    void print_fat_date(WORD wdate);
    void print_fat_time(WORD wtime);

    static void static_fatfs_ls(EmbeddedCli *cli, char *args, void *context);
    static void static_fatfs_cd(EmbeddedCli* cli, char* args, void* context);
    static void static_fatfs_chdrive(EmbeddedCli *cli, char *args, void *);
    static void static_fatfs_pwd(EmbeddedCli *, char *, void *);
    static void static_set_date(EmbeddedCli *cli, char *args, void *context);
    static void static_set_time(EmbeddedCli *cli, char *args, void *context);
    static void static_get_fat_time(EmbeddedCli *cli, char *args, void *context);
};
}