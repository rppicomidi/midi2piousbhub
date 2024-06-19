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
#include "pico_fatfs_cli.h"
#include "diskio.h"
#include "rp2040_rtc.h"
#include <string>

rppicomidi::Pico_fatfs_cli::Pico_fatfs_cli(EmbeddedCli* cli)
{
    volatile bool result = embeddedCliAddBinding(cli, {
        "f-cd",
        "change current directory on the USB flash drive. usage: f-cd <path>",
        true,
        this,
        static_fatfs_cd
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "f-chdrive",
        "change current drive number for the USB flash drive. usage: f-chdrive <drive number 0-3>",
        true,
        this,
        static_fatfs_chdrive
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "f-ls",
        "list contents of the current directory on the current USB flash drive. usage: f-ls [path]",
        false,
        this,
        static_fatfs_ls
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
        "f-pwd",
        "print the current directory path of the current USB flash drive. usage: f-pwd",
        false,
        this,
        static_fatfs_pwd
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
            "set-date",
            "change the date for file timestamps; usage set-date year(2022-9999) month(1-12) day(1-31)",
            true,
            NULL,
            static_set_date
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
            "set-time",
            "change the time of day for file timestamps; usage set-time hour(0-23) minute(0-59) second(0-59)",
            true,
            NULL,
            static_set_time
    });
    assert(result);
    result = embeddedCliAddBinding(cli, {
            "get-datetime",
            "print the current date and time",
            false,
            NULL,
            static_get_fat_time
    });
    assert(result);
    (void)result;
}


FRESULT rppicomidi::Pico_fatfs_cli::scan_files(const char* path)
{
    FRESULT res;
    DIR dir;
    static FILINFO fno;

    res = f_opendir(&dir, path);                       /* Open the directory */
    if (res == FR_OK) {
        for (;;) {
            res = f_readdir(&dir, &fno);                   /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
            printf("%lu\t",fno.fsize);
            print_fat_date(fno.fdate);
            print_fat_time(fno.ftime);
            printf("%s%c\r\n",fno.fname, (fno.fattrib & AM_DIR) ? '/' : ' ');
        }
        f_closedir(&dir);
    }

    return res;
}

void rppicomidi::Pico_fatfs_cli::static_fatfs_ls(EmbeddedCli * cli, char *args, void * context)
{
    (void)cli;
    (void)args;
    const char* path = ".";
    uint16_t argc = embeddedCliGetTokenCount(args);
    if (argc == 1) {
        path = embeddedCliGetToken(args, 1);
    }
    else if (argc > 1) {
        printf("usage: fatls [path]\r\n");
    }
    FRESULT res = reinterpret_cast<Pico_fatfs_cli*>(context)->scan_files(path);
    if (res != FR_OK) {
        printf("Error %u listing files on drive\r\n", res);
    }
}


void rppicomidi::Pico_fatfs_cli::print_fat_date(WORD wdate)
{
    uint16_t year = 1980 + ((wdate >> 9) & 0x7f);
    uint16_t month = (wdate >> 5) & 0xf;
    uint16_t day = wdate & 0x1f;
    printf("%02u/%02u/%04u\t", month, day, year);
}

void rppicomidi::Pico_fatfs_cli::print_fat_time(WORD wtime)
{
    uint8_t hour = ((wtime >> 11) & 0x1f);
    uint8_t min = ((wtime >> 5) & 0x3F);
    uint8_t sec = ((wtime &0x1f)*2);
    printf("%02u:%02u:%02u\t", hour, min, sec);
}

void rppicomidi::Pico_fatfs_cli::static_fatfs_cd(EmbeddedCli* cli, char* args, void* context)
{
    (void)cli;
    (void)context;
    uint16_t argc = embeddedCliGetTokenCount(args);
    FRESULT res;
    char temp_cwd[256] = {'/', '\0'};
    if (argc == 0) {
        res = f_chdir(temp_cwd);
    }
    else if (argc == 1) {
        strncpy(temp_cwd, embeddedCliGetToken(args, 1), sizeof(temp_cwd)-1);
        temp_cwd[sizeof(temp_cwd)-1] = '\0';
        res = f_chdir(temp_cwd);
    }
    else {
        printf("usage: fatcd <new path>\r\n");
        return;
    }
    if (res != FR_OK) {
        printf("error %u setting cwd to %s\r\n", res, temp_cwd);
    }
    res = f_getcwd(temp_cwd, sizeof(temp_cwd));
    if (res == FR_OK)
        printf("cwd=\"%s\"\r\n", temp_cwd);
    else
        printf("error %u getting cwd\r\n", res);
}

void rppicomidi::Pico_fatfs_cli::static_fatfs_chdrive(EmbeddedCli *cli, char *args, void *)
{
    (void)cli;
    uint16_t argc = embeddedCliGetTokenCount(args);
    FRESULT res;
    char dstr[] = "0:";
    if (argc != 1) {
        printf("usage chdrive drive_number(0-3)");
    }
    else {
        int drive = atoi(embeddedCliGetToken(args, 1));
        if (drive >=0 && drive <= 3) {
            dstr[0] += drive;
            res = f_chdrive(dstr);
            if (res != FR_OK) {
                printf("error %u setting drive to %d\r\n", res, drive);
            }
        }
        else {
            printf("usage chdrive drive_number(0-3)");
        }
    }
}

void rppicomidi::Pico_fatfs_cli::static_fatfs_pwd(EmbeddedCli *, char *, void *)
{
    char cwd[256];
    FRESULT res = f_getcwd(cwd, sizeof(cwd));
    if (res != FR_OK) {
        printf("error %u reading the current working directory\r\n", res);
    }
    else {
        printf("cwd=%s\r\n", cwd);
    }
}

void rppicomidi::Pico_fatfs_cli::static_set_date(EmbeddedCli *cli, char *args, void *context)
{
    (void)cli;
    (void)context;
    bool usage_ok = false;
    if (embeddedCliGetTokenCount(args) == 3) {
        int16_t year = atoi(embeddedCliGetToken(args, 1));
        int8_t month = atoi(embeddedCliGetToken(args, 2));
        int8_t day = atoi(embeddedCliGetToken(args, 3));
        usage_ok = rppicomidi::Rp2040_rtc::instance().set_date(year, month, day);
    }
    if (!usage_ok) {
        printf("usage: set-date YYYY(1980-2107) MM(1-12) DD(1-31)\r\n");
    }
}

void rppicomidi::Pico_fatfs_cli::static_set_time(EmbeddedCli *cli, char *args, void *context)
{
    (void)cli;
    (void)context;
    bool usage_ok = false;
    if (embeddedCliGetTokenCount(args) == 3) {
        int8_t hour = atoi(embeddedCliGetToken(args, 1));
        int8_t min = atoi(embeddedCliGetToken(args, 2));
        int8_t sec = atoi(embeddedCliGetToken(args, 3));
        usage_ok = rppicomidi::Rp2040_rtc::instance().set_time(hour, min, sec);
    }
    if (!usage_ok) {
        printf("usage: set-time hour(0-23) min(0-59) sec(0-59)\r\n");
    }
}

void rppicomidi::Pico_fatfs_cli::static_get_fat_time(EmbeddedCli *cli, char *args, void *context)
{
    (void)cli;
    (void)args;
    (void)context;
    DWORD f_time=get_fattime();
    int16_t year = ((f_time >> 25 ) & 0x7F) + 1980;
    int8_t month = ((f_time >> 21) & 0xf);
    int8_t day = ((f_time >> 16) & 0x1f);
    int8_t hour = ((f_time >> 11) & 0x1f);
    int8_t min = ((f_time >> 5) & 0x3F);
    int8_t sec = ((f_time &0x1f)*2);
    printf("%02d/%02d/%04d %02d:%02d:%02d\r\n", month, day, year, hour, min, sec);
}
