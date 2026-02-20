/**
 * @file rp2040_rtc.cpp
 * @brief this class is a wrapper for managing the RP2350 always-on timer
 * implementation of a real-time clock
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
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "pico/aon_timer.h"
#include "rp2040_rtc.h"
#include <cmath>
#include <cstring>
#include <cstdlib>

rppicomidi::Rp2040_rtc::Rp2040_rtc()
{
    char compile_date[] = __DATE__; // the compile date string: e.g. "Nov 03 2022"
    char* cdate_str = strtok(compile_date, " ");
    int8_t month = get_month_number(cdate_str);
    int8_t day = atoi(strtok(NULL, " "));
    int16_t year = atoi(strtok(NULL, " "));
    char compile_time[] = __TIME__; // the compile time string: e.g. 07:35:23
    char* ctime_str = strtok(compile_time, ":");
    int8_t hour = atoi(ctime_str);
    int8_t min = atoi(strtok(NULL, ":"));
    int8_t sec = atoi(strtok(NULL," "));
    const struct tm datetime = {
           .tm_sec = sec,
           .tm_min = min,
           .tm_hour = hour,
           .tm_mday = day,
           .tm_mon = month - 1,
           .tm_year = year - 1900,
           .tm_wday = get_day_of_the_week(year, month, day),
           .tm_yday = get_day_of_the_year(year, month, day),
           .tm_isdst = false, // TODO: add summertime support
    };

    // Start the always on timer
    aon_timer_start_calendar(&datetime);
}

bool rppicomidi::Rp2040_rtc::set_date(uint16_t year, uint8_t month, uint8_t day)
{
    bool success = ((year <= 9999) && (month >=1 && month <=12) && day_ok_for_month(year, month, day));
    if (success) {
        struct tm datetime;
        aon_timer_get_time_calendar(&datetime);
        datetime.tm_year = year - 1900;
        datetime.tm_mon = month - 1;
        datetime.tm_mday = day;
        datetime.tm_wday = get_day_of_the_week(year, month, day);
        datetime.tm_yday = get_day_of_the_year(year, month, day);
        aon_timer_set_time_calendar(&datetime);
    }
    return success;
}

bool rppicomidi::Rp2040_rtc::set_time(uint8_t hour, uint8_t min, uint8_t sec)
{
    bool success = ((hour <= 24) && (min < 60) && (sec < 60));
    if (success) {
        struct tm datetime;
        aon_timer_get_time_calendar(&datetime);
        datetime.tm_hour = hour;
        datetime.tm_min = min;
        datetime.tm_sec = sec;
        aon_timer_set_time_calendar(&datetime);
    }
    return success;
}

void rppicomidi::Rp2040_rtc::get_date(uint16_t& year, uint8_t& month, uint8_t& day, uint8_t& dow)
{
    struct tm datetime;
    aon_timer_get_time_calendar(&datetime);
    year = datetime.tm_year + 1900;
    month = datetime.tm_mon + 1;
    day = datetime.tm_mday;
    dow = datetime.tm_wday;
}

void rppicomidi::Rp2040_rtc::get_time(uint8_t& hour, uint8_t& min, uint8_t& sec)
{
    struct tm datetime;
    aon_timer_get_time_calendar(&datetime);
    hour = datetime.tm_hour;
    min = datetime.tm_min;
    sec = datetime.tm_sec;
}

uint32_t rppicomidi::Rp2040_rtc::get_fat_date_time()
{
    uint32_t result = ~0; //error value
    struct tm datetime;
    aon_timer_get_time_calendar(&datetime);
    uint32_t yearfield = datetime.tm_year - 80; // the tm_year field is the year minus 1900; we need year - 1980
    if (yearfield < 127) {
        result = (yearfield << 25) | ((datetime.tm_mon + 1) << 21) | (datetime.tm_mday << 16) | (datetime.tm_hour << 11) | (datetime.tm_min << 5) | (datetime.tm_sec / 2);
    }
    return result;
}
