/**
 * @file rp2040_rtc.cpp
 * @brief this class is the implementation of the Rp2040_rtc class using
 * the RP2040's real-time clock
 *
 * MIT License
 *
 * Copyright (c) 2022, 2026 rppicomidi
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
#include "hardware/rtc.h"
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
    datetime_t t = {
            .year  = year,
            .month = month,
            .day   = day,
            .dotw  = (int8_t)get_day_of_the_week(year, month, day),
            .hour  = hour,
            .min   = min,
            .sec   = sec
    };

    // Start the RTC
    rtc_init();
    rtc_set_datetime(&t);
    // clk_sys is >2000x faster than clk_rtc, so datetime is not updated immediately when rtc_set_datetime() is called.
    // tbe delay is up to 3 RTC clock cycles (which is 64us with the default clock settings)
    sleep_us(64);
}

bool rppicomidi::Rp2040_rtc::set_date(uint16_t year, uint8_t month, uint8_t day)
{
    bool success = ((year <= 9999) && (month >=1 && month <=12) && day_ok_for_month(year, month, day));
    if (success) {
        datetime_t t;
        rtc_get_datetime(&t);
        t.year = year;
        t.month = month;
        t.day = day;
        t.dotw = get_day_of_the_week(year, month, day);
        rtc_set_datetime(&t);
        sleep_us(64);
    }
    return success;
}

bool rppicomidi::Rp2040_rtc::set_time(uint8_t hour, uint8_t min, uint8_t sec)
{
    bool success = ((hour <= 24) && (min < 60) && (sec < 60));
    if (success) {
        datetime_t t;
        rtc_get_datetime(&t);
        t.hour = hour;
        t.min = min;
        t.sec = sec;
        rtc_set_datetime(&t);
        sleep_us(64);
    }
    return success;
}

void rppicomidi::Rp2040_rtc::get_date(uint16_t& year, uint8_t& month, uint8_t& day, uint8_t& dow)
{
    datetime_t t;
    rtc_get_datetime(&t);
    year = t.year;
    month = t.month;
    day = t.day;
    dow = t.dotw;
}

void rppicomidi::Rp2040_rtc::get_time(uint8_t& hour, uint8_t& min, uint8_t& sec)
{
    datetime_t t;
    rtc_get_datetime(&t);
    hour = t.hour;
    min = t.min;
    sec = t.sec;
}

uint32_t rppicomidi::Rp2040_rtc::get_fat_date_time()
{
    uint32_t result = ~0; //error value
    datetime_t t;
    rtc_get_datetime(&t);
    uint32_t yearfield = (t.year-1980);
    if (yearfield < 127) {        
        result = (yearfield << 25) | (t.month << 21) | (t.day << 16) | (t.hour << 11) | (t.min << 5) | (t.sec / 2);
    }
    return result;
}
