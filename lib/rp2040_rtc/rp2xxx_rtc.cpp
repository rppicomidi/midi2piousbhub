/**
 * @file rp2xxx_rtc.cpp
 * @brief implements the common functions for the Rp2040_rtc class
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
#include <cmath>
#include <cstring>
#include <cstdlib>
#include "rp2040_rtc.h"

const char* const rppicomidi::Rp2040_rtc::month_name[] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"
};

uint8_t rppicomidi::Rp2040_rtc::get_day_of_the_week(uint16_t year, uint8_t month, uint8_t day)
{
    // https://en.wikipedia.org/wiki/Determination_of_the_day_of_the_week
    // Keith's and Cramer's algorithm
    return (day+=month<3?year--:year-2,23*month/9+day+4+year/4-year/100+year/400)%7;
}

uint8_t rppicomidi::Rp2040_rtc::get_month_number(char* month_str)
{
    uint8_t month_num=0;
    for(; month_num<12 && strncmp(month_str, month_name[month_num], 3) != 0; month_num++) {
    }

    return month_num+1;
}


uint8_t rppicomidi::Rp2040_rtc::get_max_days_for_month(uint16_t year, uint8_t month)
{
    static const uint8_t days_in_month[] {
        31, // Jan
        28, // Feb (non leap year)
        31, // Mar
        30, // Apr
        31, // May
        30, // Jun
        31, // Jul
        31, // Aug
        30, // Sep
        31, // Oct
        30, // Nov
        31, // Dec
    };
    uint8_t max_day = 0; // an illegal day number
    if (month == 2 && (year % 4) == 0 && ((year % 100) != 0 || (year % 400) == 0)) {
        max_day = 29;
    }
    else if (month <= sizeof(days_in_month) && year < 10000) {
        max_day = days_in_month[month-1];
    }
    return max_day;
}

bool rppicomidi::Rp2040_rtc::day_ok_for_month(uint16_t year, uint8_t month, uint8_t day)
{
    return day >0 && day <= get_max_days_for_month(year, month);
}

const char* rppicomidi::Rp2040_rtc::get_month_name(uint8_t month_num)
{
    const char* name_ptr = nullptr;
    if (month_num > 0 && month_num <= 12) {
        name_ptr = month_name[month_num - 1];
    }
    return name_ptr;
}

uint16_t rppicomidi::Rp2040_rtc::get_day_of_the_year(uint16_t year, uint8_t month, uint8_t day)
{
	uint16_t elapsed_days_in_month[] = { 0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334 };
    uint16_t day_of_the_year = elapsed_days_in_month[month - 1] + day;
    //If leap year and after February
	if ((year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)) && month >= 2) {
		day_of_the_year += 1;
    }

	return day_of_the_year;
}

// For FatFs library
extern "C" uint32_t get_fattime(void)
{
    return rppicomidi::Rp2040_rtc::instance().get_fat_date_time();
}
