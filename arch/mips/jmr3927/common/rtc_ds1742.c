/*
 * Copyright 2001 MontaVista Software Inc.
 * Author: MontaVista Software, Inc.
 *              ahennessy@mvista.com
 *
 * arch/mips/jmr3927/common/rtc_ds1742.c
 * Based on arch/mips/ddb5xxx/common/rtc_ds1386.c
 *     low-level RTC hookups for s for Dallas 1742 chip.
 *
 * Copyright (C) 2000-2001 Toshiba Corporation
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 */


/*
 * This file exports a function, rtc_ds1386_init(), which expects an
 * uncached base address as the argument.  It will set the two function
 * pointers expected by the MIPS generic timer code.
 */

#include <linux/bcd.h>
#include <linux/types.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/ds1742rtc.h>

#include <asm/time.h>
#include <asm/addrspace.h>

#include <asm/debug.h>

#define	EPOCH		2000

static unsigned long rtc_base;

static unsigned long
rtc_ds1742_get_time(void)
{
	unsigned int year, month, day, hour, minute, second;
	unsigned int century;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	rtc_write(RTC_READ, RTC_CONTROL);
	second = BCD2BIN(rtc_read(RTC_SECONDS) & RTC_SECONDS_MASK);
	minute = BCD2BIN(rtc_read(RTC_MINUTES));
	hour = BCD2BIN(rtc_read(RTC_HOURS));
	day = BCD2BIN(rtc_read(RTC_DATE));
	month = BCD2BIN(rtc_read(RTC_MONTH));
	year = BCD2BIN(rtc_read(RTC_YEAR));
	century = BCD2BIN(rtc_read(RTC_CENTURY) & RTC_CENTURY_MASK);
	rtc_write(0, RTC_CONTROL);
	spin_unlock_irqrestore(&rtc_lock, flags);

	year += century * 100;

	return mktime(year, month, day, hour, minute, second);
}
extern void to_tm(unsigned long tim, struct rtc_time * tm);

static int
rtc_ds1742_set_time(unsigned long t)
{
	struct rtc_time tm;
	u8 year, month, day, hour, minute, second;
	u8 cmos_year, cmos_month, cmos_day, cmos_hour, cmos_minute, cmos_second;
	int cmos_century;
	unsigned long flags;

	spin_lock_irqsave(&rtc_lock, flags);
	rtc_write(RTC_READ, RTC_CONTROL);
	cmos_second = (u8)(rtc_read(RTC_SECONDS) & RTC_SECONDS_MASK);
	cmos_minute = (u8)rtc_read(RTC_MINUTES);
	cmos_hour = (u8)rtc_read(RTC_HOURS);
	cmos_day = (u8)rtc_read(RTC_DATE);
	cmos_month = (u8)rtc_read(RTC_MONTH);
	cmos_year = (u8)rtc_read(RTC_YEAR);
	cmos_century = rtc_read(RTC_CENTURY) & RTC_CENTURY_MASK;

	rtc_write(RTC_WRITE, RTC_CONTROL);

	/* convert */
	to_tm(t, &tm);

	/* check each field one by one */
	year = BIN2BCD(tm.tm_year - EPOCH);
	if (year != cmos_year) {
		rtc_write(year,RTC_YEAR);
	}

	month = BIN2BCD(tm.tm_mon);
	if (month != (cmos_month & 0x1f)) {
		rtc_write((month & 0x1f) | (cmos_month & ~0x1f),RTC_MONTH);
	}

	day = BIN2BCD(tm.tm_mday);
	if (day != cmos_day) {

		rtc_write(day, RTC_DATE);
	}

	if (cmos_hour & 0x40) {
		/* 12 hour format */
		hour = 0x40;
		if (tm.tm_hour > 12) {
			hour |= 0x20 | (BIN2BCD(hour-12) & 0x1f);
		} else {
			hour |= BIN2BCD(tm.tm_hour);
		}
	} else {
		/* 24 hour format */
		hour = BIN2BCD(tm.tm_hour) & 0x3f;
	}
	if (hour != cmos_hour) rtc_write(hour, RTC_HOURS);

	minute = BIN2BCD(tm.tm_min);
	if (minute !=  cmos_minute) {
		rtc_write(minute, RTC_MINUTES);
	}

	second = BIN2BCD(tm.tm_sec);
	if (second !=  cmos_second) {
		rtc_write(second & RTC_SECONDS_MASK,RTC_SECONDS);
	}

	/* RTC_CENTURY and RTC_CONTROL share same address... */
	rtc_write(cmos_century, RTC_CONTROL);
	spin_unlock_irqrestore(&rtc_lock, flags);

	return 0;
}

void
rtc_ds1742_init(unsigned long base)
{
	u8  cmos_second;

	/* remember the base */
	rtc_base = base;
	db_assert((rtc_base & 0xe0000000) == KSEG1);

	/* set the function pointers */
	rtc_mips_get_time = rtc_ds1742_get_time;
	rtc_mips_set_time = rtc_ds1742_set_time;

	/* clear oscillator stop bit */
	rtc_write(RTC_READ, RTC_CONTROL);
	cmos_second = (u8)(rtc_read(RTC_SECONDS) & RTC_SECONDS_MASK);
	rtc_write(RTC_WRITE, RTC_CONTROL);
	rtc_write(cmos_second, RTC_SECONDS); /* clear msb */
	rtc_write(0, RTC_CONTROL);
}
