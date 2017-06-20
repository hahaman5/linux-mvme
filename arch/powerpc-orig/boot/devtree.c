/*
 * devtree.c - convenience functions for device tree manipulation
 * Copyright 2007 David Gibson, IBM Corporation.
 * Copyright (c) 2007 Freescale Semiconductor, Inc.
 *
 * Authors: David Gibson <david@gibson.dropbear.id.au>
 *	    Scott Wood <scottwood@freescale.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version
 * 2 of the License, or (at your option) any later version.
 */
#include <stdarg.h>
#include <stddef.h>
#include "types.h"
#include "string.h"
#include "stdio.h"
#include "ops.h"

void dt_fixup_memory(u64 start, u64 size)
{
	void *root, *memory;
	int naddr, nsize, i;
	u32 memreg[4];

	root = finddevice("/");
	if (getprop(root, "#address-cells", &naddr, sizeof(naddr)) < 0)
		naddr = 2;
	if (naddr < 1 || naddr > 2)
		fatal("Can't cope with #address-cells == %d in /\n\r", naddr);

	if (getprop(root, "#size-cells", &nsize, sizeof(nsize)) < 0)
		nsize = 1;
	if (nsize < 1 || nsize > 2)
		fatal("Can't cope with #size-cells == %d in /\n\r", nsize);

	i = 0;
	if (naddr == 2)
		memreg[i++] = start >> 32;
	memreg[i++] = start & 0xffffffff;
	if (nsize == 2)
		memreg[i++] = size >> 32;
	memreg[i++] = size & 0xffffffff;

	memory = finddevice("/memory");
	if (! memory) {
		memory = create_node(NULL, "memory");
		setprop_str(memory, "device_type", "memory");
	}

	printf("Memory <- <0x%x", memreg[0]);
	for (i = 1; i < (naddr + nsize); i++)
		printf(" 0x%x", memreg[i]);
	printf("> (%ldMB)\n\r", (unsigned long)(size >> 20));

	setprop(memory, "reg", memreg, (naddr + nsize)*sizeof(u32));
}

#define MHZ(x)	((x + 500000) / 1000000)

void dt_fixup_cpu_clocks(u32 cpu, u32 tb, u32 bus)
{
	void *devp = NULL;

	printf("CPU clock-frequency <- 0x%x (%dMHz)\n\r", cpu, MHZ(cpu));
	printf("CPU timebase-frequency <- 0x%x (%dMHz)\n\r", tb, MHZ(tb));
	if (bus > 0)
		printf("CPU bus-frequency <- 0x%x (%dMHz)\n\r", bus, MHZ(bus));

	while ((devp = find_node_by_devtype(devp, "cpu"))) {
		setprop_val(devp, "clock-frequency", cpu);
		setprop_val(devp, "timebase-frequency", tb);
		if (bus > 0)
			setprop_val(devp, "bus-frequency", bus);
	}
}

void dt_fixup_clock(const char *path, u32 freq)
{
	void *devp = finddevice(path);

	if (devp) {
		printf("%s: clock-frequency <- %x (%dMHz)\n\r", path, freq, MHZ(freq));
		setprop_val(devp, "clock-frequency", freq);
	}
}

void __dt_fixup_mac_addresses(u32 startindex, ...)
{
	va_list ap;
	u32 index = startindex;
	void *devp;
	const u8 *addr;

	va_start(ap, startindex);
	while ((addr = va_arg(ap, const u8 *))) {
		devp = find_node_by_prop_value(NULL, "linux,network-index",
					       (void*)&index, sizeof(index));

		printf("ENET%d: local-mac-address <-"
		       " %02x:%02x:%02x:%02x:%02x:%02x\n\r", index,
		       addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

		if (devp)
			setprop(devp, "local-mac-address", addr, 6);

		index++;
	}
	va_end(ap);
}

#define MAX_ADDR_CELLS 4
#define MAX_RANGES 8

static void get_reg_format(void *node, u32 *naddr, u32 *nsize)
{
	if (getprop(node, "#address-cells", naddr, 4) != 4)
		*naddr = 2;
	if (getprop(node, "#size-cells", nsize, 4) != 4)
		*nsize = 1;
}

static void copy_val(u32 *dest, u32 *src, int naddr)
{
	memset(dest, 0, (MAX_ADDR_CELLS - naddr) * 4);
	memcpy(dest, src, naddr * 4);
}

static int sub_reg(u32 *reg, u32 *sub)
{
	int i, borrow = 0;

	for (i = 0; i < MAX_ADDR_CELLS; i++) {
		int prev_borrow = borrow;
		borrow = reg[i] < sub[i] + prev_borrow;
		reg[i] -= sub[i] + prev_borrow;
	}

	return !borrow;
}

static int add_reg(u32 *reg, u32 *add)
{
	int i, carry = 0;

	for (i = 0; i < MAX_ADDR_CELLS; i++) {
		u64 tmp = (u64)reg[i] + add[i] + carry;
		carry = tmp >> 32;
		reg[i] = (u32)tmp;
	}

	return !carry;
}

/* It is assumed that if the first byte of reg fits in a
 * range, then the whole reg block fits.
 */
static int compare_reg(u32 *reg, u32 *range, u32 *rangesize)
{
	int i;
	u32 end;

	for (i = 0; i < MAX_ADDR_CELLS; i++) {
		if (reg[i] < range[i])
			return 0;
		if (reg[i] > range[i])
			break;
	}

	for (i = 0; i < MAX_ADDR_CELLS; i++) {
		end = range[i] + rangesize[i];

		if (reg[i] < end)
			break;
		if (reg[i] > end)
			return 0;
	}

	return reg[i] != end;
}

/* reg must be MAX_ADDR_CELLS */
static int find_range(u32 *reg, u32 *ranges, int nregaddr,
                      int naddr, int nsize, int buflen)
{
	int nrange = nregaddr + naddr + nsize;
	int i;

	for (i = 0; i + nrange <= buflen; i += nrange) {
		u32 range_addr[MAX_ADDR_CELLS];
		u32 range_size[MAX_ADDR_CELLS];

		copy_val(range_addr, ranges + i, naddr);
		copy_val(range_size, ranges + i + nregaddr + naddr, nsize);

		if (compare_reg(reg, range_addr, range_size))
			return i;
	}

	return -1;
}

/* Currently only generic buses without special encodings are supported.
 * In particular, PCI is not supported.  Also, only the beginning of the
 * reg block is tracked; size is ignored except in ranges.
 */
int dt_xlate_reg(void *node, int res, unsigned long *addr,
                 unsigned long *size)
{
	u32 last_addr[MAX_ADDR_CELLS];
	u32 this_addr[MAX_ADDR_CELLS];
	u32 buf[MAX_ADDR_CELLS * MAX_RANGES * 3];
	void *parent;
	u64 ret_addr, ret_size;
	u32 naddr, nsize, prev_naddr;
	int buflen, offset;

	parent = get_parent(node);
	if (!parent)
		return 0;

	get_reg_format(parent, &naddr, &nsize);

	if (nsize > 2)
		return 0;

	buflen = getprop(node, "reg", buf, sizeof(buf)) / 4;
	offset = (naddr + nsize) * res;

	if (buflen < offset + naddr + nsize)
		return 0;

	copy_val(last_addr, buf + offset, naddr);

	ret_size = buf[offset + naddr];
	if (nsize == 2) {
		ret_size <<= 32;
		ret_size |= buf[offset + naddr + 1];
	}

	while ((node = get_parent(node))) {
		prev_naddr = naddr;

		get_reg_format(node, &naddr, &nsize);

		buflen = getprop(node, "ranges", buf, sizeof(buf));
		if (buflen < 0)
			continue;
		if (buflen > sizeof(buf))
			return 0;

		offset = find_range(last_addr, buf, prev_naddr,
		                    naddr, nsize, buflen / 4);

		if (offset < 0)
			return 0;

		copy_val(this_addr, buf + offset, prev_naddr);

		if (!sub_reg(last_addr, this_addr))
			return 0;

		copy_val(this_addr, buf + offset + prev_naddr, naddr);

		if (!add_reg(last_addr, this_addr))
			return 0;
	}

	if (naddr > 2)
		return 0;

	ret_addr = last_addr[0];
	if (naddr == 2) {
		ret_addr <<= 32;
		ret_addr |= last_addr[1];
	}

	if (sizeof(void *) == 4 &&
	    (ret_addr >= 0x100000000ULL || ret_size > 0x100000000ULL ||
	     ret_addr + ret_size > 0x100000000ULL))
		return 0;

	*addr = ret_addr;
	if (size)
		*size = ret_size;

	return 1;
}
