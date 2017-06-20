/*
 * mcp805.h
 * 
 * Definitions for the Motorola MCP805 board
 *
 * Author: Ajit Prem <Ajit.Prem@motorola.com>
 *
 * Copyright 2002-2007 Motorola Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */
 /*
    * From Processor to PCI:
    *   PCI Mem Space: 0xd0000000 - 0xefffffff -> 0xd0000000 - 0xefffffff (512 MB)
    *   PCI I/O Space: 0xfe000000 - 0xfe7fffff -> 0x00000000 - 0x00800000 (8 MB)
    *
    * From PCI to Processor:
    *   System Memory: 0x00000000 -> 0x00000000
  */


#ifndef __MCP805_H
#define __MCP805_H

#define MCP805_PCI_CONFIG_1_ADDR		0xfe000cf8U
#define MCP805_PCI_CONFIG_1_DATA		0xfe000cfcU
#define MCP805_PCI_CONFIG_2_ADDR		0xfe400cf8U
#define MCP805_PCI_CONFIG_2_DATA		0xfe400cfcU

#define MCP805_PROC_PCI_IO_START		0xfe000000U
#define MCP805_PROC_PCI_IO_END			0xfe7fffffU
#define MCP805_PROC_PCI_IO_1_START		0xfe000000U
#define MCP805_PROC_PCI_IO_1_END		0xfe3fffffU
#define MCP805_PROC_PCI_IO_2_START		0xfe400000U
#define MCP805_PROC_PCI_IO_2_END		0xfe7fffffU

#define MCP805_PCI_IO_1_START			0x00000000U
#define MCP805_PCI_IO_1_END			0x003fffffU
#define MCP805_PCI_IO_2_START			0x00400000U
#define MCP805_PCI_IO_2_END			0x007fffffU

#define MCP805_PROC_PCI_MEM_START		0xd0000000U
#define MCP805_PROC_PCI_MEM_END			0xefffffffU
#define MCP805_PROC_PCI_MEM_1_START		0xd0000000U
#define MCP805_PROC_PCI_MEM_1_END		0xdfffffffU
#define MCP805_PROC_PCI_MEM_2_START		0xe0000000U
#define MCP805_PROC_PCI_MEM_2_END		0xefffffffU

#define MCP805_PCI_MEM_1_START			0xd0000000U
#define MCP805_PCI_MEM_1_END			0xdfffffffU
#define MCP805_PCI_MEM_2_START			0xe0000000U
#define MCP805_PCI_MEM_2_END			0xefffffffU

#define MCP805_PCI_DRAM_OFFSET			0x00000000U
#define MCP805_PCI_PHY_MEM_OFFSET		0x00000000U

#define MCP805_ISA_IO_BASE			MCP805_PROC_PCI_IO_START
#define MCP805_ISA_MEM_BASE			0x00000000U

#define MCP805_HARRIER_1_XCSR_BASE		0xfeff0000U
#define MCP805_HARRIER_1_MPIC_BASE		0xff000000U

#define MCP805_HARRIER_2_XCSR_BASE		0xfeff1000U
#define MCP805_HARRIER_2_MPIC_BASE		0xff040000U

#define MCP805_HARRIER_1_SERIAL_1		0xfeff00c0U
#define MCP805_HARRIER_1_SERIAL_2		0xfeff00c8U

#define MCP805_HARRIER_2_SERIAL_1		0xfeff10c0U
#define MCP805_HARRIER_2_SERIAL_2		0xfeff10c8U

#define MCP805_BASE_BAUD			1843200

#define	MCP805_NVRAM_BASE_ADDRESS		0xff110000U
#define	MCP805_NVRAM_SIZE			0x8000

#define MCP805_GEO_ADDR_REG			0xff100003
#define MCP805_GEO_ADDR_MASK			0x1f

#ifdef CONFIG_SERIAL_MANY_PORTS
#define RS_TABLE_SIZE  64
#else
#define RS_TABLE_SIZE  4
#endif

/* Rate for the 1.8432 Mhz clock for the onboard serial chip */
#define BASE_BAUD (MCP805_BASE_BAUD / 16)

#define MCP805_HARRIER_1_SERIAL_1_IRQ	16
#define MCP805_HARRIER_1_SERIAL_2_IRQ	16

#ifdef CONFIG_SERIAL_DETECT_IRQ
#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF|ASYNC_SKIP_TEST|ASYNC_AUTO_IRQ)
#else
#define STD_COM_FLAGS (ASYNC_BOOT_AUTOCONF|ASYNC_SKIP_TEST)
#endif

/* Harrier A UARTS are at IRQ 16  */
#define STD_SERIAL_PORT_DFNS \
        { 0, BASE_BAUD, MCP805_HARRIER_1_SERIAL_1, 16, STD_COM_FLAGS, /* ttyS0 */\
		iomem_base: (unsigned char *)MCP805_HARRIER_1_SERIAL_1,	\
		iomem_reg_shift: 0,					\
		io_type: SERIAL_IO_MEM },				\
        { 0, BASE_BAUD, MCP805_HARRIER_1_SERIAL_2, 16, STD_COM_FLAGS, /* ttyS1 */\
		iomem_base: (unsigned char *)MCP805_HARRIER_1_SERIAL_2,		\
		iomem_reg_shift: 0,					\
		io_type: SERIAL_IO_MEM },				\
        { 0, BASE_BAUD, MCP805_HARRIER_2_SERIAL_1, 33, STD_COM_FLAGS, /* ttyS2 */\
		iomem_base: (unsigned char *)MCP805_HARRIER_2_SERIAL_1,		\
		iomem_reg_shift: 0,					\
		io_type: SERIAL_IO_MEM },				\
        { 0, BASE_BAUD, MCP805_HARRIER_2_SERIAL_2, 33, STD_COM_FLAGS, /* ttyS3 */\
		iomem_base: (unsigned char *)MCP805_HARRIER_2_SERIAL_2,		\
		iomem_reg_shift: 0,					\
		io_type: SERIAL_IO_MEM },				

#define SERIAL_PORT_DFNS \
        STD_SERIAL_PORT_DFNS

#endif				/* __MCP805_H */
