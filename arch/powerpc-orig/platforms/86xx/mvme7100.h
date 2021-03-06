/*
 * MVME7100 board definitions
 *
 * Copyright 2008 Emerson Network Power Embedded Computing
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 * Author: Ajit Prem <ajit.prem@emerson.com>
 */

#ifndef __MVME7100_H__
#define __MVME7100_H__

#include <linux/init.h>

#define MPC86XX_RSTCR_OFFSET	(0xe00b0)	/* Reset Control Register */

/* Flash */
#define MVME7100_FLASH_BASE	0xf8000000
#define MVME7100_FLASH_SIZE	0x08000000


/* System Control and Status Registers */
#define MVME7100_SYSTEM_STATUS_REG		0xF2000000
#define MVME7100_SYSTEM_CONTROL_REG		0xF2000001
#define MVME7100_STATUS_INDICATOR_REG		0xF2000002
#define MVME7100_NOR_FLASH_CTRL_STAT_REG	0xF2000003
#define MVME7100_INTERRUPT_REG_1		0xF2000004
#define MVME7100_INTERRUPT_REG_2		0xF2000005
#define MVME7100_PRESENCE_DETECT_REG		0xF2000006
#define MVME7100_NAND_FLASH1_CONTROL_REG	0xF2000010
#define MVME7100_NAND_FLASH1_SELECT_REG		0xF2000011
#define MVME7100_NAND_FLASH1_PRESENCE_REG	0xF2000014
#define MVME7100_NAND_FLASH1_STATUS_REG		0xF2000015
#define MVME7100_NAND_FLASH1_DATA_REG		0xF2030000
#define MVME7100_NAND_FLASH2_CONTROL_REG	0xF2000018
#define MVME7100_NAND_FLASH2_SELECT_REG		0xF2000019
#define MVME7100_NAND_FLASH2_PRESENCE_REG	0xF200001C
#define MVME7100_NAND_FLASH2_STATUS_REG		0xF200001D
#define MVME7100_NAND_FLASH2_DATA_REG		0xF2031000
#define MVME7100_WATCHDOG_TIMER_LOAD_REG	0xF2000020
#define MVME7100_WATCHDOG_CONTROL_REG		0xF2000024
#define MVME7100_WATCHDOG_TIMER_RESOLUTION_REG	0xF2000025
#define MVME7100_WATCHDOG_TIMER_COUNT_REG	0xF2000026
#define MVME7100_PLD_REVISION_REG		0xF2000030
#define MVME7100_PLD_DATE_CODE_REG		0xF2000034
#define MVME7100_TEST_1_REG			0xF2000038
#define MVME7100_TEST_2_REG			0xF200003C

/* Register offsets */
#define MVME7100_SYSTEM_STATUS_REG_OFFSET		0x00000000
#define MVME7100_SYSTEM_CONTROL_REG_OFFSET		0x00000001
#define MVME7100_STATUS_INDICATOR_REG_OFFSET		0x00000002
#define MVME7100_NOR_FLASH_CTRL_STAT_REG_OFFSET		0x00000003
#define MVME7100_INTERRUPT_REG_1_OFFSET			0x00000004
#define MVME7100_INTERRUPT_REG_2_OFFSET			0x00000005
#define MVME7100_PRESENCE_DETECT_REG_OFFSET		0x00000006
#define MVME7100_NAND_FLASH1_CTRL_REG_OFFSET		0x00000010
#define MVME7100_NAND_FLASH1_SELECT_REG_OFFSET		0x00000011
#define MVME7100_NAND_FLASH1_PRESENCE_REG_OFFSET	0x00000014
#define MVME7100_NAND_FLASH1_STATUS_REG_OFFSET		0x00000015
#define MVME7100_NAND_FLASH2_CTRL_REG_OFFSET		0x00000018
#define MVME7100_NAND_FLASH2_SELECT_REG_OFFSET		0x00000019
#define MVME7100_NAND_FLASH2_PRESENCE_REG_OFFSET	0x0000001C
#define MVME7100_NAND_FLASH2_STATUS_REG_OFFSET		0x0000001D
#define MVME7100_WATCHDOG_TIMER_LOAD_REG_OFFSET		0x00000020
#define MVME7100_WATCHDOG_CONTROL_REG_OFFSET		0x00000024
#define MVME7100_WATCHDOG_TIMER_RESOLUTION_REG_OFFSET	0x00000025
#define MVME7100_WATCHDOG_TIMER_COUNT_REG_OFFSET	0x00000026
#define MVME7100_PLD_REVISION_REG_OFFSET		0x00000030
#define MVME7100_PLD_DATE_CODE_REG_OFFSET		0x00000034
#define MVME7100_TEST_1_REG_OFFSET			0x00000038
#define MVME7100_TEST_2_REG_OFFSET			0x0000003C

/* System Status Register */
#define MVME7100_STATE_SW8		0x80
#define MVME7100_STATE_SW7		0x40
#define MVME7100_STATE_SW6		0x20
#define MVME7100_STATE_SW5		0x10
#define MVME7100_SAFE_START		0x08
#define MVME7100_PEX8525_ERROR		0x04
#define MVME7100_BOARD_TYPE_MASK	0x03
#define MVME7100_BOARD_TYPE_PRPMC	0x01
#define MVME7100_BOARD_TYPE_VME		0x00

/* System Control Register */
#define MVME7100_BOARD_RESET		0xA0
#define MVME7100_EEPROM_WP		0x02

/* Status Indicator Register */
#define MVME7100_USR1_RED_LED		0x01
#define MVME7100_USR1_YELLOW_LED	0x02
#define MVME7100_USR2_LED		0x04
#define MVME7100_USR3_LED		0x08

/* NOR Flash Control/Status Register */
#define MVME7100_NOR_FLASH_MAP_SELECT	0x10
#define MVME7100_NOR_FLASH_WP_SW	0x08
#define MVME7100_NOR_FLASH_WP_HW	0x04
#define MVME7100_NOR_FLASH_BLK_SEL	0x02
#define MVME7100_NOR_FLASH_RDY		0x01

/* Interrupt Register 1 */
#define MVME7100_TSEC4_PHY_INTERRUPT	0x08
#define MVME7100_TSEC3_PHY_INTERRUPT	0x04
#define MVME7100_TSEC2_PHY_INTERRUPT	0x02
#define MVME7100_TSEC1_PHY_INTERRUPT	0x01

/* Interrupt Register 2 */
#define MVME7100_RTC_MASK		0x40
#define MVME7100_TEMP_MASK		0x20
#define MVME7100_ABORT_MASK		0x10
#define MVME7100_RTC_STATUS		0x04
#define MVME7100_TEMP_STATUS		0x02
#define MVME7100_ABORT_STATUS		0x01

/* Presence Detect Register */
#define MVME7100_ERDY2			0x20
#define MVME7100_ERDY1			0x10
#define MVME7100_XMCSPAN_PRESENT	0x04
#define MVME7100_PMC2_PRESENT		0x02
#define MVME7100_PMC1_PRESENT		0x01

/* NAND Flash Chip Control Register */
#define MVME7100_NAND_FLASH_CLE		0x80
#define MVME7100_NAND_FLASH_ALE		0x40
#define MVME7100_NAND_FLASH_WP		0x20

/* NAND Flash Chip Select Register */
#define MVME7100_NAND_FLASH_CE1		0x80
#define MVME7100_NAND_FLASH_CE2		0x40
#define MVME7100_NAND_FLASH_CE3		0x20
#define MVME7100_NAND_FLASH_CE4		0x10

/* NAND Flash Chip Presence Register */
#define MVME7100_NAND_FLASH_CP		0x80

/* NAND Flash Chip Status Register */
#define MVME7100_NAND_FLASH_RB1		0x80
#define MVME7100_NAND_FLASH_RB2		0x40
#define MVME7100_NAND_FLASH_RB3		0x20
#define MVME7100_NAND_FLASH_RB4		0x10

#define MVME7100_BASE_BAUD		1843200
#define QUART_BASE_BAUD			(MVME7100_BASE_BAUD / 16)
#define MVME7100_UART_SIZE		0x8

#define MVME7100_SERIAL_1	0xF2011000U
#define MVME7100_SERIAL_2	0xF2012000U
#define MVME7100_SERIAL_3	0xF2013000U
#define MVME7100_SERIAL_4	0xF2014000U

#endif	/* __MVME7100_H__ */
