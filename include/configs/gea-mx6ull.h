/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright (C) 2016 Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 *
 * Configuration settings for the Freescale i.MX6UL 14x14 EVK board.
 */
#ifndef __MICROGEA_CONFIG_H
#define __MICROGEA_CONFIG_H


#include <asm/arch/imx-regs.h>
#include <linux/sizes.h>
#include "mx6_common.h"
#include <asm/mach-imx/gpio.h>
#include "imx_env.h"


#undef CONFIG_LDO_BYPASS_CHECK

/* Size of malloc() pool */
#define CONFIG_SYS_MALLOC_LEN		(16 * SZ_1M)

#define MAX_SDRAM_SIZE			0x20000000  /* Maximum 512MB for GEA M6UL */

#define CONFIG_MXC_UART
#define CONFIG_MXC_UART_BASE		UART1_BASE

/* MMC Configs */
#ifdef CONFIG_FSL_USDHC
#define CONFIG_SYS_FSL_ESDHC_ADDR	USDHC2_BASE_ADDR

/* NAND pin conflicts with usdhc2 */
#ifdef CONFIG_NAND_MXS
	#define CONFIG_SYS_FSL_USDHC_NUM	1
#else
	#define CONFIG_SYS_FSL_USDHC_NUM	2
#endif
#endif

/* I2C configs */
#ifdef CONFIG_CMD_I2C
#define CONFIG_SYS_I2C_MXC
#define CONFIG_SYS_I2C_MXC_I2C1		/* enable I2C bus 1 */
#define CONFIG_SYS_I2C_MXC_I2C2		/* enable I2C bus 2 */
#define CONFIG_SYS_I2C_SPEED		100000
#endif

#define CONFIG_SYS_MMC_IMG_LOAD_PART	1

#define CONFIG_CMD_READ
#define CONFIG_SERIAL_TAG
#define CONFIG_FASTBOOT_USB_DEV 0

#define BOOTCMD_FROM_NET	 "run bootargs_net; tftp uImage; tftp ${fdt_addr} uImage.dtb; bootz ${loadaddr} - ${fdt_addr} \0"

#ifdef CONFIG_NAND_BOOT
	#define BOOTCMD		"bootcmd=run bootcmd_ubi\0"
#else
	#define BOOTCMD		"bootcmd=run bootcmd_mmc\0"
#endif

#define CONFIG_EXTRA_ENV_SETTINGS \
		"bootargs_base=setenv bootargs_tmp console=ttymxc0,115200\0"			\
		"bootargs_mmc=run bootargs_base; setenv bootargs ${bootargs_tmp} ${mtdparts} root=/dev/mmcblk${mmcdev}p2 rootwait rw\0" \
		"bootcmd_mmc=setenv mmcdev 0; run bootargs_mmc; run loadfdt; run loadzImage; bootz ${loadaddr} - ${fdt_addr}\0"	\
		"bootargs_net=run bootargs_base; setenv bootargs ${bootargs_tmp} root=/dev/nfs ip=dhcp nfsroot=${serverip}:${nfsroot},v3,tcp\0" \
		"bootcmd_net="   "run bootargs_net; tftp uImage; tftp ${fdt_addr} uImage.dtb; bootz ${loadaddr} - ${fdt_addr}\0"	\
        "bootargs_ubi=run bootargs_base; setenv bootargs ${bootargs_tmp} ${mtdparts} ubi.mtd=3 root=ubi0:rootfs rootfstype=ubifs\0"	\
        "bootcmd_ubi=run bootargs_ubi;nand read ${loadaddr} 0x400000 0x800000;nand read ${fdt_addr} 0xc00000 0x100000;bootz ${loadaddr} - ${fdt_addr}\0" \
		BOOTCMD	\
		"loadfdt=fatload mmc ${mmcdev}:${mmcpart} ${fdt_addr} ${fdt_file}\0" \
		"mmcpart=1\0"					\
		"loadzImage=fatload mmc ${mmcdev}:${mmcpart} ${loadaddr} zImage;\0"	\
		"mtdparts=mtdparts=gpmi-nand:4m(boot),8m(kernel),1m(dtb),-(rootfs)\0" \
		"fdt_file=" CONFIG_DEFAULT_FDT_FILE "\0" 	\
		"netdev=eth0\0" \
		"ethprime=FEC0\0" \
		"nfsroot=/nfs_icore\0"	\
		"fdt_addr=0x83000000\0"	\
		"fdt_high=0xffffffff\0"


/* Miscellaneous configurable options */
#define CONFIG_SYS_MEMTEST_START	0x80000000
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + 0x8000000)

#define CONFIG_SYS_LOAD_ADDR		CONFIG_LOADADDR
#define CONFIG_SYS_HZ			1000

/* Physical Memory Map */
#define PHYS_SDRAM			MMDC0_ARB_BASE_ADDR

#define CONFIG_SYS_SDRAM_BASE		PHYS_SDRAM
#define CONFIG_SYS_INIT_RAM_ADDR	IRAM_BASE_ADDR
#define CONFIG_SYS_INIT_RAM_SIZE	IRAM_SIZE

#define CONFIG_SYS_INIT_SP_OFFSET \
	(CONFIG_SYS_INIT_RAM_SIZE - GENERATED_GBL_DATA_SIZE)
#define CONFIG_SYS_INIT_SP_ADDR \
	(CONFIG_SYS_INIT_RAM_ADDR + CONFIG_SYS_INIT_SP_OFFSET)

/* environment organization */
#define CONFIG_SYS_MMC_ENV_DEV		0	/* USDHC2 */
#define CONFIG_SYS_MMC_ENV_PART		0	/* user area */
#define CONFIG_MMCROOT			"/dev/mmcblk0p2"  /* USDHC2 */

#define CONFIG_IOMUX_LPSR

#ifdef CONFIG_FSL_QSPI
#define CONFIG_SYS_FSL_QSPI_AHB
#define FSL_QSPI_FLASH_NUM		1
#define FSL_QSPI_FLASH_SIZE		SZ_32M
#endif

/* NAND stuff */
#ifdef CONFIG_NAND_MXS
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_SYS_NAND_BASE		0x40000000
#define CONFIG_SYS_NAND_5_ADDR_CYCLE
#define CONFIG_SYS_NAND_ONFI_DETECTION
#define CONFIG_SYS_NAND_USE_FLASH_BBT

/* DMA stuff, needed for GPMI/MXS NAND support */
#endif

#undef CONFIG_ENV_SIZE
#undef CONFIG_ENV_OFFSET
#define CONFIG_ENV_SIZE			SZ_8K
#if defined(CONFIG_ENV_IS_IN_MMC)
#define CONFIG_ENV_OFFSET		(14 * SZ_64K)
#elif defined(CONFIG_ENV_IS_IN_SPI_FLASH)
#define CONFIG_ENV_OFFSET		(896 * 1024)
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_SPI_BUS		CONFIG_SF_DEFAULT_BUS
#define CONFIG_ENV_SPI_CS		CONFIG_SF_DEFAULT_CS
#define CONFIG_ENV_SPI_MODE		CONFIG_SF_DEFAULT_MODE
#define CONFIG_ENV_SPI_MAX_HZ		CONFIG_SF_DEFAULT_SPEED
#elif defined(CONFIG_ENV_IS_IN_NAND)
#undef CONFIG_ENV_SIZE
#define CONFIG_ENV_OFFSET		(0x1c0000)
#define CONFIG_ENV_SECT_SIZE		(0x20000)
#define CONFIG_ENV_SIZE			CONFIG_ENV_SECT_SIZE
#endif

/* USB Configs */
#ifdef CONFIG_CMD_USB
#define CONFIG_EHCI_HCD_INIT_AFTER_RESET
#define CONFIG_MXC_USB_PORTSC  (PORT_PTS_UTMI | PORT_PTS_PTW)
#define CONFIG_MXC_USB_FLAGS   0
#define CONFIG_USB_MAX_CONTROLLER_COUNT 2
#endif

#define CONFIG_CMD_MII
#define IMX_FEC_BASE			ENET_BASE_ADDR
#define CONFIG_FEC_MXC_PHYADDR          0x0
#define CONFIG_FEC_XCV_TYPE             RMII
#define CONFIG_ETHPRIME			"FEC0"
#define CONFIG_FEC_MXC_MDIO_BASE ENET_BASE_ADDR
#define CONFIG_FEC_ENET_DEV 0

#define CONFIG_MODULE_FUSE
#define CONFIG_OF_SYSTEM_SETUP


#define CONFIG_IMX_THERMAL


#define CONFIG_MODULE_FUSE
#define CONFIG_OF_SYSTEM_SETUP

#endif
