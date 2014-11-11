/*
 * arch/arm/include/asm/arch-rmobile/r8a7790.h
 *
 * Copyright (C) 2013,2014 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: GPL-2.0
*/

#ifndef __ASM_ARCH_R8A7790_H
#define __ASM_ARCH_R8A7790_H

#include "rcar-base.h"

/* SH-I2C */
#define CONFIG_SYS_I2C_SH_BASE2	0xE6520000
#define CONFIG_SYS_I2C_SH_BASE3	0xE60B0000

/* SDHI */
#define CONFIG_SYS_SH_SDHI1_BASE 0xEE120000
#define CONFIG_SYS_SH_SDHI2_BASE 0xEE140000
#define CONFIG_SYS_SH_SDHI3_BASE 0xEE160000
#define CONFIG_SYS_SH_SDHI_NR_CHANNEL 4

#define R8A7790_CUT_ES2X	2
#define IS_R8A7790_ES2()	\
	(rmobile_get_cpu_rev_integer() == R8A7790_CUT_ES2X)

#endif /* __ASM_ARCH_R8A7790_H */
