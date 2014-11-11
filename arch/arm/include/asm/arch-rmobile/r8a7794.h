/*
 * arch/arm/include/asm/arch-rmobile/r8a7794.h
 *
 * Copyright (C) 2014 Renesas Electronics Corporation
 *
 * SPDX-License-Identifier: GPL-2.0
*/

#ifndef __ASM_ARCH_R8A7794_H
#define __ASM_ARCH_R8A7794_H

#include "rcar-base.h"

/* SH-I2C */
#define CONFIG_SYS_I2C_SH_BASE2	0xE60B0000

/* SDHI */
#define CONFIG_SYS_SH_SDHI1_BASE 0xEE140000
#define CONFIG_SYS_SH_SDHI2_BASE 0xEE160000
#define CONFIG_SYS_SH_SDHI_NR_CHANNEL 3

#endif /* __ASM_ARCH_R8A7794_H */
