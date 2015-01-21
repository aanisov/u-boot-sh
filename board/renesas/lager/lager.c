/*
 * board/renesas/lager/lager.c
 *     This file is lager board support.
 *
 * Copyright (C) 2013 Renesas Electronics Corporation
 * Copyright (C) 2013 Nobuhiro Iwamatsu <nobuhiro.iwamatsu.yj@renesas.com>
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#include <common.h>
#include <malloc.h>
#include <netdev.h>
#include <asm/processor.h>
#include <asm/mach-types.h>
#include <asm/io.h>
#include <asm/errno.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/arch/rmobile.h>
#include <miiphy.h>
#include <i2c.h>
#include "qos.h"

DECLARE_GLOBAL_DATA_PTR;

#ifdef CONFIG_ARMV7_VIRT
extern void shmobile_boot_vector(void);
extern unsigned long shmobile_boot_size;
#endif

#define CLK2MHZ(clk)	(clk / 1000 / 1000)
void s_init(void)
{
	struct rcar_rwdt *rwdt = (struct rcar_rwdt *)RWDT_BASE;
	struct rcar_swdt *swdt = (struct rcar_swdt *)SWDT_BASE;

	/* Watchdog init */
	writel(0xA5A5A500, &rwdt->rwtcsra);
	writel(0xA5A5A500, &swdt->swtcsra);

	/* CPU frequency setting. Set to 1.4GHz */
	if (rmobile_get_cpu_rev_integer() >= R8A7790_CUT_ES2X) {
		u32 stc = ((1400 / CLK2MHZ(CONFIG_SYS_CLK_FREQ)) - 1)
			<< PLL0_STC_BIT;
		clrsetbits_le32(PLL0CR, PLL0_STC_MASK, stc);
	}

	/* QoS(Quality-of-Service) Init */
	qos_init();
}

#define MSTPSR1	0xE6150038
#define SMSTPCR1	0xE6150134
#define TMU0_MSTP125	(1 << 25)

#define MSTPSR7	0xE61501C4
#define SMSTPCR7	0xE615014C
#define SCIF0_MSTP721	(1 << 21)

#define MSTPSR8	0xE61509A0
#define SMSTPCR8	0xE6150990
#define ETHER_MSTP813	(1 << 13)

#define mstp_setbits(type, addr, saddr, set) \
	out_##type((saddr), in_##type(addr) | (set))
#define mstp_clrbits(type, addr, saddr, clear) \
	out_##type((saddr), in_##type(addr) & ~(clear))
#define mstp_setbits_le32(addr, saddr, set)	\
		mstp_setbits(le32, addr, saddr, set)
#define mstp_clrbits_le32(addr, saddr, clear)	\
		mstp_clrbits(le32, addr, saddr, clear)

int board_early_init_f(void)
{
	/* TMU0 */
	mstp_clrbits_le32(MSTPSR1, SMSTPCR1, TMU0_MSTP125);
	/* SCIF0 */
	mstp_clrbits_le32(MSTPSR7, SMSTPCR7, SCIF0_MSTP721);
	/* ETHER */
	mstp_clrbits_le32(MSTPSR8, SMSTPCR8, ETHER_MSTP813);

	return 0;
}

void arch_preboot_os(void)
{
	/* Disable TMU0 */
	mstp_setbits_le32(MSTPSR1, SMSTPCR1, TMU0_MSTP125);
}

#ifdef CONFIG_ARMV7_VIRT
#define TIMER_BASE                  0xE6080000
#define TIMER_CNTCR                 0x0
#define TIMER_CNTFID0               0x20
#define MODEMR                      0xE6160060
#define BIT(x)                      (1 << (x))
#define MD(nr)                      BIT(nr)

static int shmobile_init_time(void)
{
    uint32_t freq;
    int extal_mhz = 0;
    unsigned int mode = readl(MODEMR);

    /* At Linux boot time the r8a7790 arch timer comes up
     * with the counter disabled. Moreover, it may also report
     * a potentially incorrect fixed 13 MHz frequency. To be
     * correct these registers need to be updated to use the
     * frequency EXTAL / 2 which can be determined by the MD pins.
     */

    switch ( mode & (MD(14) | MD(13)) ) {
    case 0:
        extal_mhz = 15;
        break;
    case MD(13):
        extal_mhz = 20;
        break;
    case MD(14):
        extal_mhz = 26;
        break;
    case MD(13) | MD(14):
        extal_mhz = 30;
        break;
    }

    /* The arch timer frequency equals EXTAL / 2 */
    freq = extal_mhz * (1000000 / 2);

    /*
     * Update the timer if it is either not running, or is not at the
     * right frequency. The timer is only configurable in secure mode
     * so this avoids an abort if the loader started the timer and
     * entered the kernel in non-secure mode.
     */

    if ( (readl(TIMER_BASE + TIMER_CNTCR) & 1) == 0 ||
            readl(TIMER_BASE + TIMER_CNTFID0) != freq ) {
        /* Update registers with correct frequency */
        writel(freq, TIMER_BASE + TIMER_CNTFID0);
        asm volatile("mcr p15, 0, %0, c14, c0, 0" : : "r" (freq));

       /* make sure arch timer is started by setting bit 0 of CNTCR */
        writel(1, TIMER_BASE + TIMER_CNTCR);
    }
    return 0;
}
#endif

DECLARE_GLOBAL_DATA_PTR;
int board_init(void)
{
	/* adress of boot parameters */
	gd->bd->bi_boot_params = LAGER_SDRAM_BASE + 0x100;

	/* Init PFC controller */
	r8a7790_pinmux_init();

	/* ETHER Enable */
	gpio_request(GPIO_FN_ETH_CRS_DV, NULL);
	gpio_request(GPIO_FN_ETH_RX_ER, NULL);
	gpio_request(GPIO_FN_ETH_RXD0, NULL);
	gpio_request(GPIO_FN_ETH_RXD1, NULL);
	gpio_request(GPIO_FN_ETH_LINK, NULL);
	gpio_request(GPIO_FN_ETH_REF_CLK, NULL);
	gpio_request(GPIO_FN_ETH_MDIO, NULL);
	gpio_request(GPIO_FN_ETH_TXD1, NULL);
	gpio_request(GPIO_FN_ETH_TX_EN, NULL);
	gpio_request(GPIO_FN_ETH_MAGIC, NULL);
	gpio_request(GPIO_FN_ETH_TXD0, NULL);
	gpio_request(GPIO_FN_ETH_MDC, NULL);
	gpio_request(GPIO_FN_IRQ0, NULL);

	gpio_request(GPIO_GP_5_31, NULL);	/* PHY_RST */
	gpio_direction_output(GPIO_GP_5_31, 0);
	mdelay(20);
	gpio_set_value(GPIO_GP_5_31, 1);
	udelay(1);

#ifdef CONFIG_ARMV7_VIRT
	/* init timer */
	shmobile_init_time();
#endif

	return 0;
}

#define CXR24 0xEE7003C0 /* MAC address high register */
#define CXR25 0xEE7003C8 /* MAC address low register */
int board_eth_init(bd_t *bis)
{
	int ret = -ENODEV;

#ifdef CONFIG_SH_ETHER
	u32 val;
	unsigned char enetaddr[6];

	ret = sh_eth_initialize(bis);
	if (!eth_getenv_enetaddr("ethaddr", enetaddr))
		return ret;

	/* Set Mac address */
	val = enetaddr[0] << 24 | enetaddr[1] << 16 |
	    enetaddr[2] << 8 | enetaddr[3];
	writel(val, CXR24);

	val = enetaddr[4] << 8 | enetaddr[5];
	writel(val, CXR25);

#endif

	return ret;
}

/* lager has KSZ8041NL/RNL */
#define PHY_CONTROL1	0x1E
#define PHY_LED_MODE	0xC0000
#define PHY_LED_MODE_ACK	0x4000
int board_phy_config(struct phy_device *phydev)
{
	int ret = phy_read(phydev, MDIO_DEVAD_NONE, PHY_CONTROL1);
	ret &= ~PHY_LED_MODE;
	ret |= PHY_LED_MODE_ACK;
	ret = phy_write(phydev, MDIO_DEVAD_NONE, PHY_CONTROL1, (u16)ret);

	return 0;
}

int dram_init(void)
{
	gd->bd->bi_dram[0].start = CONFIG_SYS_SDRAM_BASE;
	gd->ram_size = CONFIG_SYS_SDRAM_SIZE;

	return 0;
}

const struct rmobile_sysinfo sysinfo = {
	CONFIG_RMOBILE_BOARD_STRING
};

void dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = LAGER_SDRAM_BASE;
	gd->bd->bi_dram[0].size = LAGER_SDRAM_SIZE;
}

int board_late_init(void)
{
	return 0;
}

void reset_cpu(ulong addr)
{
	u8 val;

	i2c_set_bus_num(3); /* PowerIC connected to ch3 */
	i2c_init(400000, 0);
	i2c_read(CONFIG_SYS_I2C_POWERIC_ADDR, 0x13, 1, &val, 1);
	val |= 0x02;
	i2c_write(CONFIG_SYS_I2C_POWERIC_ADDR, 0x13, 1, &val, 1);
}

#ifdef CONFIG_ARMV7_VIRT
#define r8a7790_clst_id(cpu) ((cpu & 4) > 0 ? 1 : 0)
#define r8a7790_cpu_id(cpu) ((cpu) & 0x3)
#define LAGER_APMU_BASE                        0xE6150000
#define LAGER_APMU_CA15WUPCR_OFFSET            0x2010
#define LAGER_APMU_CA15CPUCMCR_OFFSET          0x2184
#define LAGER_APMU_CA7WUPCR_OFFSET             0x1010
#define LAGER_APMU_CA7CPUCMCR_OFFSET           0x1184
#define LAGER_RST_BASE                         0xE6160000
#define LAGER_RST_CA15BAR_OFFSET               0x20
#define LAGER_RST_CA7BAR_OFFSET                0x30
#define LAGER_RST_CA15BAR_BAREN                (1 << 4)
#define LAGER_RST_CA7BAR_BAREN                 (1 << 4)
#define LAGER_RST_CA15RESCNT_OFFSET            0x40
#define LAGER_RST_CA7RESCNT_OFFSET             0x44
#define LAGER_XEN_INIT_SECONDARY_START         0xE63C0FFC
#define LAGER_RST_BASE                         0xE6160000
#define LAGER_RST_CA15BAR                      0xE6160020
#define LAGER_RST_CA7BAR                       0xE6160030
#define LAGER_LAGER_RAM                        0xE63C0000
#define LAGER_MAX_CPUS                         4


enum { R8A7790_CLST_CA15, R8A7790_CLST_CA7, R8A7790_CLST_NR };
static struct {
	unsigned int wupcr;
	unsigned int bar;
	unsigned int rescnt;
	unsigned int rescnt_magic;
} r8a7790_clst[R8A7790_CLST_NR] = {
	[R8A7790_CLST_CA15] = {
		.wupcr = LAGER_APMU_CA15WUPCR_OFFSET,
		.bar = LAGER_RST_CA15BAR_OFFSET,
		.rescnt = LAGER_RST_CA15RESCNT_OFFSET,
		.rescnt_magic = 0xa5a50000,
	},
	[R8A7790_CLST_CA7] = {
		.wupcr = LAGER_APMU_CA7WUPCR_OFFSET,
		.bar = LAGER_RST_CA7BAR_OFFSET,
		.rescnt = LAGER_RST_CA7RESCNT_OFFSET,
		.rescnt_magic = 0x5a5a0000,
	},
};

static void assert_reset(unsigned int cpu)
{
	void *rescnt;
	u32 mask, magic;
	unsigned int clst_id = r8a7790_clst_id(cpu);

	/* disable per-core clocks */
	mask = BIT(3 - r8a7790_cpu_id(cpu));
	magic = r8a7790_clst[clst_id].rescnt_magic;
	rescnt = (void *) (LAGER_RST_BASE + r8a7790_clst[clst_id].rescnt);
	writel((readl(rescnt) | mask) | magic, rescnt);
}

static void deassert_reset(unsigned int cpu)
{
	void *rescnt;
	u32 mask, magic;
	unsigned int clst_id = r8a7790_clst_id(cpu);

	/* enable per-core clocks */
	mask = BIT(3 - r8a7790_cpu_id(cpu));
	magic = r8a7790_clst[clst_id].rescnt_magic;
	rescnt = (void *) (LAGER_RST_BASE + r8a7790_clst[clst_id].rescnt);
	writel((readl(rescnt) & ~mask) | magic, rescnt);
}

static void power_on(unsigned int cpu)
{
	void *cawupcr;
	unsigned int clst_id = r8a7790_clst_id(cpu);

	cawupcr = (void *) (LAGER_APMU_BASE + r8a7790_clst[clst_id].wupcr);
	writel(BIT(r8a7790_cpu_id(cpu)), cawupcr);

	/* wait for APMU to finish */
	while (readl(cawupcr) != 0);
}

void smp_kick_all_cpus(void)
{
	int i;
	for (i = 1; i < LAGER_MAX_CPUS; i++)
	{
		assert_reset(i);
		power_on(i);
		deassert_reset(i);
	}
}

void smp_set_core_boot_addr(unsigned long addr, int corenr)
{

	void __iomem *p;
	unsigned long *f;
	unsigned long bar;

	p = (void __iomem*) LAGER_LAGER_RAM;
	memcpy_toio(p, shmobile_boot_vector, shmobile_boot_size);
	f = (void __iomem *)((long unsigned)p + shmobile_boot_size - 4);
	*((unsigned long *) f) = addr;

	bar = (LAGER_LAGER_RAM >> 8) & 0xfffffc00;

	writel(bar, LAGER_RST_CA15BAR);
	writel(bar | 0x10, LAGER_RST_CA15BAR);
	writel(bar, LAGER_RST_CA7BAR);
	writel(bar | 0x10, LAGER_RST_CA7BAR);

	f = (unsigned long *)(LAGER_XEN_INIT_SECONDARY_START);
	*f = 0;

	/* make sure this write is really executed */
	__asm__ volatile ("dsb\n");
}


asm(".arm \n"
	".align 2 \n"
	".global smp_waitloop \n"
	"smp_waitloop: \n"
	"1: 	wfe \n"
	"ldr 	r0, =0xE63C0FFC \n"
	"ldr	r0, [r0] \n"
	"teq	r0, #0x0 \n"
	"beq 	1b \n"

	"b		_do_nonsec_entry \n"
	".type smp_waitloop, %function \n"
	".size smp_waitloop, .-smp_waitloop \n");

asm(
	".arm \n"
	".globl shmobile_boot_vector \n"
	".align 2 \n"
	"shmobile_boot_vector: \n"
	"ldr    pc, 1f \n"
	".type shmobile_boot_vector, %function \n"
	".size shmobile_boot_vector, .-shmobile_boot_vector \n"
    ".align	2 \n"
		"func:\n"
"1:	.space	4 \n"
	".globl	shmobile_boot_size \n"
"shmobile_boot_size: \n"
	".long	.-shmobile_boot_vector \n");
#endif
