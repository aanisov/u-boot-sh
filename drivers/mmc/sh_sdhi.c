/*
 * drivers/mmc/sh_sdhi.c
 *
 * SD/MMC driver for Renesas rmobile ARM SoCs.
 *
 * Copyright (C) 2011,2013-2014 Renesas Electronics Corporation
 * Copyright (C) 2014 Nobuhiro Iwamatsu <nobuhiro.iwamatsu.yj@renesas.com>
 * Copyright (C) 2008-2009 Renesas Solutions Corp.
 *
 * SPDX-License-Identifier:	GPL-2.0
 */

#include <common.h>
#include <malloc.h>
#include <mmc.h>
#include <asm/errno.h>
#include <asm/io.h>
#include <asm/arch/rmobile.h>
#include <asm/arch/sh_sdhi.h>

#define DRIVER_NAME "sh-sdhi"

struct sh_sdhi_host {
	unsigned long	addr;
	int		ch;
	int		bus_shift;
};

static unsigned short g_wait_int[CONFIG_SYS_SH_SDHI_NR_CHANNEL];
static unsigned short g_sd_error[CONFIG_SYS_SH_SDHI_NR_CHANNEL];

static inline void sh_sdhi_writew(struct sh_sdhi_host *host, int reg, u16 val)
{
	writew(val, host->addr + (reg << host->bus_shift));
}

static inline u16 sh_sdhi_readw(struct sh_sdhi_host *host, int reg)
{
	return readw(host->addr + (reg << host->bus_shift));
}

static void *mmc_priv(struct mmc *mmc)
{
	return (void *)mmc->priv;
}

static int detect_waiting;
static void sh_sdhi_detect(struct sh_sdhi_host *host)
{
	sh_sdhi_writew(host, SDHI_OPTION,
		       OPT_BUS_WIDTH_1 | sh_sdhi_readw(host, SDHI_OPTION));

	detect_waiting = 0;
}

static int sh_sdhi_intr(void *dev_id)
{
	struct sh_sdhi_host *host = dev_id;
	int state1 = 0, state2 = 0;

	state1 = sh_sdhi_readw(host, SDHI_INFO1);
	state2 = sh_sdhi_readw(host, SDHI_INFO2);

	debug("%s: state1 = %x, state2 = %x\n", __func__, state1, state2);

	/* CARD Insert */
	if (state1 & INFO1_CARD_IN) {
		sh_sdhi_writew(host, SDHI_INFO1, ~INFO1_CARD_IN);
		if (!detect_waiting) {
			detect_waiting = 1;
			sh_sdhi_detect(host);
		}
		sh_sdhi_writew(host, SDHI_INFO1_MASK, INFO1M_RESP_END |
			INFO1M_ACCESS_END | INFO1M_CARD_IN |
			INFO1M_DATA3_CARD_RE | INFO1M_DATA3_CARD_IN);
		return -EAGAIN;
	}
	/* CARD Removal */
	if (state1 & INFO1_CARD_RE) {
		sh_sdhi_writew(host, SDHI_INFO1, ~INFO1_CARD_RE);
		if (!detect_waiting) {
			detect_waiting = 1;
			sh_sdhi_detect(host);
		}
		sh_sdhi_writew(host, SDHI_INFO1_MASK, INFO1M_RESP_END |
				INFO1M_ACCESS_END | INFO1M_CARD_RE |
				INFO1M_DATA3_CARD_RE | INFO1M_DATA3_CARD_IN);
		sh_sdhi_writew(host, SDHI_SDIO_INFO1_MASK, SDIO_INFO1M_ON);
		sh_sdhi_writew(host, SDHI_SDIO_MODE, SDIO_MODE_OFF);
		return -EAGAIN;
	}

	if (state2 & INFO2_ALL_ERR) {
		sh_sdhi_writew(host, SDHI_INFO2,
			       (unsigned short)~(INFO2_ALL_ERR));
		sh_sdhi_writew(host, SDHI_INFO2_MASK,
			       INFO2M_ALL_ERR |
			       sh_sdhi_readw(host, SDHI_INFO2_MASK));
		g_sd_error[host->ch] = 1;
		g_wait_int[host->ch] = 1;
		return 0;
	}
	/* Respons End */
	if (state1 & INFO1_RESP_END) {
		sh_sdhi_writew(host, SDHI_INFO1, ~INFO1_RESP_END);
		sh_sdhi_writew(host, SDHI_INFO1_MASK,
			       INFO1M_RESP_END |
			       sh_sdhi_readw(host, SDHI_INFO1_MASK));
		g_wait_int[host->ch] = 1;
		return 0;
	}
	/* SD_BUF Read Enable */
	if (state2 & INFO2_BRE_ENABLE) {
		sh_sdhi_writew(host, SDHI_INFO2, ~INFO2_BRE_ENABLE);
		sh_sdhi_writew(host, SDHI_INFO2_MASK,
			       INFO2M_BRE_ENABLE | INFO2M_BUF_ILL_READ |
			       sh_sdhi_readw(host, SDHI_INFO2_MASK));
		g_wait_int[host->ch] = 1;
		return 0;
	}
	/* SD_BUF Write Enable */
	if (state2 & INFO2_BWE_ENABLE) {
		sh_sdhi_writew(host, SDHI_INFO2, ~INFO2_BWE_ENABLE);
		sh_sdhi_writew(host, SDHI_INFO2_MASK,
			       INFO2_BWE_ENABLE | INFO2M_BUF_ILL_WRITE |
			       sh_sdhi_readw(host, SDHI_INFO2_MASK));
		g_wait_int[host->ch] = 1;
		return 0;
	}
	/* Access End */
	if (state1 & INFO1_ACCESS_END) {
		sh_sdhi_writew(host, SDHI_INFO1, ~INFO1_ACCESS_END);
		sh_sdhi_writew(host, SDHI_INFO1_MASK,
			       INFO1_ACCESS_END |
			       sh_sdhi_readw(host, SDHI_INFO1_MASK));
		g_wait_int[host->ch] = 1;
		return 0;
	}
	return -EAGAIN;
}

static int sh_sdhi_wait_interrupt_flag(struct sh_sdhi_host *host)
{
	int timeout = 10000000;

	while (1) {
		timeout--;
		if (timeout < 0) {
			puts("timeout\n");
			return 0;
		}

		if (!sh_sdhi_intr(host))
			break;

		udelay(1);	/* 1 usec */
	}

	return 1; /* Return value: NOT 0 = complete waiting */
}

static void sh_sdhi_clock_control(struct sh_sdhi_host *host, unsigned long clk)
{
	u32 clkdiv, i;

	if (sh_sdhi_readw(host, SDHI_INFO2) & (1 << 14)) {
		printf(DRIVER_NAME": Busy state ! Cannot change the clock\n");
		return;
	}

	sh_sdhi_writew(host, SDHI_CLK_CTRL,
		       ~CLK_ENABLE & sh_sdhi_readw(host, SDHI_CLK_CTRL));

	if (clk == 0)
		return;

	clkdiv = 0x80;
	i = CONFIG_SH_SDHI_FREQ >> (0x8 + 1);
	for (; clkdiv && clk >= (i << 1); (clkdiv >>= 1))
		i <<= 1;

	sh_sdhi_writew(host, SDHI_CLK_CTRL, clkdiv);

	/* Waiting for SD Bus busy to be cleared */
	while ((sh_sdhi_readw(host, SDHI_INFO2) & 0x2000) == 0)
		;

	sh_sdhi_writew(host, SDHI_CLK_CTRL,
		       CLK_ENABLE | sh_sdhi_readw(host, SDHI_CLK_CTRL));
}

static void sh_sdhi_sync_reset(struct sh_sdhi_host *host)
{
	sh_sdhi_writew(host, SDHI_SOFT_RST, SOFT_RST_ON);
	sh_sdhi_writew(host, SDHI_SOFT_RST, SOFT_RST_OFF);
	sh_sdhi_writew(host, SDHI_CLK_CTRL,
		       CLK_ENABLE | sh_sdhi_readw(host, SDHI_CLK_CTRL));

	while (sh_sdhi_readw(host, SDHI_INFO2) & INFO2_CBUSY)
		udelay(100);

#if defined(CONFIG_R8A7790)
	if (host->ch < 2)
		sh_sdhi_writew(host, SDHI_HOST_MODE, 1); /* 16bit access */
#elif defined(CONFIG_R8A7791) || defined(CONFIG_R8A7793) || \
	defined(CONFIG_R8A7790)
	if (host->ch == 0)
		sh_sdhi_writew(host, SDHI_HOST_MODE, 1); /* 16bit access */
#endif
}

static int sh_sdhi_error_manage(struct sh_sdhi_host *host)
{
	unsigned short e_state1, e_state2;
	int ret;

	g_sd_error[host->ch] = 0;
	g_wait_int[host->ch] = 0;

	e_state1 = sh_sdhi_readw(host, SDHI_ERR_STS1);
	e_state2 = sh_sdhi_readw(host, SDHI_ERR_STS2);
	if (e_state2 & ERR_STS2_SYS_ERROR) {
		if (e_state2 & ERR_STS2_RES_STOP_TIMEOUT)
			ret = TIMEOUT;
		else
			ret = -EILSEQ;
		debug("%s: ERR_STS2 = %04x\n",
		      DRIVER_NAME, sh_sdhi_readw(host, SDHI_ERR_STS2));
		sh_sdhi_sync_reset(host);
		sh_sdhi_writew(host, SDHI_INFO1_MASK,
			       INFO1M_DATA3_CARD_RE | INFO1M_DATA3_CARD_IN);
		return ret;
	}
	if (e_state1 & ERR_STS1_CRC_ERROR || e_state1 & ERR_STS1_CMD_ERROR)
		ret = -EILSEQ;
	else
		ret = TIMEOUT;

	debug("%s: ERR_STS1 = %04x\n",
	      DRIVER_NAME, sh_sdhi_readw(host, SDHI_ERR_STS1));
	sh_sdhi_sync_reset(host);
	sh_sdhi_writew(host, SDHI_INFO1_MASK,
		       INFO1M_DATA3_CARD_RE | INFO1M_DATA3_CARD_IN);
	return ret;
}

static int sh_sdhi_single_read(struct sh_sdhi_host *host, struct mmc_data *data)
{
	int ch = host->ch;
	long time;
	unsigned short blocksize, i;
	unsigned short *p = (unsigned short *)data->dest;

	if ((unsigned long)p & 0x00000001) {
		printf(DRIVER_NAME": %s: The data pointer is unaligned.",
		       __func__);
		return -EIO;
	}

	g_wait_int[ch] = 0;
	sh_sdhi_writew(host, SDHI_INFO2_MASK,
		       ~(INFO2M_BRE_ENABLE | INFO2M_BUF_ILL_READ) &
		       sh_sdhi_readw(host, SDHI_INFO2_MASK));
	sh_sdhi_writew(host, SDHI_INFO1_MASK,
		       ~INFO1M_ACCESS_END &
		       sh_sdhi_readw(host, SDHI_INFO1_MASK));
	time = sh_sdhi_wait_interrupt_flag(host);
	if (time == 0 || g_sd_error[ch] != 0)
		return sh_sdhi_error_manage(host);

	g_wait_int[ch] = 0;
	blocksize = sh_sdhi_readw(host, SDHI_SIZE);
	for (i = 0; i < blocksize / 2; i++)
		*p++ = sh_sdhi_readw(host, SDHI_BUF0);

	time = sh_sdhi_wait_interrupt_flag(host);
	if (time == 0 || g_sd_error[ch] != 0)
		return sh_sdhi_error_manage(host);

	g_wait_int[ch] = 0;
	return 0;
}

static int sh_sdhi_multi_read(struct sh_sdhi_host *host, struct mmc_data *data)
{
	int ch = host->ch;
	long time;
	unsigned short blocksize, i, sec;
	unsigned short *p = (unsigned short *)data->dest;

	if ((unsigned long)p & 0x00000001) {
		printf(DRIVER_NAME": %s: The data pointer is unaligned.",
		       __func__);
		return -EIO;
	}

	debug("%s: blocks = %d, blocksize = %d\n",
	      __func__, data->blocks, data->blocksize);

	g_wait_int[ch] = 0;
	for (sec = 0; sec < data->blocks; sec++) {
		sh_sdhi_writew(host, SDHI_INFO2_MASK,
			       ~(INFO2M_BRE_ENABLE | INFO2M_BUF_ILL_READ) &
			       sh_sdhi_readw(host, SDHI_INFO2_MASK));

		time = sh_sdhi_wait_interrupt_flag(host);
		if (time == 0 || g_sd_error[ch] != 0)
			return sh_sdhi_error_manage(host);

		g_wait_int[ch] = 0;
		blocksize = sh_sdhi_readw(host, SDHI_SIZE);
		for (i = 0; i < blocksize / 2; i++)
			*p++ = sh_sdhi_readw(host, SDHI_BUF0);
	}

	return 0;
}

static int sh_sdhi_single_write(struct sh_sdhi_host *host,
		struct mmc_data *data)
{
	int ch = host->ch;
	long time;
	unsigned short blocksize, i;
	const unsigned short *p = (const unsigned short *)data->src;

	if ((unsigned long)p & 0x00000001) {
		printf(DRIVER_NAME": %s: The data pointer is unaligned.",
		       __func__);
		return -EIO;
	}

	debug("%s: blocks = %d, blocksize = %d\n",
	      __func__, data->blocks, data->blocksize);

	g_wait_int[ch] = 0;
	sh_sdhi_writew(host, SDHI_INFO2_MASK,
		       ~(INFO2M_BWE_ENABLE | INFO2M_BUF_ILL_WRITE) &
		       sh_sdhi_readw(host, SDHI_INFO2_MASK));
	sh_sdhi_writew(host, SDHI_INFO1_MASK,
		       ~INFO1M_ACCESS_END &
		       sh_sdhi_readw(host, SDHI_INFO1_MASK));

	time = sh_sdhi_wait_interrupt_flag(host);
	if (time == 0 || g_sd_error[ch] != 0)
		return sh_sdhi_error_manage(host);

	g_wait_int[ch] = 0;
	blocksize = sh_sdhi_readw(host, SDHI_SIZE);
	for (i = 0; i < blocksize / 2; i++)
		sh_sdhi_writew(host, SDHI_BUF0, *p++);

	time = sh_sdhi_wait_interrupt_flag(host);
	if (time == 0 || g_sd_error[ch] != 0)
		return sh_sdhi_error_manage(host);

	g_wait_int[ch] = 0;
	return 0;
}

static int sh_sdhi_multi_write(struct sh_sdhi_host *host, struct mmc_data *data)
{
	int ch = host->ch;
	long time;
	unsigned short i, sec, blocksize;
	const unsigned short *p = (const unsigned short *)data->src;

	debug("%s: blocks = %d, blocksize = %d\n",
	      __func__, data->blocks, data->blocksize);

	g_wait_int[ch] = 0;
	for (sec = 0; sec < data->blocks; sec++) {
		sh_sdhi_writew(host, SDHI_INFO2_MASK,
			       ~(INFO2M_BWE_ENABLE | INFO2M_BUF_ILL_WRITE) &
			       sh_sdhi_readw(host, SDHI_INFO2_MASK));

		time = sh_sdhi_wait_interrupt_flag(host);
		if (time == 0 || g_sd_error[ch] != 0)
			return sh_sdhi_error_manage(host);

		g_wait_int[ch] = 0;
		blocksize = sh_sdhi_readw(host, SDHI_SIZE);
		for (i = 0; i < blocksize / 2; i++)
			sh_sdhi_writew(host, SDHI_BUF0, *p++);
	}

	return 0;
}

static void sh_sdhi_get_response(struct sh_sdhi_host *host, struct mmc_cmd *cmd)
{
	unsigned short i, j, cnt = 1;
	volatile unsigned short resp[8];
	volatile unsigned long *p1, *p2;

	if (cmd->resp_type & MMC_RSP_136) {
		cnt = 4;
		resp[0] = sh_sdhi_readw(host, SDHI_RSP00);
		resp[1] = sh_sdhi_readw(host, SDHI_RSP01);
		resp[2] = sh_sdhi_readw(host, SDHI_RSP02);
		resp[3] = sh_sdhi_readw(host, SDHI_RSP03);
		resp[4] = sh_sdhi_readw(host, SDHI_RSP04);
		resp[5] = sh_sdhi_readw(host, SDHI_RSP05);
		resp[6] = sh_sdhi_readw(host, SDHI_RSP06);
		resp[7] = sh_sdhi_readw(host, SDHI_RSP07);

		/* SDHI REGISTER SPECIFICATION */
		for (i = 7, j = 6; i > 0; i--) {
			resp[i] = (resp[i] << 8) & 0xff00;
			resp[i] |= (resp[j--] >> 8) & 0x00ff;
		}
		resp[0] = (resp[0] << 8) & 0xff00;

		/* SDHI REGISTER SPECIFICATION */
		p1 = ((unsigned long *)resp) + 3;

	} else {
		resp[0] = sh_sdhi_readw(host, SDHI_RSP00);
		resp[1] = sh_sdhi_readw(host, SDHI_RSP01);

		p1 = ((unsigned long *)resp);
	}

	p2 = (unsigned long *)cmd->response;
#if defined(__BIG_ENDIAN_BITFIELD)
	for (i = 0; i < cnt; i++) {
		*p2++ = ((*p1 >> 16) & 0x0000ffff) |
				((*p1 << 16) & 0xffff0000);
		p1--;
	}
#else
	for (i = 0; i < cnt; i++)
		*p2++ = *p1--;
#endif /* __BIG_ENDIAN_BITFIELD */
}

static unsigned short sh_sdhi_set_cmd(struct sh_sdhi_host *host,
			struct mmc_data *data, unsigned short opc)
{
	switch (opc) {
	case SD_CMD_APP_SEND_OP_COND:
	case SD_CMD_APP_SEND_SCR:
		opc |= SDHI_APP;
		break;
	case SD_CMD_APP_SET_BUS_WIDTH:
		 /* SD_APP_SET_BUS_WIDTH*/
		if (!data)
			opc |= SDHI_APP;
		else /* SD_SWITCH */
			opc = SDHI_SD_SWITCH;
		break;
	default:
		break;
	}
	return opc;
}

static unsigned short sh_sdhi_data_trans(struct sh_sdhi_host *host,
			struct mmc_data *data, unsigned short opc)
{
	unsigned short ret;

	switch (opc) {
	case MMC_CMD_READ_MULTIPLE_BLOCK:
		ret = sh_sdhi_multi_read(host, data);
		break;
	case MMC_CMD_WRITE_MULTIPLE_BLOCK:
		ret = sh_sdhi_multi_write(host, data);
		break;
	case MMC_CMD_WRITE_SINGLE_BLOCK:
		ret = sh_sdhi_single_write(host, data);
		break;
	case MMC_CMD_READ_SINGLE_BLOCK:
	case SDHI_SD_APP_SEND_SCR:
	case SDHI_SD_SWITCH: /* SD_SWITCH */
		ret = sh_sdhi_single_read(host, data);
		break;
	default:
		printf(DRIVER_NAME": SD: NOT SUPPORT CMD = d'%04d\n", opc);
		ret = -EINVAL;
		break;
	}
	return ret;
}

static int sh_sdhi_start_cmd(struct sh_sdhi_host *host,
			struct mmc_data *data, struct mmc_cmd *cmd)
{
	long time;
	unsigned short opc = cmd->cmdidx;
	int ret = 0;

	debug("opc = %d, arg = %x, resp_type = %x\n",
	      opc, cmd->cmdarg, cmd->resp_type);

	if (opc == MMC_CMD_STOP_TRANSMISSION) {
		/* SDHI sends the STOP command automatically by STOP reg */
		sh_sdhi_writew(host, SDHI_INFO1_MASK, ~INFO1M_ACCESS_END &
				sh_sdhi_readw(host, SDHI_INFO1_MASK));

		time = sh_sdhi_wait_interrupt_flag(host);
		if (time == 0 || g_sd_error[host->ch] != 0)
			return sh_sdhi_error_manage(host);

		sh_sdhi_get_response(host, cmd);
		return 0;
	}

	if (data) {
		if ((opc == MMC_CMD_READ_MULTIPLE_BLOCK) ||
		    opc == MMC_CMD_WRITE_MULTIPLE_BLOCK) {
			sh_sdhi_writew(host, SDHI_STOP, STOP_SEC_ENABLE);
			sh_sdhi_writew(host, SDHI_SECCNT, data->blocks);
		}
		sh_sdhi_writew(host, SDHI_SIZE, data->blocksize);
	}
	opc = sh_sdhi_set_cmd(host, data, opc);

	/*
	 *  U-boot cannot use interrupt.
	 *  So this flag may not be clear by timing
	 */
	sh_sdhi_writew(host, SDHI_INFO1, ~INFO1_RESP_END);

	sh_sdhi_writew(host, SDHI_INFO1_MASK,
		       INFO1M_RESP_END | sh_sdhi_readw(host, SDHI_INFO1_MASK));
	sh_sdhi_writew(host, SDHI_ARG0,
		       (unsigned short)(cmd->cmdarg & ARG0_MASK));
	sh_sdhi_writew(host, SDHI_ARG1,
		       (unsigned short)((cmd->cmdarg >> 16) & ARG1_MASK));

	/* Waiting for SD Bus busy to be cleared */
	while ((sh_sdhi_readw(host, SDHI_INFO2) & 0x2000) == 0)
		;

	sh_sdhi_writew(host, SDHI_CMD, (unsigned short)(opc & CMD_MASK));

	g_wait_int[host->ch] = 0;
	sh_sdhi_writew(host, SDHI_INFO1_MASK,
		       ~INFO1M_RESP_END & sh_sdhi_readw(host, SDHI_INFO1_MASK));
	sh_sdhi_writew(host, SDHI_INFO2_MASK,
		       ~(INFO2M_CMD_ERROR | INFO2M_CRC_ERROR |
		       INFO2M_END_ERROR | INFO2M_TIMEOUT |
		       INFO2M_RESP_TIMEOUT | INFO2M_ILA) &
		       sh_sdhi_readw(host, SDHI_INFO2_MASK));

	time = sh_sdhi_wait_interrupt_flag(host);
	if (!time)
		return sh_sdhi_error_manage(host);

	if (g_sd_error[host->ch]) {
		switch (cmd->cmdidx) {
		case MMC_CMD_ALL_SEND_CID:
		case MMC_CMD_SELECT_CARD:
		case SD_CMD_SEND_IF_COND:
		case MMC_CMD_APP_CMD:
			ret = TIMEOUT;
			break;
		default:
			printf(DRIVER_NAME": Cmd(d'%d) err\n", opc);
			printf(DRIVER_NAME": cmdidx = %d\n", cmd->cmdidx);
			ret = sh_sdhi_error_manage(host);
			break;
		}
		g_sd_error[host->ch] = 0;
		g_wait_int[host->ch] = 0;
		return ret;
	}
	if (sh_sdhi_readw(host, SDHI_INFO1) & INFO1_RESP_END)
		return -EINVAL;

	if (g_wait_int[host->ch]) {
		sh_sdhi_get_response(host, cmd);
		g_wait_int[host->ch] = 0;
	}
	if (data)
		ret = sh_sdhi_data_trans(host, data, opc);

	debug("ret = %d, resp = %08x, %08x, %08x, %08x\n",
	      ret, cmd->response[0], cmd->response[1],
	      cmd->response[2], cmd->response[3]);
	return ret;
}

static int sh_sdhi_send_cmd(struct mmc *mmc, struct mmc_cmd *cmd,
			struct mmc_data *data)
{
	struct sh_sdhi_host *host = mmc_priv(mmc);
	int ret;

	g_sd_error[host->ch] = 0;

	ret = sh_sdhi_start_cmd(host, data, cmd);

	return ret;
}

static void sh_sdhi_set_ios(struct mmc *mmc)
{
	struct sh_sdhi_host *host = mmc_priv(mmc);

	sh_sdhi_clock_control(host, mmc->clock);

	if (mmc->bus_width == 4)
		sh_sdhi_writew(host, SDHI_OPTION, ~OPT_BUS_WIDTH_1 &
			       sh_sdhi_readw(host, SDHI_OPTION));
	else
		sh_sdhi_writew(host, SDHI_OPTION, OPT_BUS_WIDTH_1 |
			       sh_sdhi_readw(host, SDHI_OPTION));

	debug("clock = %d, buswidth = %d\n", mmc->clock, mmc->bus_width);
}

static int sh_sdhi_initialize(struct mmc *mmc)
{
	struct sh_sdhi_host *host = mmc_priv(mmc);

	sh_sdhi_sync_reset(host);
	sh_sdhi_writew(host, SDHI_PORTSEL, USE_1PORT);

#if defined(__BIG_ENDIAN_BITFIELD)
	sh_sdhi_writew(host, SDHI_EXT_SWAP, SET_SWAP);
#endif

	sh_sdhi_writew(host, SDHI_INFO1_MASK, INFO1M_RESP_END |
		       INFO1M_ACCESS_END | INFO1M_CARD_RE |
		       INFO1M_DATA3_CARD_RE | INFO1M_DATA3_CARD_IN);

	return 0;
}

static const struct mmc_ops sh_sdhi_ops = {
	.send_cmd       = sh_sdhi_send_cmd,
	.set_ios        = sh_sdhi_set_ios,
	.init           = sh_sdhi_initialize,
};

static struct mmc_config sh_sdhi_cfg = {
	.name           = DRIVER_NAME,
	.ops            = &sh_sdhi_ops,
	.f_min          = CLKDEV_INIT,
	.f_max          = CLKDEV_HS_DATA,
	.voltages       = MMC_VDD_32_33 | MMC_VDD_33_34,
	.host_caps      = MMC_MODE_4BIT | MMC_MODE_HS,
	.part_type      = PART_TYPE_DOS,
	.b_max          = CONFIG_SYS_MMC_MAX_BLK_COUNT,
};

int sh_sdhi_init(unsigned long addr, int ch)
{
	int ret = 0;
	struct mmc *mmc;
	struct sh_sdhi_host *host = NULL;

	if (ch >= CONFIG_SYS_SH_SDHI_NR_CHANNEL)
		return -ENODEV;

	host = malloc(sizeof(struct sh_sdhi_host));
	if (!host)
		return -ENOMEM;

	mmc = mmc_create(&sh_sdhi_cfg, host);
	if (!mmc) {
		ret = -1;
		goto error;
	}

	host->ch = ch;
	host->addr = addr;
#if defined(CONFIG_R8A7790)
	if (ch < 2)
		host->bus_shift = 1;
	else
		host->bus_shift = 0;
#elif defined(CONFIG_R8A7791) || defined(CONFIG_R8A7793) || \
	defined(CONFIG_R8A7794)
	if (ch == 0)
		host->bus_shift = 1;
	else
		host->bus_shift = 0;
#endif
	return ret;
error:
	if (host)
		free(host);
	return ret;
}

