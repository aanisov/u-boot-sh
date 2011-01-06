/*
 * (C) Copyright 2008
 * Texas Instruments, <www.ti.com>
 * Syed Mohammed Khasim <khasim@ti.com>
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation's version 2 of
 * the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <config.h>
#include <common.h>
#include <fat.h>
#include <mmc.h>
#include <part.h>
#include <i2c.h>
#include <asm/io.h>
#undef SECTOR_SIZE
#include <ext2fs.h>

#define RETRY_NUM	12

#define MMC_CMD_INIT	0x40

#define CMD_INITIALIZE	0
#define CMD_ENABLE	1
#define	CMD_CHECK	8
#define	CMD_CSD		9
#define	CMD_CID		10
#define CMD_READ	17
#define CMD_MREAD	18
#define CMD_WRITE	24
#define CMD_MWRITE	25
#define ACMD_COND	41
#define CMD_APP		55
#define CMD_OCD		58

#define HCS	0x40000000

#define LOW_SPD		144
#define MID_SPD		32
#define MMC_TIME_OVER	1500

// SMR
#define	SCI_CA	0x80
#define	SCI_CHR	0x40
#define	SCI_PE	0x20
#define	SCI_OE	0x10
#define	SCI_STOP	0x08
#define	SCI_MP	0x04

// SCR
#define	SCI_TIE	0x80
#define	SCI_RIE	0x40
#define	SCI_TE	0x20
#define	SCI_RE	0x10
#define	SCI_MPIE	0x08
#define	SCI_TEIE	0x04
#define SCK_OUT	0
#define SCK_IN	2

// SSR
#define	SCI_TDRE	0x80
#define	SCI_RDRF	0x40
#define	SCI_ORER	0x20
#define	SCI_FER	0x10
#define	SCI_PER	0x08
#define	SCI_TEND	0x04
#define	SCI_MPB	0x02
#define	SCI_MPBT	0x01

#define PGCR   0xA400010C /* 16 bit EJECT */
#define PGDR   0xA400012C /* 8  bit EJECT */
#define SCPCR  0xA4000116 /* 16 bit SCI */
#define SCPDR  0xA4000136 /* 8  bit SCI */
#define SMR0   0xFFFFFE80 /* 8  bit SCI */
#define BRR0   0xFFFFFE82 /* 8  bit SCI */
#define SCR0   0xFFFFFE84 /* 8  bit SCI */
#define TDR0   0xFFFFFE86 /* 8  bit SCI */
#define SSR0   0xFFFFFE88 /* 8  bit SCI */
#define RDR0   0xFFFFFE8A /* 8  bit SCI */
#define SCMR0  0xFFFFFE8C /* 8  bit SCI */

#define SCPCR_CLK_MASK	0x000C
#define SCPCR_CLK_IN	0x000C
#define SCPCR_CLK_OUT	0x0004
#define SCPDR_CLK	0x02
#define SCPCR_DAT_MASK	0x0003
#define SCPCR_DAT_IN	0x0003
#define SCPCR_DAT_OUT	0x0001
#define SCPDR_DAT	0x01
#define SCPCR_CMD_MASK	0x0030
#define SCPCR_CMD_IN	0x0030
#define SCPCR_CMD_OUT	0x0010
#define SCPDR_CMD	0x04
#define V1_SCPCR_CS_MASK	0x00C0
#define V1_SCPCR_CS_IN	0x00C0
#define V1_SCPCR_CS_OUT	0x0040
#define V1_SCPDR_CS	0x08
#define V1_PGCR_EJECT	0x0300
#define V1_PGDR_EJECT	0x10
#define V2_SCPCR_CS_MASK	0x000C
#define V2_SCPCR_CS_OUT	0x0004
#define V2_SCPDR_CS	0x02
#define V2_SCPCR_EJECT	0x00C0
#define V2_SCPDR_EJECT	0x08

#define MMC_COUNT	1
#define DISK_COUNT	8

static block_dev_desc_t mmc_blk_dev;
static char	board_id;
#define	SH7706LANV1	0
#define	SH7706LANV2	1
#define	SH7706LANV3	3
#define	SH7706USB	2

#define ctrl_inw readw
#define ctrl_outw writew
#define ctrl_outb writeb
#define ctrl_inb readb

#define READ 0x02
#define BLOCK_SIZE 512
#define MMCSD_SECTOR_SIZE 512

static int disksize;
static int mmc_blocksize;

block_dev_desc_t *mmc_get_dev(int dev)
{
	return (block_dev_desc_t *) &mmc_blk_dev;
}


static void sci_putch(int c) {
	int	w;

	w = (ctrl_inw(SCPCR) & ~SCPCR_CLK_MASK) | SCPCR_CLK_OUT; ctrl_outw(w, SCPCR);
	ctrl_outb(SCI_TE + SCK_OUT, SCR0);
	while((ctrl_inb(SSR0) & SCI_TDRE) == 0);
	w = ctrl_inw(SCPCR) & ~SCPCR_CLK_MASK; ctrl_outw(w, SCPCR);
	ctrl_outb(c, TDR0);
	w = ctrl_inb(SSR0);
	ctrl_outb(0, SSR0);
}

static void sci_putch_sw(void) {
	int	w;

	while(!(ctrl_inb(SSR0) & SCI_TEND));

	w = ctrl_inw(SCPCR) | SCPCR_CLK_IN; ctrl_outw(w, SCPCR);
	w = ctrl_inb(SCR0) | SCK_IN; ctrl_outb(w, SCR0);
	w = ctrl_inw(SCPCR) & ~SCPCR_CLK_MASK; ctrl_outw(w, SCPCR);
	ctrl_outb(0, SMR0);
	ctrl_outb(SCK_OUT, SCR0);
	ctrl_outb(0, SSR0);
	ctrl_outb(SCI_CA, SMR0);
}

static int sci_getch(void) {
	int	w, c;

#if defined(CONFIG_CPU_SH2)
	w = (ctrl_inw(SCPCR) & ~SCPCR_CLK_MASK) | SCPCR_CLK_OUT; ctrl_outw(w, SCPCR);
#endif
	ctrl_outb(SCI_RE + SCK_OUT, SCR0);
	if(ctrl_inb(SSR0) & SCI_ORER) {
		w = ctrl_inb(SSR0) & ~SCI_ORER; ctrl_outb(w, SSR0);
		return -1;
	}
	w = ctrl_inw(SCPCR) & ~SCPCR_CLK_MASK; ctrl_outw(w, SCPCR);
	while((ctrl_inb(SSR0) & SCI_RDRF) == 0);
	c = ctrl_inb(RDR0);
	w = ctrl_inb(SSR0);
	ctrl_outb(0, SSR0);
	return c & 0xff;
}
static void sci_getch_sw(void) {
	unsigned short	w;

	ctrl_outb(LOW_SPD, BRR0);
	while(!(ctrl_inb(SSR0) & SCI_RDRF));
	w = ctrl_inb(RDR0);
	w = ctrl_inw(SCPCR) | SCPCR_CLK_IN; ctrl_outw(w, SCPCR);
	w = ctrl_inb(SCR0) | SCK_IN; ctrl_outb(w, SCR0);
	w = ctrl_inw(SCPCR) & ~SCPCR_CLK_MASK; ctrl_outw(w, SCPCR);
	ctrl_outb(0, SMR0);
	ctrl_outb(SCK_OUT, SCR0);
	ctrl_outb(0, SSR0);
	ctrl_outb(SCI_CA, SMR0);
}

static int mmc_command(char command, char *arg, int arglen) {
	int		datalen, over;
	unsigned char	data;

	ctrl_outb((command == 0) ? LOW_SPD : MID_SPD, BRR0);
	sci_putch(0xff);
	sci_putch(MMC_CMD_INIT | (command & 0x3f));
	datalen = arglen;
	while(datalen) {
		sci_putch(arg[arglen - datalen]);
		datalen--;
	}
	datalen = 4 - arglen;
	while(datalen){
		sci_putch(0x00);
		datalen--;
	}
	sci_putch((command == 0) ? 0x95 : 0);
	sci_putch(0xff);
	sci_putch_sw();
	over = MMC_TIME_OVER;
	while((data = sci_getch()) & 0x80){
		if(--over == 0) return -1;
	}
	sci_getch_sw();
	return data;
}

static int mmc_reset(void) {
	int		ret;
	int		w, i ,flag;

	w = (ctrl_inw(SCPCR) & ~(SCPCR_CLK_MASK | SCPCR_CMD_MASK)) | (SCPCR_CLK_OUT | SCPCR_CMD_OUT); ctrl_outw(w, SCPCR);
	w = ctrl_inw(SCPCR) & ~SCPCR_CMD_MASK; ctrl_outw(w, SCPCR);
	w = ctrl_inb(SCPDR) | SCPDR_CLK | SCPDR_CMD; ctrl_outb(w, SCPDR);
	w = (ctrl_inw(SCPCR) & ~V1_SCPCR_CS_MASK) | V1_SCPCR_CS_OUT; ctrl_outw(w, SCPCR);
	ctrl_outb(0x00, SCR0);
	ctrl_outb(0x00, SSR0);
	ctrl_outb(0xFA, SCMR0);
	ctrl_outb(0x80, SMR0);
	ctrl_outb(LOW_SPD, BRR0);
	for(i = 0;i < 1000;i++);
	w = ctrl_inb(SCPDR) | V1_SCPDR_CS; ctrl_outb(w, SCPDR);
	flag = disable_interrupts();
	for(i = 0;i < 20;i++) sci_putch(0xff);
	sci_putch_sw();
	w = ctrl_inb(SCPDR) & ~V1_SCPDR_CS; ctrl_outb(w, SCPDR);
	ret = mmc_command(CMD_INITIALIZE, 0, 0);
	if(ret != 1) {
		if (flag){
			enable_interrupts();
		}
		return -1;
	}
	for(;;) {
		ret = mmc_command(CMD_ENABLE, 0, 0);
		if(ret == 0x00) break;
		if(ret != 0x01) {
			if (flag){
				enable_interrupts();
			}
			return -1;
		}
	}
	if (flag){
		enable_interrupts();
	}
	return 0;
}

static int mmc_config_read(char *data, int command, int len) {
	int		i, ret,flag;

	flag = disable_interrupts();
	ret = mmc_command(command, 0, 0);
	if(ret != 0) {
		if (flag){
			enable_interrupts();
		}
		return -1;
	}
	for(i = 0;i < len;i++) {
		data[i] = sci_getch();
		if(i == 0 && data[0] == -2) i--;
	}
	ctrl_outb(LOW_SPD, BRR0);
	sci_getch();
	sci_getch();
	sci_getch();
	sci_getch_sw();
	if (flag){
		enable_interrupts();
	}
	return 0;
}

static void sh_mmc_sci0_read(char *buffer) {
	asm("mov %0,r1" :: "r" (buffer));
	asm(
	"mov	#0xe8,r3" "\n\r"
	"shll2	r3" "\n\r"
	"shll2	r3" "\n\r"
"_mmc_sci_read:" "\n\r"
	"mov	#4,r0" "\n\r"
	"mov.b	r0,@(2,r3)" "\n\r"
	"mov	#0x10,r0" "\n\r"
	"mov.b	r0,@(4,r3)" "\n\r"
	"mov	#2,r2" "\n\r"
	"shll8	r2" "\n\r"
"_mmc_sci_read_L0:" "\n\r"
	"mov.b	@(8,r3),r0" "\n\r"
	"tst	#0x40,r0" "\n\r"
	"bt	_mmc_sci_read_L0" "\n\r"
	"mov.b	@(10,r3),r0" "\n\r"
	"mov.b	r0,@r1" "\n\r"
	"add	#1,r1" "\n\r"
	"mov.b	@(8,r3),r0" "\n\r"
	"mov	#0,r0" "\n\r"
	"mov.b	r0,@(8,r3)" "\n\r"
	"dt	r2" "\n\r"
	"bf	_mmc_sci_read_L0" "\n\r"
	);
}

static int sci_readmmc(int addr, char *buffer) {
	int		ret, over,flag;
	unsigned char	address[4];
	unsigned char	c;

	address[0] = addr >> 24;
	address[1] = addr >> 16;
	address[2] = addr >> 8;
	address[3] = addr;
	flag = disable_interrupts();
	ret = mmc_command(CMD_READ, address, 4);
	if(ret != 0) {
		if (flag){
			enable_interrupts();
		}
		return -1;
	}
	over = MMC_TIME_OVER;
	for(;;) {
		c = sci_getch();
		if(c == 0xfe) break;
		if(--over == 0) {
			if (flag){
				enable_interrupts();
			}
			return -1;
		}
	}
	sh_mmc_sci0_read(buffer);
	ctrl_outb(LOW_SPD, BRR0);
	sci_getch();
	sci_getch();
	sci_getch();
	sci_getch_sw();
	if (flag){
		enable_interrupts();
	}
	return 0;
}

static void sh_mmc_sci0_write(char *buffer) {
	asm("mov %0,r1" :: "r" (buffer));
	asm(
	"mov	#0xe8,r3" "\n\r"
	"shll2	r3" "\n\r"
	"shll2	r3" "\n\r"
"_mmc_sci_write:" "\n\r"
	"mov	#4,r0" "\n\r"
	"mov.b	r0,@(2,r3)" "\n\r"
	"mov	#0x20,r0" "\n\r"
	"mov.b	r0,@(4,r3)" "\n\r"
	"mov	#2,r2" "\n\r"
	"shll8	r2" "\n\r"
"_mmc_sci_write_L0:" "\n\r"
	"mov.b	@(8,r3),r0" "\n\r"
	"tst	#0x80,r0" "\n\r"
	"bt	_mmc_sci_write_L0" "\n\r"
	"mov.b	@r1,r0" "\n\r"
	"mov.b	r0,@(6,r3)" "\n\r"
	"add	#1,r1" "\n\r"
	"mov.b	@(8,r3),r0" "\n\r"
	"mov	#0,r0" "\n\r"
	"mov.b	r0,@(8,r3)" "\n\r"
	"dt	r2" "\n\r"
	"bf	_mmc_sci_write_L0" "\n\r"
	);
}

static int sci_writemmc(int addr, char *buffer) {
	int		ret, over,flag;
	unsigned char	address[4];

	address[0] = addr >> 24;
	address[1] = addr >> 16;
	address[2] = addr >> 8;
	address[3] = addr;
	flag = disable_interrupts();
	ret = mmc_command(CMD_WRITE, address, 4);
	if(ret != 0) {
		if (flag){
			enable_interrupts();
		}
		return -1;
	}
	sci_putch(0xfe);
	sh_mmc_sci0_write(buffer);
	ctrl_outb(LOW_SPD, BRR0);
	sci_putch(0);
	sci_putch(0);
	sci_putch(0);
	sci_putch_sw();
	over = MMC_TIME_OVER;
	while(sci_getch() != 0xff) {
		if(--over == 0) {
			if (flag){
				enable_interrupts();
			}
			return -1;
		}
	}
	sci_getch_sw();
	if (flag){
		enable_interrupts();
	}
	return 0;
}

static volatile unsigned char *spidr, *spibr, *spisr;
static unsigned char sdhc;

static int ssu_cmd(int media, unsigned char cmd, unsigned int address, unsigned char *data, unsigned int len, unsigned int rw) {
	volatile int w;
	unsigned char crc;
	unsigned int cnt = 0, status = 0;
	int	i,flag;

	switch(board_id) {
	case SH7706LANV2:
		w = ctrl_inb(SCPDR) & ~V2_SCPDR_CS; ctrl_outb(w, SCPDR);
		break;
	case SH7706LANV3:
		*(spisr) &= ~(0x08 << media);
		break;
	}
	flag = disable_interrupts();
	switch(cmd) {
	case CMD_INITIALIZE:
		crc = 0x95;
		break;
	case CMD_CHECK:
		crc = 0x87;
		break;
	default:
		crc = 0x00;
	}
	*(spidr) = 0xff;
	while((*(spisr) & 0x80) == 0x00);
	*(spidr) = cmd + 0x40;
	while((*(spisr) & 0x80) == 0x00);
	for(i = 24;i >= 0;i -= 8) {
		*(spidr) = address >> i;
		while((*(spisr) & 0x80) == 0x00);
	}
	*(spidr) = crc;
	for(cnt = 64;cnt != 0;cnt--) {
		while((*(spisr) & 0x80) == 0x00);
		status = *(spibr);
		if(!(status & 0x80) && cnt <= 62) break;
	}
	if(cnt == 0) {
		if (flag){
			enable_interrupts();
		}
		switch(board_id) {
		case SH7706LANV2:
			w = (ctrl_inw(SCPCR) & ~V2_SCPCR_CS_MASK) | V2_SCPCR_CS_OUT; ctrl_outw(w, SCPCR);
			w = ctrl_inb(SCPDR) | V2_SCPDR_CS; ctrl_outb(w, SCPDR);
			break;
		case SH7706LANV3:
			(*spisr) |= 0x38;
			break;
		}
		return -1;
	}
	if((status & 0x04) == 0) {
		switch(cmd) {
		case CMD_CHECK:
		case CMD_OCD:
			status = 0;
			for(i = 0;i < 4;i++) {
				status <<= 8;
				while((*(spisr) & 0x80) == 0x00);
				status |= *(spibr);
			}
			break;
		}
	}
	if(len > 0) {
		if(rw == READ) {
			for(cnt = 20000;cnt != 0;cnt--) {
				while((*(spisr) & 0x80) == 0x00);
				if(*(spibr) == 0xfe) {
					for(i = 0;i < len;i++) {
						while((*(spisr) & 0x80) == 0x00);
						data[i] = *(spibr);
					}
					while((*(spisr) & 0x80) == 0x00);
					crc = *(spibr);
					while((*(spisr) & 0x80) == 0x00);
					crc = *(spibr);
					while((*(spisr) & 0x80) == 0x00);
					break;
				}
			}
		} else {
			while((*(spisr) & 0x80) == 0x00);
			*(spidr) = 0xfe;
			while((*(spisr) & 0x80) == 0x00);
			for(i = 0;i < len;i++) {
				*(spidr) = data[i];
				while((*(spisr) & 0x80) == 0x00);
			}
			*(spidr) = 0;
			while((*(spisr) & 0x80) == 0x00);
			*(spidr) = 0;
			while((*(spisr) & 0x80) == 0x00);
			*(spidr) = 0xff;
			while((*(spisr) & 0x80) == 0x00);
			if ((*(spidr) & 0x0f) != 5) {
				cnt = 0;
			} else {
				for(;;) {
					*(spidr) = 0xff;
					while((*(spisr) & 0x80) == 0x00);
					if(*(spidr) == 0xff) break;
				}
			}
		}
	} else {
		while((*(spisr) & 0x80) == 0x00);
	}
	if (flag){
		enable_interrupts();
	}
	switch(board_id) {
	case SH7706LANV2:
		w = (ctrl_inw(SCPCR) & ~V2_SCPCR_CS_MASK) | V2_SCPCR_CS_OUT; ctrl_outw(w, SCPCR);
		w = ctrl_inb(SCPDR) | V2_SCPDR_CS; ctrl_outb(w, SCPDR);
		break;
	case SH7706LANV3:
		(*spisr) |= 0x38;
		break;
	}
	return (cnt == 0) ? -1 : status;
}

static int ssu_reset(int media) {
	volatile int w;
	int	i, s, ver;

	switch(board_id) {
	case SH7706LANV2:
		w = (ctrl_inw(SCPCR) & ~V2_SCPCR_CS_MASK) | V2_SCPCR_CS_OUT; ctrl_outw(w, SCPCR);
		w = ctrl_inb(SCPDR) | V2_SCPDR_CS; ctrl_outb(w, SCPDR);
		break;
	case SH7706LANV3:
		(*spisr) |= 0x38;
		break;
	}
	for(i = 0;i < 10;i++) {
		*(spidr) = 0xff;
		while((*(spisr) & 0x80) == 0x00);
	}
	if(ssu_cmd(media, CMD_INITIALIZE, 0, 0, 0, READ) == -1) {
		return -1;
	}
	s = ssu_cmd(media, CMD_CHECK, 0x1aa, 0, 0, READ);
	ver = (s == 0x1aa) ? 2 : 1;
	do {
		if(ver == 2) {
			ssu_cmd(media, CMD_APP, 0, 0, 0, READ);
			s = ssu_cmd(media, ACMD_COND, 0x40000000, 0, 0, READ);
		} else {
			s = ssu_cmd(media, CMD_ENABLE, 0, 0, 0, READ);
		}
		if(s == -1) return -1;
	} while(s != 0);
	for(w = 0;w < 100;w++);
	return 0;
}

static int mmc_card_status(void) {
	int	w, status;

	status = -1;
	switch(board_id) {
	case SH7706LANV1:
		w = ctrl_inw(PGCR); ctrl_outw(w | V1_PGCR_EJECT, PGCR);
		status = (ctrl_inb(PGDR) & V1_PGDR_EJECT) ? -1 : 0;
		break;
	case SH7706LANV2:
		w = ctrl_inw(SCPCR); ctrl_outw(w | V2_SCPCR_EJECT, SCPCR);
		status = (ctrl_inb(SCPDR) & V2_SCPDR_EJECT) ? -1 : 0;
		break;
	}
	return status;
}


/*******************************************************/
/* mmc card からブロックリード                           */
/*  戻り値　読み込んだブロック数                          */
/********************************************************/
static unsigned long mmc_bread(int dev_num, unsigned long blknr,
		lbaint_t blkcnt, void *dst)
{
	unsigned char *output_buf = dst;
	unsigned int rw, address;
	unsigned long long start, len, offset, adr;

	if (blkcnt == 0)
		return 0;

	start = blknr << 9;
	len = blkcnt << 9;
	rw = READ;
	for(offset = 0;offset < len;offset += 0x200) {
		switch(board_id) {
		case SH7706LANV1:
			address = (unsigned int)(start + offset);
			if (rw == READ) {
				sci_readmmc(address,(char *) &output_buf[offset]);
			} else {
				sci_writemmc(address,(char *) &output_buf[offset]);
			}
			break;
		case SH7706LANV2:
			if(sdhc) {
				adr = start + offset;
				adr >>= 9;
				address = (unsigned int)adr;
			} else {
				address = (unsigned int)(start + offset);
			}
			if (rw == READ) {
				ssu_cmd(0,CMD_READ, address, &output_buf[offset], 512, rw);
			} else {
				ssu_cmd(0,CMD_WRITE, address, &output_buf[offset], 512, rw);
			}
			break;
		}
	}
	return blkcnt;
}

/*******************************************************/
/* mmc card の初期化                                    */
/*  戻り値　-1:エラー　　正の数:ディスクブロックサイズ     */
/********************************************************/
static int shmin_mmc_init(void) {
	int	ret, read_len, c_size, c_size_mult;
	unsigned char	csd[16];
	

	mmc_blocksize = BLOCK_SIZE;

	board_id = SH7706LANV1;
	if(ctrl_inb(0xb0008006) == 0xab) {
		board_id = SH7706LANV2;
		spidr = (volatile unsigned char *)0xb0008000;
		spibr = (volatile unsigned char *)0xb0008004;
		spisr =(volatile unsigned char *) 0xb0008002;
	}

	if(mmc_card_status() == -1) {
		printf("mmc: Media not found.\n");
		return -1;
	}

	switch(board_id) {
	case SH7706LANV1:
		ret = mmc_reset();
		break;
	case SH7706LANV2:
		ret = ssu_reset(0);
		break;
	}
	if(ret == -1) {
		printf("mmc: Media I/O Error.\n");
		return -1;
	}
	printf("mmc: MMC Reset OK.\n");

	switch(board_id) {
	case SH7706LANV1:
		ret = mmc_config_read((char *)csd, CMD_CSD, 16);
		break;
	case SH7706LANV2:
		ret = ssu_cmd(0,CMD_CSD, 0, csd, 16, READ);
		break;
	}
	if(ret == -1) {
		printf("mmc: MMCDISK CSD Read Error.\n");
		return -1;
	}

	sdhc = 0;
	if(board_id == SH7706LANV2) {
		ret = ssu_cmd(0,CMD_OCD, 0, 0, 0, READ);
		if(ret & HCS) sdhc = 1;
//		if(ssu_cmd(0,CMD_CSD, 0, csd, 16, 0) == -1) return -1;
	}
	if(sdhc) {
		c_size = (((int)csd[7] & 0x3f) << 16) | (((int)csd[8] & 0xff) << 8) | ((int)csd[9] & 0xff);
		disksize = c_size + 1;
		disksize <<= 10;
		printf("mmc: SDHC Card\n");
	} else {
		read_len = (int)csd[5] & 0x0f;
		c_size = (((int)csd[6] & 0x03) << 10) | ((int)csd[7] << 2) | ((int)(csd[8] & 0xc0) >> 6);
		c_size_mult = (((int)csd[9] & 0x03) << 1) | (((int)csd[10] & 0x80) >> 7);
		disksize = c_size + 1;
		disksize <<= c_size_mult + 2;
		disksize <<= read_len - 9;
	}

	printf("mmc: %dKbyte %d blocksize\n", disksize >> 1, mmc_blocksize);

	return disksize;
}


/*******************************************************/
/* mmc card の登録                                      */
/*  戻り値　0意外:登録失敗　　0:登録成功                  */
/********************************************************/
int mmc_legacy_init(int dev)
{
	int dsize;

	dsize = shmin_mmc_init();
	if (dsize < 0)
		return 1;

	mmc_blk_dev.if_type = IF_TYPE_MMC;
	mmc_blk_dev.part_type = PART_TYPE_DOS;
	mmc_blk_dev.dev = 0;/* device number */
	mmc_blk_dev.lun = 0;/* target LUN */
	mmc_blk_dev.type = DEV_TYPE_HARDDISK;

	/* FIXME fill in the correct size (is set to 32MByte) */
	mmc_blk_dev.blksz = MMCSD_SECTOR_SIZE;
	mmc_blk_dev.lba = dsize; /* number of blocks */
	mmc_blk_dev.removable = 0;
	mmc_blk_dev.block_read = mmc_bread;

	//fat_register_device(&mmc_blk_dev, 1);/* 第2引数はパーティション番号 */
	//ext2fs_set_blk_dev (&mmc_blk_dev, 2);
	return 0;
}

