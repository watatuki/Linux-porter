/*
 * r8a7791 processor support
 *
 * Copyright (C) 2013-2014  Renesas Electronics Corporation
 * Copyright (C) 2013  Renesas Solutions Corp.
 * Copyright (C) 2013  Magnus Damm
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/of_platform.h>
#include <linux/platform_data/dma-rcar-audmapp.h>
#include <linux/platform_data/gpio-rcar.h>
#include <linux/platform_data/irq-renesas-irqc.h>
#include <linux/serial_sci.h>
#include <linux/sh_dma.h>
#include <linux/sh_timer.h>
#include <linux/spi/sh_msiof.h>
#include <asm/mach/arch.h>

#include "common.h"
#include "dma-register.h"
#include "irqs.h"
#include "r8a7791.h"
#include "rcar-gen2.h"

/* Audio-DMAC */
#define AUDIO_DMAC_SLAVE(_id, _addr, toffset, roffset, t, r)	\
{								\
	.slave_id	= AUDIO_DMAC_SLAVE_## _id ##_TX,	\
	.addr		= _addr + toffset,			\
	.chcr		= CHCR_TX(XMIT_SZ_32BIT),		\
	.mid_rid	= t,					\
}, {								\
	.slave_id	= AUDIO_DMAC_SLAVE_## _id ##_RX,	\
	.addr		= _addr + roffset,			\
	.chcr		= CHCR_RX(XMIT_SZ_32BIT),		\
	.mid_rid	= r,					\
}

#define AUDIO_DMAC_SCU_SLAVE(_id, _addr, t, r)			\
	AUDIO_DMAC_SLAVE(SCU##_id, _addr, 0x0, 0x4000, t, r)

#define AUDIO_DMAC_SSI_SLAVE(_id, _addr, t, r)			\
	AUDIO_DMAC_SLAVE(SSI##_id, _addr, 0x8, 0xc, t, r)

#define AUDIO_DMAC_SSIU_SLAVE(_id, _addr, t, r)		\
	AUDIO_DMAC_SLAVE(SSIU##_id, _addr, 0, 0, t, r)

#define AUDIO_DMAC_CMD_SLAVE(_id, _addr, t)			\
{								\
	.slave_id	= AUDIO_DMAC_SLAVE_CMD## _id ##_TO_MEM,	\
	.addr		= _addr,				\
	.chcr		= CHCR_RX(XMIT_SZ_32BIT),		\
	.mid_rid	= t,					\
}

static const struct sh_dmae_slave_config r8a7791_audio_dmac_slaves[] = {
	AUDIO_DMAC_SCU_SLAVE(0, 0, 0x85, 0x9a),
	AUDIO_DMAC_SCU_SLAVE(1, 0, 0x87, 0x9c),
	AUDIO_DMAC_SCU_SLAVE(2, 0, 0x89, 0x9e),
	AUDIO_DMAC_SCU_SLAVE(3, 0, 0x8b, 0xa0),
	AUDIO_DMAC_SCU_SLAVE(4, 0, 0x8d, 0xb0),
	AUDIO_DMAC_SCU_SLAVE(5, 0, 0x8f, 0xb2),
	AUDIO_DMAC_SCU_SLAVE(6, 0, 0x91, 0xb4),
	AUDIO_DMAC_SCU_SLAVE(7, 0, 0x93, 0xb6),
	AUDIO_DMAC_SCU_SLAVE(8, 0, 0x95, 0xb8),
	AUDIO_DMAC_SCU_SLAVE(9, 0, 0x97, 0xba),
	AUDIO_DMAC_SSI_SLAVE(0, 0, 0x01, 0x02),
	AUDIO_DMAC_SSI_SLAVE(1, 0, 0x03, 0x04),
	AUDIO_DMAC_SSI_SLAVE(2, 0, 0x05, 0x06),
	AUDIO_DMAC_SSI_SLAVE(3, 0, 0x07, 0x08),
	AUDIO_DMAC_SSI_SLAVE(4, 0, 0x09, 0x0a),
	AUDIO_DMAC_SSI_SLAVE(5, 0, 0x0b, 0x0c),
	AUDIO_DMAC_SSI_SLAVE(6, 0, 0x0d, 0x0e),
	AUDIO_DMAC_SSI_SLAVE(7, 0, 0x0f, 0x10),
	AUDIO_DMAC_SSI_SLAVE(8, 0, 0x11, 0x12),
	AUDIO_DMAC_SSI_SLAVE(9, 0, 0x13, 0x14),
	AUDIO_DMAC_SSIU_SLAVE(0, 0, 0x15, 0x16),
	AUDIO_DMAC_SSIU_SLAVE(1, 0, 0x49, 0x4a),
	AUDIO_DMAC_SSIU_SLAVE(2, 0, 0x63, 0x64),
	AUDIO_DMAC_SSIU_SLAVE(3, 0, 0x6f, 0x70),
	AUDIO_DMAC_SSIU_SLAVE(4, 0, 0x71, 0x72),
	AUDIO_DMAC_SSIU_SLAVE(5, 0, 0x73, 0x74),
	AUDIO_DMAC_SSIU_SLAVE(6, 0, 0x75, 0x76),
	AUDIO_DMAC_SSIU_SLAVE(7, 0, 0x79, 0x7a),
	AUDIO_DMAC_SSIU_SLAVE(8, 0, 0x7b, 0x7c),
	AUDIO_DMAC_SSIU_SLAVE(9, 0, 0x7d, 0x7e),
	AUDIO_DMAC_CMD_SLAVE(1, 0, 0xbe),
};

#define DMAE_CHANNEL(a, b)			\
{						\
	.offset		= (a) - 0x20,		\
	.dmars		= (a) - 0x20 + 0x40,	\
	.chclr_bit	= (b),			\
	.chclr_offset	= 0x80 - 0x20,		\
}

static const struct sh_dmae_channel r8a7791_audio_dmac_channels[] = {
	DMAE_CHANNEL(0x8000, 0),
	DMAE_CHANNEL(0x8080, 1),
	DMAE_CHANNEL(0x8100, 2),
	DMAE_CHANNEL(0x8180, 3),
	DMAE_CHANNEL(0x8200, 4),
	DMAE_CHANNEL(0x8280, 5),
	DMAE_CHANNEL(0x8300, 6),
	DMAE_CHANNEL(0x8380, 7),
	DMAE_CHANNEL(0x8400, 8),
	DMAE_CHANNEL(0x8480, 9),
	DMAE_CHANNEL(0x8500, 10),
	DMAE_CHANNEL(0x8580, 11),
	DMAE_CHANNEL(0x8600, 12),
};

static struct sh_dmae_pdata r8a7791_audio_dmac_platform_data = {
	.slave		= r8a7791_audio_dmac_slaves,
	.slave_num	= ARRAY_SIZE(r8a7791_audio_dmac_slaves),
	.channel	= r8a7791_audio_dmac_channels,
	.channel_num	= ARRAY_SIZE(r8a7791_audio_dmac_channels),
	.ts_low_shift	= TS_LOW_SHIFT,
	.ts_low_mask	= TS_LOW_BIT << TS_LOW_SHIFT,
	.ts_high_shift	= TS_HI_SHIFT,
	.ts_high_mask	= TS_HI_BIT << TS_HI_SHIFT,
	.ts_shift	= dma_ts_shift,
	.ts_shift_num	= ARRAY_SIZE(dma_ts_shift),
	.dmaor_init	= DMAOR_DME,
	.chclr_present	= 1,
	.chclr_bitwise	= 1,
};

static struct resource r8a7791_audio_dmac_resources[] = {
	/* Channel registers and DMAOR for low */
	DEFINE_RES_MEM(0xec700020, 0x8663 - 0x20),
	DEFINE_RES_IRQ(gic_spi(346)),
	DEFINE_RES_NAMED(gic_spi(320), 13, NULL, IORESOURCE_IRQ),

	/* Channel registers and DMAOR for hi */
	DEFINE_RES_MEM(0xec720020, 0x8663 - 0x20), /* hi */
	DEFINE_RES_IRQ(gic_spi(347)),
	DEFINE_RES_NAMED(gic_spi(333), 13, NULL, IORESOURCE_IRQ),
};

#define r8a7791_register_audio_dmac(id)				\
	platform_device_register_resndata(			\
		&platform_bus, "sh-dma-engine", id,		\
		&r8a7791_audio_dmac_resources[id * 3],	3,	\
		&r8a7791_audio_dmac_platform_data,		\
		sizeof(r8a7791_audio_dmac_platform_data))

/* Audio DMAC peri peri */
#define AUDMAPP_CFG(id, _src, _dst, _chcr)				\
	{ .slave_id = AUDIOPP_DMAC_SLAVE_##id,				\
			.src = _src, .dst = _dst, .chcr = _chcr << 16}

static struct audmapp_slave_config r8a7791_audmapp_slaves[] = {
	AUDMAPP_CFG(SCU0_TO_SSI0, 0, 0, 0x2d00),
	AUDMAPP_CFG(SCU1_TO_SSI1, 0, 0, 0x2e04),
	AUDMAPP_CFG(SCU2_TO_SSI2, 0, 0, 0x2f08),
	AUDMAPP_CFG(SCU3_TO_SSI3, 0, 0, 0x300c),
	AUDMAPP_CFG(SCU4_TO_SSI4, 0, 0, 0x310d),
	AUDMAPP_CFG(SCU5_TO_SSI5, 0, 0, 0x320e),
	AUDMAPP_CFG(SCU6_TO_SSI6, 0, 0, 0x330f),
	AUDMAPP_CFG(SCU7_TO_SSI7, 0, 0, 0x3410),
	AUDMAPP_CFG(SCU8_TO_SSI8, 0, 0, 0x3511),
	AUDMAPP_CFG(SCU9_TO_SSI9, 0, 0, 0x3612),
	AUDMAPP_CFG(CMD0_TO_SSI0, 0, 0, 0x3700),

	AUDMAPP_CFG(SSI0_TO_SCU0, 0, 0, 0x002d),
	AUDMAPP_CFG(SSI1_TO_SCU1, 0, 0, 0x042e),
	AUDMAPP_CFG(SSI2_TO_SCU2, 0, 0, 0x082f),
	AUDMAPP_CFG(SSI3_TO_SCU3, 0, 0, 0x0c30),
	AUDMAPP_CFG(SSI4_TO_SCU4, 0, 0, 0x0d31),
	AUDMAPP_CFG(SSI5_TO_SCU5, 0, 0, 0x0e32),
	AUDMAPP_CFG(SSI6_TO_SCU6, 0, 0, 0x0f33),
	AUDMAPP_CFG(SSI7_TO_SCU7, 0, 0, 0x1034),
	AUDMAPP_CFG(SSI8_TO_SCU8, 0, 0, 0x1135),
	AUDMAPP_CFG(SSI9_TO_SCU9, 0, 0, 0x1236),
};

struct audmapp_pdata r8a7791_audmapp_platform_data = {
	.slave		= r8a7791_audmapp_slaves,
	.slave_num	= ARRAY_SIZE(r8a7791_audmapp_slaves),
};

static const struct resource r8a7791_audmapp_resources[] __initconst = {
	DEFINE_RES_MEM(0xec740000, 0x200),
};

#define r8a7791_register_audmapp()				\
	platform_device_register_resndata(&platform_bus,	\
		"rcar-audmapp-engine", -1,			\
		r8a7791_audmapp_resources,			\
		ARRAY_SIZE(r8a7791_audmapp_resources),		\
		&r8a7791_audmapp_platform_data,			\
		sizeof(r8a7791_audmapp_platform_data));

static const struct resource pfc_resources[] __initconst = {
	DEFINE_RES_MEM(0xe6060000, 0x250),
};

#define r8a7791_register_pfc()						\
	platform_device_register_simple("pfc-r8a7791", -1, pfc_resources, \
					ARRAY_SIZE(pfc_resources))

#define R8A7791_GPIO(idx, base, nr)					\
static const struct resource r8a7791_gpio##idx##_resources[] __initconst = { \
	DEFINE_RES_MEM((base), 0x50),					\
	DEFINE_RES_IRQ(gic_spi(4 + (idx))),				\
};									\
									\
static const struct gpio_rcar_config					\
r8a7791_gpio##idx##_platform_data __initconst = {			\
	.gpio_base	= 32 * (idx),					\
	.irq_base	= 0,						\
	.number_of_pins	= (nr),						\
	.pctl_name	= "pfc-r8a7791",				\
	.has_both_edge_trigger = 1,					\
};									\

R8A7791_GPIO(0, 0xe6050000, 32);
R8A7791_GPIO(1, 0xe6051000, 32);
R8A7791_GPIO(2, 0xe6052000, 32);
R8A7791_GPIO(3, 0xe6053000, 32);
R8A7791_GPIO(4, 0xe6054000, 32);
R8A7791_GPIO(5, 0xe6055000, 32);
R8A7791_GPIO(6, 0xe6055400, 32);
R8A7791_GPIO(7, 0xe6055800, 26);

#define r8a7791_register_gpio(idx)					\
	platform_device_register_resndata(&platform_bus, "gpio_rcar", idx, \
		r8a7791_gpio##idx##_resources,				\
		ARRAY_SIZE(r8a7791_gpio##idx##_resources),		\
		&r8a7791_gpio##idx##_platform_data,			\
		sizeof(r8a7791_gpio##idx##_platform_data))

void __init r8a7791_pinmux_init(void)
{
	r8a7791_register_pfc();
	r8a7791_register_gpio(0);
	r8a7791_register_gpio(1);
	r8a7791_register_gpio(2);
	r8a7791_register_gpio(3);
	r8a7791_register_gpio(4);
	r8a7791_register_gpio(5);
	r8a7791_register_gpio(6);
	r8a7791_register_gpio(7);
}

#define __R8A7791_SCIF(scif_type, index, baseaddr, irq)			\
static struct plat_sci_port scif##index##_platform_data = {		\
	.type		= scif_type,					\
	.flags		= UPF_BOOT_AUTOCONF | UPF_IOREMAP,		\
	.scscr		= SCSCR_RE | SCSCR_TE,				\
};									\
									\
static struct resource scif##index##_resources[] = {			\
	DEFINE_RES_MEM(baseaddr, 0x100),				\
	DEFINE_RES_IRQ(irq),						\
}

#define R8A7791_SCIF(index, baseaddr, irq)				\
	__R8A7791_SCIF(PORT_SCIF, index, baseaddr, irq)

#define R8A7791_SCIFA(index, baseaddr, irq)				\
	__R8A7791_SCIF(PORT_SCIFA, index, baseaddr, irq)

#define R8A7791_SCIFB(index, baseaddr, irq)				\
	__R8A7791_SCIF(PORT_SCIFB, index, baseaddr, irq)

#define R8A7791_HSCIF(index, baseaddr, irq)				\
	__R8A7791_SCIF(PORT_HSCIF, index, baseaddr, irq)

R8A7791_SCIFA(0,  0xe6c40000, gic_spi(144)); /* SCIFA0 */
R8A7791_SCIFA(1,  0xe6c50000, gic_spi(145)); /* SCIFA1 */
R8A7791_SCIFB(2,  0xe6c20000, gic_spi(148)); /* SCIFB0 */
R8A7791_SCIFB(3,  0xe6c30000, gic_spi(149)); /* SCIFB1 */
R8A7791_SCIFB(4,  0xe6ce0000, gic_spi(150)); /* SCIFB2 */
R8A7791_SCIFA(5,  0xe6c60000, gic_spi(151)); /* SCIFA2 */
R8A7791_SCIF(6,   0xe6e60000, gic_spi(152)); /* SCIF0 */
R8A7791_SCIF(7,   0xe6e68000, gic_spi(153)); /* SCIF1 */
R8A7791_HSCIF(8,  0xe62c0000, gic_spi(154)); /* HSCIF0 */
R8A7791_HSCIF(9,  0xe62c8000, gic_spi(155)); /* HSCIF1 */
R8A7791_SCIF(10,  0xe6e58000, gic_spi(22)); /* SCIF2 */
R8A7791_SCIF(11,  0xe6ea8000, gic_spi(23)); /* SCIF3 */
R8A7791_SCIF(12,  0xe6ee0000, gic_spi(24)); /* SCIF4 */
R8A7791_SCIF(13,  0xe6ee8000, gic_spi(25)); /* SCIF5 */
R8A7791_SCIFA(14, 0xe6c70000, gic_spi(29)); /* SCIFA3 */
R8A7791_SCIFA(15, 0xe6c78000, gic_spi(30)); /* SCIFA4 */
R8A7791_SCIFA(16, 0xe6c80000, gic_spi(31)); /* SCIFA5 */
R8A7791_HSCIF(17, 0xe62d0000, gic_spi(21)); /* HSCIF2 */

#define r8a7791_register_scif(index)					       \
	platform_device_register_resndata(&platform_bus, "sh-sci", index,      \
					  scif##index##_resources,	       \
					  ARRAY_SIZE(scif##index##_resources), \
					  &scif##index##_platform_data,	       \
					  sizeof(scif##index##_platform_data))

static const struct sh_timer_config cmt00_platform_data __initconst = {
	.name = "CMT00",
	.timer_bit = 0,
	.clockevent_rating = 80,
};

static const struct resource cmt00_resources[] __initconst = {
	DEFINE_RES_MEM(0xffca0510, 0x0c),
	DEFINE_RES_MEM(0xffca0500, 0x04),
	DEFINE_RES_IRQ(gic_spi(142)), /* CMT0_0 */
};

#define r8a7791_register_cmt(idx)					\
	platform_device_register_resndata(&platform_bus, "sh_cmt",	\
					  idx, cmt##idx##_resources,	\
					  ARRAY_SIZE(cmt##idx##_resources), \
					  &cmt##idx##_platform_data,	\
					  sizeof(struct sh_timer_config))

static struct renesas_irqc_config irqc0_data = {
	.irq_base = irq_pin(0), /* IRQ0 -> IRQ9 */
};

static struct resource irqc0_resources[] = {
	DEFINE_RES_MEM(0xe61c0000, 0x200), /* IRQC Event Detector Block_0 */
	DEFINE_RES_IRQ(gic_spi(0)), /* IRQ0 */
	DEFINE_RES_IRQ(gic_spi(1)), /* IRQ1 */
	DEFINE_RES_IRQ(gic_spi(2)), /* IRQ2 */
	DEFINE_RES_IRQ(gic_spi(3)), /* IRQ3 */
	DEFINE_RES_IRQ(gic_spi(12)), /* IRQ4 */
	DEFINE_RES_IRQ(gic_spi(13)), /* IRQ5 */
	DEFINE_RES_IRQ(gic_spi(14)), /* IRQ6 */
	DEFINE_RES_IRQ(gic_spi(15)), /* IRQ7 */
	DEFINE_RES_IRQ(gic_spi(16)), /* IRQ8 */
	DEFINE_RES_IRQ(gic_spi(17)), /* IRQ9 */
};

#define r8a7791_register_irqc(idx)					\
	platform_device_register_resndata(&platform_bus, "renesas_irqc", \
					  idx, irqc##idx##_resources,	\
					  ARRAY_SIZE(irqc##idx##_resources), \
					  &irqc##idx##_data,		\
					  sizeof(struct renesas_irqc_config))

static const struct resource thermal_resources[] __initconst = {
	DEFINE_RES_MEM(0xe61f0000, 0x14),
	DEFINE_RES_MEM(0xe61f0100, 0x38),
	DEFINE_RES_IRQ(gic_spi(69)),
};

#define r8a7791_register_thermal()					\
	platform_device_register_simple("rcar_thermal", -1,		\
					thermal_resources,		\
					ARRAY_SIZE(thermal_resources))

/* MSIOF */
#define MSIOF_COMMON				\
	.rx_fifo_override	= 256,			\
	.num_chipselect		= 1

static const struct sh_msiof_spi_info sh_msiof_info[] __initconst = {
	{
		MSIOF_COMMON,
	},
	{
		MSIOF_COMMON,
	},
	{
		MSIOF_COMMON,
	},
};

static const struct resource sh_msiof0_resources[] __initconst = {
	DEFINE_RES_MEM(0xe6e20000, 0x0064),
	DEFINE_RES_IRQ(gic_spi(156)),
};

static const struct resource sh_msiof1_resources[] __initconst = {
	DEFINE_RES_MEM(0xe6e10000, 0x0064),
	DEFINE_RES_IRQ(gic_spi(157)),
};

static const struct resource sh_msiof2_resources[] __initconst = {
	DEFINE_RES_MEM(0xe6e00000, 0x0064),
	DEFINE_RES_IRQ(gic_spi(158)),
};

#define r8a7791_register_msiof(idx)					\
	platform_device_register_resndata(&platform_bus, "spi_sh_msiof", \
				  (idx+1), sh_msiof##idx##_resources,	\
				  ARRAY_SIZE(sh_msiof##idx##_resources), \
				  &sh_msiof_info[idx],		\
				  sizeof(struct sh_msiof_spi_info))

/* POWERVR */
static const struct resource powervr_resources[] __initconst = {
	DEFINE_RES_MEM(0xfd800000, 0x10000),
	DEFINE_RES_IRQ(gic_spi(119)),
};

#define r8a7791_register_pvrsrvkm()					\
	platform_device_register_simple("pvrsrvkm", -1,			\
					powervr_resources,		\
					ARRAY_SIZE(powervr_resources))

/* SSP */
void __init r8a7791_register_ssp(void)
{
	struct platform_device_info info = {
		.name = "ssp_dev",
		.id = -1,
	};
	platform_device_register_full(&info);
}

void __init r8a7791_add_dt_devices(void)
{
	r8a7791_pm_init();
	r8a7791_init_pm_domains();
	r8a7791_register_cmt(00);
	r8a7791_register_audio_dmac(0);
	r8a7791_register_audio_dmac(1);
	r8a7791_register_audmapp();
	r8a7791_register_pvrsrvkm();
	r8a7791_register_ssp();
}

void __init r8a7791_add_standard_devices(void)
{
	r8a7791_register_scif(0);
	r8a7791_register_scif(1);
	r8a7791_register_scif(2);
	r8a7791_register_scif(3);
	r8a7791_register_scif(4);
	r8a7791_register_scif(5);
	r8a7791_register_scif(6);
	r8a7791_register_scif(7);
	r8a7791_register_scif(8);
	r8a7791_register_scif(9);
	r8a7791_register_scif(10);
	r8a7791_register_scif(11);
	r8a7791_register_scif(12);
	r8a7791_register_scif(13);
	r8a7791_register_scif(14);
	r8a7791_register_scif(15);
	r8a7791_register_scif(16);
	r8a7791_register_scif(17);
	r8a7791_add_dt_devices();
	r8a7791_register_irqc(0);
	r8a7791_register_thermal();
	r8a7791_register_msiof(0);
	r8a7791_register_msiof(1);
	r8a7791_register_msiof(2);
}

#ifdef CONFIG_USE_OF
static const char *r8a7791_boards_compat_dt[] __initdata = {
	"renesas,r8a7791",
	NULL,
};

DT_MACHINE_START(R8A7791_DT, "Generic R8A7791 (Flattened Device Tree)")
	.smp		= smp_ops(r8a7791_smp_ops),
	.init_early	= shmobile_init_delay,
	.init_time	= rcar_gen2_timer_init,
	.init_late	= shmobile_init_late,
	.reserve	= rcar_gen2_reserve,
	.dt_compat	= r8a7791_boards_compat_dt,
MACHINE_END
#endif /* CONFIG_USE_OF */
