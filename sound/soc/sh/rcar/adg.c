/*
 * Helper routines for R-Car sound ADG.
 *
 *  Copyright (C) 2013  Kuninori Morimoto <kuninori.morimoto.gx@renesas.com>
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/sh_clk.h>
#include "rsnd.h"

#define CLKA	0
#define CLKB	1
#define CLKC	2
#define CLKI	3
#define CLKMAX	4

static struct rsnd_mod_ops adg_ops = {
	.name = "adg"
};

struct rsnd_adg {
	struct clk *clk[CLKMAX];
	struct rsnd_mod mod;

	int rbga_rate_for_441khz_div_6;	/* RBGA */
	int rbgb_rate_for_48khz_div_6;	/* RBGB */
	u32 ckr;
#ifdef QUICK_HACK
	unsigned int quick_hack_rate[CLKMAX];
	int quick_hack_used_clk;
#endif
};

#define for_each_rsnd_clk(pos, adg, i)		\
	for (i = 0;				\
	     (i < CLKMAX) &&			\
	     ((pos) = adg->clk[i]);		\
	     i++)
#define rsnd_priv_to_adg(priv) ((struct rsnd_adg *)(priv)->adg)


static u32 rsnd_adg_ssi_ws_timing_gen2(struct rsnd_dai_stream *io)
{
	struct rsnd_mod *mod = rsnd_io_to_mod_ssi(io);
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	int id = rsnd_mod_id(mod);
	int ws = id;

	if (rsnd_ssi_is_pin_sharing(rsnd_ssi_mod_get(priv, id))) {
		switch (id) {
		case 1:
		case 2:
			ws = 0;
			break;
		case 4:
			ws = 3;
			break;
		case 8:
			ws = 7;
			break;
		}
	}

	return (0x6 + ws) << 8;
}


static void __rsnd_adg_get_timesel_ratio(struct rsnd_priv *priv,
				       struct rsnd_dai_stream *io,
				       unsigned int target_rate,
				       unsigned int *target_val,
				       unsigned int *target_en)
{
	struct rsnd_adg *adg = rsnd_priv_to_adg(priv);
	struct device *dev = rsnd_priv_to_dev(priv);
	int idx, sel, div, step;
	unsigned int val, en;
	unsigned int min, diff;
	unsigned int sel_rate[] = {
		clk_get_rate(adg->clk[CLKA]),	/* 0000: CLKA */
		clk_get_rate(adg->clk[CLKB]),	/* 0001: CLKB */
		clk_get_rate(adg->clk[CLKC]),	/* 0010: CLKC */
		adg->rbga_rate_for_441khz_div_6,/* 0011: RBGA */
		adg->rbgb_rate_for_48khz_div_6,	/* 0100: RBGB */
	};
#ifdef QUICK_HACK
	int i;
	for (i = 0; i < CLKMAX; i++)
		sel_rate[i] = adg->quick_hack_rate[i];
#endif

	min = ~0;
	val = 0;
	en = 0;
	for (sel = 0; sel < ARRAY_SIZE(sel_rate); sel++) {
		idx = 0;
		step = 2;

		if (!sel_rate[sel])
			continue;

		for (div = 2; div <= 98304; div += step) {
			diff = abs(target_rate - sel_rate[sel] / div);
			if (min > diff) {
				val = (sel << 8) | idx;
				min = diff;
				en = 1 << (sel + 1); /* fixme */
			}

			/*
			 * step of 0_0000 / 0_0001 / 0_1101
			 * are out of order
			 */
			if ((idx > 2) && (idx % 2))
				step *= 2;
			if (idx == 0x1c) {
				div += step;
				step *= 2;
			}
			idx++;
		}
	}

	if (min == ~0) {
		dev_err(dev, "no Input clock\n");
		return;
	}

	*target_val = val;
	if (target_en)
		*target_en = en;
}

static void rsnd_adg_get_timesel_ratio(struct rsnd_priv *priv,
				       struct rsnd_dai_stream *io,
				       unsigned int in_rate,
				       unsigned int out_rate,
				       u32 *in, u32 *out, u32 *en)
{
	struct snd_pcm_runtime *runtime = rsnd_io_to_runtime(io);
	unsigned int target_rate;
	u32 *target_val;
	u32 _in;
	u32 _out;
	u32 _en;

	/* default = SSI WS */
	_in =
	_out = rsnd_adg_ssi_ws_timing_gen2(io);

	target_rate = 0;
	target_val = NULL;
	_en = 0;
	if (runtime->rate != in_rate) {
		target_rate = out_rate;
		target_val  = &_out;
	} else if (runtime->rate != out_rate) {
		target_rate = in_rate;
		target_val  = &_in;
	}

	if (target_rate)
		__rsnd_adg_get_timesel_ratio(priv, io,
					     target_rate,
					     target_val, &_en);

	if (in)
		*in = _in;
	if (out)
		*out = _out;
	if (en)
		*en = _en;
}

int rsnd_adg_set_cmd_timsel_gen2(struct rsnd_dai *rdai,
				 struct rsnd_mod *mod,
				 struct rsnd_dai_stream *io)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct rsnd_adg *adg = rsnd_priv_to_adg(priv);
	struct rsnd_mod *adg_mod = rsnd_mod_get(adg);
	int id = rsnd_mod_id(mod);
	int shift = (id % 2) ? 16 : 0;
	u32 mask, val;

	rsnd_adg_get_timesel_ratio(priv, io,
				   rsnd_src_get_in_rate(priv, io),
				   rsnd_src_get_out_rate(priv, io),
				   NULL, &val, NULL);

	val  = val	<< shift;
	mask = 0xffff	<< shift;

	rsnd_mod_bset(adg_mod, CMDOUT_TIMSEL, mask, val);

	return 0;
}


int rsnd_adg_set_src_timesel_gen2(struct rsnd_mod *src_mod,
				  struct rsnd_dai_stream *io,
				  unsigned int in_rate,
				  unsigned int out_rate)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(src_mod);
	struct rsnd_adg *adg = rsnd_priv_to_adg(priv);
	struct rsnd_mod *adg_mod = rsnd_mod_get(adg);
	int id = rsnd_mod_id(src_mod);
	int shift = (id % 2) ? 16 : 0;
	u32 mask, en;
	u32 in, out;
#ifdef QUICK_HACK
	unsigned int rate = rsnd_io_is_play(io) ? out_rate : in_rate;

	rsnd_adg_clk_set_rate(adg_mod, rate);
#endif
	rsnd_adg_get_timesel_ratio(priv, io,
				  in_rate, out_rate,
				  &in, &out, &en);
	in   = in	<< shift;
	out  = out	<< shift;
	mask = 0xffff	<< shift;

	switch (id / 2) {
	case 0:
		rsnd_mod_bset(adg_mod, SRCIN_TIMSEL0,  mask, in);
		rsnd_mod_bset(adg_mod, SRCOUT_TIMSEL0, mask, out);
		break;
	case 1:
		rsnd_mod_bset(adg_mod, SRCIN_TIMSEL1,  mask, in);
		rsnd_mod_bset(adg_mod, SRCOUT_TIMSEL1, mask, out);
		break;
	case 2:
		rsnd_mod_bset(adg_mod, SRCIN_TIMSEL2,  mask, in);
		rsnd_mod_bset(adg_mod, SRCOUT_TIMSEL2, mask, out);
		break;
	case 3:
		rsnd_mod_bset(adg_mod, SRCIN_TIMSEL3,  mask, in);
		rsnd_mod_bset(adg_mod, SRCOUT_TIMSEL3, mask, out);
		break;
	case 4:
		rsnd_mod_bset(adg_mod, SRCIN_TIMSEL4,  mask, in);
		rsnd_mod_bset(adg_mod, SRCOUT_TIMSEL4, mask, out);
		break;
	}

	if (en)
		rsnd_mod_bset(adg_mod, DIV_EN, en, en);

	return 0;
}

int rsnd_adg_set_convert_clk_gen1(struct rsnd_priv *priv,
				  struct rsnd_mod *mod,
				  unsigned int src_rate,
				  unsigned int dst_rate)
{
	struct rsnd_adg *adg = rsnd_priv_to_adg(priv);
	struct rsnd_mod *adg_mod = rsnd_mod_get(adg);
	struct device *dev = rsnd_priv_to_dev(priv);
	int idx, sel, div, shift;
	u32 mask, val;
	int id = rsnd_mod_id(mod);
	unsigned int sel_rate [] = {
		clk_get_rate(adg->clk[CLKA]),	/* 000: CLKA */
		clk_get_rate(adg->clk[CLKB]),	/* 001: CLKB */
		clk_get_rate(adg->clk[CLKC]),	/* 010: CLKC */
		0,				/* 011: MLBCLK (not used) */
		adg->rbga_rate_for_441khz_div_6,/* 100: RBGA */
		adg->rbgb_rate_for_48khz_div_6,	/* 101: RBGB */
	};

	/* find div (= 1/128, 1/256, 1/512, 1/1024, 1/2048 */
	for (sel = 0; sel < ARRAY_SIZE(sel_rate); sel++) {
		for (div  = 128,	idx = 0;
		     div <= 2048;
		     div *= 2,		idx++) {
			if (src_rate == sel_rate[sel] / div) {
				val = (idx << 4) | sel;
				goto find_rate;
			}
		}
	}
	dev_err(dev, "can't find convert src clk\n");
	return -EINVAL;

find_rate:
	shift	= (id % 4) * 8;
	mask	= 0xFF << shift;
	val	= val << shift;

	dev_dbg(dev, "adg convert src clk = %02x\n", val);

	switch (id / 4) {
	case 0:
		rsnd_mod_bset(adg_mod, AUDIO_CLK_SEL3, mask, val);
		break;
	case 1:
		rsnd_mod_bset(adg_mod, AUDIO_CLK_SEL4, mask, val);
		break;
	case 2:
		rsnd_mod_bset(adg_mod, AUDIO_CLK_SEL5, mask, val);
		break;
	}

	/*
	 * Gen1 doesn't need dst_rate settings,
	 * since it uses SSI WS pin.
	 * see also rsnd_src_set_route_if_gen1()
	 */

	return 0;
}

static void rsnd_adg_set_ssi_clk(struct rsnd_mod *mod, u32 val)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct rsnd_adg *adg = rsnd_priv_to_adg(priv);
	struct rsnd_mod *adg_mod = rsnd_mod_get(adg);
	int id = rsnd_mod_id(mod);
	int shift = (id % 4) * 8;
	u32 mask = 0xFF << shift;

	val = val << shift;

	/*
	 * SSI 8 is not connected to ADG.
	 * it works with SSI 7
	 */
	if (id == 8)
		return;

	switch (id / 4) {
	case 0:
		rsnd_mod_bset(adg_mod, AUDIO_CLK_SEL0, mask, val);
		break;
	case 1:
		rsnd_mod_bset(adg_mod, AUDIO_CLK_SEL1, mask, val);
		break;
	case 2:
		rsnd_mod_bset(adg_mod, AUDIO_CLK_SEL2, mask, val);
		break;
	}
}

int rsnd_adg_ssi_clk_stop(struct rsnd_mod *mod)
{
	/*
	 * "mod" = "ssi" here.
	 * we can get "ssi id" from mod
	 */
	rsnd_adg_set_ssi_clk(mod, 0);

	return 0;
}

int rsnd_adg_ssi_clk_try_start(struct rsnd_mod *mod, unsigned int rate)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct rsnd_adg *adg = rsnd_priv_to_adg(priv);
	struct rsnd_mod *adg_mod = rsnd_mod_get(adg);
	struct device *dev = rsnd_priv_to_dev(priv);
	struct clk *clk;
	int i;
	u32 data;
	int sel_table[] = {
		[CLKA] = 0x1,
		[CLKB] = 0x2,
		[CLKC] = 0x3,
		[CLKI] = 0x0,
	};

	dev_dbg(dev, "request clock = %d\n", rate);

	/*
	 * find suitable clock from
	 * AUDIO_CLKA/AUDIO_CLKB/AUDIO_CLKC/AUDIO_CLKI.
	 */
	data = 0;
	for_each_rsnd_clk(clk, adg, i) {
#ifndef QUICK_HACK
		if (rate == clk_get_rate(clk)) {
#else
		if (rate == adg->quick_hack_rate[i]) {
#endif
			data = sel_table[i];
			goto found_clock;
		}
	}

	/*
	 * find 1/6 clock from BRGA/BRGB
	 */
	if (rate == adg->rbga_rate_for_441khz_div_6) {
		data = 0x10;
		goto found_clock;
	}

	if (rate == adg->rbgb_rate_for_48khz_div_6) {
		data = 0x20;
		goto found_clock;
	}

	return -EIO;

found_clock:

	/* see rsnd_adg_ssi_clk_init() */
	rsnd_mod_bset(adg_mod, SSICKR, 0x00FF0000, adg->ckr);
	rsnd_mod_write(adg_mod, BRRA,  0x00000002); /* 1/6 */
	rsnd_mod_write(adg_mod, BRRB,  0x00000002); /* 1/6 */

	/*
	 * This "mod" = "ssi" here.
	 * we can get "ssi id" from mod
	 */
	rsnd_adg_set_ssi_clk(mod, data);

	dev_dbg(dev, "ADG: ssi%d selects clk%d = %d",
		rsnd_mod_id(mod), i, rate);

	return 0;
}

#ifdef QUICK_HACK
void rsnd_adg_clk_set_rate(struct rsnd_mod *mod, unsigned int rate)
{
	struct rsnd_priv *priv = rsnd_mod_to_priv(mod);
	struct rsnd_adg *adg = rsnd_priv_to_adg(priv);

	adg->quick_hack_rate[adg->quick_hack_used_clk] = rate / 1 * 32 * 2 * 4;
}
#endif

static void rsnd_adg_ssi_clk_init(struct rsnd_priv *priv, struct rsnd_adg *adg)
{
	struct clk *clk;
	unsigned long rate;
	u32 ckr;
	int i;
	int brg_table[] = {
		[CLKA] = 0x0,
		[CLKB] = 0x1,
		[CLKC] = 0x4,
		[CLKI] = 0x2,
	};

	/*
	 * This driver is assuming that AUDIO_CLKA/AUDIO_CLKB/AUDIO_CLKC
	 * have 44.1kHz or 48kHz base clocks for now.
	 *
	 * SSI itself can divide parent clock by 1/1 - 1/16
	 * So,  BRGA outputs 44.1kHz base parent clock 1/32,
	 * and, BRGB outputs 48.0kHz base parent clock 1/32 here.
	 * see
	 *	rsnd_adg_ssi_clk_try_start()
	 */
	ckr = 0;
	adg->rbga_rate_for_441khz_div_6 = 0;
	adg->rbgb_rate_for_48khz_div_6  = 0;
	for_each_rsnd_clk(clk, adg, i) {
#ifndef QUICK_HACK
		rate = clk_get_rate(clk);
#else
		rate = adg->quick_hack_rate[i];
#endif

		if (0 == rate) /* not used */
			continue;

		/* RBGA */
		if (!adg->rbga_rate_for_441khz_div_6 && (0 == rate % 44100)) {
			adg->rbga_rate_for_441khz_div_6 = rate / 6;
			ckr |= brg_table[i] << 20;
		}

		/* RBGB */
		if (!adg->rbgb_rate_for_48khz_div_6 && (0 == rate % 48000)) {
			adg->rbgb_rate_for_48khz_div_6 = rate / 6;
			ckr |= brg_table[i] << 16;
		}
	}

	adg->ckr = ckr;
}

int rsnd_adg_probe(struct platform_device *pdev,
		   const struct rsnd_of_data *of_data,
		   struct rsnd_priv *priv)
{
	struct rsnd_adg *adg;
	struct device *dev = rsnd_priv_to_dev(priv);
	struct clk *clk;
	int i;
#ifdef QUICK_HACK
	struct device_node *np_root, *np_pfc, *np_clk;
	const char *name;
	const char *clk_name[CLKMAX] = {
		"audio_clk_a", "audio_clk_b", "audio_clk_c", "audio_clk_i" };
#endif

	adg = devm_kzalloc(dev, sizeof(*adg), GFP_KERNEL);
	if (!adg) {
		dev_err(dev, "ADG allocate failed\n");
		return -ENOMEM;
	}

	/*
	 * ADG is special module.
	 * Use ADG mod without rsnd_mod_init() to make debug easy
	 * for rsnd_write/rsnd_read
	 */
	adg->mod.ops = &adg_ops;
	adg->mod.priv = priv;

	adg->clk[CLKA]	= devm_clk_get(dev, "clk_a");
	adg->clk[CLKB]	= devm_clk_get(dev, "clk_b");
	adg->clk[CLKC]	= devm_clk_get(dev, "clk_c");
	adg->clk[CLKI]	= devm_clk_get(dev, "clk_i");

	for_each_rsnd_clk(clk, adg, i)
		dev_dbg(dev, "clk %d : %p\n", i, clk);

#ifdef QUICK_HACK
	np_root = of_find_node_by_path("/");
	np_pfc = of_find_node_by_name(np_root, "pfc");
	np_clk = of_get_child_by_name(np_pfc, "sound_clk");
	of_property_read_string(np_clk, "renesas,groups", &name);

	for (i = 0; i < CLKMAX; i++) {
		if (!strcmp(name, clk_name[i])) {
			adg->quick_hack_used_clk = i;
			break;
		}
	}
	dev_dbg(dev, "used clk%d (name:%s)\n", i, name);
#endif

	rsnd_adg_ssi_clk_init(priv, adg);

	priv->adg = adg;

	dev_dbg(dev, "adg probed\n");

	return 0;
}
