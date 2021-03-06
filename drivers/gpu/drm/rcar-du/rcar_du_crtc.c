/*
 * rcar_du_crtc.c  --  R-Car Display Unit CRTCs
 *
 * Copyright (C) 2013-2015 Renesas Electronics Corporation
 *
 * Contact: Laurent Pinchart (laurent.pinchart@ideasonboard.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/clk.h>
#include <linux/mutex.h>
#include <linux/rcar-du-frm-interface.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "rcar_du_crtc.h"
#include "rcar_du_drv.h"
#include "rcar_du_kms.h"
#include "rcar_du_plane.h"
#include "rcar_du_regs.h"

#define  RCAR_DU_MAX_CH  3
static int du_frmend[RCAR_DU_MAX_CH] = {0};

static u32 rcar_du_crtc_read(struct rcar_du_crtc *rcrtc, u32 reg)
{
	struct rcar_du_device *rcdu = rcrtc->group->dev;

	return rcar_du_read(rcdu, rcrtc->mmio_offset + reg);
}

static void rcar_du_crtc_write(struct rcar_du_crtc *rcrtc, u32 reg, u32 data)
{
	struct rcar_du_device *rcdu = rcrtc->group->dev;

	rcar_du_write(rcdu, rcrtc->mmio_offset + reg, data);
}

static void rcar_du_crtc_clr(struct rcar_du_crtc *rcrtc, u32 reg, u32 clr)
{
	struct rcar_du_device *rcdu = rcrtc->group->dev;

	rcar_du_write(rcdu, rcrtc->mmio_offset + reg,
		      rcar_du_read(rcdu, rcrtc->mmio_offset + reg) & ~clr);
}

static void rcar_du_crtc_set(struct rcar_du_crtc *rcrtc, u32 reg, u32 set)
{
	struct rcar_du_device *rcdu = rcrtc->group->dev;

	rcar_du_write(rcdu, rcrtc->mmio_offset + reg,
		      rcar_du_read(rcdu, rcrtc->mmio_offset + reg) | set);
}

static void rcar_du_crtc_clr_set(struct rcar_du_crtc *rcrtc, u32 reg,
				 u32 clr, u32 set)
{
	struct rcar_du_device *rcdu = rcrtc->group->dev;
	u32 value = rcar_du_read(rcdu, rcrtc->mmio_offset + reg);

	rcar_du_write(rcdu, rcrtc->mmio_offset + reg, (value & ~clr) | set);
}

static int rcar_du_crtc_get(struct rcar_du_crtc *rcrtc)
{
	int ret;

	if (rcrtc->use_count != 0)
		return 0;

	rcrtc->use_count++;

	ret = clk_prepare_enable(rcrtc->clock);
	if (ret < 0)
		return ret;

	ret = rcar_du_group_get(rcrtc->group);
	if (ret < 0)
		clk_disable_unprepare(rcrtc->clock);

	return ret;
}

static void rcar_du_crtc_put(struct rcar_du_crtc *rcrtc)
{
	if (rcrtc->use_count == 0)
		return;

	rcrtc->use_count--;

	rcar_du_group_put(rcrtc->group);
	clk_disable_unprepare(rcrtc->clock);
}

static void rcar_du_crtc_set_display_timing(struct rcar_du_crtc *rcrtc)
{
	const struct drm_display_mode *mode = &rcrtc->crtc.mode;
	unsigned long clk_in = 0, clk_ex = 0;
	u32 value;
	u32 div = 0, div_in = 0, div_ex = 0;
	u32 abs_in = 0, abs_ex = 0;
	u32 dclksel_bit = 0, dclkoinv_bit = 0;
	const struct rcar_du_crtc_data *pdata =
			&rcrtc->group->dev->pdata->crtcs[rcrtc->index];
	int vdsr, vder;

	/* Internal dot clock */
	clk_in = clk_get_rate(rcrtc->clock);
	div_in = DIV_ROUND_CLOSEST(clk_in, mode->clock * 1000);

	/* External dot clock */
	if (pdata->exclk != 0) {
		clk_ex = pdata->exclk;
		div_ex = DIV_ROUND_CLOSEST(clk_ex, mode->clock * 1000);
	}

	if (div_ex) {
		/* Select recommand dot clock */
		abs_ex = abs((mode->clock * 1000) - (clk_ex / div_ex));
		abs_in = abs((mode->clock * 1000) - (clk_in / div_in));
		if (abs_ex < abs_in) {
			div = div_ex;
			dclksel_bit = ESCR_DCLKSEL_DCLKIN;
		} else {
			div = div_in;
			dclksel_bit = ESCR_DCLKSEL_CLKS;
		}
	} else {
		div = div_in;
		dclksel_bit = ESCR_DCLKSEL_CLKS;
	}

	if (dclksel_bit & ESCR_DCLKSEL_CLKS)
		dev_dbg(rcrtc->group->dev->dev,
		      "Internal clock is used in CRTC[%d]. Dot clock:%ldkHz\n",
		       rcrtc->index, ((clk_in / div_in) / 1000));
	else
		dev_dbg(rcrtc->group->dev->dev,
		      "External clock is used in CRTC[%d]. Dot clock:%ldkHz\n",
		       rcrtc->index, ((clk_ex / div_ex) / 1000));

	div = clamp(div, 1U, 64U) - 1;

	rcar_du_group_write(rcrtc->group, rcrtc->index % 2 ?
				ESCR2 : ESCR, dclksel_bit | dclkoinv_bit | div);
	rcar_du_group_write(rcrtc->group, rcrtc->index % 2 ? OTAR2 : OTAR, 0);

	/* Signal polarities */
	value = ((mode->flags & DRM_MODE_FLAG_PVSYNC) ? 0 : DSMR_VSL)
	      | ((mode->flags & DRM_MODE_FLAG_PHSYNC) ? 0 : DSMR_HSL)
	      | ((mode->flags & DRM_MODE_FLAG_INTERLACE) ? DSMR_ODEV : 0)
	      | DSMR_DIPM_DE;
	rcar_du_crtc_write(rcrtc, DSMR, value | DSMR_CSPM); /* for HDMI */

	/* Display timings */
	rcar_du_crtc_write(rcrtc, HDSR, mode->htotal - mode->hsync_start - 19);
	rcar_du_crtc_write(rcrtc, HDER, mode->htotal - mode->hsync_start +
					mode->hdisplay - 19);
	rcar_du_crtc_write(rcrtc, HSWR, mode->hsync_end -
					mode->hsync_start - 1);
	rcar_du_crtc_write(rcrtc, HCR,  mode->htotal - 1);

	if (mode->flags & DRM_MODE_FLAG_INTERLACE) {
		vdsr = (mode->vtotal / 2) - (mode->vsync_end / 2) - 2;
		vder = (mode->vtotal / 2) - (mode->vsync_end / 2) +
		       (mode->vdisplay / 2) - 2;
		if (vdsr < 1) {
			vder = vder - vdsr + 1;
			vdsr = 1;
		}
		rcar_du_crtc_write(rcrtc, VDSR, vdsr);
		rcar_du_crtc_write(rcrtc, VDER, vder);
		rcar_du_crtc_write(rcrtc, VSPR, (mode->vtotal / 2)
						 - (mode->vsync_end / 2)
						 + (mode->vsync_start / 2) - 1);
		rcar_du_crtc_write(rcrtc, VCR,  (mode->vtotal / 2) - 1);
	} else {
		vdsr = mode->vtotal - mode->vsync_end - 2;
		vder = mode->vtotal - mode->vsync_end + mode->vdisplay - 2;
		if (vdsr < 1) {
			vder = vder - vdsr + 1;
			vdsr = 1;
		}
		rcar_du_crtc_write(rcrtc, VDSR, vdsr);
		rcar_du_crtc_write(rcrtc, VDER, vder);
		rcar_du_crtc_write(rcrtc, VSPR, mode->vtotal - mode->vsync_end
						 + mode->vsync_start - 1);
		rcar_du_crtc_write(rcrtc, VCR,  mode->vtotal - 1);
	}

	rcar_du_crtc_write(rcrtc, DESR,  mode->htotal - mode->hsync_start - 1);
	rcar_du_crtc_write(rcrtc, DEWR,  mode->hdisplay);
}

void rcar_du_crtc_route_output(struct drm_crtc *crtc,
			       enum rcar_du_output output)
{
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(crtc);
	struct rcar_du_device *rcdu = rcrtc->group->dev;

	/* Store the route from the CRTC output to the DU output. The DU will be
	 * configured when starting the CRTC.
	 */
	rcrtc->outputs |= BIT(output);

	/* Store RGB routing to DPAD0 for R8A7790. */
	if (rcar_du_has(rcdu, RCAR_DU_FEATURE_DEFR8) &&
	    output == RCAR_DU_OUTPUT_DPAD0)
		rcdu->dpad0_source = rcrtc->index;
}

void rcar_du_crtc_update_planes(struct drm_crtc *crtc)
{
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(crtc);
	struct rcar_du_plane *planes[RCAR_DU_NUM_HW_PLANES];
	unsigned int num_planes = 0;
	unsigned int prio = 0;
	unsigned int i;
	u32 dptsr = 0;
	u32 dspr = 0;

	for (i = 0; i < ARRAY_SIZE(rcrtc->group->planes.planes); ++i) {
		struct rcar_du_plane *plane = &rcrtc->group->planes.planes[i];
		unsigned int j;

		if (plane->crtc != &rcrtc->crtc || !plane->enabled)
			continue;

		/* Insert the plane in the sorted planes array. */
		for (j = num_planes++; j > 0; --j) {
			if (planes[j-1]->zpos <= plane->zpos)
				break;
			planes[j] = planes[j-1];
		}

		planes[j] = plane;
		prio += plane->format->planes * 4;
	}

	for (i = 0; i < num_planes; ++i) {
		struct rcar_du_plane *plane = planes[i];
		unsigned int index = plane->hwindex;

#ifdef CONFIG_DRM_RCAR_DESKTOP_TURN_OFF
		if (!plane->fb_plane) {
			prio -= 4;
			dspr |= (index + 1) << prio;
			dptsr |= DPTSR_PnDK(index) | DPTSR_PnTS(index);
		}
#else
		prio -= 4;
		dspr |= (index + 1) << prio;
		dptsr |= DPTSR_PnDK(index) | DPTSR_PnTS(index);
#endif

		if (plane->format->planes == 2) {
			index = (index + 1) % 8;

#ifdef CONFIG_DRM_RCAR_DESKTOP_TURN_OFF
			if (!plane->fb_plane) {
				prio -= 4;
				dspr |= (index + 1) << prio;
				dptsr |= DPTSR_PnDK(index) | DPTSR_PnTS(index);
			}
#else
			prio -= 4;
			dspr |= (index + 1) << prio;
			dptsr |= DPTSR_PnDK(index) | DPTSR_PnTS(index);
#endif
		}
	}

	/* Select display timing and dot clock generator 1 for planes associated
	 * with superposition controller 1.
	 */
	if (rcrtc->index < DU_CH_2) {
		u32 value = DPTSR_MASK &
			 rcar_du_group_read(rcrtc->group, DPTSR);

		/* The DPTSR register is updated when the display controller is
		 * stopped. We thus need to restart the DU. Once again, sorry
		 * for the flicker. One way to mitigate the issue would be to
		 * pre-associate planes with CRTCs (either with a fixed 4/4
		 * split, or through a module parameter). Flicker would then
		 * occur only if we need to break the pre-association.
		 */
		if ((rcrtc->index == DU_CH_0) && ((~value & dptsr) != dptsr)) {
			rcar_du_group_write(rcrtc->group, DPTSR,
					 (value & ~dptsr));
			rcar_du_group_restart(rcrtc->group);
		}
		if ((rcrtc->index == DU_CH_1) && ((value & dptsr) != dptsr)) {
			rcar_du_group_write(rcrtc->group, DPTSR,
					 (value | dptsr));
			rcar_du_group_restart(rcrtc->group);
		}
		rcrtc->dptsr_read = DPTSR_MASK &
				 rcar_du_group_read(rcrtc->group, DPTSR);
	}

	rcar_du_group_write(rcrtc->group, rcrtc->index % 2 ? DS2PR : DS1PR,
			    dspr);
}

static void rcar_du_crtc_start(struct rcar_du_crtc *rcrtc)
{
	struct drm_crtc *crtc = &rcrtc->crtc;
	unsigned int i;
	unsigned int dptsr;

	if (rcrtc->started)
		return;

	if (WARN_ON(rcrtc->plane->format == NULL))
		return;

#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	if (rcrtc->lif_enable)
		vsp_du_if_start(rcrtc->vpsd_handle);
#endif

	/* Set display off and background to black */
	rcar_du_crtc_write(rcrtc, DOOR, DOOR_RGB(0, 0, 0));
	rcar_du_crtc_write(rcrtc, BPOR, BPOR_RGB(0, 0, 0));

	/* Initialized DPTSR register */
	if ((rcrtc->group->dptsr_init) && (rcrtc->index < DU_CH_2)) {
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
		dptsr = rcrtc->group->dptsr_init_val;
#else
		dptsr = ((CONFIG_DRM_RCAR_DU_OVERLAY_CH << DPTSR_DK_BIT_SHIFT) |
			(CONFIG_DRM_RCAR_DU_OVERLAY_CH << DPTSR_TS_BIT_SHIFT));
#endif
		rcar_du_group_write(rcrtc->group, DPTSR, DPTSR_MASK
		     & (rcar_du_group_read(rcrtc->group, DPTSR) | dptsr));
		rcar_du_group_restart(rcrtc->group);
		rcrtc->group->dptsr_init = false;
	}

	/* Configure display timings and output routing */
	rcar_du_crtc_set_display_timing(rcrtc);
	rcar_du_group_set_routing(rcrtc->group);

	mutex_lock(&rcrtc->group->planes.lock);
	rcrtc->plane->enabled = true;
	rcar_du_crtc_update_planes(crtc);
	mutex_unlock(&rcrtc->group->planes.lock);

	/* Setup planes. */
	for (i = 0; i < ARRAY_SIZE(rcrtc->group->planes.planes); ++i) {
		struct rcar_du_plane *plane = &rcrtc->group->planes.planes[i];

		if (plane->crtc != crtc || !plane->enabled)
			continue;

		if (rcrtc->crtc.mode.flags & DRM_MODE_FLAG_INTERLACE)
			plane->interlace_flag = true;
		else
			plane->interlace_flag = false;
		rcar_du_plane_setup(plane);
	}
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	if (rcrtc->lif_enable) {
		const struct rcar_du_crtc_data *pdata =
			&rcrtc->group->dev->pdata->crtcs[rcrtc->index];

		if ((pdata->vsp == RCAR_DU_VSPD_1) ||
			(pdata->vsp == RCAR_DU_VSPD_1_RGB)) {
			struct rcar_du_plane *rplane = rcrtc->plane;
			struct rcar_du_device *rcdu = rplane->group->dev;
			unsigned int vspd1_sink =
				rplane->group->index ? 2 : 0;

			if (rcdu->vspd1_sink != vspd1_sink) {
				rcdu->vspd1_sink = vspd1_sink;
				rcar_du_set_dpad0_vsp1_routing(rcdu);
			}
		}
		rcar_du_group_restart(rcrtc->group);
	}
#endif

	/* Select master sync mode. This enables display operation in master
	 * sync mode (with the HSYNC and VSYNC signals configured as outputs and
	 * actively driven).
	 */
	rcar_du_crtc_clr_set(rcrtc, DSYSR, DSYSR_TVM_MASK, DSYSR_TVM_MASTER);

	if (rcrtc->crtc.mode.flags & DRM_MODE_FLAG_INTERLACE) {
		if (rcrtc->index == 1)
			rcar_du_crtc_clr_set(rcrtc, DSYSR,
				DSYSR_SCM_INT_VIDEO, DSYSR_SCM_INT_VIDEO);
		else
			rcrtc->group->interlace_grp = true;
	} else {
		if (rcrtc->index == 1)
			rcar_du_crtc_clr_set(rcrtc, DSYSR,
				DSYSR_SCM_INT_VIDEO, 0);
		else
			rcrtc->group->interlace_grp = false;
	}
	rcar_du_group_start_stop(rcrtc->group, true);

	rcrtc->started = true;

#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	rcar_du_crtc_write(rcrtc, DSRCR, DSRCR_VBCL);
	rcar_du_crtc_set(rcrtc, DIER, DIER_VBE);
#endif
}

static void rcar_du_crtc_stop(struct rcar_du_crtc *rcrtc)
{
	struct drm_crtc *crtc = &rcrtc->crtc;

	if (!rcrtc->started)
		return;

#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	rcar_du_crtc_clr(rcrtc, DIER, DIER_VBE);
#endif
	mutex_lock(&rcrtc->group->planes.lock);
	rcrtc->plane->enabled = false;
	rcar_du_crtc_update_planes(crtc);
	mutex_unlock(&rcrtc->group->planes.lock);

	/* Select switch sync mode. This stops display operation and configures
	 * the HSYNC and VSYNC signals as inputs.
	 */
	rcar_du_crtc_clr_set(rcrtc, DSYSR, DSYSR_TVM_MASK, DSYSR_TVM_SWITCH);

	if (rcrtc->crtc.mode.flags & DRM_MODE_FLAG_INTERLACE) {
		if (rcrtc->index == 1)
			rcar_du_crtc_clr_set(rcrtc,
				DSYSR, DSYSR_SCM_INT_VIDEO, 0);
		else
			rcrtc->group->interlace_grp = false;
	}

	rcar_du_group_start_stop(rcrtc->group, false);

	rcrtc->started = false;

#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	if (rcrtc->lif_enable)
		vsp_du_if_stop(rcrtc->vpsd_handle);
#endif
}

void rcar_du_crtc_suspend(struct rcar_du_crtc *rcrtc)
{
	rcar_du_crtc_stop(rcrtc);
	rcar_du_crtc_put(rcrtc);
}

void rcar_du_crtc_resume(struct rcar_du_crtc *rcrtc)
{
	if (rcrtc->dpms != DRM_MODE_DPMS_ON)
		return;

	rcar_du_crtc_get(rcrtc);
	rcar_du_crtc_start(rcrtc);
}

static void rcar_du_crtc_update_base(struct rcar_du_crtc *rcrtc)
{
	struct drm_crtc *crtc = &rcrtc->crtc;

#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	rcar_du_plane_compute_base(rcrtc->plane, crtc->fb);

	if (rcrtc->lif_enable)
		vsp_du_if_update_base(rcrtc->vpsd_handle, rcrtc->plane);
	else
		rcar_du_plane_update_base(rcrtc->plane);
#else
	rcrtc->plane->pitch = crtc->fb->pitches[0];

	rcar_du_plane_compute_base(rcrtc->plane, crtc->fb);

	rcar_du_plane_update_base(rcrtc->plane);
#endif
}


static void rcar_du_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(crtc);

	if (((rcrtc->dpms == DRM_MODE_DPMS_ON)
		&& (mode == DRM_MODE_DPMS_ON))
		|| ((rcrtc->dpms != DRM_MODE_DPMS_ON)
		&& (mode != DRM_MODE_DPMS_ON)))
		return;

	if (mode == DRM_MODE_DPMS_ON) {
		rcar_du_crtc_get(rcrtc);
		rcar_du_crtc_start(rcrtc);
	} else {
		rcar_du_crtc_stop(rcrtc);
		rcar_du_crtc_put(rcrtc);
	}

	rcrtc->dpms = mode;
}

static bool rcar_du_crtc_mode_fixup(struct drm_crtc *crtc,
				    const struct drm_display_mode *mode,
				    struct drm_display_mode *adjusted_mode)
{
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(crtc);
	struct rcar_du_device *rcdu = rcrtc->group->dev;

	/* It is prohibition to set up the width of the multiple of 16
	   for the specification of H/W in R-Car series */
	if ((mode) && (mode->hdisplay % 16)) {
		dev_err(rcdu->dev,
			"Error! width of the multiple of 16 is prohibition\n");
		return false;
	}
	return true;
}

static void rcar_du_crtc_mode_prepare(struct drm_crtc *crtc)
{
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(crtc);

	/* We need to access the hardware during mode set, acquire a reference
	 * to the CRTC.
	 */
	rcar_du_crtc_get(rcrtc);

	/* Stop the CRTC and release the plane. Force the DPMS mode to off as a
	 * result.
	 */
	rcar_du_crtc_stop(rcrtc);
	rcar_du_plane_release(rcrtc->plane);

	rcrtc->dpms = DRM_MODE_DPMS_OFF;
}

static int rcar_du_crtc_mode_set(struct drm_crtc *crtc,
				 struct drm_display_mode *mode,
				 struct drm_display_mode *adjusted_mode,
				 int x, int y,
				 struct drm_framebuffer *old_fb)
{
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(crtc);
	struct rcar_du_device *rcdu = rcrtc->group->dev;
	const struct rcar_du_format_info *format;
	int ret;
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	enum rcar_du_plane_source source;
	struct rcar_du_crtc_data *pdata =
			&rcrtc->group->dev->pdata->crtcs[rcrtc->index];
#endif

	format = rcar_du_format_info(crtc->fb->pixel_format);
	if (format == NULL) {
		dev_dbg(rcdu->dev, "mode_set: unsupported format %08x\n",
			crtc->fb->pixel_format);
		ret = -EINVAL;
		goto error;
	}

#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	if ((pdata->vsp == RCAR_DU_VSPD_0) || (pdata->vsp == RCAR_DU_VSPD_1)) {
		if (rcrtc->vpsd_handle) {
			source = pdata->vsp == RCAR_DU_VSPD_0 ?
				RCAR_DU_PLANE_VSPD0 : RCAR_DU_PLANE_VSPD1;
			rcrtc->lif_enable = 1;
		}
	} else if ((pdata->vsp == RCAR_DU_VSPD_0_RGB) ||
		   (pdata->vsp == RCAR_DU_VSPD_1_RGB)) {
		switch (format->fourcc) {
		case DRM_FORMAT_XRGB8888:
		case DRM_FORMAT_ARGB8888:
			if (rcrtc->vpsd_handle) {
				source = pdata->vsp == RCAR_DU_VSPD_0_RGB ?
				      RCAR_DU_PLANE_VSPD0 : RCAR_DU_PLANE_VSPD1;
				rcrtc->lif_enable = 1;
				break;
			}
		default:
			source = RCAR_DU_PLANE_MEMORY;
			rcrtc->lif_enable = 0;
			break;
		}
	} else if (pdata->vsp == RCAR_DU_VSPD_UNUSED) {
		source = RCAR_DU_PLANE_MEMORY;
		rcrtc->lif_enable = 0;
	}
	ret = rcar_du_plane_reserve_src(rcrtc->plane, format, source);
#else
	ret = rcar_du_plane_reserve(rcrtc->plane, format);
#endif
	if (ret < 0)
		goto error;

	rcrtc->plane->format = format;

	rcrtc->plane->src_x = x;
	rcrtc->plane->src_y = y;
	rcrtc->plane->width = mode->hdisplay;
	rcrtc->plane->height = mode->vdisplay;
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	rcrtc->plane->d_width = mode->hdisplay;
	rcrtc->plane->d_height = mode->vdisplay;
#endif
	rcar_du_plane_compute_base(rcrtc->plane, crtc->fb);

	rcrtc->outputs = 0;

#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	if (rcrtc->lif_enable)
		vsp_du_if_setup_base(rcrtc->vpsd_handle, rcrtc->plane,
			rcrtc->crtc.mode.flags & DRM_MODE_FLAG_INTERLACE ?
							true : false);
#endif

	return 0;

error:
	/* There's no rollback/abort operation to clean up in case of error. We
	 * thus need to release the reference to the CRTC acquired in prepare()
	 * here.
	 */
	rcar_du_crtc_put(rcrtc);
	return ret;
}

static void rcar_du_crtc_mode_commit(struct drm_crtc *crtc)
{
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(crtc);

	/* We're done, restart the CRTC and set the DPMS mode to on. The
	 * reference to the DU acquired at prepare() time will thus be released
	 * by the DPMS handler (possibly called by the disable() handler).
	 */
	rcar_du_crtc_start(rcrtc);
	rcrtc->dpms = DRM_MODE_DPMS_ON;

}

static int rcar_du_crtc_mode_set_base(struct drm_crtc *crtc, int x, int y,
				      struct drm_framebuffer *old_fb)
{
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(crtc);

	rcrtc->plane->src_x = x;
	rcrtc->plane->src_y = y;

	rcar_du_crtc_update_base(rcrtc);

	return 0;
}

static void rcar_du_crtc_disable(struct drm_crtc *crtc)
{
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(crtc);

	rcar_du_crtc_dpms(crtc, DRM_MODE_DPMS_OFF);
	rcar_du_plane_release(rcrtc->plane);
}

static void rcar_du_crtc_load_lut(struct drm_crtc *crtc)
{
}

static const struct drm_crtc_helper_funcs crtc_helper_funcs = {
	.dpms = rcar_du_crtc_dpms,
	.mode_fixup = rcar_du_crtc_mode_fixup,
	.prepare = rcar_du_crtc_mode_prepare,
	.commit = rcar_du_crtc_mode_commit,
	.mode_set = rcar_du_crtc_mode_set,
	.mode_set_base = rcar_du_crtc_mode_set_base,
	.disable = rcar_du_crtc_disable,
	.load_lut = rcar_du_crtc_load_lut,
};

void rcar_du_crtc_cancel_page_flip(struct rcar_du_crtc *rcrtc,
				   struct drm_file *file)
{
	struct drm_pending_vblank_event *event;
	struct drm_device *dev = rcrtc->crtc.dev;
	unsigned long flags;

	/* Destroy the pending vertical blanking event associated with the
	 * pending page flip, if any, and disable vertical blanking interrupts.
	 */
	spin_lock_irqsave(&dev->event_lock, flags);
	event = rcrtc->event;
	if (event && event->base.file_priv == file) {
		rcrtc->event = NULL;
		event->base.destroy(&event->base);
		drm_vblank_put(dev, rcrtc->index);
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);
}

static void rcar_du_crtc_finish_page_flip(struct rcar_du_crtc *rcrtc)
{
	struct drm_pending_vblank_event *event;
	struct drm_device *dev = rcrtc->crtc.dev;
	unsigned long flags;

	spin_lock_irqsave(&dev->event_lock, flags);
	event = rcrtc->event;
	rcrtc->event = NULL;
	spin_unlock_irqrestore(&dev->event_lock, flags);

	if (event == NULL)
		return;

	spin_lock_irqsave(&dev->event_lock, flags);
	drm_send_vblank_event(dev, rcrtc->index, event);
	spin_unlock_irqrestore(&dev->event_lock, flags);

	drm_vblank_put(dev, rcrtc->index);
}

int rcar_du_get_frmend(unsigned int ch)
{
	if (ch < RCAR_DU_MAX_CH)
		return du_frmend[ch];
	else
		return -EINVAL;
}
EXPORT_SYMBOL(rcar_du_get_frmend);

static void rcar_du_set_frmend(int frmend, unsigned int ch)
{
	du_frmend[ch] = frmend;
}


#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
static void rcar_du_crtc_irq_callback(void *data)
{
	struct rcar_du_crtc *rcrtc = data;

	drm_handle_vblank(rcrtc->crtc.dev, rcrtc->index);
	rcar_du_crtc_finish_page_flip(rcrtc);
}
#endif

static irqreturn_t rcar_du_crtc_irq(int irq, void *arg)
{
	struct rcar_du_crtc *rcrtc = arg;
	irqreturn_t ret = IRQ_NONE;
	u32 status;

	status = rcar_du_crtc_read(rcrtc, DSSR);
	rcar_du_crtc_write(rcrtc, DSRCR, status & DSRCR_MASK);

	if (status & DSSR_FRM) {
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
		if (!rcrtc->lif_enable)
			rcar_du_crtc_irq_callback(rcrtc);
#else
		drm_handle_vblank(rcrtc->crtc.dev, rcrtc->index);
		rcar_du_crtc_finish_page_flip(rcrtc);
#endif
		ret = IRQ_HANDLED;
	}

	rcar_du_set_frmend((status & DSSR_FRM) ? 1 : 0, rcrtc->index);

	return ret;
}

static int rcar_du_crtc_page_flip(struct drm_crtc *crtc,
				  struct drm_framebuffer *fb,
				  struct drm_pending_vblank_event *event,
				  uint32_t page_flip_flags)
{
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(crtc);
	struct drm_device *dev = rcrtc->crtc.dev;
	unsigned long flags;

	spin_lock_irqsave(&dev->event_lock, flags);
	if (rcrtc->event != NULL) {
		spin_unlock_irqrestore(&dev->event_lock, flags);
		return -EBUSY;
	}
	spin_unlock_irqrestore(&dev->event_lock, flags);

	crtc->fb = fb;
	rcar_du_crtc_update_base(rcrtc);

	if (event) {
		event->pipe = rcrtc->index;
		drm_vblank_get(dev, rcrtc->index);
		spin_lock_irqsave(&dev->event_lock, flags);
		rcrtc->event = event;
		spin_unlock_irqrestore(&dev->event_lock, flags);
	}

	return 0;
}

#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
void rcar_du_crtc_cleanup(struct drm_crtc *crtc)
{
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(crtc);

	if (rcrtc->vpsd_handle)
		vsp_du_if_deinit(rcrtc->vpsd_handle);
	drm_crtc_cleanup(crtc);
}
#endif

static const struct drm_crtc_funcs crtc_funcs = {
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	.destroy = rcar_du_crtc_cleanup,
#else
	.destroy = drm_crtc_cleanup,
#endif
	.set_config = drm_crtc_helper_set_config,
	.page_flip = rcar_du_crtc_page_flip,
};

int rcar_du_crtc_create(struct rcar_du_group *rgrp, unsigned int index)
{
	static const unsigned int mmio_offsets[] = {
		DU0_REG_OFFSET, DU1_REG_OFFSET, DU2_REG_OFFSET
	};

	struct rcar_du_device *rcdu = rgrp->dev;
	struct platform_device *pdev = to_platform_device(rcdu->dev);
	struct rcar_du_crtc *rcrtc = &rcdu->crtcs[index];
	struct drm_crtc *crtc = &rcrtc->crtc;
	unsigned int irqflags;
	char clk_name[5];
	char *name;
	int irq;
	int ret;
	struct rcar_du_crtc_data *pdata =
			&rgrp->dev->pdata->crtcs[index];
#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	int plane_bit;
	unsigned int vsp_ch;
	bool error = false;
#endif

	/* Get the CRTC clock. */
	if (rcar_du_has(rcdu, RCAR_DU_FEATURE_CRTC_IRQ_CLOCK)) {
		sprintf(clk_name, "du.%u", index);
		name = clk_name;
	} else {
		name = NULL;
	}

	rcrtc->clock = devm_clk_get(rcdu->dev, name);
	if (IS_ERR(rcrtc->clock)) {
		dev_err(rcdu->dev, "no clock for CRTC %u\n", index);
		return PTR_ERR(rcrtc->clock);
	}

	rcrtc->group = rgrp;
	rcrtc->mmio_offset = mmio_offsets[index];
	rcrtc->index = index;
	rcrtc->dpms = DRM_MODE_DPMS_OFF;
	rcrtc->plane = &rgrp->planes.planes[index % 2];
	rcrtc->lvds_ch = -1;

	if (pdata->init_conn_type)
		crtc->connector_type = pdata->init_conn_type;
	else
		crtc->connector_type = DRM_MODE_CONNECTOR_Unknown;

#if !defined(CONFIG_DRM_ADV7511) && !defined(CONFIG_DRM_ADV7511_MODULE)
	if ((pdata->init_conn_type) &&
		(crtc->connector_type == DRM_MODE_CONNECTOR_HDMIA)) {
		if (rcdu->info->chip == RCAR_H2)
			crtc->connector_type = DRM_MODE_CONNECTOR_VGA;
		if ((rcdu->info->chip == RCAR_M2) ||
			(rcdu->info->chip == RCAR_M2N))
			crtc->connector_type = DRM_MODE_CONNECTOR_Unknown;
		if (rcdu->info->chip == RCAR_E2)
			crtc->connector_type = DRM_MODE_CONNECTOR_LVDS;
	}
#endif
#ifndef CONFIG_DRM_RCAR_LVDS
	if (pdata->init_conn_type) {
		if ((crtc->connector_type == DRM_MODE_CONNECTOR_HDMIA) &&
			(rcdu->info->chip == RCAR_H2))
			crtc->connector_type = DRM_MODE_CONNECTOR_VGA;
		if ((crtc->connector_type == DRM_MODE_CONNECTOR_LVDS) &&
			((rcdu->info->chip == RCAR_M2) ||
			(rcdu->info->chip == RCAR_M2N) ||
			(rcdu->info->chip == RCAR_E2)))
			crtc->connector_type = DRM_MODE_CONNECTOR_Unknown;
	}
#endif
	rcrtc->plane->crtc = crtc;
	rcrtc->plane->fb_plane = true;

	ret = drm_crtc_init(rcdu->ddev, crtc, &crtc_funcs);
	if (ret < 0)
		return ret;

	rcdu->crtcs_connect_id[index] = rcrtc->plane->crtc->base.id;

	drm_crtc_helper_add(crtc, &crtc_helper_funcs);

	/* Register the interrupt handler. */
	if (rcar_du_has(rcdu, RCAR_DU_FEATURE_CRTC_IRQ_CLOCK)) {
		irq = platform_get_irq(pdev, index);
		irqflags = 0;
	} else {
		irq = platform_get_irq(pdev, 0);
		irqflags = IRQF_SHARED;
	}

	if (irq < 0) {
		dev_err(rcdu->dev, "no IRQ for CRTC %u\n", index);
		return ret;
	}

	rcdu->ddev->irq_enabled = true;

	ret = devm_request_irq(rcdu->dev, irq, rcar_du_crtc_irq, irqflags,
			       dev_name(rcdu->dev), rcrtc);
	if (ret < 0) {
		dev_err(rcdu->dev,
			"failed to register IRQ for CRTC %u\n", index);
		return ret;
	}

#ifdef CONFIG_DRM_RCAR_DU_CONNECT_VSP
	/* Check configuration of VSPD channel used by DU */
	if ((pdata->vsp >= RCAR_DU_VSPD_MAX) ||
		(pdata->vsp < RCAR_DU_VSPD_UNUSED)) {
		dev_err(rcdu->dev,
		 "failed to set configuration value[%d] specified in DU%u\n"
		 , pdata->vsp, index);
		return -EINVAL;
	}

	if (pdata->vsp >= RCAR_DU_VSPD_0_RGB)
		vsp_ch = pdata->vsp - RCAR_DU_VSPD_0_RGB;
	else
		vsp_ch = pdata->vsp;

	if (vsp_ch != RCAR_DU_VSPD_UNUSED) {
		/* R8A7794 is not supported VSPD1 */
		if ((rcdu->info->chip == RCAR_E2) &&
			((0x01 << vsp_ch) & BIT(1)))
			vsp_ch = pdata->vsp = RCAR_DU_VSPD_UNUSED;
		else if ((rcdu->info->chip == RCAR_H2) && (index == DU_CH_2) &&
			((0x01 << vsp_ch) & BIT(0))) {
			error = true;
		} else if ((rcdu->vsp_reserve) & (0x01 << vsp_ch))
			rcdu->vsp_reserve &= ~(0x01 << vsp_ch);
		else {
			error = true;
		}
	}
	if (error) {
		dev_err(rcdu->dev,
		 "failed to set configuration DU%u->VSPD%u.\n"
		 , index, vsp_ch);
		return -EINVAL;
	}

	rcrtc->lif_enable = 0;
	if (vsp_ch == RCAR_DU_VSPD_UNUSED) {
		rcrtc->vpsd_handle = NULL;
		goto end;
	} else if (DU_CH_2 <= rcrtc->index) {
		if (vsp_ch == RCAR_DU_VSPD_0) {
			rcrtc->vpsd_handle = NULL;
			goto end;
		}
		dev_info(rcdu->dev, "DU%d use VSPD%d\n", index, vsp_ch);
		plane_bit = 0x01;
	} else {
		dev_info(rcdu->dev, "DU%d use VSPD%d\n", index, vsp_ch);
		switch (vsp_ch) {
		case RCAR_DU_VSPD_0:
			plane_bit = 0x01;
			break;
		case RCAR_DU_VSPD_1:
			plane_bit = 0x02;
			break;
		default:
			/* for build warning */
			break;
		}
	}

	plane_bit = (plane_bit << DPTSR_DK_BIT_SHIFT) |
		    (plane_bit << DPTSR_TS_BIT_SHIFT);

	rcrtc->vpsd_handle = vsp_du_if_init(rcdu->dev, vsp_ch);
	if (rcrtc->vpsd_handle == NULL) {
		dev_err(rcdu->dev, "[Error] vsp_du_if_init\n");
	} else {
		if ((rcrtc->index % 2) == 0)
			rcrtc->group->dptsr_init_val &= ~plane_bit;
		else
			rcrtc->group->dptsr_init_val |= plane_bit;

		vsp_du_if_set_callback(rcrtc->vpsd_handle,
			    rcar_du_crtc_irq_callback, rcrtc);
	}
end:
#endif

	return 0;
}

void rcar_du_crtc_enable_vblank(struct rcar_du_crtc *rcrtc, bool enable)
{
	if (enable) {
		rcar_du_crtc_write(rcrtc, DSRCR, DSRCR_VBCL);
		rcar_du_crtc_set(rcrtc, DIER, DIER_VBE);
	} else {
		rcar_du_crtc_clr(rcrtc, DIER, DIER_VBE);
	}
}
