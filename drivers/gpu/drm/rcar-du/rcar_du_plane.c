/*
 * rcar_du_plane.c  --  R-Car Display Unit Planes
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

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_gem_cma_helper.h>

#include "rcar_du_drv.h"
#include "rcar_du_group.h"
#include "rcar_du_kms.h"
#include "rcar_du_plane.h"
#include "rcar_du_regs.h"

/* -----------------------------------------------------------------------------
 * Live Sources
 */

struct rcar_du_vsp1_source {
	struct drm_live_source base;

	enum rcar_du_plane_source source;
};

static inline struct rcar_du_vsp1_source *
to_rcar_vsp1_source(struct drm_live_source *src)
{
	return container_of(src, struct rcar_du_vsp1_source, base);
}

static const struct drm_live_source_funcs rcar_du_live_source_funcs = {
	.destroy = drm_live_source_cleanup,
};

static const uint32_t source_formats[] = {
	DRM_FORMAT_XRGB8888,
};

int rcar_du_vsp1_sources_init(struct rcar_du_device *rcdu)
{
	static const struct {
		enum rcar_du_plane_source source;
		unsigned int planes;
	} sources[] = {
		{ RCAR_DU_PLANE_VSPD0, BIT(RCAR_DU01_NUM_KMS_PLANES - 1) },
		{ RCAR_DU_PLANE_VSPD1, BIT(RCAR_DU01_NUM_KMS_PLANES - 2) |
				       BIT(2 * RCAR_DU2_NUM_KMS_PLANES - 2) },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(sources); ++i) {
		struct rcar_du_vsp1_source *src;
		char name[6];
		int ret;

		if ((rcdu->info->vspd_crtc & (0x01 << i)) == 0)
			continue;

		src = devm_kzalloc(rcdu->dev, sizeof(*src), GFP_KERNEL);
		if (src == NULL)
			return -ENOMEM;

		src->source = sources[i].source;

		sprintf(name, "vspd%u", i);
		ret = drm_live_source_init(rcdu->ddev, &src->base, name,
					   sources[i].planes, source_formats,
					   ARRAY_SIZE(source_formats),
					   &rcar_du_live_source_funcs);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/* -----------------------------------------------------------------------------
 * Planes
 */

#define RCAR_DU_COLORKEY_NONE		(0 << 24)
#define RCAR_DU_COLORKEY_SOURCE		(1 << 24)
#define RCAR_DU_COLORKEY_MASK		(1 << 24)

struct rcar_du_kms_plane {
	struct drm_plane plane;
	struct rcar_du_plane *hwplane;
};

static inline struct rcar_du_plane *to_rcar_plane(struct drm_plane *plane)
{
	return container_of(plane, struct rcar_du_kms_plane, plane)->hwplane;
}

static void rcar_du_plane_write(struct rcar_du_group *rgrp,
				unsigned int index, u32 reg, u32 data)
{
	rcar_du_write(rgrp->dev, rgrp->mmio_offset + index * PLANE_OFF + reg,
		      data);
}

/*
 * The R8A7790 DU can source frames directly from the VSP1 devices VSPD0 and
 * VSPD1. VSPD0 feeds DU0/1 plane 0, and VSPD1 feeds either DU2 plane 0 or
 * DU0/1 plane 1.
 *
 * Allocate the correct fixed plane when sourcing frames from VSPD0 or VSPD1,
 * and allocate planes in reverse index order otherwise to ensure maximum
 * availability of planes 0 and 1.
 *
 * The caller is responsible for ensuring that the requested source is
 * compatible with the DU revision.
 */
static int rcar_du_plane_find(struct rcar_du_group *rgrp, unsigned int count,
			      enum rcar_du_plane_source source)
{
	int fixed = -1;
	int i;

	if (source == RCAR_DU_PLANE_VSPD0) {
		/* VSPD0 feeds plane 0 on DU0/1. */
		if (rgrp->index != 0)
			return -EINVAL;

		fixed = 0;
	} else if (source == RCAR_DU_PLANE_VSPD1) {
		/* VSPD1 feeds plane 1 on DU0/1 or plane 0 on DU2. */
		fixed = rgrp->index == 0 ? 1 : 0;
	}

	if (fixed >= 0)
		return rgrp->planes.free & (1 << fixed) ? fixed : -EBUSY;

#ifdef RCAR_DU_CONNECT_VSP
	for (i = 2; i < (ARRAY_SIZE(rgrp->planes.planes) - 1); i++) {
#else
	for (i = ARRAY_SIZE(rgrp->planes.planes) - 1; i >= 0; --i) {
#endif
		if (!(rgrp->planes.free & (1 << i)))
			continue;

		if (count == 1 ||
		    rgrp->planes.free & (1 << ((i + 1) % 8)))
			break;
	}

	return i < 0 ? -EBUSY : i;
}

static int __rcar_du_plane_reserve(struct rcar_du_plane *plane,
				   const struct rcar_du_format_info *format,
				   enum rcar_du_plane_source source)
{
	struct rcar_du_group *rgrp = plane->group;
	unsigned int hwindex;
	int ret;

	mutex_lock(&rgrp->planes.lock);

	ret = rcar_du_plane_find(rgrp, format->planes, source);
	if (ret < 0)
		goto done;

	hwindex = ret;

	rgrp->planes.free &= ~(1 << hwindex);
	if (format->planes == 2)
		rgrp->planes.free &= ~(1 << ((hwindex + 1) % 8));

	plane->hwindex = hwindex;
	plane->source = source;

	ret = 0;

done:
	mutex_unlock(&rgrp->planes.lock);
	return ret;
}

int rcar_du_plane_reserve(struct rcar_du_plane *plane,
			  const struct rcar_du_format_info *format)
{
	return __rcar_du_plane_reserve(plane, format, RCAR_DU_PLANE_MEMORY);
}

#ifdef RCAR_DU_CONNECT_VSP
int rcar_du_plane_reserve_src(struct rcar_du_plane *plane,
			  const struct rcar_du_format_info *format,
			  enum rcar_du_plane_source source)
{
	return __rcar_du_plane_reserve(plane, format, source);
}
#endif

void rcar_du_plane_release(struct rcar_du_plane *plane)
{
	struct rcar_du_group *rgrp = plane->group;

	if (plane->hwindex == -1)
		return;

	mutex_lock(&rgrp->planes.lock);
	rgrp->planes.free |= 1 << plane->hwindex;
	if (plane->format->planes == 2)
		rgrp->planes.free |= 1 << ((plane->hwindex + 1) % 8);
	mutex_unlock(&rgrp->planes.lock);

	plane->hwindex = -1;
}

void rcar_du_plane_update_base(struct rcar_du_plane *plane)
{
	struct rcar_du_group *rgrp = plane->group;
	unsigned int index = plane->hwindex;
	u32 mwr;

	/* Memory pitch (expressed in pixels) */
	if (plane->format->planes == 2)
		mwr = plane->pitch;
	else
		mwr = plane->pitch * 8 / plane->format->bpp;

	if ((plane->interlace_flag) && (plane->format->bpp == 32))
		rcar_du_plane_write(rgrp, index, PnMWR, mwr * 2);
	else
		rcar_du_plane_write(rgrp, index, PnMWR, mwr);

	/* The Y position is expressed in raster line units and must be doubled
	 * for 32bpp formats, according to the R8A7790 datasheet. No mention of
	 * doubling the Y position is found in the R8A7779 datasheet, but the
	 * rule seems to apply there as well.
	 *
	 * Similarly, for the second plane, NV12 and NV21 formats seem to
	 * require a halved Y position value.
	 */
	rcar_du_plane_write(rgrp, index, PnSPXR, plane->src_x);
	if ((!plane->interlace_flag) && (plane->format->bpp == 32))
		rcar_du_plane_write(rgrp, index, PnSPYR, plane->src_y * 2);
	else
		rcar_du_plane_write(rgrp, index, PnSPYR, plane->src_y);
	rcar_du_plane_write(rgrp, index, PnDSA0R, plane->dma[0]);

	if (plane->format->planes == 2) {
		index = (index + 1) % 8;

		rcar_du_plane_write(rgrp, index, PnSPXR, plane->src_x);
		rcar_du_plane_write(rgrp, index, PnSPYR, plane->src_y *
				    (plane->format->bpp == 16 ? 2 : 1) / 2);
		rcar_du_plane_write(rgrp, index, PnDSA0R, plane->dma[1]);
	}
}

void rcar_du_plane_compute_base(struct rcar_du_plane *plane,
				struct drm_framebuffer *fb)
{
	struct drm_gem_cma_object *gem;

	plane->pitch = fb->pitches[0];

	gem = drm_fb_cma_get_gem_obj(fb, 0);
	plane->dma[0] = gem->paddr + fb->offsets[0];

	if (plane->format->planes == 2) {
		gem = drm_fb_cma_get_gem_obj(fb, 1);
		plane->dma[1] = gem->paddr + fb->offsets[1];
	}
}

static void rcar_du_plane_setup_mode(struct rcar_du_plane *plane,
				     unsigned int index)
{
	struct rcar_du_group *rgrp = plane->group;
	u32 colorkey;
	u32 pnmr;
	u32 alpha_bit;

	/* The PnALPHAR register controls alpha-blending in formats
	 * (XRGB0565, XRGB1555, ARGB1555 and XRGB8888)
	 * by setting the alpha value.
	 *
	 * For ARGB1555, set the alpha value from 0 to 255, and enable
	 * the pixel alpha-blending when the value of A bit is 0 or 1.
	 * The pixel alpha-blending by the A bit depends on setting the config
	 * of DRM_RCAR_ALPHA_BIT_ARGB1555
	 *
	 * For ARGB8888, set the value of the A bit from 0 to 255, and enable
	 * the pixel alpha-blending. A=0 is transparent.
	 * A=128 is semi-transparent. A=255 is non-transparent.
	 *
	 * For XRGB, set the alpha value to the plane-wide alpha value and
	 * enable alpha-blending regardless of the X bit value.
	 */
	if (CONFIG_DRM_RCAR_ALPHA_BIT_ARGB1555 == 1)
		alpha_bit = PnALPHAR_ABIT_1;
	else
		alpha_bit = PnALPHAR_ABIT_0;

	if (plane->format->fourcc == DRM_FORMAT_ARGB1555)
		rcar_du_plane_write(rgrp, index, PnALPHAR,
				    alpha_bit | plane->alpha);
	else
		rcar_du_plane_write(rgrp, index, PnALPHAR,
				    PnALPHAR_ABIT_X | plane->alpha);

	/* Disable alpha blending of lowest plane */
	if (plane->fb_plane == true)
		pnmr = PnMR_BM_MD | ((plane->format->pnmr) & ~PnMR_SPIM_MASK) |
		       PnMR_SPIM_TP_OFF;
	else
		pnmr = PnMR_BM_MD | plane->format->pnmr;

	/* Disable color keying when requested. YUV formats have the
	 * PnMR_SPIM_TP_OFF bit set in their pnmr field, disabling color keying
	 * automatically.
	 */
	if ((plane->colorkey & RCAR_DU_COLORKEY_MASK) == RCAR_DU_COLORKEY_NONE)
		pnmr |= PnMR_SPIM_TP_OFF;

	/* For packed YUV formats we need to select the U/V order. */
	if (plane->format->fourcc == DRM_FORMAT_YUYV)
		pnmr |= PnMR_YCDF_YUYV;

	rcar_du_plane_write(rgrp, index, PnMR, pnmr);

	switch (plane->format->fourcc) {
	case DRM_FORMAT_RGB565:
		colorkey = ((plane->colorkey & 0xf80000) >> 8)
			 | ((plane->colorkey & 0x00fc00) >> 5)
			 | ((plane->colorkey & 0x0000f8) >> 3);
		rcar_du_plane_write(rgrp, index, PnTC2R, colorkey);
		break;

	case DRM_FORMAT_ARGB1555:
	case DRM_FORMAT_XRGB1555:
		colorkey = ((plane->colorkey & 0xf80000) >> 9)
			 | ((plane->colorkey & 0x00f800) >> 6)
			 | ((plane->colorkey & 0x0000f8) >> 3);
		rcar_du_plane_write(rgrp, index, PnTC2R, colorkey);
		break;

	case DRM_FORMAT_XRGB8888:
	case DRM_FORMAT_ARGB8888:
		rcar_du_plane_write(rgrp, index, PnTC3R,
				    PnTC3R_CODE | (plane->colorkey & 0xffffff));
		break;
	}
}

static void __rcar_du_plane_setup(struct rcar_du_plane *plane,
				  unsigned int index)
{
	struct rcar_du_group *rgrp = plane->group;
	u32 ddcr2 = PnDDCR2_CODE;
	u32 ddcr4;
	u32 mwr;

	/* Data format
	 *
	 * The data format is selected by the DDDF field in PnMR and the EDF
	 * field in DDCR4.
	 */
	rcar_du_plane_setup_mode(plane, index);

	if (plane->format->planes == 2) {
		if (plane->hwindex != index) {
			if (plane->format->fourcc == DRM_FORMAT_NV12 ||
			    plane->format->fourcc == DRM_FORMAT_NV21)
				ddcr2 |= PnDDCR2_Y420;

			if (plane->format->fourcc == DRM_FORMAT_NV21)
				ddcr2 |= PnDDCR2_NV21;

			ddcr2 |= PnDDCR2_DIVU;
		} else {
			ddcr2 |= PnDDCR2_DIVY;
		}
	}

	rcar_du_plane_write(rgrp, index, PnDDCR2, ddcr2);

	ddcr4 = plane->format->edf | PnDDCR4_CODE;
	if (plane->source != RCAR_DU_PLANE_MEMORY)
		ddcr4 |= PnDDCR4_VSPS;

	rcar_du_plane_write(rgrp, index, PnDDCR4, ddcr4);

	/* Memory pitch (expressed in pixels) */
	if (plane->format->planes == 2)
		mwr = plane->pitch;
	else
		mwr = plane->pitch * 8 / plane->format->bpp;

	if ((plane->interlace_flag) && (plane->format->bpp == 32))
		rcar_du_plane_write(rgrp, index, PnMWR, mwr * 2);
	else
		rcar_du_plane_write(rgrp, index, PnMWR, mwr);

	/* Destination position and size */
	rcar_du_plane_write(rgrp, index, PnDSXR, plane->width);
	rcar_du_plane_write(rgrp, index, PnDSYR, plane->height);
	rcar_du_plane_write(rgrp, index, PnDPXR, plane->dst_x);
	rcar_du_plane_write(rgrp, index, PnDPYR, plane->dst_y);

	/* Wrap-around and blinking, disabled */
	rcar_du_plane_write(rgrp, index, PnWASPR, 0);
	rcar_du_plane_write(rgrp, index, PnWAMWR, 4095);
	rcar_du_plane_write(rgrp, index, PnBTR, 0);
	rcar_du_plane_write(rgrp, index, PnMLR, 0);
}

void rcar_du_plane_setup(struct rcar_du_plane *plane)
{
	__rcar_du_plane_setup(plane, plane->hwindex);
	if (plane->format->planes == 2)
		__rcar_du_plane_setup(plane, (plane->hwindex + 1) % 8);

	rcar_du_plane_update_base(plane);
}

#ifdef RCAR_DU_CONNECT_VSP
static int rcar_du_plane_order_sort_update(struct rcar_du_plane *plane)
{
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(plane->crtc);
	struct rcar_du_group *rgrp = plane->group;
	struct rcar_du_planes *planes = &rgrp->planes;
	struct rcar_du_plane *tmp_planes[VSPD_NUM_KMS_PLANES];
	unsigned int ret, i, j, num_planes = 0, ch_index;
	unsigned int planes_order[VSPD_NUM_KMS_PLANES];
	bool blend;

	switch (rcrtc->index) {
	case DU_CH_0:
		ch_index = 0;
		break;
	case DU_CH_1:
		ch_index = 1;
		break;
	case DU_CH_2:
		ch_index = 0;
		break;
	default:
		ret = -EINVAL;
		goto done;
	}

	/* sort order */
	for (i = 0; i < VSPD_NUM_KMS_PLANES; ++i) {
		struct rcar_du_plane *tmp_plane =
			&planes->vspd_planes[ch_index][i];
		for (j = num_planes++; j > 0; --j) {
			if (tmp_planes[j-1]->zpos <= tmp_plane->zpos)
				break;
			tmp_planes[j] = tmp_planes[j-1];
		}
		tmp_planes[j] = tmp_plane;
	}

	for (i = 0; i < VSPD_NUM_KMS_PLANES; ++i) {
		tmp_planes[i]->order = i + 1;
		planes_order[(tmp_planes[i]->hwindex - 1)]
					 = tmp_planes[i]->order;
	}

	for (i = 0; i < VSPD_NUM_KMS_PLANES; ++i) {
		struct rcar_du_plane *tmp_plane =
			&planes->vspd_planes[ch_index][i];

		if (i == (VSPD_NUM_KMS_PLANES - 1))
			blend = true;
		else
			blend = false;
		tmp_plane->hwindex = planes_order[(tmp_plane->hwindex - 1)];
		if (tmp_plane->enabled) {
			ret = vsp_du_if_update_plane(rcrtc->vpsd_handle,
					 tmp_plane->hwindex, tmp_plane, blend);
		} else {
			ret = vsp_du_if_update_plane(rcrtc->vpsd_handle,
					 tmp_plane->hwindex, NULL, blend);
		}
	}
done:
	return ret;
}

static int
rcar_du_plane_update_core(struct drm_plane *plane, struct drm_crtc *crtc,
		       struct drm_framebuffer *fb, struct drm_live_source *src,
		       int crtc_x, int crtc_y,
		       unsigned int crtc_w, unsigned int crtc_h,
		       uint32_t src_x, uint32_t src_y,
		       uint32_t src_w, uint32_t src_h,
		       int use_sync)
{
	struct rcar_du_plane *rplane = to_rcar_plane(plane);
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(crtc);
	const struct rcar_du_format_info *format;

	if (!rcrtc->lif_enable)
		return -EINVAL;

	if (!fb)
		return -EINVAL;

	format = rcar_du_format_info(fb->pixel_format);

	rplane->crtc = crtc;
	rplane->format = format;
	rplane->pitch = fb->pitches[0];

	rplane->src_x = src_x >> 16;
	rplane->src_y = src_y >> 16;
	rplane->dst_x = crtc_x;
	rplane->dst_y = crtc_y;
	rplane->width = src_w >> 16;
	rplane->height = src_h >> 16;
	rplane->d_width = crtc_w;
	rplane->d_height = crtc_h;

	rcar_du_plane_compute_base(rplane, fb);
	if (crtc->mode.flags & DRM_MODE_FLAG_INTERLACE)
		rplane->interlace_flag = true;
	else
		rplane->interlace_flag = false;

	mutex_lock(&rplane->group->planes.lock);
	rplane->enabled = true;
	mutex_unlock(&rplane->group->planes.lock);

	return rcar_du_plane_order_sort_update(rplane);
}

static int
rcar_du_plane_update(struct drm_plane *plane, struct drm_crtc *crtc,
		       struct drm_framebuffer *fb, struct drm_live_source *src,
		       int crtc_x, int crtc_y,
		       unsigned int crtc_w, unsigned int crtc_h,
		       uint32_t src_x, uint32_t src_y,
		       uint32_t src_w, uint32_t src_h)
{
	return rcar_du_plane_update_core(plane, crtc, fb, src,
		       crtc_x, crtc_y, crtc_w, crtc_h,
		       src_x, src_y, src_w, src_h,
		       0);
}

static int rcar_du_plane_disable(struct drm_plane *plane)
{
	struct rcar_du_plane *rplane = to_rcar_plane(plane);
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(plane->crtc);

	if (!rplane->enabled)
		return 0;

	vsp_du_if_update_plane(rcrtc->vpsd_handle, rplane->hwindex, NULL, true);

	mutex_lock(&rplane->group->planes.lock);
	rplane->enabled = false;
	mutex_unlock(&rplane->group->planes.lock);

	rplane->crtc = NULL;
	rplane->format = NULL;

	return 0;
}

static void rcar_du_plane_set_premultiplied(struct rcar_du_plane *plane,
					u32 premultiplied)
{
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(plane->crtc);

	if (plane->premultiplied == premultiplied)
		return;

	plane->premultiplied = premultiplied;
	if ((!plane->enabled) || (plane->format->fourcc == DRM_FORMAT_UYVY)
			      || (plane->format->fourcc == DRM_FORMAT_YUYV)
			      || (plane->format->fourcc == DRM_FORMAT_NV12)
			      || (plane->format->fourcc == DRM_FORMAT_NV21)
			      || (plane->format->fourcc == DRM_FORMAT_NV16))
		return;

	vsp_du_if_update_plane(rcrtc->vpsd_handle, plane->hwindex, plane, true);
}

static void rcar_du_plane_set_alpha(struct rcar_du_plane *plane, u32 alpha)
{
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(plane->crtc);

	if (plane->alpha == alpha)
		return;

	plane->alpha = alpha;
	if ((!plane->enabled) || (plane->format->fourcc == DRM_FORMAT_ARGB8888)
			      || (plane->format->fourcc == DRM_FORMAT_UYVY)
			      || (plane->format->fourcc == DRM_FORMAT_YUYV)
			      || (plane->format->fourcc == DRM_FORMAT_NV12)
			      || (plane->format->fourcc == DRM_FORMAT_NV21)
			      || (plane->format->fourcc == DRM_FORMAT_NV16))
		return;

	vsp_du_if_update_plane(rcrtc->vpsd_handle, plane->hwindex, plane, true);
}

static void rcar_du_plane_set_zpos(struct rcar_du_plane *plane,
				   unsigned int zpos)
{
	mutex_lock(&plane->group->planes.lock);

	if (plane->zpos == zpos)
		goto done;

	plane->zpos = zpos;
	if (!plane->enabled)
		goto done;

	rcar_du_plane_order_sort_update(plane);
done:
	mutex_unlock(&plane->group->planes.lock);
}

static int rcar_du_plane_set_property(struct drm_plane *plane,
				      struct drm_property *property,
				      uint64_t value)
{
	struct rcar_du_plane *rplane = to_rcar_plane(plane);
	struct rcar_du_group *rgrp = rplane->group;

	if (property == rgrp->planes.alpha)
		rcar_du_plane_set_alpha(rplane, value);
	else if (property == rgrp->planes.premultiplied)
		rcar_du_plane_set_premultiplied(rplane, value);
	else if (property == rgrp->planes.zpos)
		rcar_du_plane_set_zpos(rplane, value);
	else
		return -EINVAL;

	return 0;
}

static const struct drm_plane_funcs rcar_du_plane_lif_funcs = {
	.update_plane = rcar_du_plane_update,
	.disable_plane = rcar_du_plane_disable,
	.set_property = rcar_du_plane_set_property,
};

#else /* RCAR_DU_CONNECT_VSP */

static int
rcar_du_plane_update(struct drm_plane *plane, struct drm_crtc *crtc,
		       struct drm_framebuffer *fb, struct drm_live_source *src,
		       int crtc_x, int crtc_y,
		       unsigned int crtc_w, unsigned int crtc_h,
		       uint32_t src_x, uint32_t src_y,
		       uint32_t src_w, uint32_t src_h)
{
	struct rcar_du_plane *rplane = to_rcar_plane(plane);
	struct rcar_du_device *rcdu = rplane->group->dev;
	struct rcar_du_crtc *rcrtc = to_rcar_crtc(crtc);
	const struct rcar_du_format_info *format;
	enum rcar_du_plane_source source;
	uint32_t pixel_format;
	unsigned int nplanes;
	bool source_changed;
	int ret;
	unsigned int value;

	pixel_format = fb ? fb->pixel_format : src->pixel_format;

	format = rcar_du_format_info(pixel_format);
	if (format == NULL) {
		dev_dbg(rcdu->dev, "%s: unsupported format %08x\n", __func__,
			pixel_format);
		return -EINVAL;
	}

	if (src_w >> 16 != crtc_w || src_h >> 16 != crtc_h) {
		dev_dbg(rcdu->dev, "%s: scaling not supported\n", __func__);
		return -EINVAL;
	}

	nplanes = rplane->format ? rplane->format->planes : 0;

	/* Check whether the source has changed from memory to live source or
	 * from live source to memory. The source will be configured by the VSPS
	 * bit in the PnDDCR4 register. Although the datasheet states that the
	 * bit is updated during vertical blanking, it seems that updates only
	 * occur when the DU group is held in reset through the DSYSR.DRES bit.
	 * We thus need to restart the group if the source changes.
	 */
	if (src)
		source = to_rcar_vsp1_source(src)->source;
	else
		source = RCAR_DU_PLANE_MEMORY;

	source_changed = (rplane->source == RCAR_DU_PLANE_MEMORY) !=
			 (source == RCAR_DU_PLANE_MEMORY);

	/* Reallocate hardware planes if the source or the number of required
	 * planes has changed.
	 */
	if (format->planes != nplanes || rplane->source != source) {
		rcar_du_plane_release(rplane);
		ret = __rcar_du_plane_reserve(rplane, format, source);
		if (ret < 0)
			return ret;
	}

	rplane->crtc = crtc;
	rplane->format = format;
	rplane->pitch = fb ? fb->pitches[0] : (src_w >> 16) * format->bpp / 8;

	rplane->src_x = src_x >> 16;
	rplane->src_y = src_y >> 16;
	rplane->dst_x = crtc_x;
	rplane->dst_y = crtc_y;
	rplane->width = crtc_w;
	rplane->height = crtc_h;

	if (fb) {
		rcar_du_plane_compute_base(rplane, fb);
	} else {
		rplane->dma[0] = 0;
		rplane->dma[1] = 0;
	}

	if (source == RCAR_DU_PLANE_VSPD1) {
		unsigned int vspd1_sink = rplane->group->index ? 2 : 0;

		if (rcdu->vspd1_sink != vspd1_sink) {
			rcdu->vspd1_sink = vspd1_sink;
			rcar_du_set_dpad0_vsp1_routing(rcdu);
		}
	}

	if (crtc->mode.flags & DRM_MODE_FLAG_INTERLACE)
		rplane->interlace_flag = true;
	else
		rplane->interlace_flag = false;
	rcar_du_plane_setup(rplane);

	mutex_lock(&rplane->group->planes.lock);

	/* If the source has changed we will need to restart the group for the
	 * change to take effect. Set the need_restart flag and proceed to
	 * update the planes. The update function might restart the group, in
	 * which case the need_restart flag will be cleared. If the flag is
	 * still set, force a group restart.
	 */

	if (source_changed)
		rplane->group->planes.need_restart = true;

	rplane->enabled = true;
	rcar_du_crtc_update_planes(rplane->crtc);

	if (rplane->group->planes.need_restart)
		rcar_du_group_restart(rplane->group);

	if (rplane->group->index == 1)
		value = DU_CH_2;
	else {
		if (rcrtc->dptsr_read & (1 << rplane->hwindex))
			value = DU_CH_1;
		else
			value = DU_CH_0;
	}
	drm_object_property_set_value(&plane->base,
				  rplane->group->planes.channel, value);

	mutex_unlock(&rplane->group->planes.lock);

	return 0;
}

static int rcar_du_plane_disable(struct drm_plane *plane)
{
	struct rcar_du_plane *rplane = to_rcar_plane(plane);

	if (!rplane->enabled)
		return 0;

	mutex_lock(&rplane->group->planes.lock);
	rplane->enabled = false;
	rcar_du_crtc_update_planes(rplane->crtc);
	mutex_unlock(&rplane->group->planes.lock);

	rcar_du_plane_release(rplane);

	rplane->crtc = NULL;
	rplane->format = NULL;

	return 0;
}

/* Both the .set_property and the .update_plane operations are called with the
 * mode_config lock held. There is this no need to explicitly protect access to
 * the alpha and colorkey fields and the mode register.
 */
static void rcar_du_plane_set_alpha(struct rcar_du_plane *plane, u32 alpha)
{
	if (plane->alpha == alpha)
		return;

	plane->alpha = alpha;
	if ((!plane->enabled) || (plane->format->fourcc == DRM_FORMAT_ARGB8888)
			      || (plane->format->fourcc == DRM_FORMAT_UYVY)
			      || (plane->format->fourcc == DRM_FORMAT_YUYV)
			      || (plane->format->fourcc == DRM_FORMAT_NV12)
			      || (plane->format->fourcc == DRM_FORMAT_NV21)
			      || (plane->format->fourcc == DRM_FORMAT_NV16))
		return;

	rcar_du_plane_setup_mode(plane, plane->hwindex);
}

static void rcar_du_plane_set_colorkey(struct rcar_du_plane *plane,
				       u32 colorkey)
{
	if (plane->colorkey == colorkey)
		return;

	plane->colorkey = colorkey;
	if (!plane->enabled)
		return;

	rcar_du_plane_setup_mode(plane, plane->hwindex);
}

static void rcar_du_plane_set_zpos(struct rcar_du_plane *plane,
				   unsigned int zpos)
{
	mutex_lock(&plane->group->planes.lock);
	if (plane->zpos == zpos)
		goto done;

	plane->zpos = zpos;
	if (!plane->enabled)
		goto done;

	rcar_du_crtc_update_planes(plane->crtc);

done:
	mutex_unlock(&plane->group->planes.lock);
}

static int rcar_du_plane_set_property(struct drm_plane *plane,
				      struct drm_property *property,
				      uint64_t value)
{
	struct rcar_du_plane *rplane = to_rcar_plane(plane);
	struct rcar_du_group *rgrp = rplane->group;

	if (property == rgrp->planes.alpha)
		rcar_du_plane_set_alpha(rplane, value);
	else if (property == rgrp->planes.colorkey)
		rcar_du_plane_set_colorkey(rplane, value);
	else if (property == rgrp->planes.zpos)
		rcar_du_plane_set_zpos(rplane, value);
	else
		return -EINVAL;

	return 0;
}

static const struct drm_plane_funcs rcar_du_plane_funcs = {
	.update_plane = rcar_du_plane_update,
	.disable_plane = rcar_du_plane_disable,
	.set_property = rcar_du_plane_set_property,
	.destroy = drm_plane_cleanup,
};
#endif

static const uint32_t plane_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_ARGB1555,
	DRM_FORMAT_XRGB1555,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_YUYV,
	DRM_FORMAT_NV12,
	DRM_FORMAT_NV21,
	DRM_FORMAT_NV16,
};

static const struct drm_prop_enum_list rcar_du0_du1_info_enum_list[] = {
	{ DU_CH_0, "channel0" },
	{ DU_CH_1, "channel1" },
};

static const struct drm_prop_enum_list rcar_du2_info_enum_list[] = {
	{ DU_CH_2, "channel2" },
};

int rcar_du_planes_init(struct rcar_du_group *rgrp)
{
	struct rcar_du_planes *planes = &rgrp->planes;
	struct rcar_du_device *rcdu = rgrp->dev;
	unsigned int i;

	mutex_init(&planes->lock);
	planes->free = 0xff;

	planes->alpha =
		drm_property_create_range(rcdu->ddev, 0, "alpha", 0, 255);
	if (planes->alpha == NULL)
		return -ENOMEM;

	/* The color key is expressed as an RGB888 triplet stored in a 32-bit
	 * integer in XRGB8888 format. Bit 24 is used as a flag to disable (0)
	 * or enable source color keying (1).
	 */
	planes->colorkey =
		drm_property_create_range(rcdu->ddev, 0, "colorkey",
					  0, 0x01ffffff);
	if (planes->colorkey == NULL)
		return -ENOMEM;

#ifdef RCAR_DU_CONNECT_VSP
	planes->zpos =
		drm_property_create_range(rcdu->ddev, 0, "zpos", 1, 3);
#else
	planes->zpos =
		drm_property_create_range(rcdu->ddev, 0, "zpos", 1, 7);
#endif
	if (planes->zpos == NULL)
		return -ENOMEM;

	if (rgrp->index == 1)
		planes->channel = drm_property_create_enum(
				 rcdu->ddev, DRM_MODE_PROP_IMMUTABLE,
				 "channel",
				 rcar_du2_info_enum_list,
				 ARRAY_SIZE(rcar_du2_info_enum_list));
	else
		planes->channel = drm_property_create_enum(
				 rcdu->ddev, DRM_MODE_PROP_IMMUTABLE,
				 "channel",
				 rcar_du0_du1_info_enum_list,
				 ARRAY_SIZE(rcar_du0_du1_info_enum_list));

	if (planes->channel == NULL)
		return -ENOMEM;

	for (i = 0; i < ARRAY_SIZE(planes->planes); ++i) {
		struct rcar_du_plane *plane = &planes->planes[i];

		plane->group = rgrp;
		plane->hwindex = -1;
		plane->source = RCAR_DU_PLANE_MEMORY;
		plane->alpha = 255;
		plane->colorkey = RCAR_DU_COLORKEY_NONE;
		plane->zpos = 0;
	}

#ifdef RCAR_DU_CONNECT_VSP
	for (i = 0; i < VSPD_NUM_KMS_PLANES; ++i) {
		int j;
		for (j = 0; j < 2; j++) {
			struct rcar_du_plane *plane =
				&planes->vspd_planes[j][i];

			plane->group = rgrp;
			plane->hwindex = i + 1;
			plane->source = RCAR_DU_PLANE_MEMORY;
			plane->alpha = 255;
			plane->colorkey = RCAR_DU_COLORKEY_NONE;
			plane->zpos = 1;
			plane->premultiplied = 0;
		}
	}
#endif

	return 0;
}


#ifdef RCAR_DU_CONNECT_VSP
static int vpsd_planes_register(struct rcar_du_group *rgrp, int du_ch)
{
	struct rcar_du_planes *planes = &rgrp->planes;
	struct rcar_du_device *rcdu = rgrp->dev;
	int i;
	int ret;
	unsigned int crtc;
	int ch_index;

	switch (du_ch) {
	case DU_CH_0:
		ch_index = 0;
		crtc = 1 << 0;
		break;
	case DU_CH_1:
		ch_index = 1;
		crtc = 1 << 1;
		break;
	case DU_CH_2:
		ch_index = 0;
		crtc = 1 << 2;
		break;
	default:
		return -EINVAL;
	}

	for (i = 0; i < VSPD_NUM_KMS_PLANES; ++i) {
		struct rcar_du_kms_plane *plane;

		plane = devm_kzalloc(rcdu->dev, sizeof(*plane), GFP_KERNEL);
		if (plane == NULL)
			return -ENOMEM;

		plane->hwplane = &planes->vspd_planes[ch_index][i];
		plane->hwplane->zpos = 1;
		plane->hwplane->fb_plane = false;
		plane->hwplane->interlace_flag = false;
		ret = drm_plane_init(
				rcdu->ddev, &plane->plane, crtc,
				&rcar_du_plane_lif_funcs, plane_formats,
				ARRAY_SIZE(plane_formats), false);

		if (ret < 0)
			return ret;

		drm_object_attach_property(&plane->plane.base,
					   planes->alpha, 255);
#ifndef RCAR_DU_CONNECT_VSP
		drm_object_attach_property(&plane->plane.base,
					   planes->colorkey,
					   RCAR_DU_COLORKEY_NONE);
#endif
		drm_object_attach_property(&plane->plane.base,
					   planes->zpos, 1);
		drm_object_attach_property(&plane->plane.base,
					   planes->channel, du_ch);
		drm_object_attach_property(&plane->plane.base,
					   planes->premultiplied, 0);
	}

	return 0;
}
#endif


int rcar_du_planes_register(struct rcar_du_group *rgrp)
{
#ifdef RCAR_DU_CONNECT_VSP
	struct rcar_du_device *rcdu = rgrp->dev;
	struct rcar_du_planes *planes = &rgrp->planes;
	unsigned int crtcs;
	unsigned int i;
	int ret;
	const struct rcar_du_crtc_data *pdata =
			rgrp->dev->pdata->crtcs;

	planes->premultiplied =
		drm_property_create_range(rcdu->ddev, 0, "premultiplied", 0, 1);
	if (planes->premultiplied == NULL)
		return -ENOMEM;

	crtcs = ((1 << rcdu->num_crtcs) - 1) & (3 << (2 * rgrp->index));

	for (i = 0; i < rcdu->num_crtcs; i++) {
		if (pdata[i].vsp == RCAR_DU_VSPD_UNUSED)
			continue;

		if (crtcs & (1 << i)) {
			ret = vpsd_planes_register(rgrp, i);
			if (ret < 0)
				return ret;
		}
	}
#else
	struct rcar_du_planes *planes = &rgrp->planes;
	struct rcar_du_device *rcdu = rgrp->dev;
	unsigned int crtcs;
	unsigned int i;
	int ret;
	unsigned int plane_num;
	unsigned int dptsr_ch;

	if (rgrp->index == 1)
		plane_num = RCAR_DU2_NUM_KMS_PLANES;
	else
		plane_num = RCAR_DU01_NUM_KMS_PLANES;

	crtcs = ((1 << rcdu->num_crtcs) - 1) & (3 << (2 * rgrp->index));

	for (i = 0; i < plane_num; ++i) {
		struct rcar_du_kms_plane *plane;

		plane = devm_kzalloc(rcdu->dev, sizeof(*plane), GFP_KERNEL);
		if (plane == NULL)
			return -ENOMEM;

		plane->hwplane = &planes->planes[i + 2];
		plane->hwplane->zpos = 1;
		plane->hwplane->fb_plane = false;
		plane->hwplane->interlace_flag = false;
		ret = drm_plane_init(rcdu->ddev, &plane->plane, crtcs,
				     &rcar_du_plane_funcs, plane_formats,
				     ARRAY_SIZE(plane_formats), false);
		if (ret < 0)
			return ret;

		drm_object_attach_property(&plane->plane.base,
					   planes->alpha, 255);
		drm_object_attach_property(&plane->plane.base,
					   planes->colorkey,
					   RCAR_DU_COLORKEY_NONE);
		drm_object_attach_property(&plane->plane.base,
					   planes->zpos, 1);
		if (rgrp->index == 1)
			dptsr_ch = DU_CH_2;
		else {
			if ((0x01 << ((plane_num - 1) - i)) &
				CONFIG_DRM_RCAR_DU_OVERLAY_CH)
				dptsr_ch = DU_CH_1;
			else
				dptsr_ch = DU_CH_0;
		}
		drm_object_attach_property(&plane->plane.base,
					   planes->channel, dptsr_ch);
	}
#endif

	return 0;
}
