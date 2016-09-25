/*
 * vsp1_video.c  --  R-Car VSP1 Video Node
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

#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/v4l2-mediabus.h>
#include <linux/videodev2.h>
#if IS_ENABLED(CONFIG_DRM)
#include <linux/rcar-du-frm-interface.h>
#endif

#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include "vsp1.h"
#include "vsp1_bru.h"
#include "vsp1_entity.h"
#include "vsp1_rwpf.h"
#include "vsp1_uds.h"
#include "vsp1_video.h"

#define VSP1_VIDEO_DEF_FORMAT		V4L2_PIX_FMT_YUYV
#define VSP1_VIDEO_DEF_WIDTH		1024
#define VSP1_VIDEO_DEF_HEIGHT		768

#define VSP1_VIDEO_MIN_WIDTH		2U
#define VSP1_VIDEO_MAX_WIDTH		8190U
#define VSP1_VIDEO_MIN_HEIGHT		2U
#define VSP1_VIDEO_MAX_HEIGHT		8190U

/* DU Registers */
#define PLANEREG0_PLN1_ADDR			0xFEB00190
#define PLANEREG0_PLN2_ADDR			0xFEB00290
#define PLANEREG2_PLN1_ADDR			0xFEB40190
#define PRIOREG_ADDR				0xFEB11020

#define PLANEREG_PNVSPS				0x2000
#define PLANE_NO_1					0x0001
#define PLANE_NO_2					0x0002

/* Product Registers */
#define PRR_ADDRESS					0xFF000044
#define PRODUCT_ID_H2				0x4500
#define PRODUCT_MASK				0x7F00

/* DU channel ID */
#define DU_CH_NONE					-1
#define DU_CH_0						0
#define DU_CH_1						1
#define DU_CH_2						2

/* VSPD channel ID */
#define VSPD_CH_ID_1				2
#define VSPD_CH_ID_2				3

/* size of register */
#define REG_SIZE					0x04

/* -----------------------------------------------------------------------------
 * Helper functions
 */

static const struct vsp1_format_info vsp1_video_formats[] = {
	{ V4L2_PIX_FMT_RGB332, V4L2_MBUS_FMT_ARGB8888_1X32,
	  VI6_FMT_RGB_332, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS | VI6_RPF_DSWAP_P_BTS,
	  1, { 8, 0, 0 }, false, false, 1, 1, false },
	{ V4L2_PIX_FMT_ARGB444, V4L2_MBUS_FMT_ARGB8888_1X32,
	  VI6_FMT_ARGB_4444, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS,
	  1, { 16, 0, 0 }, false, false, 1, 1, true },
	{ V4L2_PIX_FMT_XRGB444, V4L2_MBUS_FMT_ARGB8888_1X32,
	  VI6_FMT_XRGB_4444, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS,
	  1, { 16, 0, 0 }, false, false, 1, 1, true },
	{ V4L2_PIX_FMT_ARGB555, V4L2_MBUS_FMT_ARGB8888_1X32,
	  VI6_FMT_ARGB_1555, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS,
	  1, { 16, 0, 0 }, false, false, 1, 1, true },
	{ V4L2_PIX_FMT_XRGB555, V4L2_MBUS_FMT_ARGB8888_1X32,
	  VI6_FMT_XRGB_1555, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS,
	  1, { 16, 0, 0 }, false, false, 1, 1, false },
	{ V4L2_PIX_FMT_RGB565, V4L2_MBUS_FMT_ARGB8888_1X32,
	  VI6_FMT_RGB_565, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS,
	  1, { 16, 0, 0 }, false, false, 1, 1, false },
	{ V4L2_PIX_FMT_BGR24, V4L2_MBUS_FMT_ARGB8888_1X32,
	  VI6_FMT_BGR_888, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS | VI6_RPF_DSWAP_P_BTS,
	  1, { 24, 0, 0 }, false, false, 1, 1, false },
	{ V4L2_PIX_FMT_RGB24, V4L2_MBUS_FMT_ARGB8888_1X32,
	  VI6_FMT_RGB_888, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS | VI6_RPF_DSWAP_P_BTS,
	  1, { 24, 0, 0 }, false, false, 1, 1, false },
	{ V4L2_PIX_FMT_ABGR32, V4L2_MBUS_FMT_ARGB8888_1X32,
	  VI6_FMT_ARGB_8888, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS,
	  1, { 32, 0, 0 }, false, false, 1, 1, true },
	{ V4L2_PIX_FMT_XBGR32, V4L2_MBUS_FMT_ARGB8888_1X32,
	  VI6_FMT_ARGB_8888, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS,
	  1, { 32, 0, 0 }, false, false, 1, 1, false },
	{ V4L2_PIX_FMT_ARGB32, V4L2_MBUS_FMT_ARGB8888_1X32,
	  VI6_FMT_ARGB_8888, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS | VI6_RPF_DSWAP_P_BTS,
	  1, { 32, 0, 0 }, false, false, 1, 1, true },
	{ V4L2_PIX_FMT_XRGB32, V4L2_MBUS_FMT_ARGB8888_1X32,
	  VI6_FMT_ARGB_8888, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS | VI6_RPF_DSWAP_P_BTS,
	  1, { 32, 0, 0 }, false, false, 1, 1, false },
	{ V4L2_PIX_FMT_UYVY, V4L2_MBUS_FMT_AYUV8_1X32,
	  VI6_FMT_YUYV_422, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS | VI6_RPF_DSWAP_P_BTS,
	  1, { 16, 0, 0 }, false, false, 2, 1, false },
	{ V4L2_PIX_FMT_VYUY, V4L2_MBUS_FMT_AYUV8_1X32,
	  VI6_FMT_YUYV_422, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS | VI6_RPF_DSWAP_P_BTS,
	  1, { 16, 0, 0 }, false, true, 2, 1, false },
	{ V4L2_PIX_FMT_YUYV, V4L2_MBUS_FMT_AYUV8_1X32,
	  VI6_FMT_YUYV_422, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS | VI6_RPF_DSWAP_P_BTS,
	  1, { 16, 0, 0 }, true, false, 2, 1, false },
	{ V4L2_PIX_FMT_YVYU, V4L2_MBUS_FMT_AYUV8_1X32,
	  VI6_FMT_YUYV_422, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS | VI6_RPF_DSWAP_P_BTS,
	  1, { 16, 0, 0 }, true, true, 2, 1, false },
	{ V4L2_PIX_FMT_NV12M, V4L2_MBUS_FMT_AYUV8_1X32,
	  VI6_FMT_Y_UV_420, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS | VI6_RPF_DSWAP_P_BTS,
	  2, { 8, 16, 0 }, false, false, 2, 2, false },
	{ V4L2_PIX_FMT_NV21M, V4L2_MBUS_FMT_AYUV8_1X32,
	  VI6_FMT_Y_UV_420, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS | VI6_RPF_DSWAP_P_BTS,
	  2, { 8, 16, 0 }, false, true, 2, 2, false },
	{ V4L2_PIX_FMT_NV16M, V4L2_MBUS_FMT_AYUV8_1X32,
	  VI6_FMT_Y_UV_422, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS | VI6_RPF_DSWAP_P_BTS,
	  2, { 8, 16, 0 }, false, false, 2, 1, false },
	{ V4L2_PIX_FMT_NV61M, V4L2_MBUS_FMT_AYUV8_1X32,
	  VI6_FMT_Y_UV_422, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS | VI6_RPF_DSWAP_P_BTS,
	  2, { 8, 16, 0 }, false, true, 2, 1, false },
	{ V4L2_PIX_FMT_YUV420M, V4L2_MBUS_FMT_AYUV8_1X32,
	  VI6_FMT_Y_U_V_420, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS |
	  VI6_RPF_DSWAP_P_WDS | VI6_RPF_DSWAP_P_BTS,
	  3, { 8, 8, 8 }, false, false, 2, 2, false },
	{ V4L2_PIX_FMT_RGB32S, V4L2_MBUS_FMT_ARGB8888_1X32,
	  VI6_FMT_ARGB_8888, VI6_RPF_DSWAP_P_LLS | VI6_RPF_DSWAP_P_LWS,
	  1, { 32, 0, 0 }, false, false, 1, 1, false },
};

/*
 * vsp1_get_format_info - Retrieve format information for a 4CC
 * @fourcc: the format 4CC
 *
 * Return a pointer to the format information structure corresponding to the
 * given V4L2 format 4CC, or NULL if no corresponding format can be found.
 */
static const struct vsp1_format_info *vsp1_get_format_info(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(vsp1_video_formats); ++i) {
		const struct vsp1_format_info *info = &vsp1_video_formats[i];

		if (info->fourcc == fourcc)
			return info;
	}

	return NULL;
}


static struct v4l2_subdev *
vsp1_video_remote_subdev(struct media_pad *local, u32 *pad)
{
	struct media_pad *remote;

	remote = media_entity_remote_pad(local);
	if (remote == NULL ||
	    media_entity_type(remote->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
		return NULL;

	if (pad)
		*pad = remote->index;

	return media_entity_to_v4l2_subdev(remote->entity);
}

static int vsp1_video_verify_format(struct vsp1_video *video)
{
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev *subdev;
	int ret;

	subdev = vsp1_video_remote_subdev(&video->pad, &fmt.pad);
	if (subdev == NULL)
		return -EINVAL;

	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt);
	if (ret < 0)
		return ret == -ENOIOCTLCMD ? -EINVAL : ret;

	if (video->fmtinfo->mbus != fmt.format.code ||
	    video->format.height != fmt.format.height ||
	    video->format.width != fmt.format.width)
		return -EINVAL;

	return 0;
}

static int __vsp1_video_try_format(struct vsp1_video *video,
				   struct v4l2_pix_format_mplane *pix,
				   const struct vsp1_format_info **fmtinfo)
{
	static const u32 xrgb_formats[][2] = {
		{ V4L2_PIX_FMT_RGB444, V4L2_PIX_FMT_XRGB444 },
		{ V4L2_PIX_FMT_RGB555, V4L2_PIX_FMT_XRGB555 },
		{ V4L2_PIX_FMT_BGR32, V4L2_PIX_FMT_XBGR32 },
		{ V4L2_PIX_FMT_RGB32, V4L2_PIX_FMT_XRGB32 },
	};

	const struct vsp1_format_info *info;
	unsigned int width = pix->width;
	unsigned int height = pix->height;
	unsigned int i;

	/* Backward compatibility: replace deprecated RGB formats by their XRGB
	 * equivalent. This selects the format older userspace applications want
	 * while still exposing the new format.
	 */
	for (i = 0; i < ARRAY_SIZE(xrgb_formats); ++i) {
		if (xrgb_formats[i][0] == pix->pixelformat) {
			pix->pixelformat = xrgb_formats[i][1];
			break;
		}
	}

	/* Retrieve format information and select the default format if the
	 * requested format isn't supported.
	 */
	info = vsp1_get_format_info(pix->pixelformat);
	if (info == NULL)
		info = vsp1_get_format_info(VSP1_VIDEO_DEF_FORMAT);

	pix->pixelformat = info->fourcc;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	memset(pix->reserved, 0, sizeof(pix->reserved));

	/* Align the width and height for YUV 4:2:2 and 4:2:0 formats. */
	width = round_down(width, info->hsub);
	height = round_down(height, info->vsub);

	/* Clamp the width and height. */
	pix->width = clamp(width, VSP1_VIDEO_MIN_WIDTH, VSP1_VIDEO_MAX_WIDTH);
	pix->height = clamp(height, VSP1_VIDEO_MIN_HEIGHT,
			    VSP1_VIDEO_MAX_HEIGHT);

	for (i = 0; i < max(info->planes, 2U); ++i) {
		unsigned int hsub = i > 0 ? info->hsub : 1;
		unsigned int vsub = i > 0 ? info->vsub : 1;
		unsigned int bpl;

		bpl = clamp_t(unsigned int, pix->plane_fmt[i].bytesperline,
			      pix->width / hsub * info->bpp[i] / 8, 65535U);

		pix->plane_fmt[i].bytesperline = bpl;
		pix->plane_fmt[i].sizeimage = pix->plane_fmt[i].bytesperline
					    * pix->height / vsub;
	}

	if (info->planes == 3) {
		/* The second and third planes must have the same stride. */
		pix->plane_fmt[2].bytesperline = pix->plane_fmt[1].bytesperline;
		pix->plane_fmt[2].sizeimage = pix->plane_fmt[1].sizeimage;
	}

	pix->num_planes = info->planes;

	if (fmtinfo)
		*fmtinfo = info;

	return 0;
}

static bool
vsp1_video_format_adjust(struct vsp1_video *video,
			 const struct v4l2_pix_format_mplane *format,
			 struct v4l2_pix_format_mplane *adjust)
{
	unsigned int i;

	*adjust = *format;
	__vsp1_video_try_format(video, adjust, NULL);

	if (format->width != adjust->width ||
	    format->height != adjust->height ||
	    format->pixelformat != adjust->pixelformat ||
	    format->num_planes != adjust->num_planes)
		return false;

	for (i = 0; i < format->num_planes; ++i) {
		if (format->plane_fmt[i].bytesperline !=
		    adjust->plane_fmt[i].bytesperline)
			return false;

		adjust->plane_fmt[i].sizeimage =
			max(adjust->plane_fmt[i].sizeimage,
			    format->plane_fmt[i].sizeimage);
	}

	return true;
}

static void vsp1_set_cropupdate(struct vsp1_video *video,
					struct vsp1_video_buffer *buf)
{
	if (video->update_crop == VSP1_UPDATE_CROP_REQUESTED) {
		buf->update_reg = true;
		video->update_crop = VSP1_UPDATE_CROP_UPDATE;
	} else
		buf->update_reg = false;
}

static void vsp1_update_reg(struct vsp1_pipeline *pipe,
					struct vsp1_video *video,
					struct vsp1_video_buffer *buf)
{
	struct vsp1_entity *entity;
	if (buf->update_reg == true) {
		list_for_each_entry(entity, &pipe->entities, list_pipe)
			v4l2_subdev_call(&entity->subdev, video,
				       s_stream, 1);
		buf->update_reg = false;
		video->update_crop = VSP1_UPDATE_CROP_NORMAL;
	}
}

/* -----------------------------------------------------------------------------
 * Pipeline Management
 */

static int vsp1_pipeline_validate_branch(struct vsp1_pipeline *pipe,
					 struct vsp1_rwpf *input,
					 struct vsp1_rwpf *output)
{
	struct vsp1_entity *entity;
	unsigned int entities = 0;
	struct media_pad *pad;
	bool bru_found = false;

	pad = media_entity_remote_pad(&input->entity.pads[RWPF_PAD_SOURCE]);

	input->location.left = 0;
	input->location.top = 0;

	input->location.left = 0;
	input->location.top = 0;

	while (1) {
		if (pad == NULL)
			return -EPIPE;

		/* We've reached a video node, that shouldn't have happened. */
		if (media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
			return -EPIPE;

		entity = to_vsp1_entity(media_entity_to_v4l2_subdev(pad->entity));

		/* A BRU is present in the pipeline, store the compose rectangle
		 * location in the input RPF for use when configuring the RPF.
		 */
		if (entity->type == VSP1_ENTITY_BRU) {
			struct vsp1_bru *bru = to_bru(&entity->subdev);
			struct v4l2_rect *rect =
				&bru->inputs[pad->index].compose;

			bru->inputs[pad->index].rpf = input;

			input->location.left = rect->left;
			input->location.top = rect->top;

			bru_found = true;
		}

		/* We've reached the WPF, we're done. */
		if (entity->type == VSP1_ENTITY_WPF)
			break;

		/* Ensure the branch has no loop. */
		if (entities & (1 << entity->subdev.entity.id))
			return -EPIPE;

		entities |= 1 << entity->subdev.entity.id;

		/* UDS can't be chained. */
		if (entity->type == VSP1_ENTITY_UDS) {
			if (pipe->uds)
				return -EPIPE;

			pipe->uds = entity;
			pipe->uds_input = bru_found ? pipe->bru
					: &input->entity;
		}

		/* Follow the source link. The link setup operations ensure
		 * that the output fan-out can't be more than one, there is thus
		 * no need to verify here that only a single source link is
		 * activated.
		 */
		pad = &entity->pads[entity->source_pad];
		pad = media_entity_remote_pad(pad);
	}

	/* The last entity must be the output WPF. */
	if (entity != &output->entity)
		return -EPIPE;

	return 0;
}

static void __vsp1_pipeline_cleanup(struct vsp1_pipeline *pipe)
{
	if (pipe->bru) {
		struct vsp1_bru *bru = to_bru(&pipe->bru->subdev);
		unsigned int i;

		for (i = 0; i < ARRAY_SIZE(bru->inputs); ++i)
			bru->inputs[i].rpf = NULL;
	}

	INIT_LIST_HEAD(&pipe->entities);
	pipe->state = VSP1_PIPELINE_STOPPED;
	pipe->buffers_ready = 0;
	pipe->num_video = 0;
	pipe->num_inputs = 0;
	pipe->output = NULL;
	pipe->bru = NULL;
	pipe->lif = NULL;
	pipe->uds = NULL;
}

static int vsp1_pipeline_validate(struct vsp1_pipeline *pipe,
				  struct vsp1_video *video)
{
	struct media_entity_graph graph;
	struct media_entity *entity = &video->video.entity;
	struct media_device *mdev = entity->parent;
	unsigned int i;
	int ret;

	mutex_lock(&mdev->graph_mutex);

	/* Walk the graph to locate the entities and video nodes. */
	media_entity_graph_walk_start(&graph, entity);

	while ((entity = media_entity_graph_walk_next(&graph))) {
		struct v4l2_subdev *subdev;
		struct vsp1_rwpf *rwpf;
		struct vsp1_entity *e;

		if (media_entity_type(entity) != MEDIA_ENT_T_V4L2_SUBDEV) {
			pipe->num_video++;
			continue;
		}

		subdev = media_entity_to_v4l2_subdev(entity);
		e = to_vsp1_entity(subdev);
		list_add_tail(&e->list_pipe, &pipe->entities);

		if (e->type == VSP1_ENTITY_RPF) {
			rwpf = to_rwpf(subdev);
			pipe->inputs[pipe->num_inputs++] = rwpf;
			rwpf->video.pipe_index = pipe->num_inputs;
		} else if (e->type == VSP1_ENTITY_WPF) {
			rwpf = to_rwpf(subdev);
			pipe->output = to_rwpf(subdev);
			rwpf->video.pipe_index = 0;
		} else if (e->type == VSP1_ENTITY_LIF) {
			pipe->lif = e;
		} else if (e->type == VSP1_ENTITY_BRU) {
			pipe->bru = e;
		}
	}

	mutex_unlock(&mdev->graph_mutex);

	/* We need one output and at least one input. */
	if (pipe->num_inputs == 0 || !pipe->output) {
		ret = -EPIPE;
		goto error;
	}

	/* Follow links downstream for each input and make sure the graph
	 * contains no loop and that all branches end at the output WPF.
	 */
	for (i = 0; i < pipe->num_inputs; ++i) {
		ret = vsp1_pipeline_validate_branch(pipe, pipe->inputs[i],
						    pipe->output);
		if (ret < 0)
			goto error;
	}

	return 0;

error:
	__vsp1_pipeline_cleanup(pipe);
	return ret;
}

static int vsp1_pipeline_init(struct vsp1_pipeline *pipe,
			      struct vsp1_video *video)
{
	int ret;

	mutex_lock(&pipe->lock);

	/* If we're the first user validate and initialize the pipeline. */
	if (pipe->use_count == 0) {
		ret = vsp1_pipeline_validate(pipe, video);
		if (ret < 0)
			goto done;
	}

	pipe->use_count++;
	ret = 0;

done:
	mutex_unlock(&pipe->lock);
	return ret;
}

static void vsp1_pipeline_cleanup(struct vsp1_pipeline *pipe)
{
	mutex_lock(&pipe->lock);

	/* If we're the last user clean up the pipeline. */
	if (--pipe->use_count == 0)
		__vsp1_pipeline_cleanup(pipe);

	mutex_unlock(&pipe->lock);
}

static void vsp1_pipeline_run(struct vsp1_pipeline *pipe)
{
	struct vsp1_device *vsp1 = pipe->output->entity.vsp1;

	vsp1_write(vsp1, VI6_CMD(pipe->output->entity.index), VI6_CMD_STRCMD);
	pipe->state = VSP1_PIPELINE_RUNNING;
	pipe->buffers_ready = 0;
}

static int vsp1_pipeline_stop(struct vsp1_pipeline *pipe)
{
	struct vsp1_entity *entity;
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&pipe->irqlock, flags);
	if (pipe->state == VSP1_PIPELINE_RUNNING)
		pipe->state = VSP1_PIPELINE_STOPPING;
	spin_unlock_irqrestore(&pipe->irqlock, flags);

	ret = wait_event_timeout(pipe->wq, pipe->state == VSP1_PIPELINE_STOPPED,
				 msecs_to_jiffies(500));
	ret = ret == 0 ? -ETIMEDOUT : 0;

	list_for_each_entry(entity, &pipe->entities, list_pipe) {
		if (entity->route && entity->route->reg)
			vsp1_write(entity->vsp1, entity->route->reg,
				   VI6_DPR_NODE_UNUSED);

		v4l2_subdev_call(&entity->subdev, video, s_stream, 0);
	}

	return ret;
}

static bool vsp1_pipeline_ready(struct vsp1_pipeline *pipe)
{
	unsigned int mask;

	mask = ((1 << pipe->num_inputs) - 1) << 1;
	if (!pipe->lif)
		mask |= 1 << 0;

	return pipe->buffers_ready == mask;
}

/*
 * vsp1_video_complete_buffer - Complete the current buffer
 * @video: the video node
 *
 * This function completes the current buffer by filling its sequence number,
 * time stamp and payload size, and hands it back to the videobuf core.
 *
 * When operating in DU output mode (deep pipeline to the DU through the LIF),
 * the VSP1 needs to constantly supply frames to the display. In that case, if
 * no other buffer is queued, reuse the one that has just been processed instead
 * of handing it back to the videobuf core.
 *
 * Return the next queued buffer or NULL if the queue is empty.
 */
static struct vsp1_video_buffer *
vsp1_video_complete_buffer(struct vsp1_video *video, bool is_input)
{
	struct vsp1_pipeline *pipe = to_vsp1_pipeline(&video->video.entity);
	struct vsp1_video_buffer *next = NULL;
	struct vsp1_video_buffer *done;
	unsigned long flags;
	unsigned int i;
	struct vsp1_device *vsp1 = pipe->output->entity.vsp1;


	spin_lock_irqsave(&video->irqlock, flags);

	if (list_empty(&video->irqqueue)) {
		spin_unlock_irqrestore(&video->irqlock, flags);
		return NULL;
	}

	done = list_first_entry(&video->irqqueue,
				struct vsp1_video_buffer, queue);

	if (!list_is_singular(&video->irqqueue))
		next = list_entry(done->queue.next,
					struct vsp1_video_buffer, queue);
	else if (pipe->lif
		|| V4L2_FIELD_IS_PICONV(video->format.field)) {
		/* Reuse the buffer if the list is singular. */
		spin_unlock_irqrestore(&video->irqlock, flags);
		return done;
	}

	if (video->format.field == V4L2_FIELD_PICONV_DIVIDE
		|| (video->format.field == V4L2_FIELD_PICONV_COMPOSITE
		&& !is_input)) {
		if (vsp1->display_field == V4L2_FIELD_BOTTOM) {
			/* Use the bottom field of the buffer
			 * under the following conditions.
			 * (1) In DU output mode, PI conversion mode
			 *     is DIVIDE mode.
			 * (2) In memory output mode, PI conversion mode
			 *     is COMPOSITE mode, and the buffer
			 *     is for output. */
			spin_unlock_irqrestore(&video->irqlock, flags);
			return done;
		}
	}

	if (video->format.field == V4L2_FIELD_PICONV_EXTRACT
		|| (video->format.field == V4L2_FIELD_PICONV_COMPOSITE
		&& is_input)) {
		if (vsp1->display_field != next->buf.v4l2_buf.field) {
			/* Reuse the buffer under the following conditions
			 * (1) DU's field differs form the next buffer's field.
			 * (2) In DU output mode, PI conversion mode
			 *     is EXTRACT mode.
			 * (3) In memory output mode, PI conversion mode
			 *     is COMPOSITE mode, and the buffer
			 *     is for input. */
			spin_unlock_irqrestore(&video->irqlock, flags);
			return done;
		}
	}

	list_del(&done->queue);

	spin_unlock_irqrestore(&video->irqlock, flags);

	done->buf.v4l2_buf.sequence = video->sequence++;
	v4l2_get_timestamp(&done->buf.v4l2_buf.timestamp);
	for (i = 0; i < done->buf.num_planes; ++i)
		vb2_set_plane_payload(&done->buf, i, done->length[i]);
	vb2_buffer_done(&done->buf, VB2_BUF_STATE_DONE);

	return next;
}

static void vsp1_video_frame_end(struct vsp1_pipeline *pipe,
				 struct vsp1_video *video,
				 bool is_input)
{
	struct vsp1_video_buffer *buf;
	unsigned long flags;

	buf = vsp1_video_complete_buffer(video, is_input);
	if (buf == NULL)
		return;

	spin_lock_irqsave(&pipe->irqlock, flags);

	vsp1_update_reg(pipe, video, buf);

	video->ops->queue(video, buf);
	pipe->buffers_ready |= 1 << video->pipe_index;

	spin_unlock_irqrestore(&pipe->irqlock, flags);
}

void vsp1_pipeline_frame_end(struct vsp1_pipeline *pipe)
{
	enum vsp1_pipeline_state state;
	unsigned long flags;
	unsigned int i;

	if (pipe == NULL)
		return;

	/* Complete buffers on all video nodes. */
	for (i = 0; i < pipe->num_inputs; ++i)
		vsp1_video_frame_end(pipe, &pipe->inputs[i]->video, true);

	if (!pipe->lif)
		vsp1_video_frame_end(pipe, &pipe->output->video, false);

	spin_lock_irqsave(&pipe->irqlock, flags);

	state = pipe->state;
	pipe->state = VSP1_PIPELINE_STOPPED;

	/* If a stop has been requested, mark the pipeline as stopped and
	 * return.
	 */
	if (state == VSP1_PIPELINE_STOPPING) {
		wake_up(&pipe->wq);
		goto done;
	}

	/* Restart the pipeline if ready. */
	if (vsp1_pipeline_ready(pipe))
		vsp1_pipeline_run(pipe);

done:
	spin_unlock_irqrestore(&pipe->irqlock, flags);
}

int vsp1_pipeline_suspend(struct vsp1_pipeline *pipe)
{
	unsigned long flags;
	int ret;

	if (pipe == NULL)
		return 0;

	spin_lock_irqsave(&pipe->irqlock, flags);
	if (pipe->state == VSP1_PIPELINE_RUNNING)
		pipe->state = VSP1_PIPELINE_STOPPING;
	spin_unlock_irqrestore(&pipe->irqlock, flags);

	ret = wait_event_timeout(pipe->wq, pipe->state == VSP1_PIPELINE_STOPPED,
				 msecs_to_jiffies(500));
	ret = ret == 0 ? -ETIMEDOUT : 0;

	return ret;
}

void vsp1_pipeline_resume(struct vsp1_pipeline *pipe)
{
	if (pipe == NULL)
		return;

	if (vsp1_pipeline_ready(pipe))
		vsp1_pipeline_run(pipe);
}

/*
 * Propagate the alpha value through the pipeline.
 *
 * As the UDS has restricted scaling capabilities when the alpha component needs
 * to be scaled, we disable alpha scaling when the UDS input has a fixed alpha
 * value. The UDS then outputs a fixed alpha value which needs to be programmed
 * from the input RPF alpha.
 */
void vsp1_pipeline_propagate_alpha(struct vsp1_pipeline *pipe,
				   struct vsp1_entity *input,
				   unsigned int alpha)
{
	struct vsp1_entity *entity;
	struct media_pad *pad;

	pad = media_entity_remote_pad(&input->pads[RWPF_PAD_SOURCE]);

	while (pad) {
		if (media_entity_type(pad->entity) != MEDIA_ENT_T_V4L2_SUBDEV)
			break;

		entity = to_vsp1_entity(media_entity_to_v4l2_subdev(pad->entity));

		/* The BRU background color has a fixed alpha value set to 255,
		 * the output alpha value is thus always equal to 255.
		 */
		if (entity->type == VSP1_ENTITY_BRU)
			alpha = 255;

		if (entity->type == VSP1_ENTITY_UDS) {
			struct vsp1_uds *uds = to_uds(&entity->subdev);

			vsp1_uds_set_alpha(uds, alpha);
			break;
		}

		pad = &entity->pads[entity->source_pad];
		pad = media_entity_remote_pad(pad);
	}
}

/* -----------------------------------------------------------------------------
 * videobuf2 Queue Operations
 */

static int
vsp1_video_queue_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
		     unsigned int *nbuffers, unsigned int *nplanes,
		     unsigned int sizes[], void *alloc_ctxs[])
{
	struct vsp1_video *video = vb2_get_drv_priv(vq);
	const struct v4l2_pix_format_mplane *format;
	struct v4l2_pix_format_mplane pix_mp;
	unsigned int i;

	if (fmt) {
		/* Make sure the format is valid and adjust the sizeimage field
		 * if needed.
		 */
		if (!vsp1_video_format_adjust(video, &fmt->fmt.pix_mp, &pix_mp))
			return -EINVAL;

		format = &pix_mp;
	} else {
		format = &video->format;
	}

	*nplanes = format->num_planes;

	for (i = 0; i < format->num_planes; ++i) {
		sizes[i] = format->plane_fmt[i].sizeimage;
		alloc_ctxs[i] = video->alloc_ctx;
	}

	return 0;
}

static int vsp1_video_buffer_prepare(struct vb2_buffer *vb)
{
	struct vsp1_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct vsp1_video_buffer *buf = to_vsp1_video_buffer(vb);
	const struct v4l2_pix_format_mplane *format = &video->format;
	unsigned int i;

	if (vb->num_planes < format->num_planes)
		return -EINVAL;

	buf->video = video;

	for (i = 0; i < vb->num_planes; ++i) {
		buf->addr[i] = vb2_dma_contig_plane_dma_addr(vb, i);
		buf->length[i] = vb2_plane_size(vb, i);

		if (buf->length[i] < format->plane_fmt[i].sizeimage)
			return -EINVAL;
	}

	return 0;
}

static void vsp1_video_set_bottom(struct vsp1_video_buffer *buf,
		     struct v4l2_pix_format_mplane *format)
{
	int index = 0;

	for (index = 0; index < buf->buf.num_planes; index++)
		buf->addr_btm[index] = buf->addr[index]
			+ format->plane_fmt[index].bytesperline;
}

static int vsp1_get_du_channel(int id)
{
	void __iomem *planereg;
	void __iomem *prioreg;
	void __iomem *ppr;
	u32 product_id;
	u32 val;
	u32 mask = 0x0000000F;
	bool isDU0 = false;
	int ret = DU_CH_NONE;
	int planeno;
	int i;
	u32 plnreg0;
	u32 plnreg2;

	switch (id) {
	case VSPD_CH_ID_1:
		planeno = PLANE_NO_1;
		plnreg0 = PLANEREG0_PLN1_ADDR;
		break;
	case VSPD_CH_ID_2:
		planeno = PLANE_NO_2;
		plnreg0 = PLANEREG0_PLN2_ADDR;
		plnreg2 = PLANEREG2_PLN1_ADDR;
		break;
	default:
		/* Return if the device is not VSPD */
		return DU_CH_NONE;
	}
	/* Check connect VSP and DU (PnDDC4R0) */
	planereg = ioremap_nocache(plnreg0, REG_SIZE);
	val = ioread32(planereg) & PLANEREG_PNVSPS;
	iounmap(planereg);
	if (val != 0x0) {
		/* check connect DU0 or DU1 (DS0PR0.S0Sx)*/
		prioreg = ioremap_nocache(PRIOREG_ADDR, REG_SIZE);
		val = ioread32(prioreg);
		iounmap(prioreg);
		for (i = 0; i < 8; i++) {
			if ((val & mask) == planeno) {
				isDU0 = true;
				break;
			}
			val = val >> 4;
		}
		if (isDU0)
			/*connect DU0*/
			ret = DU_CH_0;
		else
			/*connect DU1*/
			ret = DU_CH_1;
	} else {
		/* Check connect VSP and DU (PnDDC4R2, H2 only.) */
		ppr = ioremap_nocache(PRR_ADDRESS, REG_SIZE);
		product_id = ioread32(ppr) & PRODUCT_MASK;
		iounmap(ppr);
		if (product_id == PRODUCT_ID_H2 && id == VSPD_CH_ID_2) {
			planereg = ioremap_nocache(plnreg2, REG_SIZE);
			val = ioread32(planereg) & PLANEREG_PNVSPS;
			iounmap(planereg);
			if (val != 0x0)
				/*connect DU2*/
				ret = DU_CH_2;
		}
	}

	return ret;
}

static void vsp1_sync_du(int duch, int target, int limit)
{
	int cnt;
	int frm;
	for (cnt = 0; cnt < limit; cnt++) {
#if IS_ENABLED(CONFIG_DRM)
		frm = rcar_du_get_frmend(duch);
#endif
		if (frm == target)
			break;
		usleep_range(100, 200);
	}
}

static void vsp1_video_buffer_queue(struct vb2_buffer *vb)
{
	struct vsp1_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct vsp1_pipeline *pipe = to_vsp1_pipeline(&video->video.entity);
	struct vsp1_video_buffer *buf = to_vsp1_video_buffer(vb);
	struct vsp1_device *vsp1 = pipe->output->entity.vsp1;
	unsigned long flags;
	bool empty;
	struct vsp1_rwpf *rwpf = container_of(video, struct vsp1_rwpf, video);
	int duch;

	spin_lock_irqsave(&video->irqlock, flags);
	empty = list_empty(&video->irqqueue);
	if (V4L2_FIELD_IS_PICONV(video->format.field))
		vsp1_video_set_bottom(buf, &rwpf->video.format);
	vsp1_set_cropupdate(video, buf);
	list_add_tail(&buf->queue, &video->irqqueue);
	spin_unlock_irqrestore(&video->irqlock, flags);

	if (!empty)
		return;

	vsp1->display_field = V4L2_FIELD_TOP;

	if (video->format.field == V4L2_FIELD_PICONV_DIVIDE
		|| video->format.field == V4L2_FIELD_PICONV_EXTRACT) {
		duch = vsp1_get_du_channel(vsp1->id);
		if (duch != DU_CH_NONE) {
			vsp1_sync_du(duch, 0, 1000);
			vsp1_sync_du(duch, 1, 1000);
		}
	}

	spin_lock_irqsave(&pipe->irqlock, flags);

	vsp1_update_reg(pipe, video, buf);
	video->ops->queue(video, buf);
	pipe->buffers_ready |= 1 << video->pipe_index;

	if (vb2_is_streaming(&video->queue) &&
	    vsp1_pipeline_ready(pipe))
		vsp1_pipeline_run(pipe);

	spin_unlock_irqrestore(&pipe->irqlock, flags);
}

static void vsp1_entity_route_setup(struct vsp1_entity *source)
{
	struct vsp1_entity *sink;

	if (source->route->reg == 0)
		return;

	sink = container_of(source->sink, struct vsp1_entity, subdev.entity);
	vsp1_write(source->vsp1, source->route->reg,
		   sink->route->inputs[source->sink_pad]);
}

static int vsp1_video_start_streaming(struct vb2_queue *vq, unsigned int count)
{
	struct vsp1_video *video = vb2_get_drv_priv(vq);
	struct vsp1_pipeline *pipe = to_vsp1_pipeline(&video->video.entity);
	struct vsp1_entity *entity;
	unsigned long flags;
	int ret;

	mutex_lock(&pipe->lock);
	if (pipe->stream_count == pipe->num_video - 1) {
		if (pipe->uds) {
			struct vsp1_uds *uds = to_uds(&pipe->uds->subdev);

			/* If a BRU is present in the pipeline before the UDS,
			 * the alpha component doesn't need to be scaled as the
			 * BRU output alpha value is fixed to 255. Otherwise we
			 * need to scale the alpha component only when available
			 * at the input RPF.
			 */
			if (pipe->uds_input->type == VSP1_ENTITY_BRU) {
				uds->scale_alpha = false;
			} else {
				struct vsp1_rwpf *rpf =
					to_rwpf(&pipe->uds_input->subdev);

				uds->scale_alpha = rpf->video.fmtinfo->alpha;
			}
		}

		list_for_each_entry(entity, &pipe->entities, list_pipe) {
			vsp1_entity_route_setup(entity);

			ret = v4l2_subdev_call(&entity->subdev, video,
					       s_stream, 1);
			if (ret < 0) {
				mutex_unlock(&pipe->lock);
				return ret;
			}
		}
	}

	pipe->stream_count++;
	mutex_unlock(&pipe->lock);

	spin_lock_irqsave(&pipe->irqlock, flags);
	if (vsp1_pipeline_ready(pipe))
		vsp1_pipeline_run(pipe);
	spin_unlock_irqrestore(&pipe->irqlock, flags);

	return 0;
}

static int vsp1_video_stop_streaming(struct vb2_queue *vq)
{
	struct vsp1_video *video = vb2_get_drv_priv(vq);
	struct vsp1_pipeline *pipe = to_vsp1_pipeline(&video->video.entity);
	struct vsp1_video_buffer *buffer;
	unsigned long flags;
	int ret;

	mutex_lock(&pipe->lock);
	if (--pipe->stream_count == 0) {
		/* Stop the pipeline. */
		ret = vsp1_pipeline_stop(pipe);
		if (ret == -ETIMEDOUT)
			dev_err(video->vsp1->dev, "pipeline stop timeout\n");
	}
	mutex_unlock(&pipe->lock);

	vsp1_pipeline_cleanup(pipe);
	media_entity_pipeline_stop(&video->video.entity);

	/* Remove all buffers from the IRQ queue. */
	spin_lock_irqsave(&video->irqlock, flags);
	list_for_each_entry(buffer, &video->irqqueue, queue)
		vb2_buffer_done(&buffer->buf, VB2_BUF_STATE_ERROR);
	INIT_LIST_HEAD(&video->irqqueue);
	spin_unlock_irqrestore(&video->irqlock, flags);

	return 0;
}

static struct vb2_ops vsp1_video_queue_qops = {
	.queue_setup = vsp1_video_queue_setup,
	.buf_prepare = vsp1_video_buffer_prepare,
	.buf_queue = vsp1_video_buffer_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.start_streaming = vsp1_video_start_streaming,
	.stop_streaming = vsp1_video_stop_streaming,
};

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int
vsp1_video_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
	struct v4l2_fh *vfh = file->private_data;
	struct vsp1_video *video = to_vsp1_video(vfh->vdev);

	cap->capabilities = V4L2_CAP_DEVICE_CAPS | V4L2_CAP_STREAMING
			  | V4L2_CAP_VIDEO_CAPTURE_MPLANE
			  | V4L2_CAP_VIDEO_OUTPUT_MPLANE;

	if (video->type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		cap->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE
				 | V4L2_CAP_STREAMING;
	else
		cap->device_caps = V4L2_CAP_VIDEO_OUTPUT_MPLANE
				 | V4L2_CAP_STREAMING;

	strlcpy(cap->driver, "vsp1", sizeof(cap->driver));
	strlcpy(cap->card, video->video.name, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(video->vsp1->dev));

	return 0;
}

static int
vsp1_video_get_format(struct file *file, void *fh, struct v4l2_format *format)
{
	struct v4l2_fh *vfh = file->private_data;
	struct vsp1_video *video = to_vsp1_video(vfh->vdev);

	if (format->type != video->queue.type)
		return -EINVAL;

	mutex_lock(&video->lock);
	format->fmt.pix_mp = video->format;
	mutex_unlock(&video->lock);

	return 0;
}

static int
vsp1_video_try_format(struct file *file, void *fh, struct v4l2_format *format)
{
	struct v4l2_fh *vfh = file->private_data;
	struct vsp1_video *video = to_vsp1_video(vfh->vdev);

	if (format->type != video->queue.type)
		return -EINVAL;

	return __vsp1_video_try_format(video, &format->fmt.pix_mp, NULL);
}

static int
vsp1_video_set_format(struct file *file, void *fh, struct v4l2_format *format)
{
	struct v4l2_fh *vfh = file->private_data;
	struct vsp1_video *video = to_vsp1_video(vfh->vdev);
	const struct vsp1_format_info *info;
	int ret;

	if (format->type != video->queue.type)
		return -EINVAL;

	ret = __vsp1_video_try_format(video, &format->fmt.pix_mp, &info);
	if (ret < 0)
		return ret;

	mutex_lock(&video->lock);

	if (vb2_is_busy(&video->queue)) {
		ret = -EBUSY;
		goto done;
	}

	video->format = format->fmt.pix_mp;
	video->fmtinfo = info;

done:
	mutex_unlock(&video->lock);
	return ret;
}

static int
vsp1_video_streamon(struct file *file, void *fh, enum v4l2_buf_type type)
{
	struct v4l2_fh *vfh = file->private_data;
	struct vsp1_video *video = to_vsp1_video(vfh->vdev);
	struct vsp1_pipeline *pipe;
	int ret;

	if (video->queue.owner && video->queue.owner != file->private_data)
		return -EBUSY;

	video->sequence = 0;

	/* Start streaming on the pipeline. No link touching an entity in the
	 * pipeline can be activated or deactivated once streaming is started.
	 *
	 * Use the VSP1 pipeline object embedded in the first video object that
	 * starts streaming.
	 */
	pipe = video->video.entity.pipe
	     ? to_vsp1_pipeline(&video->video.entity) : &video->pipe;

	ret = media_entity_pipeline_start(&video->video.entity, &pipe->pipe);
	if (ret < 0)
		return ret;

	/* Verify that the configured format matches the output of the connected
	 * subdev.
	 */
	ret = vsp1_video_verify_format(video);
	if (ret < 0)
		goto err_stop;

	ret = vsp1_pipeline_init(pipe, video);
	if (ret < 0)
		goto err_stop;

	/* Start the queue. */
	ret = vb2_streamon(&video->queue, type);
	if (ret < 0)
		goto err_cleanup;

	return 0;

err_cleanup:
	vsp1_pipeline_cleanup(pipe);
err_stop:
	media_entity_pipeline_stop(&video->video.entity);
	return ret;
}

static int vsp1_video_set_crop(struct file *file, void *fh, const struct v4l2_crop *a)
{
	struct v4l2_fh *vfh = file->private_data;
	struct vsp1_video *video = to_vsp1_video(vfh->vdev);
	struct vsp1_pipeline *pipe = to_vsp1_pipeline(&video->video.entity);
	unsigned long flags;

	spin_lock_irqsave(&pipe->irqlock, flags);
	if (video->update_crop != VSP1_UPDATE_CROP_NORMAL) {
		spin_unlock_irqrestore(&pipe->irqlock, flags);
		return -EPERM;
	}
	video->update_crop = VSP1_UPDATE_CROP_REQUESTED;
	spin_unlock_irqrestore(&pipe->irqlock, flags);
	return 0;
}

static const struct v4l2_ioctl_ops vsp1_video_ioctl_ops = {
	.vidioc_querycap		= vsp1_video_querycap,
	.vidioc_g_fmt_vid_cap_mplane	= vsp1_video_get_format,
	.vidioc_s_fmt_vid_cap_mplane	= vsp1_video_set_format,
	.vidioc_try_fmt_vid_cap_mplane	= vsp1_video_try_format,
	.vidioc_g_fmt_vid_out_mplane	= vsp1_video_get_format,
	.vidioc_s_fmt_vid_out_mplane	= vsp1_video_set_format,
	.vidioc_try_fmt_vid_out_mplane	= vsp1_video_try_format,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_streamon		= vsp1_video_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
	.vidioc_s_crop			= vsp1_video_set_crop,
};

/* -----------------------------------------------------------------------------
 * V4L2 File Operations
 */

static int vsp1_video_open(struct file *file)
{
	struct vsp1_video *video = video_drvdata(file);
	struct v4l2_fh *vfh;
	int ret = 0;

	vfh = kzalloc(sizeof(*vfh), GFP_KERNEL);
	if (vfh == NULL)
		return -ENOMEM;

	v4l2_fh_init(vfh, &video->video);
	v4l2_fh_add(vfh);

	file->private_data = vfh;

	ret = vsp1_device_get(video->vsp1);
	if (ret < 0) {
		v4l2_fh_del(vfh);
		kfree(vfh);
	}

	return ret;
}

static int vsp1_video_release(struct file *file)
{
	struct vsp1_video *video = video_drvdata(file);
	struct v4l2_fh *vfh = file->private_data;

	mutex_lock(&video->lock);
	if (video->queue.owner == vfh) {
		vb2_queue_release(&video->queue);
		video->queue.owner = NULL;
	}
	mutex_unlock(&video->lock);

	vsp1_device_put(video->vsp1);

	v4l2_fh_release(file);

	file->private_data = NULL;

	return 0;
}

static struct v4l2_file_operations vsp1_video_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = vsp1_video_open,
	.release = vsp1_video_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
};

/* -----------------------------------------------------------------------------
 * Initialization and Cleanup
 */

int vsp1_video_init(struct vsp1_video *video, struct vsp1_entity *rwpf)
{
	const char *direction;
	int ret;

	switch (video->type) {
	case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
		direction = "output";
		video->pad.flags = MEDIA_PAD_FL_SINK;
		break;

	case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
		direction = "input";
		video->pad.flags = MEDIA_PAD_FL_SOURCE;
		video->video.vfl_dir = VFL_DIR_TX;
		break;

	default:
		return -EINVAL;
	}

	video->rwpf = rwpf;

	mutex_init(&video->lock);
	spin_lock_init(&video->irqlock);
	INIT_LIST_HEAD(&video->irqqueue);

	mutex_init(&video->pipe.lock);
	spin_lock_init(&video->pipe.irqlock);
	INIT_LIST_HEAD(&video->pipe.entities);
	init_waitqueue_head(&video->pipe.wq);
	video->pipe.state = VSP1_PIPELINE_STOPPED;

	/* Initialize the media entity... */
	ret = media_entity_init(&video->video.entity, 1, &video->pad, 0);
	if (ret < 0)
		return ret;

	/* ... and the format ... */
	video->fmtinfo = vsp1_get_format_info(VSP1_VIDEO_DEF_FORMAT);
	video->format.pixelformat = video->fmtinfo->fourcc;
	video->format.colorspace = V4L2_COLORSPACE_SRGB;
	video->format.field = V4L2_FIELD_NONE;
	video->format.width = VSP1_VIDEO_DEF_WIDTH;
	video->format.height = VSP1_VIDEO_DEF_HEIGHT;
	video->format.num_planes = 1;
	video->format.plane_fmt[0].bytesperline =
		video->format.width * video->fmtinfo->bpp[0] / 8;
	video->format.plane_fmt[0].sizeimage =
		video->format.plane_fmt[0].bytesperline * video->format.height;

	/* ... and the video node... */
	video->video.v4l2_dev = &video->vsp1->v4l2_dev;
	video->video.fops = &vsp1_video_fops;
	snprintf(video->video.name, sizeof(video->video.name), "%s %s",
		 rwpf->subdev.name, direction);
	video->video.vfl_type = VFL_TYPE_GRABBER;
	video->video.release = video_device_release_empty;
	video->video.ioctl_ops = &vsp1_video_ioctl_ops;

	video_set_drvdata(&video->video, video);

	/* ... and the buffers queue... */
	video->alloc_ctx = vb2_dma_contig_init_ctx(video->vsp1->dev);
	if (IS_ERR(video->alloc_ctx)) {
		ret = PTR_ERR(video->alloc_ctx);
		goto error;
	}

	video->queue.type = video->type;
	video->queue.io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
	video->queue.lock = &video->lock;
	video->queue.drv_priv = video;
	video->queue.buf_struct_size = sizeof(struct vsp1_video_buffer);
	video->queue.ops = &vsp1_video_queue_qops;
	video->queue.mem_ops = &vb2_dma_contig_memops;
	video->queue.timestamp_type = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	ret = vb2_queue_init(&video->queue);
	if (ret < 0) {
		dev_err(video->vsp1->dev, "failed to initialize vb2 queue\n");
		goto error;
	}

	/* ... and register the video device. */
	video->video.queue = &video->queue;
	ret = video_register_device(&video->video, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		dev_err(video->vsp1->dev, "failed to register video device\n");
		goto error;
	}

	video->update_crop = VSP1_UPDATE_CROP_NORMAL;
	return 0;

error:
	vb2_dma_contig_cleanup_ctx(video->alloc_ctx);
	vsp1_video_cleanup(video);
	return ret;
}

void vsp1_video_cleanup(struct vsp1_video *video)
{
	if (video_is_registered(&video->video))
		video_unregister_device(&video->video);

	vb2_dma_contig_cleanup_ctx(video->alloc_ctx);
	media_entity_cleanup(&video->video.entity);
}
