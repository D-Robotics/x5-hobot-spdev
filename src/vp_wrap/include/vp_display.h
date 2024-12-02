/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2024 D-Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-03-05 15:28:19
 * @LastEditTime: 2023-03-05 16:31:05
 ***************************************************************************/
#ifndef VP_DISPLAY_H_
#define VP_DISPLAY_H_

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <drm/drm.h>
#include <drm/drm_mode.h>
#include <xf86drm.h>
#include <xf86drmMode.h>
#include <drm_fourcc.h>

#include "vp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DISPLAY_DEFAULT_WIDTH 1920
#define DISPLAY_DEFAULT_HEIGHT 1080

#define FONT_WORD_HEIGHT 16
#define FONT_ONE_ENCODE_WIDTH 8
#define FONT_CN_ENCODE_NUM 2
#define FONT_EN_ENCODE_NUM 1
#define FONT_CN_WORD_WIDTH \
    (FONT_CN_ENCODE_NUM * FONT_ONE_ENCODE_WIDTH)
#define FONT_EN_WORD_WIDTH \
    (FONT_EN_ENCODE_NUM * FONT_ONE_ENCODE_WIDTH)
#define FONT_HARD_PIXEL_BITS 4
#define ONE_BYTE_BIT_CNT 8

#define DISPLAY_ARGB_BYTES 4

#define FONT_MASK_80 0x80
#define FONT_MASK_40 0x40
#define FONT_MASK_20 0x20
#define FONT_MASK_10 0x10
#define FONT_MASK_08 0x08
#define FONT_MASK_04 0x04
#define FONT_MASK_02 0x02
#define FONT_MASK_01 0x01

#define FONT_INTERVAL_CN_WORD_CNT 94
#define FONT_CN_WORD_START_ENCODE 0xa0
#define FONT_CN_WORD_BYTES 32
#define FONT_EN_WORD_BYTES 16

#define SDK_FONT_HZK16_FILE "/usr/hobot/lib/HZK16"
#define SDK_FONT_ASC16_FILE "/usr/hobot/lib/ASC16"

#define DRM_MAX_PLANES 3
#define DRM_ION_MAX_BUFFERS 3

typedef struct
{
	uint32_t plane_id;
	uint32_t src_w;
	uint32_t src_h;
	uint32_t crtc_x;
	uint32_t crtc_y;
	uint32_t crtc_w;
	uint32_t crtc_h;
	uint32_t z_pos;
	uint32_t alpha;
	char format[8];          // 图层格式
	uint32_t rotation;       // 旋转属性
	uint32_t color_encoding; // 颜色编码
	uint32_t color_range;    // 颜色范围
	uint32_t pixel_blend_mode; // alpha模式
} drm_plane_config_t;

typedef struct
{
	int drm_fd;
	uint32_t crtc_id;
	uint32_t connector_id;
	drm_plane_config_t planes[DRM_MAX_PLANES];
	uint32_t width;
	uint32_t height;
	int plane_count;
} vp_drm_context_t;

int32_t vp_display_init(vp_drm_context_t *drm_ctx, int32_t width, int32_t height);
int32_t vp_display_deinit(vp_drm_context_t *drm_ctx);
int32_t vp_display_set_frame(vp_drm_context_t *drm_ctx, int32_t plane_idx,
	hbn_vnode_image_t *image_frame);

void vp_display_draw_rect(uint8_t *frame, int32_t x0, int32_t y0, int32_t x1, int32_t y1,
	int32_t color, int32_t fill, int32_t screen_width, int32_t screen_height,
	int32_t line_width);
int32_t vp_display_draw_word(uint8_t *addr, int32_t x, int32_t y, char *str,
	int32_t width, int32_t color, int32_t line_width);
int32_t vp_display_get_res(int32_t *width , int32_t *height);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // VP_DISPLAY_H_
