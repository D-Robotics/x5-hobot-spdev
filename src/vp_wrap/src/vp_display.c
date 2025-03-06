// Copyright (c) 2024，D-Robotics.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// #define J5
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/mman.h>
#include <sys/wait.h>

#include <xf86drm.h>
#include <xf86drmMode.h>

#include "utils/cJSON.h"
#include "utils/utils_log.h"
#include "utils/common_utils.h"

#include "vp_display.h"
#include "vp_wrap.h"

typedef struct {
	int dma_buf_fd;
	int fb_id;
} dma_buf_map_t;

#define MAX_MAPPINGS 64
dma_buf_map_t *mappings;
size_t *mapping_count;

/*定义分辨率的结构体*/
typedef struct {
    int width;
    int height;
} Resolution;

// 定义要剔除的分辨率黑名单
static const Resolution blacklist[] = {
    {1440, 900},
};
static const int blacklist_size = sizeof(blacklist) / sizeof(blacklist[0]);

/* python在调用显示模块的图形、文字绘制时会创建子进程来处理
 * 所以需要共享内存来传递dma_buf_fd和fb_id，使父子进程的修改能够同步
 */
void initialize_shared_memory() {
	mappings = mmap(NULL, sizeof(dma_buf_map_t) * MAX_MAPPINGS, PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);
	mapping_count = mmap(NULL, sizeof(size_t), PROT_READ | PROT_WRITE, MAP_SHARED | MAP_ANONYMOUS, -1, 0);

	if (mappings == MAP_FAILED || mapping_count == MAP_FAILED) {
		perror("mmap");
		exit(1);
	}

	*mapping_count = 0;
}

bool add_mapping(int dma_buf_fd, int fb_id) {
	if (*mapping_count >= MAX_MAPPINGS) {
		return false;
	}

	mappings[*mapping_count].dma_buf_fd = dma_buf_fd;
	mappings[*mapping_count].fb_id = fb_id;
	(*mapping_count)++;
	printf("add mapping dma_buf_fd:%d fb_id:%d, mapping_count: %zu\n", dma_buf_fd, fb_id, *mapping_count);
	return true;
}

int find_fb_id(int dma_buf_fd) {
	for (size_t i = 0; i < *mapping_count; i++) {
		if (mappings[i].dma_buf_fd == dma_buf_fd) {
			return mappings[i].fb_id;
		}
	}
	return -1;
}

bool remove_mapping(int dma_buf_fd) {
	for (size_t i = 0; i < *mapping_count; i++) {
		if (mappings[i].dma_buf_fd == dma_buf_fd) {
			mappings[i] = mappings[*mapping_count - 1];
			(*mapping_count)--;
			return true;
		}
	}
	return false;
}

static void draw_dot(uint8_t *frame, int32_t x, int32_t y, int32_t color,
					 int32_t screen_width, int32_t screen_height)
{
	int32_t pbyte = DISPLAY_ARGB_BYTES;

	if (x >= screen_width || y >= screen_height)
		return;

	frame += ((y * screen_width) + x) * pbyte;
	while (pbyte) {
		pbyte--;
		frame[pbyte] = (color >> (pbyte * 8)) & 0xFF;
	}
}
static void draw_hline(uint8_t *frame, int32_t x0, int32_t x1, int32_t y, int32_t color,
					   int32_t screen_width, int32_t screen_height)
{
	int32_t xi, xa;

	xi = (x0 < x1) ? x0 : x1;
	xa = (x0 > x1) ? x0 : x1;
	while (xi <= xa) {
		draw_dot(frame, xi, y, color, screen_width, screen_height);
		xi++;
	}
}

static void draw_vline(uint8_t *frame, int32_t x, int32_t y0, int32_t y1, int32_t color,
					   int32_t screen_width, int32_t screen_height)
{
	int32_t yi, ya;

	yi = (y0 < y1) ? y0 : y1;
	ya = (y0 > y1) ? y0 : y1;
	while (yi <= ya) {
		draw_dot(frame, x, yi, color, screen_width, screen_height);
		yi++;
	}
}

void vp_display_draw_rect(uint8_t *frame, int32_t x0, int32_t y0, int32_t x1, int32_t y1,
	int32_t color, int32_t fill, int32_t screen_width, int32_t screen_height,
	int32_t line_width)
{
	int32_t xi, xa, yi, ya;
	int32_t i = 0;

	xi = (x0 < x1) ? x0 : x1; // left
	xa = (x0 > x1) ? x0 : x1; // right
	yi = (y0 < y1) ? y0 : y1; // bottom
	ya = (y0 > y1) ? y0 : y1; // top
	if (fill) {
		while (yi <= ya) {
			draw_hline(frame, xi, xa, yi, color, screen_width, screen_height);
			yi++;
		}
	} else {
		if (ya < line_width || yi > (screen_height - line_width) ||
			xi > (screen_width - line_width) ||
			xa > (screen_width - line_width)) {
			//SC_LOGE("========point is 0,return========\n");
			return;
		}
		for (i = 0; i < line_width; i++) {
			draw_hline(frame, xi, xa, yi + i, color, screen_width, screen_height);
			draw_hline(frame, xi, xa, ya - i, color, screen_width, screen_height);
			draw_vline(frame, xi + i, yi, ya, color, screen_width, screen_height);
			draw_vline(frame, xa + i, yi, ya, color, screen_width, screen_height);
		}
	}
}

static void osd_draw_word_row(uint8_t *addr, uint32_t width,
	uint32_t line_width, uint32_t color)
{
	uint32_t addr_offset;
	uint32_t m, w, i;

	for (m = 0; m < line_width; m++) {
		for (w = 0; w < line_width; w++) {
			addr_offset = ((width * m) + w) * DISPLAY_ARGB_BYTES;
			for (i = 0; i < DISPLAY_ARGB_BYTES; i++) {
				addr[addr_offset + i] = (color >> (i * 8)) & 0xff;
			}
		}
	}
}

// draw chinese word
static int32_t osd_draw_cn_word(uint8_t *addr, uint32_t width,
	uint32_t line_width, uint32_t color, uint8_t *cn_word)
{
	FILE *file;
	uint8_t flag;
	uint64_t offset;
	uint8_t *addr_word, *addr_src;
	uint8_t buffer[32];
	uint8_t key[8] = {FONT_MASK_80, FONT_MASK_40, FONT_MASK_20,
					  FONT_MASK_10, FONT_MASK_08, FONT_MASK_04,
					  FONT_MASK_02, FONT_MASK_01};
	uint32_t row_bytes;
	uint32_t k, j, i;
	size_t size;

	file = fopen(SDK_FONT_HZK16_FILE, "rb");
	if (file == NULL) {
		SC_LOGE("open HZK16 file fail %d %s\n", errno, strerror(errno));
		return -1;
	}

	offset = ((FONT_INTERVAL_CN_WORD_CNT *
		((uint64_t)cn_word[0] - FONT_CN_WORD_START_ENCODE - 1u)) +
		((uint64_t)cn_word[1] - FONT_CN_WORD_START_ENCODE) - 1u)
		* FONT_CN_WORD_BYTES;
	(void)fseek(file, (int64_t)offset, SEEK_SET);
	size = fread((void *)buffer, 1, FONT_CN_WORD_BYTES, file);
	if (size != FONT_CN_WORD_BYTES) {
		SC_LOGE("fread font file:%s error\n", SDK_FONT_HZK16_FILE);
		(void)fclose(file);
		return -1;
	}

	row_bytes = FONT_CN_WORD_WIDTH / ONE_BYTE_BIT_CNT;
	addr_src = addr;

	for (k = 0; k < FONT_WORD_HEIGHT; k++) {
		addr_word = addr_src;
		for (j = 0; j < row_bytes; j++) {
			for (i = 0; i < ONE_BYTE_BIT_CNT; i++) {
				flag = buffer[(k * row_bytes) + j] & key[i];
				if (flag != 0u) {
					osd_draw_word_row(addr_word, width, line_width, color);
				}
				addr_word = &addr_word[line_width * DISPLAY_ARGB_BYTES];
			}
		}
		addr_src = &addr_src[width * line_width * DISPLAY_ARGB_BYTES];
	}

	(void)fclose(file);

	return 0;
}

// draw endlish word
static int32_t osd_draw_en_word(uint8_t *addr, uint32_t width,
	uint32_t line_width, uint32_t color, uint8_t en_word)
{
	FILE *file;
	uint8_t flag;
	uint32_t offset;
	uint8_t *addr_word, *addr_src;
	uint8_t buffer[16];
	uint8_t key[8] = {FONT_MASK_80, FONT_MASK_40, FONT_MASK_20,
					  FONT_MASK_10, FONT_MASK_08, FONT_MASK_04,
					  FONT_MASK_02, FONT_MASK_01};
	uint32_t k, i;
	size_t size;

	file = fopen(SDK_FONT_ASC16_FILE, "rb");
	if (file == NULL) {
		SC_LOGE("open ASC16 file fail\n");
		return -1;
	}

	offset = (uint32_t)en_word * FONT_EN_WORD_BYTES;
	(void)fseek(file, (int32_t)offset, SEEK_SET);
	size = fread((void *)buffer, 1, FONT_EN_WORD_BYTES, file);
	if (size != FONT_EN_WORD_BYTES) {
		SC_LOGE("fread font file:%s error\n", SDK_FONT_ASC16_FILE);
		(void)fclose(file);
		return -1;
	}

	addr_src = addr;

	for (k = 0; k < FONT_WORD_HEIGHT; k++) {
		addr_word = addr_src;
		for (i = 0; i < ONE_BYTE_BIT_CNT; i++) {
			flag = buffer[k] & key[i];
			if (flag != 0u) {
				osd_draw_word_row(addr_word, width, line_width, color);
			}
			addr_word = &addr_word[line_width * DISPLAY_ARGB_BYTES];
		}
		addr_src = &addr_src[width * line_width * DISPLAY_ARGB_BYTES];
	}

	(void)fclose(file);

	return 0;
}

int32_t vp_display_draw_word(uint8_t *addr, int32_t x, int32_t y, char *str, int32_t width, int32_t color, int32_t line_width)
{
	uint32_t str_len, i, word_offs = 1u;
	uint8_t cn_word[2], en_word;
	int32_t ret;
	uint32_t addr_offset;

	if (addr == NULL) {
		SC_LOGE("draw word addr was NULL\n");
		return -1;
	}

	str_len = (uint32_t)strlen(str);

	addr_offset = ((y * width) + x) * DISPLAY_ARGB_BYTES;
	addr = &(addr)[addr_offset];

	for (i = 0; i < str_len; i += word_offs) {
		if (str[i] >= (uint8_t)FONT_CN_WORD_START_ENCODE) {
			cn_word[0] = str[i];
			cn_word[1] = str[i + 1u];
			if (cn_word[1] == '\0') {
				word_offs = FONT_CN_ENCODE_NUM;
				continue;
			}

			ret = osd_draw_cn_word(addr, width, line_width, color, cn_word);
			if (ret < 0) {
				return ret;
			}
			word_offs = FONT_CN_ENCODE_NUM;
			addr = &addr[line_width * FONT_CN_WORD_WIDTH * DISPLAY_ARGB_BYTES];
		}
		if (str[i] < (uint8_t)FONT_CN_WORD_START_ENCODE) {
			en_word = str[i];

			ret = osd_draw_en_word(addr, width, line_width, color, en_word);
			if (ret < 0) {
				return ret;
			}
			word_offs = FONT_EN_ENCODE_NUM;
			addr = &addr[line_width * FONT_EN_WORD_WIDTH * DISPLAY_ARGB_BYTES];
		}
	}
	return 0;
}

// 检查分辨率是否在黑名单中
int is_blacklisted(Resolution res) {
    for (int i = 0; i < blacklist_size; i++) {
        if (blacklist[i].width == res.width &&
            blacklist[i].height == res.height) {
            return 1;
        }
    }
    return 0;
}

// 比较函数用于排序
int compare_resolutions(const void *a, const void *b) {
    Resolution *res_a = (Resolution *)a;
    Resolution *res_b = (Resolution *)b;

    // 首先按照宽度排序
    if (res_a->width != res_b->width) {
        return res_b->width - res_a->width;
    }
    // 宽度相同时按照高度排序
    return res_b->height - res_a->height;
}

// 检查分辨率是否已存在
int resolution_exists(Resolution *resolutions, int count, Resolution new_res) {
    for (int i = 0; i < count; i++) {
        if (resolutions[i].width == new_res.width &&
            resolutions[i].height == new_res.height) {
            return 1;
        }
    }
    return 0;
}

int32_t vp_display_get_res(int32_t *width , int32_t *height)
{
    int fd;
    drmModeRes *resources;
    drmModeConnector *connector;
    Resolution *resolutions = NULL;
    int res_count = 0;
    int res_capacity = 10;

    resolutions = (Resolution *)malloc(res_capacity * sizeof(Resolution));
    if (!resolutions) {
        printf("Memory allocation failed\n");
        return;
    }

    fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        printf("Failed to open DRM device\n");
        free(resolutions);
        return;
    }

    resources = drmModeGetResources(fd);
    if (!resources) {
        printf("Failed to get DRM resources\n");
        close(fd);
        free(resolutions);
        return;
    }

    for (int i = 0; i < resources->count_connectors; i++) {
        connector = drmModeGetConnector(fd, resources->connectors[i]);
        if (!connector) {
            continue;
        }

        if (connector->connection == DRM_MODE_CONNECTED && connector->count_modes > 0) {
            for (int j = 0; j < connector->count_modes; j++) {
                Resolution current_res = {
                    .width = connector->modes[j].hdisplay,
                    .height = connector->modes[j].vdisplay
                };

                // 跳过黑名单中的分辨率
                if (is_blacklisted(current_res)) {
                    continue;
                }

                // 检查是否需要扩展数组
                if (res_count >= res_capacity) {
                    res_capacity *= 2;
                    Resolution *temp = (Resolution *)realloc(resolutions,
                                                          res_capacity * sizeof(Resolution));
                    if (!temp) {
                        printf("Memory reallocation failed\n");
                        drmModeFreeConnector(connector);
                        drmModeFreeResources(resources);
                        close(fd);
                        free(resolutions);
                        return;
                    }
                    resolutions = temp;
                }

                // 只添加不重复的分辨率
                if (!resolution_exists(resolutions, res_count, current_res)) {
                    resolutions[res_count++] = current_res;
                }
            }
        }

        drmModeFreeConnector(connector);
    }

    // 对分辨率进行排序
    qsort(resolutions, res_count, sizeof(Resolution), compare_resolutions);


    for (int i = 0; i < res_count; i++) {
        width[i] = resolutions[i].width;
        height[i] = resolutions[i].height;
        printf("%dx%d\n", resolutions[i].width, resolutions[i].height);
    }


    // 清理资源
    drmModeFreeResources(resources);
    close(fd);
    free(resolutions);
    return;
}

static void add_property(int drm_fd, drmModeAtomicReq *req, uint32_t obj_id,
	uint32_t obj_type, const char *name, uint64_t value)
{
	drmModeObjectProperties *props =
		drmModeObjectGetProperties(drm_fd, obj_id, obj_type);
	if (!props)
	{
		fprintf(stderr, "Failed to get properties for object %u\n", obj_id);
		return;
	}

	uint32_t prop_id = 0;
	for (uint32_t i = 0; i < props->count_props; i++)
	{
		drmModePropertyRes *prop = drmModeGetProperty(drm_fd, props->props[i]);
		if (!prop)
		{
			continue;
		}

		if (strcmp(prop->name, name) == 0)
		{
			prop_id = prop->prop_id;
			drmModeFreeProperty(prop);
			break;
		}

		drmModeFreeProperty(prop);
	}

	drmModeFreeObjectProperties(props);

	if (prop_id == 0)
	{
		fprintf(stderr, "Property '%s' not found on object %u\n", name, obj_id);
		return;
	}

	if (drmModeAtomicAddProperty(req, obj_id, prop_id, value) < 0)
	{
		fprintf(stderr, "Failed to add property '%s' on object %u: %s\n", name, obj_id, strerror(errno));
	}
}

static void drm_set_client_capabilities(int drm_fd)
{
	if (drmSetClientCap(drm_fd, DRM_CLIENT_CAP_ATOMIC, 1) < 0)
	{
		perror("drmSetClientCap DRM_CLIENT_CAP_ATOMIC");
	}

	if (drmSetClientCap(drm_fd, DRM_CLIENT_CAP_UNIVERSAL_PLANES, 1) < 0)
	{
		perror("drmSetClientCap DRM_CLIENT_CAP_UNIVERSAL_PLANES");
	}
}

static void print_connector_info(int drm_fd) {
	drmModeRes *resources = drmModeGetResources(drm_fd);
	if (!resources) {
		fprintf(stderr, "Failed to get DRM resources\n");
		return;
	}

	for (int i = 0; i < resources->count_crtcs; i++) {
		printf("CRTC ID: %d\n", resources->crtcs[i]);
	}

	printf("Number of connectors: %d\n", resources->count_connectors);

	for (int i = 0; i < resources->count_connectors; i++) {
		uint32_t connector_id = resources->connectors[i];
		drmModeConnector *connector = drmModeGetConnector(drm_fd, connector_id);
		if (!connector) {
			fprintf(stderr, "Failed to get connector %u\n", connector_id);
			continue;
		}

		printf("Connector ID: %u\n", connector_id);
		printf("    Type: %d\n", connector->connector_type);
		printf("    Type Name: %s\n", drmModeGetConnectorTypeName(connector->connector_type));
		printf("    Connection: %s\n", connector->connection == DRM_MODE_CONNECTED ? "Connected" : "Disconnected");
		printf("    Modes: %d\n", connector->count_modes);
		printf("    Subpixel: %d\n", connector->subpixel);

		for (int j = 0; j < connector->count_modes; j++) {
			drmModeModeInfo *mode = &connector->modes[j];
			printf("    Mode %d: %s @ %dHz\n", j, mode->name, mode->vrefresh);
		}

		drmModeFreeConnector(connector);
	}

	drmModeFreeResources(resources);
}

static int drm_setup_kms(vp_drm_context_t *ctx)
{

	print_connector_info(ctx->drm_fd);

	drmModeRes *resources = drmModeGetResources(ctx->drm_fd);
	if (!resources)
	{
		perror("drmModeGetResources");
		return -1;
	}

	drmModeConnector *connector = drmModeGetConnector(ctx->drm_fd, ctx->connector_id);
	if (!connector)
	{
		perror("drmModeGetConnector");
		drmModeFreeResources(resources);
		return -1;
	}

	drmModeCrtc *crtc = drmModeGetCrtc(ctx->drm_fd, ctx->crtc_id);
	if (!crtc)
	{
		perror("drmModeGetCrtc");
		drmModeFreeConnector(connector);
		drmModeFreeResources(resources);
		return -1;
	}

	drmModeModeInfo *mode = NULL;
	for (int i = 0; i < connector->count_modes; i++)
	{
		if (connector->modes[i].hdisplay == ctx->width && connector->modes[i].vdisplay == ctx->height)
		{
			mode = &connector->modes[i];
			break;
		}
	}

	if (!mode)
	{
		fprintf(stderr, "Mode not found\n");
		drmModeFreeCrtc(crtc);
		drmModeFreeConnector(connector);
		drmModeFreeResources(resources);
		return -1;
	}

	uint32_t blob_id;
	if (drmModeCreatePropertyBlob(ctx->drm_fd, mode, sizeof(*mode), &blob_id) < 0)
	{
		perror("drmModeCreatePropertyBlob");
		drmModeFreeCrtc(crtc);
		drmModeFreeConnector(connector);
		drmModeFreeResources(resources);
		return -1;
	}

	drmModeAtomicReq *req = drmModeAtomicAlloc();
	if (!req)
	{
		perror("drmModeAtomicAlloc");
		drmModeFreeCrtc(crtc);
		drmModeFreeConnector(connector);
		drmModeFreeResources(resources);
		return -1;
	}

	uint32_t flags = DRM_MODE_ATOMIC_ALLOW_MODESET;
	add_property(ctx->drm_fd, req, ctx->crtc_id, DRM_MODE_OBJECT_CRTC, "ACTIVE", 1);
	add_property(ctx->drm_fd, req, ctx->crtc_id, DRM_MODE_OBJECT_CRTC, "MODE_ID", blob_id);
	add_property(ctx->drm_fd, req, ctx->connector_id, DRM_MODE_OBJECT_CONNECTOR, "CRTC_ID", ctx->crtc_id);

	for (int i = 0; i < ctx->plane_count; i++)
	{
		add_property(ctx->drm_fd, req, ctx->planes[i].plane_id, DRM_MODE_OBJECT_PLANE, "SRC_X", 0);
		add_property(ctx->drm_fd, req, ctx->planes[i].plane_id, DRM_MODE_OBJECT_PLANE, "SRC_Y", 0);
		add_property(ctx->drm_fd, req, ctx->planes[i].plane_id, DRM_MODE_OBJECT_PLANE, "SRC_W", ctx->planes[i].src_w << 16);
		add_property(ctx->drm_fd, req, ctx->planes[i].plane_id, DRM_MODE_OBJECT_PLANE, "SRC_H", ctx->planes[i].src_h << 16);

		add_property(ctx->drm_fd, req, ctx->planes[i].plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_X", ctx->planes[i].crtc_x);
		add_property(ctx->drm_fd, req, ctx->planes[i].plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_Y", ctx->planes[i].crtc_y);
		add_property(ctx->drm_fd, req, ctx->planes[i].plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_W", ctx->planes[i].crtc_w);
		add_property(ctx->drm_fd, req, ctx->planes[i].plane_id, DRM_MODE_OBJECT_PLANE, "CRTC_H", ctx->planes[i].crtc_h);

		if (ctx->planes[i].z_pos != -1)
		{
			add_property(ctx->drm_fd, req, ctx->planes[i].plane_id, DRM_MODE_OBJECT_PLANE, "zpos", ctx->planes[i].z_pos);
		}

		if (ctx->planes[i].alpha != -1)
		{
			add_property(ctx->drm_fd, req, ctx->planes[i].plane_id, DRM_MODE_OBJECT_PLANE, "alpha", ctx->planes[i].alpha);
		}

		if (ctx->planes[i].pixel_blend_mode != -1)
		{
			add_property(ctx->drm_fd, req, ctx->planes[i].plane_id, DRM_MODE_OBJECT_PLANE, "pixel blend mode", ctx->planes[i].pixel_blend_mode);
		}

		if (ctx->planes[i].rotation != -1)
		{
			add_property(ctx->drm_fd, req, ctx->planes[i].plane_id, DRM_MODE_OBJECT_PLANE, "rotation", ctx->planes[i].rotation);
		}

		if (ctx->planes[i].color_encoding != -1)
		{
			add_property(ctx->drm_fd, req, ctx->planes[i].plane_id, DRM_MODE_OBJECT_PLANE, "COLOR_ENCODING", ctx->planes[i].color_encoding);
		}

		if (ctx->planes[i].color_range != -1)
		{
			add_property(ctx->drm_fd, req, ctx->planes[i].plane_id, DRM_MODE_OBJECT_PLANE, "COLOR_RANGE", ctx->planes[i].color_range);
		}
	}

	if (drmModeAtomicCommit(ctx->drm_fd, req, flags, NULL) < 0)
	{
		perror("drmModeAtomicCommit");
		drmModeAtomicFree(req);
		drmModeFreeCrtc(crtc);
		drmModeFreeConnector(connector);
		drmModeFreeResources(resources);
		return -1;
	}

	drmModeAtomicFree(req);
	drmModeFreeCrtc(crtc);
	drmModeFreeConnector(connector);
	drmModeFreeResources(resources);

	return 0;
}

static drmModeConnector* find_connector(int fd)
{
	int i = 0;

	// drmModeRes描述了计算机所有的显卡信息：connector，encoder，crtc，modes等。
	drmModeRes *resources = drmModeGetResources(fd);
	if (!resources)
	{
		return NULL;
	}

	drmModeConnector* conn = NULL;
	for (i = 0; i < resources->count_connectors; i++)
	{
		conn = drmModeGetConnector(fd, resources->connectors[i]);
		if (conn != NULL)
		{
			// Check if the connector type is HDMI and it's connected
			if (conn->connector_type == DRM_MODE_CONNECTOR_HDMIA &&
				conn->connection == DRM_MODE_CONNECTED &&
				conn->count_modes > 0)
			{
				break; // Found an HDMI connector that is connected and has available modes
			}
			else
			{
				drmModeFreeConnector(conn);
				conn = NULL; // Reset conn to NULL if it's not what we're looking for
			}
		}
	}

	drmModeFreeResources(resources);
	return conn; // Will return NULL if no suitable connector was found
}

static uint32_t find_overlay_plane_id(int drm_fd)
{
	drmModePlaneRes *plane_res = drmModeGetPlaneResources(drm_fd);
	if (!plane_res) {
		perror("drmModeGetPlaneResources failed");
		return 0;
	}

	uint32_t plane_id = 0;
	for (uint32_t i = 0; i < plane_res->count_planes; i++) {
		drmModePlane *plane = drmModeGetPlane(drm_fd, plane_res->planes[i]);
		if (!plane) {
			perror("drmModeGetPlane failed");
			continue;
		}

		// Check if the plane type is Overlay
		drmModeObjectProperties *props = drmModeObjectGetProperties(drm_fd, plane->plane_id, DRM_MODE_OBJECT_PLANE);
		if (props) {
			for (uint32_t j = 0; j < props->count_props; j++) {
				drmModePropertyRes *prop = drmModeGetProperty(drm_fd, props->props[j]);

				if (strcmp(prop->name, "type") == 0) {
					// If the plane type is "Overlay", assign this plane_id
					for (uint32_t k = 0; k < prop->count_enums; k++) {
						if (strcmp(prop->enums[k].name, "Overlay") == 0 && prop->enums[k].value == props->prop_values[j]) {
							plane_id = plane->plane_id;
							break;
						}
					}
				}
				drmModeFreeProperty(prop);
			}
			drmModeFreeObjectProperties(props);
		}

		drmModeFreePlane(plane);

		if (plane_id) {
			break;
		}
	}

	drmModeFreePlaneResources(plane_res);

	if (plane_id == 0) {
		printf("No suitable overlay plane found\n");
	}
	return plane_id;
}

static void drm_init_config(vp_drm_context_t *drm_ctx, int32_t width, int32_t height)
{
	drm_ctx->crtc_id = 31;
	drm_ctx->connector_id = 75;
	drm_ctx->width = width;
	drm_ctx->height = height;

	drm_ctx->plane_count = 1; // Support 1 plane in this example

	const char *formats[2] = { "NV12", "AR24" }; // Formats for planes

	for (int i = 0; i < drm_ctx->plane_count; i++) {
		uint32_t plane_id = find_overlay_plane_id(drm_ctx->drm_fd);
		if (plane_id == 0) {
			printf("Failed to find overlay plane for format %s\n", formats[i]);
			continue;
		}

		drm_ctx->planes[i].plane_id = plane_id;
		drm_ctx->planes[i].src_w = width;
		drm_ctx->planes[i].src_h = height;
		drm_ctx->planes[i].crtc_x = 0;
		drm_ctx->planes[i].crtc_y = 0;
		drm_ctx->planes[i].crtc_w = width;
		drm_ctx->planes[i].crtc_h = height;
		strcpy(drm_ctx->planes[i].format, formats[i]);

		drm_ctx->planes[i].z_pos = i; // Set z_pos based on plane index
		drm_ctx->planes[i].alpha = 65535;
		drm_ctx->planes[i].pixel_blend_mode = 1;
		drm_ctx->planes[i].rotation = -1;
		drm_ctx->planes[i].color_encoding = -1;
		drm_ctx->planes[i].color_range = -1;

		// Print plane details
		printf("------------------------------------------------------\n");
		printf("Plane %d:\n", i);
		printf("  Plane ID: %d\n", drm_ctx->planes[i].plane_id);
		printf("  Src W: %d\n", drm_ctx->planes[i].src_w);
		printf("  Src H: %d\n", drm_ctx->planes[i].src_h);
		printf("  CRTC X: %d\n", drm_ctx->planes[i].crtc_x);
		printf("  CRTC Y: %d\n", drm_ctx->planes[i].crtc_y);
		printf("  CRTC W: %d\n", drm_ctx->planes[i].crtc_w);
		printf("  CRTC H: %d\n", drm_ctx->planes[i].crtc_h);
		printf("  Format: %s\n", drm_ctx->planes[i].format);
		printf("  Z Pos: %d\n", drm_ctx->planes[i].z_pos);
		printf("  Alpha: %d\n", drm_ctx->planes[i].alpha);
		printf("  Pixel Blend Mode: %d\n", drm_ctx->planes[i].pixel_blend_mode);
		printf("  Rotation: %d\n", drm_ctx->planes[i].rotation);
		printf("  Color Encoding: %d\n", drm_ctx->planes[i].color_encoding);
		printf("  Color Range: %d\n", drm_ctx->planes[i].color_range);
		printf("------------------------------------------------------\n");
	}
}

int32_t vp_display_init(vp_drm_context_t *drm_ctx, int32_t width, int32_t height)
{
	int32_t ret = 0;
	drmModeConnectorPtr connector = NULL;

	memset(drm_ctx, 0, sizeof(vp_drm_context_t));

	drm_ctx->drm_fd = drmOpen("vs-drm", NULL);
	if (drm_ctx->drm_fd < 0) {
		perror("drmOpen failed");
		return -1;
	}

	drm_init_config(drm_ctx, width, height);

	connector = find_connector(drm_ctx->drm_fd);
	if (connector == NULL) {
		perror("find_connector failed");
		return -1;
	}
	drm_ctx->connector_id = connector->connector_id;

	printf("Setting DRM client capabilities...\n");
	drm_set_client_capabilities(drm_ctx->drm_fd);

	printf("Setting up KMS...\n");
	ret = drm_setup_kms(drm_ctx);
	if (ret < 0)
	{
		printf("drm_setup_kms failed\n");
		close(drm_ctx->drm_fd);
		return -1;
	}

	initialize_shared_memory();

	return ret;
}

int32_t vp_display_deinit(vp_drm_context_t *drm_ctx)
{
	int32_t ret = 0;

	munmap(mappings, sizeof(dma_buf_map_t) * MAX_MAPPINGS);
	munmap(mapping_count, sizeof(size_t));

	drmModeSetCrtc(drm_ctx->drm_fd, drm_ctx->crtc_id, 0, 0, 0, NULL, 0, NULL);

	drmModeRes *resources = drmModeGetResources(drm_ctx->drm_fd);
	if (!resources)
	{
		perror("drmModeGetResources");
		return -1;
	}

	for (int i = 0; i < resources->count_crtcs; i++)
	{
		drmModeFreeCrtc(drmModeGetCrtc(drm_ctx->drm_fd, resources->crtcs[i]));
	}

	for (int i = 0; i < resources->count_connectors; i++)
	{
		drmModeFreeConnector(drmModeGetConnector(drm_ctx->drm_fd, resources->connectors[i]));
	}

	for (int i = 0; i < resources->count_encoders; i++)
	{
		drmModeFreeEncoder(drmModeGetEncoder(drm_ctx->drm_fd, resources->encoders[i]));
	}

	drmModePlaneRes *plane_resources = drmModeGetPlaneResources(drm_ctx->drm_fd);
	if (plane_resources)
	{
		for (uint32_t i = 0; i < plane_resources->count_planes; i++)
		{
			drmModeFreePlane(drmModeGetPlane(drm_ctx->drm_fd, plane_resources->planes[i]));
		}
		drmModeFreePlaneResources(plane_resources);
	}

	drmModeFreeResources(resources);

	if (drm_ctx->drm_fd >= 0)
	{
		close(drm_ctx->drm_fd);
		drm_ctx->drm_fd = -1;
	}

	printf("\r\nDRM resources cleaned up.\n");

	return ret;
}

static uint32_t get_format_from_string(const char *format_str)
{
	if (strcmp(format_str, "AR12") == 0)
	{
		return DRM_FORMAT_ARGB4444;
	}
	else if (strcmp(format_str, "AR15") == 0)
	{
		return DRM_FORMAT_ARGB1555;
	}
	else if (strcmp(format_str, "RG16") == 0)
	{
		return DRM_FORMAT_RGB565;
	}
	else if (strcmp(format_str, "AR24") == 0)
	{
		return DRM_FORMAT_ARGB8888;
	}
	else if (strcmp(format_str, "RA12") == 0)
	{
		return DRM_FORMAT_RGBA4444;
	}
	else if (strcmp(format_str, "RA15") == 0)
	{
		return DRM_FORMAT_RGBA5551;
	}
	else if (strcmp(format_str, "RA24") == 0)
	{
		return DRM_FORMAT_RGBA8888;
	}
	else if (strcmp(format_str, "AB12") == 0)
	{
		return DRM_FORMAT_ABGR4444;
	}
	else if (strcmp(format_str, "AB15") == 0)
	{
		return DRM_FORMAT_ABGR1555;
	}
	else if (strcmp(format_str, "BG16") == 0)
	{
		return DRM_FORMAT_BGR565;
	}
	else if (strcmp(format_str, "BG24") == 0)
	{
		return DRM_FORMAT_BGR888;
	}
	else if (strcmp(format_str, "AB24") == 0)
	{
		return DRM_FORMAT_ABGR8888;
	}
	else if (strcmp(format_str, "BA12") == 0)
	{
		return DRM_FORMAT_BGRA4444;
	}
	else if (strcmp(format_str, "BA15") == 0)
	{
		return DRM_FORMAT_BGRA5551;
	}
	else if (strcmp(format_str, "BA24") == 0)
	{
		return DRM_FORMAT_BGRA8888;
	}
	else if (strcmp(format_str, "YUYV") == 0)
	{
		return DRM_FORMAT_YUYV;
	}
	else if (strcmp(format_str, "YVYU") == 0)
	{
		return DRM_FORMAT_YVYU;
	}
	else if (strcmp(format_str, "NV12") == 0)
	{
		return DRM_FORMAT_NV12;
	}
	else if (strcmp(format_str, "NV21") == 0)
	{
		return DRM_FORMAT_NV21;
	}
	else
	{
		return 0; // Unsupported format
	}
}

static uint32_t get_framebuffer(vp_drm_context_t *drm_ctx,
	int dma_buf_fd, int plane_index)
{
	uint32_t fb_id = find_fb_id(dma_buf_fd);
	if (fb_id != -1) {
		return fb_id;
	}

	struct drm_prime_handle prime_handle = {
		.fd = dma_buf_fd,
		.flags = 0,
		.handle = 0,
	};

	if (drmIoctl(drm_ctx->drm_fd, DRM_IOCTL_PRIME_FD_TO_HANDLE, &prime_handle) < 0)
	{
		perror("DRM_IOCTL_PRIME_FD_TO_HANDLE");
		printf("Failed to map dma_buf_fd=%d to GEM handle\n", dma_buf_fd);
		return 0;
	}

	uint32_t handles[4] = {0};
	uint32_t strides[4] = {0};
	uint32_t offsets[4] = {0};
	if (strcmp(drm_ctx->planes[plane_index].format, "NV12") == 0) {
		handles[0] = prime_handle.handle;
		strides[0] = drm_ctx->planes[plane_index].src_w;
		offsets[0] = 0;
		handles[1] = prime_handle.handle;
		strides[1] = drm_ctx->planes[plane_index].src_w;
		offsets[1] = drm_ctx->planes[plane_index].src_w * drm_ctx->planes[plane_index].src_h;
	} else if (strcmp(drm_ctx->planes[plane_index].format, "RG24") == 0 ||
				strcmp(drm_ctx->planes[plane_index].format, "BG24") == 0){
		handles[0] = prime_handle.handle;
		strides[0] = drm_ctx->planes[plane_index].src_w * 3;
		offsets[0] = 0;
	} else if (strcmp(drm_ctx->planes[plane_index].format, "AR24") == 0 ||
				strcmp(drm_ctx->planes[plane_index].format, "RA24") == 0 ||
				strcmp(drm_ctx->planes[plane_index].format, "AG24") == 0 ||
				strcmp(drm_ctx->planes[plane_index].format, "GA24") == 0 ||
				strcmp(drm_ctx->planes[plane_index].format, "BA24") == 0 ||
				strcmp(drm_ctx->planes[plane_index].format, "AB24") == 0) {
		handles[0] = prime_handle.handle;
		strides[0] = drm_ctx->planes[plane_index].src_w * 4;
		offsets[0] = 0;
	}

	uint32_t drm_format = get_format_from_string(drm_ctx->planes[plane_index].format);

	if (drmModeAddFB2(drm_ctx->drm_fd,
		drm_ctx->planes[plane_index].src_w, drm_ctx->planes[plane_index].src_h,
		drm_format, handles, strides, offsets, &fb_id, 0))
	{
		perror("drmModeAddFB2");
		return 0;
	}

	printf("Created new framebuffer: fb_id=%u for dma_buf_fd=%d\n", fb_id, dma_buf_fd);

	add_mapping(dma_buf_fd, fb_id);

	return fb_id;
}

int32_t vp_display_set_frame(vp_drm_context_t *drm_ctx, int32_t plane_idx,
	hbn_vnode_image_t *image_frame)
{
	int32_t ret = 0;
	int dma_buf_fds[DRM_MAX_PLANES] = {-1, -1, -1};
	uint32_t flags = DRM_MODE_ATOMIC_ALLOW_MODESET;

	dma_buf_fds[plane_idx] = image_frame->buffer.fd[0];

	drmModeAtomicReq *req = drmModeAtomicAlloc();
	if (!req)
	{
		perror("drmModeAtomicAlloc");
		return -1;
	}

	if (dma_buf_fds[plane_idx] == -1)
	{
		return -1;
	}

	uint32_t fb_id = get_framebuffer(drm_ctx, dma_buf_fds[plane_idx], plane_idx);
	if (fb_id == 0)
	{
		fprintf(stderr, "Failed to get framebuffer for plane %d\n", plane_idx);
		drmModeAtomicFree(req);
		return -1;
	}
	add_property(drm_ctx->drm_fd, req, drm_ctx->planes[plane_idx].plane_id,
		DRM_MODE_OBJECT_PLANE, "CRTC_ID", drm_ctx->crtc_id);
	add_property(drm_ctx->drm_fd, req, drm_ctx->planes[plane_idx].plane_id,
		DRM_MODE_OBJECT_PLANE, "FB_ID", fb_id);

	ret = drmModeAtomicCommit(drm_ctx->drm_fd, req, flags, NULL);

	if (ret < 0)
	{
		perror("drmModeAtomicCommit");
		drmModeAtomicFree(req);
		return -1;
	}

	drmModeAtomicFree(req);

	return ret;
}