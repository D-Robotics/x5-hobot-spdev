#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <xf86drm.h>
#include <xf86drmMode.h>

#include "utils_log.h"
#include "vpp_display.h"

#include "sp_display.h"

using namespace std;
using namespace spdev;

typedef struct {
    int width;
    int height;
} Resolution;

// 定义要剔除的分辨率黑名单
static const Resolution blacklist[] = {
    {1440, 900},
};
static const int blacklist_size = sizeof(blacklist) / sizeof(blacklist[0]);

void *sp_init_display_module()
{
    return new VPPDisplay();
}

void sp_release_display_module(void *obj)
{
    if (obj != NULL)
    {
        delete static_cast<VPPDisplay *>(obj);
    }
}

int32_t sp_start_display(void *obj, int32_t chn,
        int32_t width, int32_t height)
{
    if (chn != 1) return 0;
    if (obj != NULL)
        return static_cast<VPPDisplay *>(obj)->OpenDisplay(width, height);
    return -1;
}

int32_t sp_stop_display(void *obj)
{
    if (obj != NULL)
        return static_cast<VPPDisplay *>(obj)->Close();
    return -1;
}

int32_t sp_display_set_image(void *obj,
        char *addr, int32_t size, int32_t chn)
{
    ImageFrame frame = {0};

    frame.data[0] = (uint8_t *)addr;
    frame.data_size[0] = size;
    frame.plane_count = 1;

    if (obj != NULL)
        return static_cast<VPPDisplay *>(obj)->SetImageFrame(&frame);
    return -1;
}

int32_t sp_display_draw_rect(void *obj,
        int32_t x0, int32_t y0, int32_t x1, int32_t y1,
        int32_t chn, int32_t flush, int32_t color, int32_t line_width)
{
    if (obj != NULL)
        return static_cast<VPPDisplay *>(obj)->SetGraphRect(x0, y0, x1, y1, flush, color, line_width);
    return -1;
}

int32_t sp_display_draw_string(void *obj,
        int32_t x, int32_t y, char *str,
        int32_t chn, int32_t flush, int32_t color, int32_t line_width)
{
    if (obj != NULL)
        return static_cast<VPPDisplay *>(obj)->SetGraphWord(x, y, str, flush, (uint32_t)color, line_width);
    return -1;
}

static int32_t exec_cmd_ex(const char *cmd, char* res, int32_t max)
{
    if(cmd == NULL || res == NULL || max <= 0)
            return -1;

    FILE *pp = popen(cmd, "r");
    if(!pp) {
        LOGE_print("[Error] Cannot popen cmd: %s\n", cmd);
        return -1;
    }

    int32_t length;
    char tmp[1024] = {0};

    length = max;
    if(max > 1024) length = 1024;

    while(fgets(tmp, length, pp) != NULL) {
        sscanf(tmp, "%s", res);
    }

    pclose(pp);

    return strlen(res);
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

void sp_get_display_resolution(int32_t *width, int32_t *height) {
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
