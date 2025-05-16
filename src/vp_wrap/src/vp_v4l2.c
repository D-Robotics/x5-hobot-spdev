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
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>

#include "vp_v4l2.h"

#define VIDEO_DEV_CSI0_SIF "/dev/video0"
#define VIDEO_DEV_CSI0_ISP "/dev/video4"
#define VIDEO_DEV_CSI0_VSE0 "/dev/video8"
#define VIDEO_DEV_CSI0_VSE1 "/dev/video9"
#define VIDEO_DEV_CSI0_VSE2 "/dev/video10"
#define VIDEO_DEV_CSI0_VSE3 "/dev/video11"
#define VIDEO_DEV_CSI0_VSE4 "/dev/video12"
#define VIDEO_DEV_CSI0_VSE5 "/dev/video13"

#define VIDEO_DEV_CSI1_SIF "/dev/video1"
#define VIDEO_DEV_CSI1_ISP "/dev/video5"
#define VIDEO_DEV_CSI1_VSE0 "/dev/video14"
#define VIDEO_DEV_CSI1_VSE1 "/dev/video15"
#define VIDEO_DEV_CSI1_VSE2 "/dev/video16"
#define VIDEO_DEV_CSI1_VSE3 "/dev/video17"
#define VIDEO_DEV_CSI1_VSE4 "/dev/video18"
#define VIDEO_DEV_CSI1_VSE5 "/dev/video19"

#define VIDEO_DEV_CSI2_SIF "/dev/video2"
#define VIDEO_DEV_CSI2_ISP "/dev/video6"
#define VIDEO_DEV_CSI2_VSE0 "/dev/video20"
#define VIDEO_DEV_CSI2_VSE1 "/dev/video21"
#define VIDEO_DEV_CSI2_VSE2 "/dev/video22"
#define VIDEO_DEV_CSI2_VSE3 "/dev/video23"
#define VIDEO_DEV_CSI2_VSE4 "/dev/video24"
#define VIDEO_DEV_CSI2_VSE5 "/dev/video25"

#define VIDEO_DEV_CSI3_SIF "/dev/video3"
#define VIDEO_DEV_CSI3_ISP "/dev/video7"
#define VIDEO_DEV_CSI3_VSE0 "/dev/video26"
#define VIDEO_DEV_CSI3_VSE1 "/dev/video27"
#define VIDEO_DEV_CSI3_VSE2 "/dev/video28"
#define VIDEO_DEV_CSI3_VSE3 "/dev/video29"
#define VIDEO_DEV_CSI3_VSE4 "/dev/video30"
#define VIDEO_DEV_CSI3_VSE5 "/dev/video31"

#define VIDEO_DEV_VSE0 "/dev/video0"
#define VIDEO_DEV_VSE1 "/dev/video1"
#define VIDEO_DEV_VSE2 "/dev/video2"
#define VIDEO_DEV_VSE3 "/dev/video3"
#define VIDEO_DEV_VSE4 "/dev/video4"
#define VIDEO_DEV_VSE5 "/dev/video5"

#define MAX_CAMERAS 4
#define BUFFER_COUNT 3

enum V4L2_Dev {
	V4L2_DEV_SIF = 0,
	V4L2_DEV_ISP = 1,
	V4L2_DEV_VSE = 2,
};

typedef struct v4l2_info
{
	uint32_t width;
	uint32_t height;
	int32_t v4l2_fd;
    int32_t chn_num;
    int32_t stream_on;
} v4l2_info_s;

v4l2_info_s vp_v4l2_info[VSE_MAX_CHN_NUM] = {0};
v4l2_info_s vp_v4l2_info_sif = {0};
v4l2_info_s vp_v4l2_info_isp = {0};

void* v4l2_buffers_vse[VSE_MAX_CHN_NUM][BUFFER_COUNT];
void* v4l2_buffers_isp[BUFFER_COUNT];
void* v4l2_buffers_sif[BUFFER_COUNT];

int32_t read_nv12_image(v4l2_info_s *v4l2_info, hbn_vnode_image_t *input_image,char *buffer, uint32_t y_size) {
	int64_t alloc_flags = 0;
	int32_t ret = 0;

	memset(input_image, 0, sizeof(hbn_vnode_image_t));
	alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED |
				HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD |
				HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN |
				HB_MEM_USAGE_CACHED |
				HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
	ret = hb_mem_alloc_graph_buf(v4l2_info->width,
								v4l2_info->height,
								MEM_PIX_FMT_NV12,
								alloc_flags,
								v4l2_info->width,
								v4l2_info->height,
								&input_image->buffer);
    if( ret != 0 ) {
        printf("hb_mem_alloc_graph_buf failed, ret = %d\n", ret);
        return -1;
    }

    memcpy((char *)(input_image->buffer.virt_addr[0]), buffer, v4l2_info->height*v4l2_info->width);
	memcpy((char *)(input_image->buffer.virt_addr[1]), buffer + y_size/3*2, v4l2_info->height*v4l2_info->width/2);

	// 设置一个时间戳
	gettimeofday(&input_image->info.tv, NULL);

	return ret;
}

int32_t read_nv12_image_sif(v4l2_info_s *v4l2_info, hbn_vnode_image_t *input_image,char *buffer, uint32_t y_size) {
	int64_t alloc_flags = 0;
	int32_t ret = 0;

	memset(input_image, 0, sizeof(hbn_vnode_image_t));
    alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED |
				HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD |
				HB_MEM_USAGE_CPU_READ_OFTEN |
				HB_MEM_USAGE_CPU_WRITE_OFTEN |
				HB_MEM_USAGE_CACHED |
				HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
	ret = hb_mem_alloc_graph_buf(v4l2_info->width,
								v4l2_info->height,
								MEM_PIX_FMT_RAW10,
								alloc_flags,
								v4l2_info->width*2,
								v4l2_info->height,
								&input_image->buffer);
    if( ret != 0 ) {
        printf("hb_mem_alloc_graph_buf failed, ret = %d\n", ret);
        return -1;
    }

    memcpy((char *)(input_image->buffer.virt_addr[0]), buffer, v4l2_info->height*v4l2_info->width*2);

	// 设置一个时间戳
	gettimeofday(&input_image->info.tv, NULL);

	return ret;
}

int32_t vp_v4l2_stream_on(v4l2_info_s *v4l2_info) 
{
    if(v4l2_info->stream_on == 1)
        return 0;

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(v4l2_info->v4l2_fd, VIDIOC_STREAMON, &type) < 0) {
        perror("Failed to start streaming");
        close(v4l2_info->v4l2_fd);
        return -1;
    }
    v4l2_info->stream_on = 1;
    return 0;
}

int32_t vp_v4l2_stream_off(v4l2_info_s *v4l2_info) 
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(v4l2_info->v4l2_fd, VIDIOC_STREAMOFF, &type) < 0) {
        perror("Failed to start streaming");
        close(v4l2_info->v4l2_fd);
        return -1;
    }

    v4l2_info->stream_on = 0;
    return 0;
}

int32_t vp_v4l2_close(v4l2_info_s *v4l2_info) 
{
    if(v4l2_info->v4l2_fd > 0)
    {
        vp_v4l2_stream_off(v4l2_info);   
        close(v4l2_info->v4l2_fd);
        v4l2_info->width = 0;
        v4l2_info->height = 0;
        v4l2_info->v4l2_fd = -1;
        v4l2_info->stream_on = 0;
        return 0;
    }
    return -1;
}

int32_t vp_v4l2_open(v4l2_info_s *v4l2_info, char *camera_dev, int dev_type) 
{
    struct v4l2_capability caps;
    struct v4l2_fmtdesc fmtdesc;
    struct v4l2_frmsizeenum frmsizeenum;
    struct v4l2_format fmt = {};
    uint32_t pixelformat = 0;
    int i, j, rc;

    v4l2_info->v4l2_fd = open(camera_dev, O_RDWR);
    if ( v4l2_info->v4l2_fd < 0) {
        printf("Failed to open video device %s\n",camera_dev);
        return -1;
    }

    if (ioctl(v4l2_info->v4l2_fd, VIDIOC_QUERYCAP, &caps) < 0) {
        perror("Failed to request buffers");
        close(v4l2_info->v4l2_fd);
        return -1;
    }

    memset(&fmtdesc, 0, sizeof(fmtdesc));
    memset(&frmsizeenum, 0, sizeof(frmsizeenum));
    // 枚举像素格式
    printf("Supported Pixel Formats:\n");
    for (i = 0; i < 20; i++) {
        memset(&fmtdesc, 0, sizeof(fmtdesc));
        fmtdesc.index = i;
        fmtdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;  // 视频捕获类型
        rc = ioctl(v4l2_info->v4l2_fd, VIDIOC_ENUM_FMT, &fmtdesc);
        if (rc < 0) {
            if (errno == EINVAL) {
                break;  // 如果没有更多格式，退出循环
            }
            perror("VIDIOC_ENUM_FMT");
            break;
        }

        printf("%2d: %s (0x%08x)\n", i, fmtdesc.description, fmtdesc.pixelformat);
        pixelformat = fmtdesc.pixelformat;

        // 枚举分辨率
        printf("  Supported Resolutions:\n");
        j = 0;
        while (j < 20) {
            memset(&frmsizeenum, 0, sizeof(frmsizeenum));
            frmsizeenum.index = j;
            frmsizeenum.pixel_format = fmtdesc.pixelformat;
            rc = ioctl(v4l2_info->v4l2_fd, VIDIOC_ENUM_FRAMESIZES, &frmsizeenum);
            if (rc < 0) {
                if (errno == EINVAL) {
                    break;
                }
                perror("VIDIOC_ENUM_FRAMESIZES");
                break;
            }

            // 打印支持的分辨率
            if (frmsizeenum.type == V4L2_FRMSIZE_TYPE_DISCRETE) {
                printf("    %d x %d\n", frmsizeenum.discrete.width, frmsizeenum.discrete.height);
            } else if (frmsizeenum.type == V4L2_FRMSIZE_TYPE_STEPWISE) {
                printf("    Width range: %d - %d, Height range: %d - %d\n",
                       frmsizeenum.stepwise.min_width, frmsizeenum.stepwise.max_width,
                       frmsizeenum.stepwise.min_height, frmsizeenum.stepwise.max_height);
            }
            j++;
        }
    }

    if (!pixelformat) {
        printf("video=/dev/video%d, Unsupported pixel format!\n", v4l2_info->v4l2_fd);
        return -1;
    }

    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = v4l2_info->width;
    fmt.fmt.pix.height = v4l2_info->height;
    fmt.fmt.pix.pixelformat = pixelformat;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;
    if (ioctl(v4l2_info->v4l2_fd, VIDIOC_S_FMT, &fmt) < 0) {
        perror("Failed to set format");
        close(v4l2_info->v4l2_fd);
        return -1;
    }

    struct v4l2_requestbuffers req = {};
    req.count = BUFFER_COUNT;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(v4l2_info->v4l2_fd, VIDIOC_REQBUFS, &req) < 0) {
        perror("Failed to request buffers");
        close(v4l2_info->v4l2_fd);
        return -1;
    }

    for (int i = 0; i < BUFFER_COUNT; i++) {
        struct v4l2_buffer buf = {};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(v4l2_info->v4l2_fd, VIDIOC_QUERYBUF, &buf) < 0) {
            perror("Failed to query buffer");
            close(v4l2_info->v4l2_fd);
            return -1;
        }

        if(dev_type == V4L2_DEV_SIF)
        {
            v4l2_buffers_sif[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, v4l2_info->v4l2_fd, buf.m.offset);
            if (v4l2_buffers_sif[i] == MAP_FAILED) {
                perror("Failed to mmap buffer");
                close(v4l2_info->v4l2_fd);
                return -1;
            }
        }
        else if(dev_type == V4L2_DEV_ISP)
        {
            v4l2_buffers_isp[i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, v4l2_info->v4l2_fd, buf.m.offset);
            if (v4l2_buffers_isp[i] == MAP_FAILED) {
                perror("Failed to mmap buffer");
                close(v4l2_info->v4l2_fd);
                return -1;
            }
        }
        else if(dev_type == V4L2_DEV_VSE)
        {
            v4l2_buffers_vse[v4l2_info->chn_num][i] = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, v4l2_info->v4l2_fd, buf.m.offset);
            if (v4l2_buffers_vse[v4l2_info->chn_num][i] == MAP_FAILED) {
                perror("Failed to mmap buffer");
                close(v4l2_info->v4l2_fd);
                return -1;
            }
            
        }
        else
        {
            perror("Invalid dev_type");
            close(v4l2_info->v4l2_fd);
            return -1; 
        }    
        if (ioctl(v4l2_info->v4l2_fd, VIDIOC_QBUF, &buf) < 0) {
            perror("Failed to queue buffer");
            close(v4l2_info->v4l2_fd);
            return -1;
        }
    }
    vp_v4l2_stream_on(v4l2_info);
    return 0;
}

int32_t vp_v4l2_get_vse_frame(int32_t chn_num, hbn_vnode_image_t *image_frame)
{
	int32_t ret = 0;
	struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (chn_num < 0) {
        printf("Failed to vp_v4l2_get_vse_frame %d\n",chn_num);
        return -1;
    }

    //V4L2模式下 取流VSE需要先关闭ISP
    vp_v4l2_close(&vp_v4l2_info_isp);

    vp_v4l2_stream_on(&vp_v4l2_info[chn_num]);

    if (ioctl(vp_v4l2_info[chn_num].v4l2_fd, VIDIOC_DQBUF, &buf) < 0) {
        perror("Failed to dequeue buffer");
        return -1;
    }

	ret = read_nv12_image(&vp_v4l2_info[chn_num], image_frame, v4l2_buffers_vse[chn_num][buf.index], buf.bytesused);

    if (ioctl(vp_v4l2_info[chn_num].v4l2_fd, VIDIOC_QBUF, &buf) < 0) {
        perror("Failed to queue buffer");
        return -1;
    }

    return 0;
}

int32_t vp_v4l2_get_isp_frame(hbn_vnode_image_t *image_frame)
{
	int32_t ret = 0;
	struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    vp_v4l2_stream_on(&vp_v4l2_info_isp);

    if (ioctl(vp_v4l2_info_isp.v4l2_fd, VIDIOC_DQBUF, &buf) < 0) {
        perror("Failed to dequeue buffer");
        return -1;
    }
    
	ret = read_nv12_image(&vp_v4l2_info_isp, image_frame, v4l2_buffers_isp[buf.index], buf.bytesused);

    if (ioctl(vp_v4l2_info_isp.v4l2_fd, VIDIOC_QBUF, &buf) < 0) {
        perror("Failed to queue buffer");
        return -1;
    }

    return 0;
}

int32_t vp_v4l2_get_sif_frame(hbn_vnode_image_t *image_frame)
{
	int32_t ret = 0;
	struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    vp_v4l2_stream_on(&vp_v4l2_info_sif);

    if (ioctl(vp_v4l2_info_sif.v4l2_fd, VIDIOC_DQBUF, &buf) < 0) {
        perror("Failed to dequeue buffer");
        return -1;
    }

	ret = read_nv12_image_sif(&vp_v4l2_info_sif, image_frame, v4l2_buffers_sif[buf.index], buf.bytesused);

    if (ioctl(vp_v4l2_info_sif.v4l2_fd, VIDIOC_QBUF, &buf) < 0) {
        perror("Failed to queue buffer");
        return -1;
    }

    return 0;
}

int32_t vp_v4l2_release_frame(hbn_vnode_image_t *image_frame)
{
	int32_t ret = 0;
    if(image_frame->buffer.fd[0]!=-1)
        hb_mem_free_buf(image_frame->buffer.fd[0]);
    if(image_frame->buffer.fd[1]!=-1)
        hb_mem_free_buf(image_frame->buffer.fd[1]);

	return ret;
}

int32_t vp_v4l2_vse_select(int *chn_en, int src_width, int src_height,
    int dst_width, int dst_height)
{
    if (((dst_width <= src_width) && (dst_height <= src_height)) &&
        (!(*chn_en & 1 << VP_VSE_DS0)) &&
        (dst_width <= 4096 && dst_height <= 3076)) {
        return VP_VSE_DS0;
    }
    if ((dst_width <= src_width) || (dst_height <= src_height)) {
        if ((dst_width <= 1920 && dst_height <= 1080) &&
            (!(*chn_en & 1 << VP_VSE_DS1))) {
            return VP_VSE_DS1;
        } else if ((dst_width <= 1920 && dst_height <= 1080) &&
                (!(*chn_en & 1 << VP_VSE_DS2))) {
            return VP_VSE_DS2;
        } else if ((dst_width <= 1280 && dst_height <= 720) &&
                (!(*chn_en & 1 << VP_VSE_DS3))) {
            return VP_VSE_DS3;
        } else if ((dst_width <= 1280 && dst_height <= 720) &&
                (!(*chn_en & 1 << VP_VSE_DS4))) {
            return VP_VSE_DS4;
        }
    }
    if (((dst_width >= src_width) && (dst_height >= src_height)) &&
            (!(*chn_en & 1 << VP_VSE_DS5)) &&
            (dst_width <= 4096 && dst_height <= 3076)) {
        return VP_VSE_DS5;
    }

    return -1;
}

char* get_video_device(int video_index, int vse_chn, int dev_type) {
    switch (video_index) {
         case 0:
            if(dev_type == V4L2_DEV_SIF)
                return VIDEO_DEV_CSI0_SIF;
            else if(dev_type == V4L2_DEV_ISP)
                return VIDEO_DEV_CSI0_ISP;
            else if(dev_type == V4L2_DEV_VSE)
            {
                switch (vse_chn) {
                    case 0: return VIDEO_DEV_CSI0_VSE0;
                    case 1: return VIDEO_DEV_CSI0_VSE1;
                    case 2: return VIDEO_DEV_CSI0_VSE2;
                    case 3: return VIDEO_DEV_CSI0_VSE3;
                    case 4: return VIDEO_DEV_CSI0_VSE4;
                    case 5: return VIDEO_DEV_CSI0_VSE5;
                    default: return NULL;
                }
            }
            break;
        case 1:
            if(dev_type == V4L2_DEV_SIF)
                return VIDEO_DEV_CSI1_SIF;
            else if(dev_type == V4L2_DEV_ISP)
                return VIDEO_DEV_CSI1_ISP;
            else if(dev_type == V4L2_DEV_VSE)
            {
                switch (vse_chn) {
                    case 0: return VIDEO_DEV_CSI1_VSE0;
                    case 1: return VIDEO_DEV_CSI1_VSE1;
                    case 2: return VIDEO_DEV_CSI1_VSE2;
                    case 3: return VIDEO_DEV_CSI1_VSE3;
                    case 4: return VIDEO_DEV_CSI1_VSE4;
                    case 5: return VIDEO_DEV_CSI1_VSE5;
                    default: return NULL;
                }
            }
            break;
        case 2:
            if(dev_type == V4L2_DEV_SIF)
                return VIDEO_DEV_CSI2_SIF;
            else if(dev_type == V4L2_DEV_ISP)
                return VIDEO_DEV_CSI2_ISP;
            else if(dev_type == V4L2_DEV_VSE)
            {
                switch (vse_chn) {
                    case 0: return VIDEO_DEV_CSI2_VSE0;
                case 1: return VIDEO_DEV_CSI2_VSE1;
                case 2: return VIDEO_DEV_CSI2_VSE2;
                case 3: return VIDEO_DEV_CSI2_VSE3;
                case 4: return VIDEO_DEV_CSI2_VSE4;
                case 5: return VIDEO_DEV_CSI2_VSE5;
                default: return NULL;
                }
            }
            break;
         case 3:
            if(dev_type == V4L2_DEV_SIF)
                return VIDEO_DEV_CSI3_SIF;
            else if(dev_type == V4L2_DEV_ISP)
                return VIDEO_DEV_CSI3_ISP;
            else if(dev_type == V4L2_DEV_VSE)
            {  
                switch (vse_chn) {
                    case 0: return VIDEO_DEV_CSI3_VSE0;
                    case 1: return VIDEO_DEV_CSI3_VSE1;
                    case 2: return VIDEO_DEV_CSI3_VSE2;
                    case 3: return VIDEO_DEV_CSI3_VSE3;
                    case 4: return VIDEO_DEV_CSI3_VSE4;
                    case 5: return VIDEO_DEV_CSI3_VSE5;
                    default: return NULL;
                }
            }
            break;
        default: return NULL;
    }
    return NULL;
}

int32_t vp_v4l2_found_chn(int32_t width, int32_t height)
{
    int i;

    for (i = 0; i < VSE_MAX_CHN_NUM; i++)
    {
        if ((vp_v4l2_info[i].v4l2_fd > 0)
				&& (vp_v4l2_info[i].width == width)
				&& (vp_v4l2_info[i].height == height))
            return vp_v4l2_info[i].chn_num;
    }
    return -1;
}

int32_t vp_v4l2_init(const int32_t video_index, int32_t src_width, int32_t src_height, 
    int32_t chn_num, int32_t *dst_width, int32_t *dst_height)
{
    int vse_chn;
    int vse_chn_en = 0;
    int ret = 0;

    char *v4l2_camera_name = NULL;
    int i;

    printf("video_index %d src_width %d src_height %d\n", video_index, src_width, src_height);
    //SIF
    v4l2_camera_name = get_video_device(video_index, 0, V4L2_DEV_SIF);
    printf("SIF v4l2_camera_name %s\n", v4l2_camera_name);
    if (v4l2_camera_name == NULL) {
        perror("get_video_device_sif");
        return -1;
    }
    vp_v4l2_info_sif.width = src_width;
    vp_v4l2_info_sif.height = src_height;
    if (vp_v4l2_open(&vp_v4l2_info_sif, v4l2_camera_name, V4L2_DEV_SIF) < 0) {
        perror("Failed vp_v4l2_open");
        return -1;
    }
    printf("SIF v4l2_fd %d\n", vp_v4l2_info_sif.v4l2_fd);

    //ISP
    v4l2_camera_name = get_video_device(video_index, 0, V4L2_DEV_ISP);
    printf("ISP v4l2_camera_name %s\n", v4l2_camera_name);
    if (v4l2_camera_name == NULL) {
        perror("get_video_device");
        return -1;
    }
    vp_v4l2_info_isp.width = src_width;
    vp_v4l2_info_isp.height = src_height;
    if (vp_v4l2_open(&vp_v4l2_info_isp, v4l2_camera_name, V4L2_DEV_ISP) < 0) {
        perror("Failed vp_v4l2_open");
        return -1;
    }
    printf("ISP v4l2_fd %d\n", vp_v4l2_info_isp.v4l2_fd);

    //VSE
    for (i = 0; i < chn_num; i++) {
        if ((dst_width[i] == 0) && (dst_height[i] == 0)) {//如果高宽为0，那么就开一个和原始高宽一致的通道
            dst_width[i] = src_width;
            dst_height[i] = src_height;
        }
        vse_chn = vp_v4l2_vse_select(&vse_chn_en, src_width, src_height, dst_width[i], dst_height[i]);
        if (vse_chn < 0) {
            printf("Invalid vp_v4l2_vse_select chn_num %d vse_chn %d dst_width %d  dst_height %d\n", i, vse_chn, dst_width[i], dst_height[i]);
            return -1;
        }
        vse_chn_en |= 1 << vse_chn;
        v4l2_camera_name = get_video_device(video_index, vse_chn, V4L2_DEV_VSE);
        if (v4l2_camera_name == NULL) {
            printf("Invalid VSE channel for video index %d chn_num %d vse_chn:%d\n", video_index, chn_num, vse_chn);
            return -1;
        }
        vp_v4l2_info[i].width = dst_width[i];
        vp_v4l2_info[i].height = dst_height[i];
        vp_v4l2_info[i].chn_num = i;
        if (vp_v4l2_open(&vp_v4l2_info[i],v4l2_camera_name, V4L2_DEV_VSE) < 0) {
            perror("Failed vp_v4l2_open");
            return -1;
        }

        printf("chn_num %d vse_chn %d src_width %d src_height %d dst_width %d  dst_height %d v4l2_camera_name %s v4l2_fd %d\n", 
            i, vse_chn, src_width, src_height, dst_width[i], dst_height[i], v4l2_camera_name,vp_v4l2_info[i].v4l2_fd);   
    }

    ret = hb_mem_module_open();
    if( ret != 0 ) {
        printf("hb_mem_module_open failed, ret = %d\n", ret);
        return -1;
    }

    return 0;
}

int32_t vp_v4l2_deinit()
{
    int i;

    vp_v4l2_close(&vp_v4l2_info_sif);
    vp_v4l2_close(&vp_v4l2_info_isp);

    for (i = 0; i < VSE_MAX_CHN_NUM; i++)
    {
        vp_v4l2_close(&vp_v4l2_info[i]);
    }

    hb_mem_module_close();
    return 0;      
}