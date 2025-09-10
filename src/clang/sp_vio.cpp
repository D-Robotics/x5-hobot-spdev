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

/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-03-05 16:55:13
 * @LastEditTime: 2023-03-05 16:56:44
 ***************************************************************************/
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>

#include "vpp_camera.h"
#include "sp_vio.h"

using namespace spdev;

void *sp_init_vio_module()
{
    return new VPPCamera();
}

void sp_release_vio_module(void *obj)
{
    if (obj != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        delete sp; // safe
    }
}

int32_t sp_open_camera(void *obj, const int32_t pipe_id,
    const int32_t video_index, int32_t chn_num,
    int32_t *input_width, int32_t *input_height)
{
    if (obj != NULL)
    {
        vp_sensors_parameters parameters;

        auto sp = static_cast<VPPCamera *>(obj);
        int32_t width[VSE_MAX_CHN_NUM] = {0};
        int32_t height[VSE_MAX_CHN_NUM] = {0};
        memcpy(width, input_width, sizeof(int) * chn_num);
        memcpy(height, input_height, sizeof(int) * chn_num);
        if (chn_num == 0)
        {
            // set 0 means default size
            width[chn_num] = 0;
            height[chn_num] = 0;
            chn_num++;
        }

        memset(&parameters, 0, sizeof(vp_sensors_parameters));
        parameters.raw_width = 1920;
        parameters.raw_height = 1080;
        parameters.fps =-1;

        return sp->OpenCamera(pipe_id, video_index, chn_num, width, height, &parameters);
    }
    return -1;
}


int32_t sp_open_camera_v2(void *obj, const int32_t pipe_id,
    const int32_t video_index, int32_t chn_num,
    sp_sensors_parameters *parameters,
    int32_t *input_width, int32_t *input_height)
{
    if (obj != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        int32_t width[VSE_MAX_CHN_NUM] = {0};
        int32_t height[VSE_MAX_CHN_NUM] = {0};
        memcpy(width, input_width, sizeof(int) * chn_num);
        memcpy(height, input_height, sizeof(int) * chn_num);
        if (chn_num < (VSE_MAX_CHN_NUM - 1))
        {
            // set 0 means default size
            width[chn_num] = 0;
            height[chn_num] = 0;
            chn_num++;
        }
        vp_sensors_parameters *_parameters = (vp_sensors_parameters *)parameters;
        return sp->OpenCamera(pipe_id, video_index,
            chn_num, width, height, _parameters);
    }
    return -1;
}

int32_t sp_open_vps(void *obj, const int32_t pipe_id, int32_t chn_num,
    int32_t proc_mode, int32_t src_width, int32_t src_height,
    int32_t *dst_width, int32_t *dst_height,
    int32_t *crop_x, int32_t *crop_y,
    int32_t *crop_width, int32_t *crop_height, int32_t *rotate)
{
    if (obj != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        return sp->OpenVPS(pipe_id, chn_num, proc_mode,
            src_width, src_height, dst_width,
            dst_height, crop_x, crop_y, crop_width, crop_height, rotate);
    }
    return -1;
}

int32_t sp_vio_close(void *obj)
{
    if (obj != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        return sp->Close();
    }
    return -1;
}

int32_t sp_vio_get_frame(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout)
{
    if (obj != NULL && frame_buffer != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        auto module_enum = static_cast<DevModule>(SP_DEV_VPS);
        ImageFrame temp_ptr = {0};
        if (!sp->GetImageFrame(&temp_ptr, module_enum, width, height, timeout))
        {
            memcpy(frame_buffer, temp_ptr.data[0], temp_ptr.data_size[0]);
            if (temp_ptr.plane_count == 2)
                memcpy(frame_buffer + temp_ptr.data_size[0], temp_ptr.data[1], temp_ptr.data_size[1]);
            sp->ReturnImageFrame(&temp_ptr, module_enum, width, height); // delete temp_ptr.frame_info
            return 0;
        }
    }
    return -1;
}

int32_t sp_vio_get_raw(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout)
{
    if (obj != NULL && frame_buffer != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        auto module_enum = static_cast<DevModule>(SP_DEV_RAW);
        ImageFrame temp_ptr = {0};
        if (!sp->GetImageFrame(&temp_ptr, module_enum, width, height, timeout))
        {
            printf("temp_ptr.data_size[0]:%d\n", temp_ptr.data_size[0]);
            memcpy(frame_buffer, temp_ptr.data[0], temp_ptr.data_size[0]);
            if (temp_ptr.plane_count > 1)
            {
                printf("temp_ptr.data_size[1]:%d\n", temp_ptr.data_size[1]);
                memcpy(frame_buffer + temp_ptr.data_size[0], temp_ptr.data[1], temp_ptr.data_size[1]);
            }
            sp->ReturnImageFrame(&temp_ptr, module_enum, width, height); // delete temp_ptr.frame_info
            return 0;
        }
    }
    return -1;
}

int32_t sp_vio_get_yuv(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout)
{
    if (obj != NULL && frame_buffer != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        auto module_enum = static_cast<DevModule>(SP_DEV_ISP);
        ImageFrame temp_ptr = {0};
        if (!sp->GetImageFrame(&temp_ptr, module_enum, width, height, timeout))
        {
            memcpy(frame_buffer, temp_ptr.data[0], temp_ptr.data_size[0]);
            if (temp_ptr.plane_count > 1)
                memcpy(frame_buffer + temp_ptr.data_size[0], temp_ptr.data[1], temp_ptr.data_size[1]);
            sp->ReturnImageFrame(&temp_ptr, module_enum, width, height); // delete temp_ptr.frame_info
            return 0;
        }
    }
    return -1;
}

int32_t sp_vio_set_frame(void *obj, void *frame_buffer, int32_t size)
{
    if (obj != NULL && frame_buffer != NULL)
    {
        auto sp = static_cast<VPPCamera *>(obj);
        ImageFrame temp_image = {0};
        temp_image.width = sp->GetModuleWidth();
        temp_image.height = sp->GetModuleHeight();
        temp_image.stride = sp->GetModuleWidth();

        temp_image.data[0] = static_cast<uint8_t *>(frame_buffer);
        temp_image.data_size[0] = temp_image.stride * temp_image.height;
        temp_image.data[1] =static_cast<uint8_t *>(frame_buffer) + temp_image.data_size[0];
        temp_image.data_size[1] = temp_image.data_size[0] / 2;
        temp_image.plane_count = 2;

        return sp->SetImageFrame(&temp_image);
    }
    return -1;
}
