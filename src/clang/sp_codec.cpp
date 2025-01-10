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
 * @Date: 2023-03-05 16:58:38
 * @LastEditTime: 2023-03-05 17:01:08
 ***************************************************************************/
#include <thread>
#include <sys/stat.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>

#include "vpp_codec.h"

#include "sp_codec.h"

using namespace std;

using namespace spdev;
void *sp_init_encoder_module()
{
    return new VPPEncode();
}

void sp_release_encoder_module(void *obj)
{
    if (obj != NULL)
    {
        delete static_cast<VPPEncode *>(obj);
        obj = NULL;
    }
}
int32_t sp_start_encode(void *obj, int32_t chn, int32_t type,
              int32_t width, int32_t height, int32_t bits)
{
    if (obj != NULL)
    {
        auto encoder_obj = static_cast<VPPEncode *>(obj);
        return encoder_obj->OpenEncode(type, width, height, bits);
    }
    return -1;
}

int sp_stop_encode(void *obj)
{
    if (obj != NULL)
    {
        auto encoder_obj = static_cast<VPPEncode *>(obj);
        return encoder_obj->Close();
    }
    return -1;
}

int sp_encoder_set_frame(void *obj, char *frame_buffer, int32_t size)
{
    if (obj != NULL)
    {
        auto encoder_obj = static_cast<VPPEncode *>(obj);
        ImageFrame frame_obj = {0};

        frame_obj.data[0] = (uint8_t *)frame_buffer;
        frame_obj.data_size[0] = size;
        frame_obj.plane_count = 1;
        return encoder_obj->SetImageFrame(&frame_obj);
    }
    return -1;
}

int sp_encoder_get_stream(void *obj, char *stream_buffer)
{
    if (obj != NULL)
    {
        auto encoder_obj = static_cast<VPPEncode *>(obj);
        ImageFrame frame_obj = {0};
        if (encoder_obj->GetImageFrame(&frame_obj) == 0)
        {
            //printf("get frame success!,data addr:0x%x,size:%d\n", frame_obj->data[0], frame_obj->data_size[0]);
            int32_t frame_size = frame_obj.data_size[0];
            memcpy(stream_buffer, frame_obj.data[0], frame_size);
            encoder_obj->ReturnImageFrame(&frame_obj);
            return frame_size;
        }
    }
    return -1;
}

// decoder
void *sp_init_decoder_module()
{
    return new VPPDecode();
}

void sp_release_decoder_module(void *decoder_object)
{
    if (decoder_object != NULL)
    {
        delete static_cast<VPPDecode *>(decoder_object);
        decoder_object = NULL;
    }
}

int32_t sp_start_decode(void *decoder_object, const char *stream_file, int32_t video_chn,
    int32_t type, int32_t width, int32_t height)
{
    int32_t frame_cnt = 0;
    if (decoder_object != NULL)
    {
        if (strlen(stream_file) > 0) {
            ;;
        }
        auto decoder_obj = static_cast<VPPDecode *>(decoder_object);
        return decoder_obj->OpenDecode(type, width, height, stream_file, &frame_cnt);
    }
    return -1;
}

int sp_decoder_get_image(void *decoder_object, char *image_buffer)
{
    size_t offset = 0;

    if (decoder_object != NULL && image_buffer != NULL)
    {
        auto decoder_obj = static_cast<VPPDecode *>(decoder_object);
        ImageFrame frame = {0};
        if (decoder_obj->GetImageFrame(&frame) == 0)
        {
            // vp_normal_buf_info_print(&frame);
            // vp_codec_print_media_codec_output_buffer_info(&frame);

            for (int i = 0; i < frame.plane_count; ++i) {
                memcpy(image_buffer + offset, frame.data[i], frame.data_size[i]);
                offset += frame.data_size[i];
            }

            decoder_obj->ReturnImageFrame(&frame);
            return 0;
        }
    }
    return -1;
}

int32_t sp_decoder_set_image(void *decoder_object, char *image_buffer,
              int32_t chn, int32_t size, int32_t eos)
{
    if (decoder_object != NULL && image_buffer != NULL)
    {
        auto decoder_obj = static_cast<VPPDecode *>(decoder_object);
        ImageFrame frame = {0};
        frame.data[0] = (uint8_t *)image_buffer;
        frame.data_size[0] = size;
        return decoder_obj->SetImageFrame(&frame, eos);
    }
    return 0;
}

int sp_stop_decode(void *decoder_object)
{
    if (decoder_object != NULL)
    {
        auto decoder_obj = static_cast<VPPDecode *>(decoder_object);
        return decoder_obj->Close();
    }
    return -1;
}
