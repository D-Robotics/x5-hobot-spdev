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
 * @Date: 2023-03-02 09:50:40
 * @LastEditTime: 2023-03-02 09:50:41
 ***************************************************************************/
#ifndef SP_CODEC_H_
#define SP_CODEC_H_
#define SP_ENCODER_H264  1
#define SP_ENCODER_H265  2
#define SP_ENCODER_MJPEG 3
#ifdef __cplusplus
extern "C"
{
#endif
       // encoder
       void *sp_init_encoder_module();
       void sp_release_encoder_module(void *obj);
       int32_t sp_start_encode(void *obj, int32_t chn, int32_t type,
              int32_t width, int32_t height, int32_t bits);
       int32_t sp_stop_encode(void *obj);
       int32_t sp_encoder_set_frame(void *obj, char *frame_buffer, int32_t size);
       int32_t sp_encoder_get_stream(void *obj, char *stream_buffer);

       // decoder
       void *sp_init_decoder_module();
       void sp_release_decoder_module(void *obj);
       int32_t sp_start_decode(void *obj, const char *stream_file, int32_t video_chn,
              int32_t type, int32_t width, int32_t height);
       int32_t sp_decoder_get_image(void *obj, char *image_buffer);
       int32_t sp_decoder_set_image(void *obj, char *image_buffer,
              int32_t chn, int32_t size, int32_t eos);
       int32_t sp_stop_decode(void *obj);
#ifdef __cplusplus
}
#endif /* End of #ifdef __cplusplus */

#endif /* SP_CODEC_H_ */