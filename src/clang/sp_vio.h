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

#ifndef SP_VIO_H_
#define SP_VIO_H_

#define SP_VPS_SCALE 1
#define SP_VPS_SCALE_CROP 2
#define SP_VPS_SCALE_ROTATE 3
#define SP_VPS_SCALE_ROTATE_CROP 4
#define FRAME_BUFFER_SIZE(w, h) ((w) * (h) * (3) / (2))
#define SP_HOST_0 0
#define SP_HOST_1 1
#define SP_HOST_2 2
#define SP_HOST_3 3
#define SP_HOST_AUTO_DETECT -1
#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        int32_t raw_height;
        int32_t raw_width;
        int32_t fps;
    } sp_sensors_parameters;

    void *sp_init_vio_module();
    void sp_release_vio_module(void *obj);

    int32_t sp_open_camera(void *obj, const int32_t pipe_id,
        const int32_t video_index, int32_t chn_num,
        int32_t *width, int32_t *height);
    int32_t sp_open_camera_v2(void *obj, const int32_t pipe_id,
        const int32_t video_index, int32_t chn_num,
        sp_sensors_parameters *parameters,
        int32_t *input_width, int32_t *input_height);
    int32_t sp_open_vps(void *obj, const int32_t pipe_id, int32_t chn_num, int32_t proc_mode,
                        int32_t src_width, int32_t src_height, int32_t *dst_width, int32_t *dst_height,
                        int32_t *crop_x, int32_t *crop_y, int32_t *crop_width, int32_t *crop_height, int32_t *rotate);
    int32_t sp_vio_close(void *obj);
    int32_t sp_vio_get_frame(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout);
    int32_t sp_vio_set_frame(void *obj, void *frame_buffer, int32_t size);
    int32_t sp_vio_get_raw(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout);
    int32_t sp_vio_get_yuv(void *obj, char *frame_buffer, int32_t width, int32_t height, const int32_t timeout);

#ifdef __cplusplus
}
#endif /* End of #ifdef __cplusplus */

#endif /* SP_VIO_H_ */