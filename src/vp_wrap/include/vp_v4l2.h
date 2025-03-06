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
 * @Copyright 2024 D-Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-01-30 11:27:41
 * @LastEditTime: 2023-03-05 15:57:35
 ***************************************************************************/
#ifndef VP_V4L2_H_
#define VP_V4L2_H_

#include "vp_common.h"

#ifdef __cplusplus
extern "C" {
#endif

int32_t vp_v4l2_init(const int32_t video_index, int32_t src_width, int32_t src_height, int32_t chn_num, int32_t *dst_width, int32_t *dst_height);
int32_t vp_v4l2_deinit();
int32_t vp_v4l2_get_vse_frame(int32_t chn_num, hbn_vnode_image_t *image_frame);
int32_t vp_v4l2_get_isp_frame(hbn_vnode_image_t *image_frame);
int32_t vp_v4l2_get_sif_frame(hbn_vnode_image_t *image_frame);
int32_t vp_v4l2_release_frame(hbn_vnode_image_t *image_frame);
int32_t vp_v4l2_found_chn(int32_t width, int32_t height);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // VP_VIN_H_