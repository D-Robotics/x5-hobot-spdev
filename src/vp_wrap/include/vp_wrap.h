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
 * @Date: 2023-03-05 15:28:19
 * @LastEditTime: 2023-03-05 16:31:05
 ***************************************************************************/
#ifndef VP_WRAP_H_
#define VP_WRAP_H_

#include <stddef.h>

#include "vp_common.h"
#include "vp_vflow.h"
#include "vp_vin.h"
#include "vp_isp.h"
#include "vp_vse.h"
#include "vp_osd.h"

#ifdef __cplusplus
extern "C" {
#endif

void vp_print_debug_infos(void);

void vp_normal_buf_info_print(ImageFrame *frame);
void vp_print_debug_infos_when_error(void);

int32_t vp_dump_nv12_to_file(char *filename, uint8_t *data_y, uint8_t *data_uv,
		int width, int height);
int32_t vp_dump_1plane_image_to_file(char *filename, uint8_t *src_buffer, uint32_t size);
int32_t vp_dump_yuv_to_file(char *filename, uint8_t *src_buffer, uint32_t size);

int32_t vp_dump_2plane_yuv_to_file(char *filename, uint8_t *src_buffer, uint8_t *src_buffer1,
		uint32_t size, uint32_t size1);

void vp_vin_print_hbn_vnode_image_t(const hbn_vnode_image_t *frame);
void fill_image_frame_from_vnode_image(ImageFrame *frame);
void fill_vnode_image_from_image_frame(ImageFrame *frame);

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif // VP_WRAP_H_
