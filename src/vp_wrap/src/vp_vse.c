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
 * @Date: 2023-02-23 14:01:59
 * @LastEditTime: 2023-03-05 15:57:48
 ***************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "utils/utils_log.h"

#include "vp_wrap.h"
#include "vp_vse.h"
#define VSE_MAX_CHANNLE 6 //max is 6

int32_t vp_vse_init(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0, i = 0;
	uint32_t hw_id = 0;
	uint32_t ichn_id = 0;
	hbn_buf_alloc_attr_t alloc_attr = {0};
	hbn_vnode_handle_t *vse_node_handle = NULL;
	vse_config_t *vse_config = NULL;

	vse_node_handle = &vp_vflow_contex->vse_node_handle;
	vse_config = &vp_vflow_contex->vse_config;

	ret = hbn_vnode_open(HB_VSE, hw_id, AUTO_ALLOC_ID, vse_node_handle);
	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_open");

	ret = hbn_vnode_set_attr(*vse_node_handle, &vse_config->vse_attr);
	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_attr");

	ret = hbn_vnode_set_ichn_attr(*vse_node_handle, ichn_id, &vse_config->vse_ichn_attr);
	SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ichn_attr");

	if(vse_config->vse_ochn_buffer_count > 0){
		alloc_attr.buffers_num = vse_config->vse_ochn_buffer_count;
	}else{
		alloc_attr.buffers_num = 3;
	}

	alloc_attr.is_contig = 1;
	alloc_attr.flags = HB_MEM_USAGE_CPU_READ_OFTEN
						| HB_MEM_USAGE_CPU_WRITE_OFTEN
						| HB_MEM_USAGE_CACHED
						| HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;

	for (i = 0; i < VSE_MAX_CHANNLE; i++) {
		if (vse_config->vse_ochn_attr[i].chn_en) {
			ret = hbn_vnode_set_ochn_attr(*vse_node_handle, i, &vse_config->vse_ochn_attr[i]);
			SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ochn_attr");
			ret = hbn_vnode_set_ochn_buf_attr(*vse_node_handle, i, &alloc_attr);
			SC_ERR_CON_EQ(ret, 0, "hbn_vnode_set_ochn_buf_attr");
		}
	}

	SC_LOGD("successful");
	return 0;
}

int32_t vp_vse_deinit(vp_vflow_contex_t *vp_vflow_contex)
{
	hbn_vnode_close(vp_vflow_contex->vse_node_handle);
	SC_LOGD("successful");
	return 0;
}

int32_t vp_vse_start(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0;

	SC_LOGD("successful");
	return ret;
}

int32_t vp_vse_stop(vp_vflow_contex_t *vp_vflow_contex)
{
	int32_t ret = 0;

	SC_LOGD("successful");
	return ret;
}

int32_t vp_vse_send_frame(vp_vflow_contex_t *vp_vflow_contex, hbn_vnode_image_t *image_frame)
{
	int32_t ret = 0;
	ret = hbn_vnode_sendframe(vp_vflow_contex->vse_node_handle, 0, image_frame);
	if (ret != 0) {
		SC_LOGE("hbn_vnode_sendframe failed(%d)", ret);
		return -1;
	}
	return ret;
}

int32_t vp_vse_get_frame(vp_vflow_contex_t *vp_vflow_contex,
	int32_t ochn_id, hbn_vnode_image_t *image_frame)
{
	int32_t ret = 0;
	hbn_vnode_handle_t vse_node_handle = vp_vflow_contex->vse_node_handle;

	ret = hbn_vnode_getframe(vse_node_handle, ochn_id, VP_GET_FRAME_TIMEOUT,
		image_frame);
	return ret;
}

int32_t vp_vse_release_frame(vp_vflow_contex_t *vp_vflow_contex,
	int32_t ochn_id, hbn_vnode_image_t *image_frame)
{
	int32_t ret = 0;
	hbn_vnode_handle_t vse_node_handle = vp_vflow_contex->vse_node_handle;

	ret = hbn_vnode_releaseframe(vse_node_handle, ochn_id, image_frame);
	return ret;
}

int32_t vp_vse_get_output_info(vp_vflow_contex_t *vp_vflow_contex,
	int32_t ochn_id, vp_vse_output_info_t *vse_out_info)
{
	int ret = 0;
	vse_ochn_attr_t vse_ochn_attr = {0};
	hbn_vnode_handle_t vse_node_handle = vp_vflow_contex->vse_node_handle;
	ret = hbn_vnode_get_ochn_attr(vse_node_handle, ochn_id, &vse_ochn_attr);
	if(ret == 0){
		vse_out_info->fps = vse_ochn_attr.fps.dst;
		vse_out_info->width = vse_ochn_attr.target_w;
		vse_out_info->height = vse_ochn_attr.target_h;
	}
	return ret;
}