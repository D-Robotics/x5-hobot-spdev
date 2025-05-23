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
 * @LastEditTime: 2023-03-05 14:15:22
 ***************************************************************************/
#ifndef VP_COMMONH_
#define VP_COMMONH_

#include <stdint.h>

#include "hb_camera_interface.h"
#include "hbn_api.h"
#include "hb_rgn.h"
#include "vse_cfg.h"
#include "hb_media_codec.h"
#include "hb_media_error.h"
#include "hbn_error.h"
#include "vp_sensors.h"

#define VP_MAX_PATH_LENGTH 128
#define VP_MAX_FILES_NUM 128

#define VP_MAX_JSON_DEPTH 10
#define VP_MAX_FIELD_LENGTH 20
#define VP_MAX_CMD_LENGTH 128
#define VP_MAX_RESULT_LENGTH 1024
#define VP_MAX_SENSOR_NAME 10

#define VP_MAX_PIPELINE_NUM 8
#define VP_PIPELINE_MASK 0xffffff00
#define VP_CODEC_MASK 0x00000000
#define VPP_DISPLAY_MASK 0xfffffffe

#define VP_GET_FRAME_TIMEOUT 2000
#define VP_DECODER_GET_FRAME_TIMEOUT 4000

#define VP_MAX_OSD_REGION (4)

#define VSE_DS_CHN_NUM 5
#define VSE_US_CHN_NUM 1
typedef enum VSE_CHANNEL_S {
	VP_VSE_DS0 = 0,  // 4K Downscale
	VP_VSE_DS1,  // 1080P Downscale
	VP_VSE_DS2,  // 1080P Downscale
	VP_VSE_DS3,  // 720P Downscale
	VP_VSE_DS4,  // 720P Downscale
	VP_VSE_DS5,  // 4K Upscale
} VSE_CHANNEL_E;

#define VSE_MAX_CHN_NUM (VSE_DS_CHN_NUM + VSE_US_CHN_NUM)

/**
 * Align by 16
 */
#define ALIGN_16(v) ((v + (16 - 1)) / 16 * 16)

#define VP_GET_MD_CODEC_TYPE(v) \
	({ \
		int32_t _v = (v); \
		( \
			_v == 1 ? MEDIA_CODEC_ID_H264 : \
			_v == 2 ? MEDIA_CODEC_ID_H265 : \
			_v == 3 ? MEDIA_CODEC_ID_MJPEG : \
			MEDIA_CODEC_ID_NONE \
		); \
	})

typedef struct {
	int32_t width;
	int32_t height;
	int32_t stride;
	int32_t vstride;

	int64_t frame_id;
	int64_t lost_image_num;
	int64_t exp_time;
	int64_t image_timestamp;

	int32_t plane_count;
	uint8_t *data[3];
	uint64_t pdata[3];
	uint32_t data_size[3];

	// media_codec_buffer_t
	media_codec_buffer_t frame_buffer;
	// media_codec_output_buffer_info_t
	media_codec_output_buffer_info_t buffer_info;
	// hbn_vnode_image_t
	hbn_vnode_image_t vnode_image;
} ImageFrame;

typedef struct vse_info_s {
	int vse_ochn_buffer_count;
	vse_attr_t vse_attr;
	vse_ichn_attr_t vse_ichn_attr;
	vse_ochn_attr_t vse_ochn_attr[6];
} vse_config_t;

typedef struct isp_info_s {
	int ochn_buffer_count;
} isp_user_config_t;

typedef struct vin_info_s {
	int ochn_buffer_count;
} vin_user_config_t;

typedef struct osd_position_s{
	int x;
	int y;
	int width;
	int height;
}osd_position_t;
typedef struct osd_info_s{
	int valid_osd_region_count;
	osd_position_t position[VP_MAX_OSD_REGION];
	hbn_rgn_handle_t handle [VP_MAX_OSD_REGION];
	hbn_rgn_bitmap_t bitmap [VP_MAX_OSD_REGION];
} osd_user_info_t;

enum GDC_STATUS{
    GDC_STATUS_INVALID = -1,
    GDC_STATUS_CLOSE = 0,
    GDC_STATUS_OPEN = 1,
};

typedef struct{
	enum GDC_STATUS status; //0: 没有gdc file， 1： 关闭 gdc, 2： 打开gdc
	char sensor_name[64];
	int bin_buf_is_valid;
	hb_mem_common_buf_t bin_buf;
	hbn_vnode_handle_t gdc_fd;
	int input_width;
	int input_height;

	int output_buffer_count;

}gdc_user_info_t;

typedef struct
{
	int32_t raw_height;
	int32_t raw_width;
	int32_t fps;
} vp_sensors_parameters;

typedef struct vp_vflow_contex_s {
	//vflow
	hbn_vflow_handle_t vflow_fd;

	//sensor
	camera_handle_t cam_fd;
	int32_t mipi_csi_rx_index;
	int32_t mclk_is_not_configed;
	vp_sensor_config_t *sensor_config;

	//vin
	vin_user_config_t vin_info;
	hbn_vnode_handle_t vin_node_handle;

	//isp
	isp_user_config_t isp_info;
	hbn_vnode_handle_t isp_node_handle;

	//vse
	vse_config_t vse_config;
	hbn_vnode_handle_t vse_node_handle;

	//gdc
	gdc_user_info_t gdc_info;
	hbn_vnode_handle_t gdc_node_handle;

	//osd
	osd_user_info_t osd_info;
} vp_vflow_contex_t;

#endif // VP_COMMONH_
