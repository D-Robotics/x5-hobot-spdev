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
 * @Date: 2023-02-20 17:14:01
 * @LastEditTime: 2023-03-05 16:37:47
 ***************************************************************************/
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>
#include <cJSON.h>

#include "utils_log.h"

#include "vp_vin.h"
#include "vp_isp.h"
#include "vp_vse.h"
#include "vp_wrap.h"
#include "vp_v4l2.h"

#include "vpp_camera.h"

#define VPP_VSE_OUTBUFFER_COUNT 3

namespace spdev
{
	static cJSON *open_json_file(const char *path)
	{
		FILE *fp = fopen(path, "r");
		int32_t ret = 0;
		if (fp == NULL)
		{
			perror("fopen");
			return NULL;
		}

		fseek(fp, 0, SEEK_END);
		long fsize = ftell(fp);
		fseek(fp, 0, SEEK_SET);

		char *buf = (char *)malloc(fsize + 1);
		ret = fread(buf, fsize, 1, fp);
		if (ret != 1)
		{
			SC_LOGE("Error fread size:%d\n", ret);
		}
		fclose(fp);

		buf[fsize] = '\0';

		cJSON *root = cJSON_Parse(buf);
		if (root == NULL)
		{
			const char *error_ptr = cJSON_GetErrorPtr();
			if (error_ptr != NULL)
			{
				SC_LOGE("Error cJSON_Parse: %s\n", error_ptr);
			}
			free(buf);
			return NULL;
		}
		free(buf);

		return root;
	}

	static int parse_cameras(cJSON *cameras, board_camera_info_t camera_info[])
	{
		cJSON *camera = NULL;
		int ret;

		for (int i = 0; i < MAX_CAMERAS; i++) {
			camera_info[i].enable = 0;

			camera = cJSON_GetArrayItem(cameras, i);
			if (camera == NULL) {
				break;
			}

			// parse i2c_bus
			cJSON *i2c_bus_obj = cJSON_GetObjectItem(camera, "i2c_bus");
			if (i2c_bus_obj == NULL) {
				ret = -1;
				goto exit;
			}
			camera_info[i].i2c_bus = i2c_bus_obj->valueint;

			// parse mipi_host
			cJSON *mipi_host_obj = cJSON_GetObjectItem(camera, "mipi_host");
			if (mipi_host_obj == NULL) {
				ret = -1;
				goto exit;
			}
			camera_info[i].mipi_host = mipi_host_obj->valueint;
			camera_info[i].enable = 1;
		}
		ret = 0;

	exit:
		return ret;
	}

	static int parse_config(const char *json_file, board_camera_info_t camera_info[])
	{
		int ret = 0;
		cJSON *root = NULL;
		cJSON *board_config = NULL;
		cJSON *cameras = NULL;
		char board_id[16];
		char board_name[32];

		FILE *fp = fopen("/sys/class/socinfo/board_id", "r");
		if (fp == NULL) {
			return -1 ;
		}
		fscanf(fp, "%s", board_id);
		fclose(fp);

		snprintf(board_name, sizeof(board_name), "board_%s", board_id);

		root = open_json_file(json_file);
		if (!root) {
			fprintf(stderr, "Failed to parse JSON string.\n");
			return -1 ;
		}

		board_config = cJSON_GetObjectItem(root, board_name);
		if (!board_config) {
			fprintf(stderr, "Failed to get board_config object.\n");
			ret = -1;
			goto exit;
		}

		cameras = cJSON_GetObjectItem(board_config, "cameras");
		if (!cameras) {
			fprintf(stderr, "Failed to get cameras array.\n");
			ret = -1;
			goto exit;
		}

		ret = parse_cameras(cameras, camera_info);

	exit:
		cJSON_Delete(root);
		return ret;
	}
	int32_t VPPCamera::SelectVseChn(int *chn_en, int src_width, int src_height,
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

	int32_t VPPCamera::CamInitParam(vp_vflow_contex_t *vp_vflow_contex,
					const int pipe_id, const int video_index,
					int chn_num, int *width, int *height,
					vp_sensors_parameters *parameters)
	{
		int i = 0;
		int ret = 0;
		int sensor_width = -1, sensor_height = -1, sensor_fps = -1;
		int vse_chn = 0, vse_chn_en = 0;
		camera_config_t *camera_config = NULL;
		isp_ichn_attr_t *isp_ichn_attr = NULL;
		vse_config_t *vse_config = NULL;
		vp_csi_config_t csi_config = {0};
		int32_t input_width = 0, input_height = 0;
		board_camera_info_t camera_info[MAX_CAMERAS];

		if(parameters != NULL)
		{
			sensor_fps = parameters->fps;
			sensor_width = parameters->raw_width;
			sensor_height = parameters->raw_height;
		}
		memset(vp_vflow_contex, 0, sizeof(vp_vflow_contex_t));
		printf("set camera fps: %d,width: %d,height: %d\n",sensor_fps,sensor_width,sensor_height);
		memset(camera_info, 0, sizeof(camera_info));
		ret = parse_config("/etc/board_config.json", camera_info);
		if (ret != 0) {
			printf("Failed to parse cameras\n");
			return -1;
		}

		for (int i = 0; i < MAX_CAMERAS; i++) {
			printf("Camera %d:\n", i);
			// printf("\tenable: %d\n", camera_info[i].enable);
			printf("\ti2c_bus: %d\n", camera_info[i].i2c_bus);
			printf("\tmipi_host: %d\n", camera_info[i].mipi_host);
		}

		// 1. Detect Sensor
		// 如果video_index 为 -1 ，那么遍历板子上的camera接口，否则使用指定的接口号
		// RDK X5 有两个 CAM 接口，分别对应 mipi host0 和 mipi host2,
		// EVB 1_B 最大支持4路 CAM 接口，有 mipi host0-3，
		// 读取 /etc/board_config.json 文件中关于camera接口的配置匹配不同的接口
		// 每个sensor会支持不同的分辨率，需要根据传入的 parameters 选择 sensor 配置
		if (video_index >= 0 && video_index < MAX_CAMERAS) {
			vp_vflow_contex->mipi_csi_rx_index = camera_info[video_index].mipi_host;
			vp_vflow_contex->sensor_config =
				vp_get_sensor_config_by_mipi_host(camera_info[video_index].mipi_host, &csi_config,sensor_height,sensor_width,sensor_fps);
		} else if (video_index == -1) {
			for (int i = 0; i < MAX_CAMERAS; i++) {
				if (!camera_info[i].enable)
					continue;
				vp_vflow_contex->mipi_csi_rx_index = camera_info[i].mipi_host;
				vp_vflow_contex->sensor_config =
					vp_get_sensor_config_by_mipi_host(camera_info[i].mipi_host, &csi_config,sensor_height,sensor_width,sensor_fps);
				if (vp_vflow_contex->sensor_config != NULL)
					break;
			}
		} else {
			SC_LOGE("The parameter video_idx=%d is not supported. Please set it to one of [-1, 0, 1, 2, 3].");
			return -1;
		}

		if (vp_vflow_contex->sensor_config == NULL) {
			SC_LOGE("No camera sensor found, "
				"please check whether the camera connection or video_idx is correct.\n");
			return -1;
		}

		// 2. Setting Vse channel
		vse_config = &vp_vflow_contex->vse_config;
		isp_ichn_attr = vp_vflow_contex->sensor_config->isp_ichn_attr;
		vp_vflow_contex->mclk_is_not_configed = csi_config.mclk_is_not_configed;

		input_width = isp_ichn_attr->width;
		input_height = isp_ichn_attr->height;
		SC_LOGD("VSE: input_width: %d input_height: %d",
			input_width, input_height);

		vse_config->vse_ichn_attr.width = input_width;
		vse_config->vse_ichn_attr.height = input_height;
		vse_config->vse_ichn_attr.fmt = FRM_FMT_NV12;
		vse_config->vse_ichn_attr.bit_width = 8;

		vse_config->vse_ochn_buffer_count = VPP_VSE_OUTBUFFER_COUNT;

		// 设置VSE通道0输出属性
		for (i = 0; i < chn_num; i++) {
			if ((width[i] == 0) && (height[i] == 0)) {//如果高宽为0，那么就开一个和原始高宽一致的通道
				width[i] = input_width;
				height[i] = input_height;
			}
			vse_chn = SelectVseChn(&vse_chn_en, input_width, input_height, width[i], height[i]);
			if (vse_chn < 0) {
				SC_LOGE("Invalid size:%dx%d\n", width[i], height[i]);
				return -1;
			}
			vse_config->vse_ochn_attr[vse_chn].chn_en = CAM_TRUE;
			vse_config->vse_ochn_attr[vse_chn].roi.x = 0;
			vse_config->vse_ochn_attr[vse_chn].roi.y = 0;
			vse_config->vse_ochn_attr[vse_chn].roi.w = input_width;
			vse_config->vse_ochn_attr[vse_chn].roi.h = input_height;
			vse_config->vse_ochn_attr[vse_chn].target_w = width[i];
			vse_config->vse_ochn_attr[vse_chn].target_h = height[i];
			vse_config->vse_ochn_attr[vse_chn].fmt = FRM_FMT_NV12;
			vse_config->vse_ochn_attr[vse_chn].bit_width = 8;
			vse_chn_en |= 1 << vse_chn;
			SC_LOGI("Setting VSE channel-%d: input_width:%d, input_height:%d, dst_w:%d, dst_h:%d",
				vse_chn,
				input_width, input_height, width[i], height[i]);
		}

		m_width = input_width;
		m_height = input_height;

		return ret;
	}

	int32_t VPPCamera::CamInitVseParam(vp_vflow_contex_t *vp_vflow_contex,
		const int pipe_id, int chn_num, int proc_mode,
		int src_width, int src_height, int *dst_width, int *dst_height,
		int *crop_x, int *crop_y, int *crop_width, int *crop_height, int *rotate)
	{
		int32_t i = 0;
		int32_t ret = 0;
		int vse_chn = 0, vse_chn_en = 0;
		vse_config_t *vse_config = NULL;

		memset(vp_vflow_contex, 0, sizeof(vp_vflow_contex_t));

		vse_config = &vp_vflow_contex->vse_config;

		vse_config->vse_ichn_attr.width = src_width;
		vse_config->vse_ichn_attr.height = src_height;
		vse_config->vse_ichn_attr.fmt = FRM_FMT_NV12;
		vse_config->vse_ichn_attr.bit_width = 8;

		SC_LOGD("VSE: input_width: %d input_height: %d",
			src_width, src_height);

		// 设置VSE通道0输出属性
		for (i = 0; i < chn_num; i++) {
			switch (proc_mode)
			{
			case VPS_SCALE_ROTATE_CROP:
			case VPS_SCALE_CROP:
				if ((crop_width[i] == 0) || (crop_height[i] == 0)) {
					crop_width[i] = src_width;
					crop_height[i] = src_height;
				}
			case VPS_SCALE_ROTATE:
			case VPS_SCALE:
				if ((dst_width[i] == 0) && (dst_height[i] == 0)) {
					if ((crop_width[i] != 0) && (crop_height[i] != 0)) {
						dst_width[i] = crop_width[i];
						dst_height[i] = crop_height[i];
					} else {
						dst_width[i] = src_width;
						dst_height[i] = src_height;
					}
				}
				break;
			default:
				break;
			}

			vse_chn = SelectVseChn(&vse_chn_en, src_width, src_height, dst_width[i], dst_height[i]);
			if (vse_chn < 0) {
				SC_LOGE("Invalid size:%dx%d\n", dst_width[i], dst_height[i]);
				return -1;
			}
			if (proc_mode >= VPS_SCALE) {
				vse_config->vse_ochn_attr[vse_chn].chn_en = CAM_TRUE;
				vse_config->vse_ochn_attr[vse_chn].roi.x = 0;
				vse_config->vse_ochn_attr[vse_chn].roi.y = 0;
				vse_config->vse_ochn_attr[vse_chn].roi.w = src_width;
				vse_config->vse_ochn_attr[vse_chn].roi.h = src_height;
				vse_config->vse_ochn_attr[vse_chn].target_w = dst_width[i];
				vse_config->vse_ochn_attr[vse_chn].target_h = dst_height[i];
				vse_config->vse_ochn_attr[vse_chn].fmt = FRM_FMT_NV12;
				vse_config->vse_ochn_attr[vse_chn].bit_width = 8;
			} else if ((proc_mode == VPS_SCALE_CROP)) {
				vse_config->vse_ochn_attr[vse_chn].chn_en = CAM_TRUE;
				vse_config->vse_ochn_attr[vse_chn].roi.x = crop_x[i];
				vse_config->vse_ochn_attr[vse_chn].roi.y = crop_y[i];
				vse_config->vse_ochn_attr[vse_chn].roi.w = crop_width[i];
				vse_config->vse_ochn_attr[vse_chn].roi.h = crop_height[i];
				vse_config->vse_ochn_attr[vse_chn].target_w = dst_width[i];
				vse_config->vse_ochn_attr[vse_chn].target_h = dst_height[i];
				vse_config->vse_ochn_attr[vse_chn].fmt = FRM_FMT_NV12;
				vse_config->vse_ochn_attr[vse_chn].bit_width = 8;
			}
			if ((proc_mode == VPS_SCALE_ROTATE)
				|| (proc_mode == VPS_SCALE_ROTATE_CROP)
				|| (proc_mode == VPS_SCALE_ROTATE_CROP)) {
				SC_LOGE("VSE does not support rotating images");
				return -1;
			}
			vse_chn_en |= 1 << vse_chn;
			SC_LOGI("Setting VSE channel-%d: input_width:%d, input_height:%d, dst_w:%d, dst_h:%d",
				vse_chn,
				src_width, src_height, dst_width[i], dst_height[i]);
		}

		return ret;
	}

	bool V4l2Loaded() {
		std::ifstream modules("/proc/modules");
		std::string line;
		while (std::getline(modules, line)) {
			if (line.find("vs_sif_v4l") != std::string::npos) {
				return true;
			}
		}
		return false;
	}

	int32_t VPPCamera::OpenCamera(const int pipe_id,
								  const int32_t video_index,
								  int32_t chn_num,
								  int32_t *width, int32_t *height,
								  vp_sensors_parameters *parameters)
	{
		int32_t ret = 0;

		if (V4l2Loaded()) {
			useV4l2 = true;
			SC_LOGI("v4l2 module");
		} else {
			useV4l2 = false;
			SC_LOGI("hbn module");
		}

		if(useV4l2)
		{
			int mipi_host = 0;
			board_camera_info_t camera_info[MAX_CAMERAS];

			memset(camera_info, 0, sizeof(camera_info));
			ret = parse_config("/etc/board_config.json", camera_info);
			if (ret != 0) {
				printf("Failed to parse cameras\n");
				return -1;
			}

			for (int i = 0; i < MAX_CAMERAS; i++) {
				printf("Camera %d:\n", i);
				printf("\tenable: %d\n", camera_info[i].enable);
				printf("\ti2c_bus: %d\n", camera_info[i].i2c_bus);
				printf("\tmipi_host: %d\n", camera_info[i].mipi_host);
			}

			if(video_index == -1)
			{
				mipi_host = 0;
			}
			else if(video_index > 0 && video_index < MAX_CAMERAS)
			{
				if(camera_info[video_index].enable == 0)
				{
					SC_LOGE("video_index error(%d)");
					return -1;
				}
				mipi_host = camera_info[video_index].mipi_host;
			}

			ret = vp_v4l2_init(mipi_host, parameters->raw_width, parameters->raw_height, chn_num, width, height);
			if (ret != 0){
				SC_LOGE("vp_v4l2_init failed error(%d)", ret);
				return -1;
			}
		}
		else
		{
			vp_vflow_contex_t *vp_vflow_contex = &m_vp_vflow_context;
			// TODO: 根据 video_index 和 chn_num 完成sensor的探测、初始化、开流
			ret = CamInitParam(vp_vflow_contex, pipe_id, video_index, chn_num,
				width, height, parameters);
			if (ret != 0){
				SC_LOGE("CamInitParam failed error(%d)", ret);
				return -1;
			}

			vp_vflow_contex->vin_info.ochn_buffer_count = 3;
			vp_vflow_contex->isp_info.ochn_buffer_count = 3;
			vp_vflow_contex->gdc_info.output_buffer_count = 3;

			ret = vp_vin_init(vp_vflow_contex);
			ret |= vp_isp_init(vp_vflow_contex);
			ret |= vp_vse_init(vp_vflow_contex);
			ret |= vp_vflow_init(vp_vflow_contex);
			if (ret != 0){
				SC_LOGE("pipeline init failed error(%d)", ret);
				return -1;
			}

			ret = vp_vin_start(vp_vflow_contex);
			ret |= vp_isp_start(vp_vflow_contex);
			ret |= vp_vse_start(vp_vflow_contex);
			ret |= vp_vflow_start(vp_vflow_contex);
			if (ret != 0){
				SC_LOGE("pipeline start failed error(%d)", ret);
				return -1;
			}

			vp_print_debug_infos();
		}
		return ret;
	}

	int32_t VPPCamera::OpenVSE(const int32_t pipe_id, int32_t chn_num, int32_t proc_mode,
		int32_t src_width, int32_t src_height, int32_t *dst_width, int32_t *dst_height,
		int32_t *crop_x, int32_t *crop_y, int32_t *crop_width, int32_t *crop_height, int32_t *rotate)
	{
		int32_t ret = 0;
		int64_t alloc_flags = 0;
		vp_vflow_contex_t *vp_vflow_contex = &m_vp_vflow_context;

		m_only_vse = true;

		ret = CamInitVseParam(vp_vflow_contex, pipe_id, chn_num, proc_mode,
			src_width, src_height,
			dst_width, dst_height, crop_x, crop_y, crop_width, crop_height,
			rotate);
		if (ret != 0){
			SC_LOGE("CamInitParam failed error(%d)", ret);
			return -1;
		}

		m_width = src_width;
		m_height = src_height;

		ret = vp_vse_init(vp_vflow_contex);
		ret |= vp_vflow_init(vp_vflow_contex);
		if (ret != 0){
			SC_LOGE("pipeline init failed error(%d)", ret);
			return -1;
		}

		ret = vp_vse_start(vp_vflow_contex);
		ret |= vp_vflow_start(vp_vflow_contex);
		if (ret != 0){
			SC_LOGE("pipeline start failed error(%d)", ret);
			return -1;
		}

		memset(&m_vse_input_image, 0, sizeof(hbn_vnode_image_t));
		alloc_flags = HB_MEM_USAGE_MAP_INITIALIZED |
					HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD |
					HB_MEM_USAGE_CPU_READ_OFTEN |
					HB_MEM_USAGE_CPU_WRITE_OFTEN |
					HB_MEM_USAGE_CACHED |
					HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;
		ret = hb_mem_alloc_graph_buf(m_width, m_height,
									MEM_PIX_FMT_NV12,
									alloc_flags,
									m_width, m_height,
									&m_vse_input_image.buffer);
		if (ret != 0){
			SC_LOGE("hb_mem_alloc_graph_buf failed error(%d)", ret);
			return -1;
		}

		vp_print_debug_infos();
		return ret;
	}

	int32_t VPPCamera::OpenVPS(const int32_t pipe_id, int32_t chn_num, int32_t proc_mode,
			int32_t src_width, int32_t src_height, int32_t *dst_width, int32_t *dst_height,
			int32_t *crop_x, int32_t *crop_y, int32_t *crop_width, int32_t *crop_height, int32_t *rotate)
	{
		return OpenVSE(pipe_id, chn_num, proc_mode, src_width, src_height,
			dst_width, dst_height, crop_x, crop_y, crop_width, crop_height,
			rotate);
	}

	int32_t VPPCamera::Close(void)
	{
		int32_t ret = 0;

		if(useV4l2)
		{
			ret = vp_v4l2_deinit();
			if (ret != 0){
				SC_LOGE("vp_v4l2_deinit failed error(%d)", ret);
				return -1;
			}
		}
		else
		{
			vp_vflow_contex_t *vp_vflow_contex = &m_vp_vflow_context;
			ret = vp_vflow_stop(vp_vflow_contex);
			if (m_only_vse == false) {
				ret |= vp_vin_stop(vp_vflow_contex);
				ret |= vp_isp_stop(vp_vflow_contex);
			}
			ret |= vp_vse_stop(vp_vflow_contex);
			if (ret != 0){
				SC_LOGE("pipeline stop failed error(%d)", ret);
				return -1;
			}
			ret = vp_vflow_deinit(vp_vflow_contex);
			ret |= vp_vse_deinit(vp_vflow_contex);
			if (m_only_vse == false) {
				ret |= vp_isp_deinit(vp_vflow_contex);
				ret |= vp_vin_deinit(vp_vflow_contex);
			}
			if (ret != 0){
				SC_LOGE("pipeline deinit failed error(%d)", ret);
				return -1;
			}
		}

		return 0;
	}

	int32_t VPPCamera::GetImageFrame(ImageFrame *frame, int32_t chn, const int32_t timeout)
	{
		int32_t ret = 0;

		if(useV4l2)
		{
			ret = vp_v4l2_get_vse_frame(chn, &frame->vnode_image);
			if (ret != 0){
				SC_LOGE("vp_v4l2_get_vse_frame failed error(%d)", ret);
				return -1;
			}
		}
		else
		{
			ret = vp_vse_get_frame(&m_vp_vflow_context, chn, &frame->vnode_image);
			if (ret)
			{
				return -1;
			}
		}

		fill_image_frame_from_vnode_image(frame);

		return ret;
	}

	// 对一路的三个数据处理模块取数据
	int32_t VPPCamera::GetImageFrame(ImageFrame *frame, DevModule module,
									 int32_t width, int32_t height, const int32_t timeout)
	{
		int32_t ret = 0;
		int32_t chn = 0;

		switch (module)
		{
		case SP_DEV_RAW:
			if(useV4l2)
				ret = vp_v4l2_get_sif_frame(&frame->vnode_image);
			else
				ret = vp_vin_get_frame(&m_vp_vflow_context, &(frame->vnode_image));
			break;
		case SP_DEV_ISP:
			if(useV4l2)
				ret = vp_v4l2_get_isp_frame(&frame->vnode_image);
			else
				ret = vp_isp_get_frame(&m_vp_vflow_context, &frame->vnode_image);
			break;
		case SP_DEV_VSE:
			chn = GetChnId(width, height);
			if (chn >= 0) {
				if(useV4l2)
					ret = vp_v4l2_get_vse_frame(chn, &frame->vnode_image);
				else
					ret = vp_vse_get_frame(&m_vp_vflow_context, chn, &frame->vnode_image);
			} else {
				SC_LOGE("get chn from %dx%d failed", width, height);
				return -1;
			}
			break;
		default:
			SC_LOGE("Error: module not supported!\n");
			return -1;
		}

		fill_image_frame_from_vnode_image(frame);
		// 计算丢帧数和更新上次帧id
		frame->lost_image_num = frame->frame_id - m_last_frame_id - 1;
		m_last_frame_id = frame->frame_id;
		return ret;
	}

	void VPPCamera::ReturnImageFrame(ImageFrame *frame, int32_t chn)
	{
		if(useV4l2)
		{
			vp_v4l2_release_frame(&frame->vnode_image);
		}
		else
			vp_vse_release_frame(&m_vp_vflow_context, chn, &frame->vnode_image);
		return;
	}

	void VPPCamera::ReturnImageFrame(ImageFrame *frame, DevModule module,
									 int32_t width, int32_t height)
	{
		int32_t chn = 0;
		if(useV4l2)
		{
			vp_v4l2_release_frame(&frame->vnode_image);
		}
		else
		{
			switch (module)
			{
			case SP_DEV_RAW:
				vp_vin_release_frame(&m_vp_vflow_context, &frame->vnode_image);
				break;
			case SP_DEV_ISP:
				vp_isp_release_frame(&m_vp_vflow_context, &frame->vnode_image);
				break;
			case SP_DEV_VSE:
				chn = GetChnId(width, height);
				if (chn >= 0) {
					vp_vse_release_frame(&m_vp_vflow_context, chn, &frame->vnode_image);
				} else {
					SC_LOGE("get chn from %dx%d failed", width, height);
					return;
				}
				break;
			default:
				SC_LOGE("Error: module not supported!\n");
			}
		}
	}

	// 对一路的三个数据处理模块传入数据
	int32_t VPPCamera::SetImageFrame(ImageFrame *frame)
	{
		// fill_vnode_image_from_image_frame(frame);
		// vp_normal_buf_info_print(frame);

		for (int i = 0; i < frame->plane_count; ++i) {
			memcpy(m_vse_input_image.buffer.virt_addr[i],
				frame->data[i], frame->data_size[i]);
		}

		return vp_vse_send_frame(&m_vp_vflow_context, &m_vse_input_image);
	}

	int32_t VPPCamera::GetChnId(int32_t width, int32_t height)
	{
		if ((width == 0) || (height == 0))
		{
			width = GetModuleWidth();
			height = GetModuleHeight();
		}
		if(useV4l2)
		{
			return vp_v4l2_found_chn(width, height);
		}
		else
		{
			// VSE 有多个输出通道，根据用户输入的分辨率，找到对应的通道号
			vse_config_t *vse_config = &m_vp_vflow_context.vse_config;
			for (int32_t i = 0; i < VSE_MAX_CHN_NUM; i++)
			{

				if ((vse_config->vse_ochn_attr[i].chn_en == CAM_TRUE)
					&& (vse_config->vse_ochn_attr[i].target_w == width)
					&& (vse_config->vse_ochn_attr[i].target_h == height))
				{
					return i;
				}
			}
			return -1;
		}
	}

	int32_t VPPCamera::GetChnIdForBind(int32_t width, int32_t height)
	{
		if ((width == 0) || (height == 0))
		{
			width = GetModuleWidth();
			height = GetModuleHeight();
		}

		if(useV4l2)
		{
			return vp_v4l2_found_chn(width, height);
		}
		else
		{
			// VSE 有多个输出通道，根据用户输入的分辨率，找到对应的通道号
			vse_config_t *vse_config = &m_vp_vflow_context.vse_config;
			for (int32_t i = 0; i < VSE_MAX_CHN_NUM; i++)
			{

				if ((vse_config->vse_ochn_attr[i].chn_en == CAM_TRUE)
					&& (vse_config->vse_ochn_attr[i].target_w == width)
					&& (vse_config->vse_ochn_attr[i].target_h == height))
				{
					return i;
				}
			}
			return -1;
		}
	}

	void VPPCamera::PutChnIdForUnBind(int32_t chn_id)
	{
		return;
	}

} // namespace spdev
