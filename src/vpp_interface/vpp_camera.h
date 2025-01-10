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
 * @Date: 2023-01-30 11:27:41
 * @LastEditTime: 2023-03-05 16:27:14
 ***************************************************************************/
#ifndef __VPP_CAMERA_H__
#define __VPP_CAMERA_H__

#include <sstream>
#include <string>

#include "vp_wrap.h"
#include "vp_sensors.h"
#include "vpp_module.h"

namespace spdev
{
enum DevModule {
	SP_DEV_RAW = 0,
	SP_DEV_ISP = 1,
	SP_DEV_VPS = 2,
	SP_DEV_VSE = 2, // Same as SP_DEV_VPS
};

enum VPS_PROCESS_MODE {
    VPS_SCALE = 1,
    VPS_SCALE_CROP = 2,
    VPS_SCALE_ROTATE = 3,
    VPS_SCALE_ROTATE_CROP = 4,
};

#define MAX_CAMERAS 4
typedef struct {
    int enable;
    int i2c_bus;
    int mipi_host;
} board_camera_info_t;

class VPPCamera :public VPPModule
{
  public:
	VPPCamera() {
	  SetModuleType(VPP_CAMERA);
	  SetModuleTypeString((char *)"Camera");
	};
	virtual ~VPPCamera() = default;

	/**
	 * @brief 指定原始分辨率并打开摄像头，适用于有多种分辨率的摄像头
	 *
	 * @param pipe_id           视频pipeline id
	 * @param video_index       相机节点
	 * @param chn_num           VPS通道数量
	 * @param parameters        raw图相关参数 参见 x3_sensors_parameters 声明部分
	 * @param width             VPS宽数组
	 * @param height            VPS高数组
	 * @return int
	 */
	int32_t OpenCamera(const int pipe_id, const int video_index = -1,
			int chn_num = 0, int *width = NULL, int *height = NULL,
		  vp_sensors_parameters* param = NULL);

		/**
	 * @brief 开启VSE
	 * @param [in] pipe_id       视频pipeline id
	 * @param [in] proc_mode     vse 处理模式
	 * @param [in] src_width     输入宽
	 * @param [in] src_height    输入高
	 * @param [in] dst_width     输出宽
	 * @param [in] dst_height    输出高
	 * @param [in] crop_x        裁剪坐标x
	 * @param [in] crop_y        裁剪坐标y
	 * @param [in] crop_width    裁剪宽
	 * @param [in] crop_height   裁剪高
	 * @param [in] rotate      旋转角度
	 *
	 * @retval 0      成功
	 * @retval -1     失败
	 */
	int32_t OpenVSE(const int32_t pipe_id, int32_t chn_num, int32_t proc_mode,
			int32_t src_width, int32_t src_height, int32_t *dst_width, int32_t *dst_height,
			int32_t *crop_x, int32_t *crop_y, int32_t *crop_width, int32_t *crop_height, int32_t *rotate);

	/**
	 * @brief 开启VPS
	 * @param [in] pipe_id       视频pipeline id
	 * @param [in] proc_mode     vps 处理模式
	 * @param [in] src_width     输入宽
	 * @param [in] src_height    输入高
	 * @param [in] dst_width     输出宽
	 * @param [in] dst_height    输出高
	 * @param [in] crop_x        裁剪坐标x
	 * @param [in] crop_y        裁剪坐标y
	 * @param [in] crop_width    裁剪宽
	 * @param [in] crop_height   裁剪高
	 * @param [in] rotate      旋转角度
	 *
	 * @retval 0      成功
	 * @retval -1     失败
	 */
	int32_t OpenVPS(const int32_t pipe_id, int32_t chn_num, int32_t proc_mode,
			int32_t src_width, int32_t src_height, int32_t *dst_width, int32_t *dst_height,
			int32_t *crop_x, int32_t *crop_y, int32_t *crop_width, int32_t *crop_height, int32_t *rotate);

	/**
	 * @brief 关闭相机
	 * @param void
	 *
	 * @retval 0      成功
	 * @retval -1      失败
	 */
	int32_t Close(void);

	/**
	 * @brief 获取图像数据（dqbuf）
	 * @param [out] frame  用于保存图像数据的内存地址
	 *
	 * @retval 0      成功
	 * @retval -1      失败
	 */
	int32_t GetImageFrame(ImageFrame *frame, int32_t chn, const int32_t timeout = VP_GET_FRAME_TIMEOUT);

	/**
	 * @brief 获取图像数据（dqbuf）
	 * @param [out] frame    用于保存YUV图像数据的内存地址
	 * @param [in] module        从哪个模块取图：0:SIF 1:ISP: 2: IPU CHN(ds2 default)
	 *
	 * @retval 0        成功
	 * @retval -1     失败
	 */
	int32_t GetImageFrame(ImageFrame *frame, DevModule module,
			int32_t width, int32_t height, const int32_t timeout = VP_GET_FRAME_TIMEOUT);

	/**
	 * @brief 设置图像数据（dqbuf）
	 * @param [in] frame    用于保存YUV图像数据的内存地址
	 * @param [in] module        从哪个模块取图：0:SIF 1:ISP: 2: IPU CHN(ds2 default)
	 *
	 * @retval 0        成功
	 * @retval -1     失败
	 */
	int32_t SetImageFrame(ImageFrame *frame);

	/**
	 * @brief 释放图像数据（qbuf）
	 * @param [in] frame   保存图像数据的内存地址
	 * @param [out] format       保存图像数据的格式， 0：YUV 1：RAW
	 *
	 * @retval 0      成功
	 * @retval -1      失败
	 */
	void ReturnImageFrame(ImageFrame *frame, int32_t chn = 0);

	/**
	 * @brief 释放图像数据（qbuf）
	 * @param [in] frame   保存图像数据的内存地址
	 * @param [out] format       保存图像数据的格式， 0：YUV 1：RAW
	 *
	 * @retval 0      成功
	 * @retval -1      失败
	 */
	void ReturnImageFrame(ImageFrame *frame, DevModule module,
			  int32_t width, int32_t height);

	/**
	 * @brief 获取chn index
	 * @param [in] chn   获取chn_index的chn id
	 *
	 * @retval 非-1      成功
	 * @retval -1        失败
	 */
	int32_t GetChnIndex(int32_t chn);

	/**
	 * @brief 获取chn id
	 * @param [in] width   当前需要获取chn id的宽
	 * @param [in] height   当前需要获取chn id的高
	 *
	 * @retval 非-1      成功
	 * @retval -1        失败
	 */
	int32_t GetChnId(int32_t width, int32_t height);

	/**
	 * @brief 获取模块绑定的chn id
	 *
	 * @retval 非-1      成功
	 * @retval -1        失败
	 */
	int32_t GetChnIdForBind(int32_t width, int32_t height);

	/**
	 * @brief 释放模块绑定的chn id
	 *
	 * @retval 非-1      成功
	 * @retval -1        失败
	 */
	void PutChnIdForUnBind(int32_t chn_id);

	/**
	 * @brief 根据输入和输出的分辨率获取合适的VSE通道
	 *
	 * @retval 非-1      成功
	 * @retval -1        失败
	 */
	int32_t SelectVseChn(int *chn_en, int src_width, int src_height,
		int dst_width, int dst_height);

	/**
	 * @brief 初始化 Camera Sensor 和 VSE 各通道的参数
	 *
	 * @retval 非-1      成功
	 * @retval -1        失败
	 */
	int32_t CamInitParam(vp_vflow_contex_t *vp_vflow_contex,
			const int pipe_id, const int video_index,
			int chn_num, int *width, int *height,
			vp_sensors_parameters *parameters);

	/**
	 * @brief 初始化 VSE 各通道的参数
	 *
	 * @retval 非-1      成功
	 * @retval -1        失败
	 */
	int32_t CamInitVseParam(vp_vflow_contex_t *vp_vflow_contex,
		const int pipe_id, int chn_num, int proc_mode,
		int src_width, int src_height, int *dst_width, int *dst_height,
		int *crop_x, int *crop_y, int *crop_width, int *crop_height, int *rotate);

	private:
		int32_t m_last_frame_id = 0;
		vp_vflow_contex_t m_vp_vflow_context;
		int32_t m_only_vse = false;
		hbn_vnode_image_t m_vse_input_image;
};

} // namespace spdev

#endif // __VPP_CAMERA_H__
