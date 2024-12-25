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
#ifndef _VPP_MODULE_H__
#define _VPP_MODULE_H__

#include <string>
#include <thread>
#include <memory>

#include "vp_wrap.h"

using namespace std;

namespace spdev
{
	typedef enum
	{
		VPP_MODULE_DEFAULT,
		VPP_CAMERA,
		VPP_ENCODE,
		VPP_DECODE,
		VPP_DISPLAY
	} VPP_Object_e;

	class VPPModule
	{
	public:
		VPPModule() = default;
		~VPPModule();

		/**
		 * @brief work func
		 * @param [in] param        传入参数
		 *
		 */
		void WorkFunc(void *param);

		/**
		 * @brief bind to another module
		 * @param [in] prev_module        模块对象
		 *
		 * @retval 0        成功
		 * @retval -1     失败
		 */
		int32_t BindTo(VPPModule *prev_module, int32_t chn = -1);

		/**
		 * @brief unbind to another module
		 * @param [in] prev_module        模块对象
		 *
		 * @retval 0        成功
		 * @retval -1     失败
		 */
		int32_t UnBind(VPPModule *prev_module, int32_t chn = -1);

		/**
		 * @brief 获取图像数据（dqbuf）
		 * @param [out] frame    用于保存YUV图像数据的内存地址
		 * @param [in] module        从哪个模块取图：0:SIF 1:ISP: 2: IPU CHN(ds2 default)
		 *
		 * @retval 0        成功
		 * @retval -1     失败
		 */
		virtual int32_t GetImageFrame(ImageFrame *frame, int32_t chn = 0, const int32_t timeout = VP_GET_FRAME_TIMEOUT) = 0;

		/**
		 * @brief 释放图像数据（qbuf）
		 * @param [in] frame   保存图像数据的内存地址
		 * @param [out] format       保存图像数据的格式， 0：YUV 1：RAW
		 *
		 * @retval 0      成功
		 * @retval -1      失败
		 */
		virtual void ReturnImageFrame(ImageFrame *frame, int32_t chn = 0) = 0;

		/**
		 * @brief 设置图像数据（dqbuf）
		 * @param [in] frame    用于保存YUV图像数据的内存地址
		 *
		 * @retval 0        成功
		 * @retval -1     失败
		 */
		virtual int32_t SetImageFrame(ImageFrame *frame) = 0;

		/**
		 * @brief 设置模块类型
		 *
		 * @retval 非-1      成功
		 * @retval -1        失败
		 */
		void SetModuleType(VPP_Object_e type);

		/**
		 * @brief 设置模块类型字符串
		 *
		 * @retval 非-1      成功
		 * @retval -1        失败
		 */
		void SetModuleTypeString(char *type);

		/**
		 * @brief 获取模块类型
		 *
		 * @retval 非-1      成功
		 * @retval -1        失败
		 */
		VPP_Object_e GetModuleType();

		/**
		 * @brief 获取模块类型字符串
		 *
		 * @retval 非-1      成功
		 * @retval -1        失败
		 */
		const char *GetModuleTypeString();

		/**
		 * @brief 获取模块输入宽度
		 *
		 * @retval 非-1      成功
		 * @retval -1        失败
		 */
		int32_t GetModuleWidth();

		/**
		 * @brief 获取模块输入高度
		 *
		 * @retval 非-1      成功
		 * @retval -1        失败
		 */
		int32_t GetModuleHeight();

		/**
		 * @brief 获取模块pipe id
		 *
		 * @retval 非-1      成功
		 * @retval -1        失败
		 */
		int32_t GetPipeId(uint32_t *pipe_mask);

		/**
		 * @brief 释放模块pipe id
		 *
		 * @retval 非-1      成功
		 * @retval -1        失败
		 */
		void PutPipeId(int32_t pipe_id, uint32_t *pipe_mask);

		/**
		 * @brief 获取模块绑定的chn id
		 *
		 * @retval 非-1      成功
		 * @retval -1        失败
		 */
		virtual int32_t GetChnIdForBind(int32_t width, int32_t height);

		/**
		 * @brief 释放模块绑定的chn id
		 *
		 * @retval 非-1      成功
		 * @retval -1        失败
		 */
		virtual void PutChnIdForUnBind(int32_t chn_id);

	protected:
		atomic_flag m_inited = ATOMIC_FLAG_INIT;
		VPP_Object_e m_module = VPP_MODULE_DEFAULT;
		string m_module_string = "Module";
		int32_t m_pipe_id = 0;
		int32_t m_is_bind = 0;
		int32_t m_width = 1920;	 // input width
		int32_t m_height = 1080; // input height

	private:
		int32_t m_prev_module_chn = 0;
		VPPModule *m_prev_module = NULL;

		thread *m_work_thread = nullptr;
		int32_t m_run = 0;
	};

} // namespace spdev

#endif // _VPP_MODULE_H__
