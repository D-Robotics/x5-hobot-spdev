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
 * @Date: 2023-03-05 11:20:22
 * @LastEditTime: 2023-03-05 16:27:23
 ***************************************************************************/
/*
 * Horizon Robotics
 *
 * Copyright (C) 2022 Horizon Robotics Inc.
 * All rights reserved.
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _VPP_DISPLAY_H__
#define _VPP_DISPLAY_H__

#include <atomic>
#include <cstdbool>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stddef.h>
#include <vector>
#include <condition_variable>
#include <thread>
#include <queue>


#include "GC820/nano2D.h"
#include "GC820/nano2D_util.h"

#include "vp_wrap.h"
#include "vpp_module.h"
#include "vp_display.h"

using namespace std;

namespace spdev
{
	class VPPDisplay : public VPPModule
	{
	public:
		VPPDisplay()
		{
			SetModuleType(VPP_DISPLAY);
			SetModuleTypeString((char *)"Display");
		}

		virtual ~VPPDisplay() = default;

	public:
		int32_t OpenDisplay(int32_t width = 1920, int32_t height = 1080);

		int32_t Close();

		int32_t SetImageFrame(ImageFrame *frame);

		int32_t GetImageFrame(ImageFrame *frame, int32_t chn = 0, const int32_t timeout = 0);

		/**
		 * @brief 释放图像数据（qbuf）
		 * @param [in] frame   保存图像数据的内存地址
		 * @param [out] format       保存图像数据的格式， 0：YUV 1：RAW
		 *
		 * @retval 0      成功
		 * @retval -1      失败
		 */
		void ReturnImageFrame(ImageFrame *frame, int32_t chn = 0);

		int32_t SetGraphRect(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
							 int32_t flush = 0, uint32_t color = 0xffff0000, int32_t line_width = 4);

		int32_t SetGraphWord(int32_t x, int32_t y, char *str,
							 int32_t flush = 0, uint32_t color = 0xffff0000, int32_t line_width = 1);

		int32_t GetDisplayRes(int32_t *width , int32_t *height);

		void startProcessingThread();
		void stopProcessingThread();

	private:
		vp_drm_context_t m_drm_ctx;
		static const int NUM_BUFFERS = 3;
		hbn_vnode_image_t m_vo_buffers[NUM_BUFFERS];
		int32_t currentBufferIndex = 0;
		hbn_vnode_image_t m_draw_buffers;
		hbn_vnode_image_t m_final_buffers[2];
		int32_t currentDrawBufferIndex = 0;

		int32_t m_display_mode = 0; /* 0: libdrm; 1: egl x11 */

		n2d_buffer_t primary_mapped_gpu_buffer[3];
		n2d_buffer_t overlay_mapped_gpu_buffer[1];
		n2d_buffer_t final_mapped_gpu_buffer[2];
		n2d_buffer_t rgba_buffer;
		void processRGBAQueue();
		int32_t convertN2DBuffer(n2d_buffer_t *n2d_buffer, hb_mem_graphic_buf_t *hbm_buffer, n2d_buffer_format_t format);
		n2d_error_t do_overlay(n2d_buffer_t *src0_nv12,n2d_buffer_t *scr1_ar24,n2d_buffer_t *dst0_nv12);
	};

}; // namespace spdev

#endif // _VPP_DISPLAY_H__
