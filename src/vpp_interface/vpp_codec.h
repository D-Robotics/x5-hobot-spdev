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

#ifndef _VPP_CODEC_H__
#define _VPP_CODEC_H__

#include <semaphore.h>
#include <atomic>
#include <cstdbool>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>

#ifdef __cplusplus
extern "C"
{
#endif

#include "vp_codec.h"
#include "vp_wrap.h"
#include "vpp_module.h"
#include "vp_common.h"

#ifdef __cplusplus
}
#endif /* extern "C" */

using namespace std;

namespace spdev
{

	class VPPEncode : public VPPModule
	{
	public:
		VPPEncode()
		{
			SetModuleType(VPP_ENCODE);
			SetModuleTypeString((char *)"Encode");
		}

		virtual ~VPPEncode() = default;

		int32_t OpenEncode(int32_t type, int32_t width,
						   int32_t height, int32_t bit_rate = 8000);

		/// default 1920 * 1080 -- H264 -- chn0 -- bits = 8000
		int32_t OpenEncode();

		int32_t Close();

		int32_t GetImageFrame(ImageFrame *frame, int32_t chn = 0, const int32_t timeout = VP_GET_FRAME_TIMEOUT);

		void ReturnImageFrame(ImageFrame *frame, int32_t chn = 0);

		int32_t SetImageFrame(ImageFrame *frame);

	private:
		media_codec_context_t m_context = {};
		media_codec_id_t m_type = MEDIA_CODEC_ID_H264;
		int32_t m_bit_rate = 8000;
	};

	class VPPDecode : public VPPModule
	{
	public:
		VPPDecode()
		{
			SetModuleType(VPP_DECODE);
			SetModuleTypeString((char *)"Decode");
		}

		virtual ~VPPDecode() = default;

	public:
		int32_t OpenDecode(int32_t type, int32_t width,
						   int32_t height, const char *file_name, int32_t *frame_cnt);

		/// default 1920 * 1080 -- H264 -- chn0 -- mode: frame
		int32_t OpenDecode();

		int32_t Close();

		int32_t GetImageFrame(ImageFrame *frame, int32_t chn = 0, const int32_t timeout = VP_GET_FRAME_TIMEOUT);

		void ReturnImageFrame(ImageFrame *frame, int32_t chn = 0);

		int32_t SetImageFrame(ImageFrame *frame);

		int32_t SetImageFrame(ImageFrame *frame, int32_t eos);

	private:
		vp_decode_param_t m_dec_param = {};
		media_codec_context_t m_context = {};
		media_codec_id_t m_type = MEDIA_CODEC_ID_H264;
		thread *m_running_thread = nullptr;
	};

}; // namespace spdev

#endif // _VPP_CODEC_H__
