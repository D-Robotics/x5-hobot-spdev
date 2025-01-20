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
#include <string>
#include <map>
#include <unistd.h>

#include "utils_log.h"
#include "vpp_module.h"

using namespace std;

namespace spdev
{
	VPPModule::~VPPModule()
	{
		if (m_work_thread != nullptr)
		{
			m_run = 0;
			if (m_work_thread->joinable())
			{
				m_work_thread->join();
				delete m_work_thread;
				m_work_thread = nullptr;
			}

			m_prev_module_chn = 0;
			m_prev_module = nullptr;
		}
	}

	void VPPModule::WorkFunc(void *param)
	{
		VPPModule *prev_module = static_cast<VPPModule *>(param);
		ImageFrame frame = {0};
		int32_t ret = 0;

		while (m_run)
		{
			ret = prev_module->GetImageFrame(&frame, m_prev_module_chn);
			if (ret < 0)
			{
				usleep(30);
				continue;
			}

			if (m_run != 0)
			{
				ret = this->SetImageFrame(&frame);
				if (ret < 0)
				{
					SC_LOGE("Module %s SetImageFrame failed\n",
						prev_module->GetModuleTypeString());
				}
			}

			if (m_run != 0)
			{
				prev_module->ReturnImageFrame(&frame, m_prev_module_chn);
			}
		}

		m_run = 0;
	}

	int32_t VPPModule::BindTo(VPPModule *prev_module, int32_t chn)
	{
		m_run = 1;
		m_prev_module = prev_module;
		SC_LOGE("BindTo_CHN:%d\n",chn);
		if (chn == -1)
		{
			m_prev_module_chn = prev_module->GetChnIdForBind(
				this->GetModuleWidth(), this->GetModuleHeight());
			SC_LOGE("m_prev_module_chn:%d\n",m_prev_module_chn);
		}
		else
		{
			m_prev_module_chn = chn;
		}

		m_work_thread = new thread(&VPPModule::WorkFunc, this,
							static_cast<void *>(prev_module));

		return 0;
	}

	int32_t VPPModule::UnBind(VPPModule *prev_module, int32_t chn)
	{
		m_run = 0;
		if (m_work_thread && m_work_thread->joinable())
		{
			m_work_thread->join();
			delete m_work_thread;
			m_work_thread = nullptr;
		}

		if (chn == -1)
		{
			prev_module->PutChnIdForUnBind(m_prev_module_chn);
		}
		m_prev_module_chn = 0;
		m_prev_module = nullptr;

		return 0;
	}

	void VPPModule::SetModuleType(VPP_Object_e type)
	{
		m_module = type;
	}

	void VPPModule::SetModuleTypeString(char *type)
	{
		m_module_string = type;
	}

	VPP_Object_e VPPModule::GetModuleType()
	{
		return m_module;
	}

	const char *VPPModule::GetModuleTypeString()
	{
		return m_module_string.c_str();
	}

	int32_t VPPModule::GetModuleWidth()
	{
		return m_width;
	}

	int32_t VPPModule::GetModuleHeight()
	{
		return m_height;
	}

	int32_t VPPModule::GetChnIdForBind(int32_t width, int32_t height)
	{
		if ((m_is_bind == 0) && (m_width == width) && (m_height == height))
		{
			m_is_bind = 1;
			return 0;
		}
		else
		{
			SC_LOGE("Module already bind:%d or size:%dx%d not match\n",
				m_is_bind, m_width, m_height);
			return -1;
		}
	}

	void VPPModule::PutChnIdForUnBind(int32_t chn_id)
	{
		m_is_bind = 0;
	}

	int32_t VPPModule::GetPipeId(uint32_t *pipe_mask)
	{
		int32_t pipe_id = -1;
		for (int32_t i = 0; i < VP_MAX_PIPELINE_NUM; i++){
			if ((*pipe_mask & (1 << i)) == 0) {
				pipe_id = i;
				*pipe_mask |= (1 << i);
				break;
			}
		}
		if (pipe_id == -1) {
			SC_LOGE("Module get pipe id failed, max pipeline num: %d!\n", VP_MAX_PIPELINE_NUM);
		}
		return pipe_id;
	}

	void VPPModule::PutPipeId(int32_t pipe_id, uint32_t *pipe_mask)
	{
		*pipe_mask &= ~(1 << pipe_id);
	}

} // namespace spdev
