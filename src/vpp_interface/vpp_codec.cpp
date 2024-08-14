/***************************************************************************
 * @COPYRIGHT NOTICE
 * @Copyright 2023 Horizon Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-03-03 21:45:00
 * @LastEditTime: 2023-03-05 16:34:22
 ***************************************************************************/
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <atomic>
#include <cstdbool>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>

#include <functional>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>

#include "vpp_codec.h"

#include "utils_log.h"

using namespace std;

namespace spdev
{

	static uint32_t s_pipe_mask = VP_CODEC_MASK;

	/// Class VPPEncode related
	int32_t VPPEncode::OpenEncode(int32_t type, int32_t width, int32_t height, int32_t bit_rate)
	{
		int32_t ret = 0;
		if (!m_inited.test_and_set())
		{
			m_pipe_id = GetPipeId(&s_pipe_mask);
			if (m_pipe_id < 0)
			{
				LOGE_print("Encode get pipe id error, pipe_id:%d type:%d width:%d h:%d bit_rate:%d\n",
					m_pipe_id, m_type, m_width, m_height, m_bit_rate);
				goto exit_reset_inited;
			}
			m_type = VP_GET_MD_CODEC_TYPE(type);
			m_width = width;
			m_height = height;
			m_bit_rate = bit_rate;

			LOGD_print("pipe:%d type:%d %dx%d bit_rate:%d begin init\n",
				m_pipe_id, m_type, m_width, m_height, m_bit_rate);

			ret = vp_encode_config_param(&m_context, m_type, m_width, m_height, 30, m_bit_rate);
			if (ret != 0)
			{
				LOGE_print("Encode config param error, pipe_id:%d type:%d width:%d h:%d bit_rate:%d\n",
					m_pipe_id, m_type, m_width, m_height, m_bit_rate);
				goto exit_put_pipe_id;
			}

			ret = vp_codec_init(&m_context);
			if (ret != 0)
			{
				LOGE_print("Encode init error, pipe_id:%d type:%d width:%d h:%d bit_rate:%d\n",
					m_pipe_id, m_type, m_width, m_height, m_bit_rate);
				goto exit_put_pipe_id;
			}

			ret = vp_codec_start(&m_context);
			if (ret != 0)
			{
				LOGE_print("Encode init error, pipe_id:%d type:%d width:%d h:%d bit_rate:%d\n",
					m_pipe_id, m_type, m_width, m_height, m_bit_rate);
				goto exit_deinit;
			}
		}
		else
		{
			LOGW_print("Encode init already, pipe_id:%d type:%d width:%d h:%d bit_rate:%d\n",
					m_pipe_id, m_type, m_width, m_height, m_bit_rate);
		}

		return 0;

	exit_deinit:
		PutPipeId(m_pipe_id, &s_pipe_mask);
	exit_put_pipe_id:
		PutPipeId(m_pipe_id, &s_pipe_mask);
	exit_reset_inited:
		m_inited.clear();

		return -1;
	}

	int32_t VPPEncode::OpenEncode()
	{
		int32_t ret = 0;
		if (!m_inited.test_and_set())
		{
			m_pipe_id = GetPipeId(&s_pipe_mask);
			if (m_pipe_id < 0)
			{
				LOGE_print("Encode get pipe id error, pipe_id:%d type:%d width:%d h:%d bit_rate:%d\n",
					m_pipe_id, m_type, m_width, m_height, m_bit_rate);
				goto exit_reset_inited;
			}

			ret = vp_encode_config_param(&m_context, m_type, m_pipe_id, m_width, m_height, m_bit_rate);
			if (ret != 0)
			{
				LOGE_print("Encode config param error, pipe_id:%d type:%d width:%d h:%d bit_rate:%d\n",
					m_pipe_id, m_type, m_width, m_height, m_bit_rate);
				goto exit_put_pipe_id;
			}

			ret = vp_codec_init(&m_context);
			if (ret != 0)
			{
				LOGE_print("Encode init error, pipe_id:%d type:%d width:%d h:%d bit_rate:%d\n",
					m_pipe_id, m_type, m_width, m_height, m_bit_rate);
				goto exit_put_pipe_id;
			}

			ret = vp_codec_start(&m_context);
			if (ret != 0)
			{
				LOGE_print("Encode init error, pipe_id:%d type:%d width:%d h:%d bit_rate:%d\n",
					m_pipe_id, m_type, m_width, m_height, m_bit_rate);
				goto exit_deinit;
			}
		}
		else
		{
			LOGW_print("Encode init already, pipe_id:%d type:%d width:%d h:%d bit_rate:%d\n",
					m_pipe_id, m_type, m_width, m_height, m_bit_rate);
		}

		return 0;

	exit_deinit:
		vp_codec_deinit(&m_context);
	exit_put_pipe_id:
		PutPipeId(m_pipe_id, &s_pipe_mask);
	exit_reset_inited:
		m_inited.clear();

		return -1;
	}

	int32_t VPPEncode::Close()
	{
		int32_t ret = 0;

		if (!m_inited.test_and_set())
		{
			LOGE_print("Encoder was not inited!\n");
			m_inited.clear();
			return -1;
		}
		vp_codec_stop(&m_context);
		vp_codec_deinit(&m_context);

		PutPipeId(m_pipe_id, &s_pipe_mask);
		m_inited.clear();

		return ret;
	}

	int32_t VPPEncode::SetImageFrame(ImageFrame *frame)
	{
		int32_t ret = 0;

		if (!m_inited.test_and_set())
		{
			LOGE_print("Encoder channel dose not created!\n");
			m_inited.clear();
			return -1;
		}

		ret = vp_codec_set_input(&m_context, frame, 0);

		return ret;
	}

	int32_t VPPEncode::GetImageFrame(ImageFrame *frame, int32_t chn, const int32_t timeout)
	{
		int32_t ret = 0;

		if (!m_inited.test_and_set())
		{
			LOGE_print("Encoder channel dose not created!\n");
			m_inited.clear();
			return -1;
		}

		ret = vp_codec_get_output(&m_context, frame, timeout);

		return ret;
	}

	void VPPEncode::ReturnImageFrame(ImageFrame *frame, int32_t chn)
	{
		if (!m_inited.test_and_set())
		{
			LOGE_print("Encoder channel dose not created!\n");
			m_inited.clear();
			return;
		}

		vp_codec_release_output(&m_context, frame);
	}

	/// Class VPPDecode related
	int32_t VPPDecode::OpenDecode(int32_t type, int32_t width, int32_t height,
		const char *file_name, int32_t *frame_cnt)
	{
		int32_t ret = 0;

		if (!m_inited.test_and_set())
		{
			m_pipe_id = GetPipeId(&s_pipe_mask);
			if (m_pipe_id < 0)
			{
				LOGE_print("Decode get pipe id error, pipe_id:%d type:%d width:%d h:%d\n",
					m_pipe_id, m_type, m_width, m_height);
				goto exit_reset_inited;
			}
			m_type = VP_GET_MD_CODEC_TYPE(type);
			m_width = width;
			m_height = height;

			ret = vp_decode_config_param(&m_context, m_type, m_width, m_height);
			if (ret != 0)
			{
				LOGE_print("Decode config param error, pipe_id:%d type:%d width:%d h:%d\n",
					m_pipe_id, m_type, m_width, m_height);
				goto exit_put_pipe_id;
			}
			ret = vp_codec_init(&m_context);
			if (ret != 0)
			{
				LOGE_print("Decode init error, pipe_id:%d type:%d width:%d h:%d\n",
					m_pipe_id, m_type, m_width, m_height);
				goto exit_put_pipe_id;
			}
			ret = vp_codec_start(&m_context);
			if (ret != 0)
			{
				LOGE_print("Decode start error, pipe_id:%d type:%d width:%d h:%d\n",
					m_pipe_id, m_type, m_width, m_height);
				goto exit_deinit;
			}

			if ((file_name != nullptr) && (strlen(file_name) > 0) &&
				(strlen(file_name) < VP_MAX_PATH_LENGTH))
			{
				m_dec_param.context = &m_context;
				strcpy(m_dec_param.stream_path, file_name);
				m_dec_param.is_quit = false;
				sem_init(&m_dec_param.read_done, 0, 0);
				m_running_thread = new thread(&vp_decode_work_func,
										static_cast<void *>(&m_dec_param));
				sem_wait(&m_dec_param.read_done);
				*frame_cnt = m_dec_param.frame_count;
			}
		}
		else
		{
			LOGW_print("Decode already init, pipe_id:%d type:%d width:%d h:%d\n",
					m_pipe_id, m_type, m_width, m_height);
		}

		return ret;

	exit_deinit:
		vp_codec_deinit(&m_context);
	exit_put_pipe_id:
		PutPipeId(m_pipe_id, &s_pipe_mask);
	exit_reset_inited:
		m_inited.clear();

		return -1;
	}

	int32_t VPPDecode::OpenDecode()
	{
		int32_t ret = 0;

		if (!m_inited.test_and_set())
		{
			m_pipe_id = GetPipeId(&s_pipe_mask);
			if (m_pipe_id < 0)
			{
				LOGE_print("Decode get pipe id error, pipe_id:%d type:%d width:%d h:%d\n",
					m_pipe_id, m_type, m_width, m_height);
				goto exit_reset_inited;
			}

			ret = vp_decode_config_param(&m_context, m_type, m_width, m_height);
			if (ret != 0)
			{
				LOGE_print("Decode config param error, pipe_id:%d type:%d width:%d h:%d\n",
					m_pipe_id, m_type, m_width, m_height);
				goto exit_put_pipe_id;
			}
			ret = vp_codec_init(&m_context);
			if (ret != 0)
			{
				LOGE_print("Decode init error, pipe_id:%d type:%d width:%d h:%d\n",
					m_pipe_id, m_type, m_width, m_height);
				goto exit_put_pipe_id;
			}
			ret = vp_codec_start(&m_context);
			if (ret != 0)
			{
				LOGE_print("Decode start error, pipe_id:%d type:%d width:%d h:%d\n",
					m_pipe_id, m_type, m_width, m_height);
				goto exit_deinit;
			}
		}
		else
		{
			LOGW_print("Decode already init, pipe_id:%d type:%d width:%d h:%d\n",
					m_pipe_id, m_type, m_width, m_height);
		}

		return ret;

	exit_deinit:
		vp_codec_deinit(&m_context);
	exit_put_pipe_id:
		PutPipeId(m_pipe_id, &s_pipe_mask);
	exit_reset_inited:
		m_inited.clear();

		return -1;
	}

	int32_t VPPDecode::Close()
	{
		if (!m_inited.test_and_set())
		{
			LOGE_print("Decoder channel dose not created!\n");
			m_inited.clear();
			return -1;
		}

		m_dec_param.is_quit = true;
		if (m_running_thread && m_running_thread->joinable())
		{
			m_running_thread->join();
			delete m_running_thread;
			m_running_thread = nullptr;
		}

		vp_codec_stop(&m_context);

		vp_codec_deinit(&m_context);

		PutPipeId(m_pipe_id, &s_pipe_mask);

		sem_destroy(&m_dec_param.read_done);
		m_inited.clear();

		return 0;
	}

	int32_t VPPDecode::GetImageFrame(ImageFrame *frame, int32_t chn, const int32_t timeout)
	{
		int32_t ret = 0;
		static int64_t frame_id = 0;

		if (!m_inited.test_and_set())
		{
			LOGE_print("Decoder channel dose not created!\n");
			m_inited.clear();
			return 0;
		}

		ret = vp_codec_get_output(&m_context, frame, timeout);
		frame->frame_id = frame_id++;
		frame->image_timestamp = static_cast<int64_t>(time(nullptr));

		return ret;
	}

	void VPPDecode::ReturnImageFrame(ImageFrame *frame, int32_t chn)
	{
		if (!m_inited.test_and_set())
		{
			LOGE_print("Decoder channel dose not created!\n");
			m_inited.clear();
			return;
		}

		vp_codec_release_output(&m_context, frame);
	}

	int32_t VPPDecode::SetImageFrame(ImageFrame *frame)
	{
		int32_t ret = 0;

		if (!m_inited.test_and_set())
		{
			LOGE_print("Decoder channel dose not created!\n");
			m_inited.clear();
			return -1;
		}

		ret = vp_codec_set_input(&m_context, frame, 0);

		return ret;
	}

	int32_t VPPDecode::SetImageFrame(ImageFrame *frame, int32_t eos)
	{
		int32_t ret = 0;

		if (!m_inited.test_and_set())
		{
			LOGE_print("Decoder channel dose not created!\n");
			m_inited.clear();
			return -1;
		}

		ret = vp_codec_set_input(&m_context, frame, eos);

		return ret;
	}

} // namespace spdev