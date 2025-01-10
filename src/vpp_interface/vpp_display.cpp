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
 * @LastEditTime: 2024-09-09 16:38:23
 ***************************************************************************/

#include <cstdint>
#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/fb.h>
#include <stdlib.h>
#include <stdbool.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <algorithm> // std::max, std::min
#include <chrono>
#include <iomanip>
#include <ctime>

#include "utils_log.h"
#include "egl_preview.h"
#include "vpp_display.h"

using namespace std;

namespace spdev
{

	struct timeval tv_timestamp;
	bool isGPUInit = false;
	int32_t currentRenderBufferIndex = 0;
	// Queue to hold RGBA buffers
	std::queue<std::vector<uint8_t>> rgbaQueue;
	std::mutex queueMutex;
	std::condition_variable queueCV;
	std::atomic<bool> stopProcessing{false}; // To control when to stop the processing thread
	constexpr size_t MAX_QUEUE_SIZE = 2;  // Adjust this size based on your needs

bool is_x11_or_wayland_available() {
	// 获取 DISPLAY 环境变量
	const char* display = getenv("DISPLAY");

	// 检查 DISPLAY 是否以 ":" 开头，且 ":" 后面是数字
	if (display != NULL && display[0] == ':' && isdigit(display[1])) {
		return true;  // X11 可用，且 DISPLAY 值符合要求
	}

	return false;
}

bool is_drm_available() {
	int fd = open("/dev/dri/card0", O_RDWR | O_CLOEXEC);
	if (fd < 0) {
		return false;
	}

	// 尝试获取 DRM master 权限, 如果失败则表示设备被占用
	if (ioctl(fd, DRM_IOCTL_SET_MASTER, 0) != 0) {
		if (errno == EBUSY) {
			close(fd);
			return false;  // 设备被占用
		}
	}

	// 设备没有被占用
	ioctl(fd, DRM_IOCTL_DROP_MASTER, 0);
	close(fd);
	return true;
}

/// Class VPPDisplay related

int32_t VPPDisplay::OpenDisplay(int32_t width, int32_t height)
{
	int32_t ret = 0;
	int64_t allocFlags = 0;

	m_width = width;
	m_height = height;

	if (is_x11_or_wayland_available()) {
		printf("X11 is available, using EGL for rendering.\n");
		// 使用 EGL 进行显示
		m_display_mode = 1;
	} else if (is_drm_available()) {
		printf("DRM is available, using libdrm for rendering.\n");
		// 使用 libdrm 进行显示
		m_display_mode = 0;
	} else {
		printf("No suitable display method found.\n");
		printf("Running without preview window\n");
		m_display_mode = 2;
		return 0;
	}

	if (2 == m_display_mode) {
		return 0;
	} else if (0 == m_display_mode) {
		hb_mem_module_open();

		vp_display_init(&m_drm_ctx, width, height);

		allocFlags = HB_MEM_USAGE_MAP_INITIALIZED |
						HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD |
						HB_MEM_USAGE_CPU_READ_OFTEN |
						HB_MEM_USAGE_CPU_WRITE_OFTEN |
						HB_MEM_USAGE_CACHED |
						HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;

		for (int i = 0; i < NUM_BUFFERS; ++i) {
			memset(&m_vo_buffers[i], 0, sizeof(hbn_vnode_image_t));
			ret = hb_mem_alloc_graph_buf(width, height,
					MEM_PIX_FMT_NV12,
					allocFlags,
					width, height,
					&m_vo_buffers[i].buffer);
			if (ret != 0) {
				SC_LOGE("hb_mem_alloc_graph_buf for buffer[%d] failed error(%d)", i, ret);
				for (int j = 0; j < i; ++j) {
					hb_mem_free_buf(m_vo_buffers[j].buffer.fd[0]);
				}
				return -1;
			}
		}
	} else if (1 == m_display_mode) {
		allocFlags = HB_MEM_USAGE_MAP_INITIALIZED |
						HB_MEM_USAGE_PRIV_HEAP_2_RESERVERD |
						HB_MEM_USAGE_CPU_READ_OFTEN |
						HB_MEM_USAGE_CPU_WRITE_OFTEN |
						HB_MEM_USAGE_CACHED |
						HB_MEM_USAGE_GRAPHIC_CONTIGUOUS_BUF;

		for (int i = 0; i < NUM_BUFFERS; ++i) {
			memset(&m_vo_buffers[i], 0, sizeof(hbn_vnode_image_t));
			ret = hb_mem_alloc_graph_buf(width, height,
					MEM_PIX_FMT_NV12,
					allocFlags,
					width, height,
					&m_vo_buffers[i].buffer);
			if (ret != 0) {
				SC_LOGE("hb_mem_alloc_graph_buf for buffer[%d] failed error(%d)", i, ret);
				for (int j = 0; j < i; ++j) {
					hb_mem_free_buf(m_vo_buffers[j].buffer.fd[0]);
				}
				return -1;
			}
		}

		startProcessingThread();
	}

	memset(&m_draw_buffers, 0, sizeof(hbn_vnode_image_t));
	ret = hb_mem_alloc_graph_buf(width, height,
			MEM_PIX_FMT_ARGB,
			allocFlags,
			width, height,
			&m_draw_buffers.buffer);

// final buffer for overlay process
	memset(&m_final_buffers[0], 0, sizeof(hbn_vnode_image_t));
	ret = hb_mem_alloc_graph_buf(width, height,
			MEM_PIX_FMT_NV12,
			allocFlags,
			width, height,
			&m_final_buffers[0].buffer);
	memset(&m_final_buffers[1], 0, sizeof(hbn_vnode_image_t));
	ret = hb_mem_alloc_graph_buf(width, height,
			MEM_PIX_FMT_NV12,
			allocFlags,
			width, height,
			&m_final_buffers[1].buffer);

	if (ret != 0) {
		SC_LOGE("hb_mem_alloc_graph_buf for m_draw_buffers failed error(%d)", ret);
		for (int j = 0; j < NUM_BUFFERS; ++j) {
			hb_mem_free_buf(m_vo_buffers[j].buffer.fd[0]);
		}
		return -1;
	}

	return ret;
}

int32_t VPPDisplay::Close()
{
	int32_t ret = 0;
	if (0 == m_display_mode) {
		hb_mem_free_buf(m_draw_buffers.buffer.fd[0]);
		for (int i = 0; i < NUM_BUFFERS; ++i) {
			hb_mem_free_buf(m_vo_buffers[i].buffer.fd[0]);
		}
		vp_display_deinit(&m_drm_ctx);
		hb_mem_module_close();
	} else if (1 == m_display_mode) {
		hb_mem_free_buf(m_draw_buffers.buffer.fd[0]);
		EGLPreviewWindow& window = EGLPreviewWindow::getInstance();
		window.close();
	}

	return ret;
}

inline int clamp(int value, int minVal, int maxVal) {
	return std::max(minVal, std::min(value, maxVal));
}

// NV12 转 RGBA 函数
void NV12ToRGBA(const uint8_t* nv12, uint8_t* rgba, int width, int height) {
	int frameSize = width * height;
	int chromaSize = frameSize / 4;

	const uint8_t* yPlane = nv12;
	const uint8_t* uvPlane = nv12 + frameSize;

	for (int j = 0; j < height; ++j) {
		for (int i = 0; i < width; ++i) {
			int yIndex = j * width + i;
			int uvIndex = (j / 2) * (width / 2) + (i / 2);

			int y = yPlane[yIndex];
			int u = uvPlane[2 * uvIndex];
			int v = uvPlane[2 * uvIndex + 1];

			// 将 YUV 转换为 RGBA
			int c = y - 16;
			int d = u - 128;
			int e = v - 128;

			int r = clamp((298 * c + 409 * e + 128) >> 8, 0, 255);
			int g = clamp((298 * c - 100 * d - 208 * e + 128) >> 8, 0, 255);
			int b = clamp((298 * c + 516 * d + 128) >> 8, 0, 255);

			rgba[4 * yIndex + 0] = r;
			rgba[4 * yIndex + 1] = g;
			rgba[4 * yIndex + 2] = b;
			rgba[4 * yIndex + 3] = 255; // Alpha 通道设为 255
		}
	}
}

class FrameRateCounter {
	public:
		FrameRateCounter() {
			// 初始化计时器
			lastTime = std::chrono::high_resolution_clock::now();
			frameCount = 0;
			totalFrames = 0;
		}

		void update() {
			frameCount++;
			totalFrames++;

			auto now = std::chrono::high_resolution_clock::now();
			std::chrono::duration<double> elapsed = now - lastTime;

			if (elapsed.count() >= 1.0) {  // 每秒更新一次
				print_fps(now, frameCount);
				frameCount = 0;
				lastTime = now;
			}
		}

	private:
		std::chrono::high_resolution_clock::time_point lastTime;
		int frameCount;
		int totalFrames;

		void print_fps(const std::chrono::high_resolution_clock::time_point& now, int fps) {
			// 获取当前系统时间用于时间戳
			auto currentTime = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
			std::tm* localTime = std::localtime(&currentTime);

			// 输出格式化时间戳和帧速率
			std::cout << "Frame#" << totalFrames << " ("
						<< std::fixed << std::setprecision(2) << fps << " fps) "
						<< "["
						<< std::put_time(localTime, "%Y-%m-%d %H:%M:%S")  // 时间戳
						<< "]" << std::endl;
		}
};

FrameRateCounter fpsCounter;

n2d_error_t create_n2d_buffer_from_phyaddr_continuous_memory(n2d_buffer_t *n2d_buffer,
    n2d_buffer_format_t format, n2d_uintptr_t phys_addr, int width, int height) {

    n2d_error_t error = N2D_SUCCESS;

    memset(n2d_buffer, 0, sizeof(n2d_buffer_t));
    n2d_buffer->width = width;
    n2d_buffer->height = height;

    n2d_buffer->format = format;
    n2d_buffer->orientation = N2D_0;
    n2d_buffer->srcType = N2D_SOURCE_DEFAULT;
    n2d_buffer->tiling = N2D_LINEAR;
    n2d_buffer->cacheMode = N2D_CACHE_128;

    n2d_uintptr_t n2d_buffer_handle;
    n2d_user_memory_desc_t n2d_buffer_mem_desc;
    n2d_buffer_mem_desc.flag = N2D_WRAP_FROM_USERMEMORY;
    n2d_buffer_mem_desc.logical = 0; //
    n2d_buffer_mem_desc.physical = phys_addr;

    if (format == N2D_RGBA8888) {
        // ARGB8888
        n2d_buffer->alignedw = gcmALIGN(n2d_buffer->width, 4);  // 宽度对齐到4字节
        n2d_buffer->alignedh = n2d_buffer->height;
        n2d_buffer->stride = gcmALIGN(n2d_buffer->alignedw * 4, 4);  // 步幅对齐到4字节

        // ARGB8888
        n2d_buffer_mem_desc.size = n2d_buffer->stride * n2d_buffer->alignedh;

    } else if (format == N2D_NV12) {
        // NV12
        n2d_buffer->alignedw = gcmALIGN(n2d_buffer->width, 64);  // 宽度对齐到64字节
        n2d_buffer->alignedh = n2d_buffer->height;

        // NV12
        float nv12_bpp = gcmALIGN(16, 8) * 1.0f / 8;
        n2d_buffer->stride = gcmALIGN(gcmFLOAT2INT(n2d_buffer->alignedw * nv12_bpp), 64);  // 步幅对齐到64字节

        // NV12
        n2d_buffer_mem_desc.size = n2d_buffer->stride * n2d_buffer->alignedh * 3 / 2;
    } else {

        return N2D_INVALID_ARGUMENT;
    }



    error = n2d_wrap(&n2d_buffer_mem_desc, &n2d_buffer_handle);
    if (N2D_IS_ERROR(error)) {
        return error;
    }

    n2d_buffer->handle = n2d_buffer_handle;
    error = n2d_map(n2d_buffer);
    if (N2D_IS_ERROR(error)) {
        return error;
    }

    return N2D_SUCCESS;
}

n2d_error_t VPPDisplay::do_overlay(n2d_buffer_t *src0_nv12,n2d_buffer_t *scr1_ar24,n2d_buffer_t *dst0_nv12){
	n2d_error_t error	       = N2D_SUCCESS;
	n2d_rectangle_t rect;
	rect.x = 0; //top left
	rect.y = 0;
	rect.width = scr1_ar24->width;
	rect.height = scr1_ar24->height;

	N2D_ON_ERROR(n2d_blit(&rgba_buffer, N2D_NULL, src0_nv12, N2D_NULL, N2D_BLEND_NONE));
	N2D_ON_ERROR(n2d_blit(&rgba_buffer, &rect , scr1_ar24, N2D_NULL, N2D_BLEND_SRC_OVER));
	N2D_ON_ERROR(n2d_blit(dst0_nv12, &rect , &rgba_buffer, N2D_NULL, N2D_BLEND_NONE));
	N2D_ON_ERROR(n2d_commit());
	return error;
on_error:
	n2d_free(src0_nv12);
	n2d_free(scr1_ar24);
	n2d_free(dst0_nv12);
	return error;
}


int32_t VPPDisplay::convertN2DBuffer(n2d_buffer_t *n2d_buffer, hb_mem_graphic_buf_t *hbm_buffer, n2d_buffer_format_t format){
	n2d_error_t error = N2D_SUCCESS;

	error = create_n2d_buffer_from_phyaddr_continuous_memory(n2d_buffer, format,
		(n2d_uintptr_t)hbm_buffer->phys_addr[0], hbm_buffer->width, hbm_buffer->height);

	return error;
}

size_t get_nv12_size(int width, int height) {
    return width * height * 1.5; // Y plane + UV plane
}

size_t get_argb8888_size(int width, int height) {
    return width * height * 4; // 每个像素4字节
}

void save_image_to_file(const char* file_name, void* buffer, size_t size) {

    FILE* file = fopen(file_name, "wb");
    if (!file) {
        printf("Failed to open file %s for writing\n", file_name);
        return;
    }


    size_t written = fwrite(buffer, 1, size, file);
    if (written != size) {
        printf("Failed to write all data to file. Written: %zu, Expected: %zu\n", written, size);
    }


    fclose(file);
    printf("Image data successfully saved to %s\n", file_name);
}

int32_t VPPDisplay::SetImageFrame(ImageFrame *frame) {
	int32_t ret = 0;

	if (2 == m_display_mode) {
		fpsCounter.update();
		return 0;
	}

	//Must ensure that the n2d_open and the final compositing operations are executed on the same thread.
	if(isGPUInit == false){
		ret = n2d_open();
		if(ret)
		{
			LOGE_print("n2d_open fail,ret:%d\n",ret);
		}

		for (size_t i = 0; i < 3; i++)
		{
			ret = convertN2DBuffer(&primary_mapped_gpu_buffer[i],&m_vo_buffers[i].buffer, N2D_NV12);
			if(ret){
				LOGE_print("convertN2DBuffer(NV12) fail,ret:%d\n",ret);
			}
		}

		ret = convertN2DBuffer(&overlay_mapped_gpu_buffer[0],&m_draw_buffers.buffer, N2D_RGBA8888);
		if(ret){
			LOGE_print("convertN2DBuffer(AR24) fail,ret:%d\n",ret);
		}

		ret = convertN2DBuffer(&final_mapped_gpu_buffer[0],&m_final_buffers[0].buffer, N2D_NV12);
		if(ret){
			LOGE_print("convertN2DBuffer(NV12 final) fail,ret:%d\n",ret);
		}
		ret = convertN2DBuffer(&final_mapped_gpu_buffer[1],&m_final_buffers[1].buffer, N2D_NV12);
		if(ret){
			LOGE_print("convertN2DBuffer(NV12 final) fail,ret:%d\n",ret);
		}
		memset(&rgba_buffer,0,sizeof(n2d_buffer_t));
		n2d_util_allocate_buffer(
        m_vo_buffers[0].buffer.width,
        m_vo_buffers[0].buffer.height,
        N2D_RGBA8888,
        N2D_0,
        N2D_LINEAR,
        N2D_TSC_DISABLE,
        &rgba_buffer);

		LOGI_print("N2D init done!\n");
		isGPUInit = true;
	}

	hbn_vnode_image_t& currentBuffer = m_vo_buffers[currentBufferIndex];
	currentBufferIndex = (currentBufferIndex + 1) % NUM_BUFFERS;

	for (int i = 0; i < frame->plane_count; ++i) {
		memcpy(currentBuffer.buffer.virt_addr[i], frame->data[i], frame->data_size[i]);
	}

	if (0 == m_display_mode) {
		// let's do overlay
		do_overlay(&primary_mapped_gpu_buffer[currentBufferIndex], &overlay_mapped_gpu_buffer[0], &final_mapped_gpu_buffer[currentRenderBufferIndex]);
		ret = vp_display_set_frame(&m_drm_ctx, 0, &m_final_buffers[currentRenderBufferIndex]);
		currentRenderBufferIndex = (currentRenderBufferIndex + 1) % 2;

	} else if (1 == m_display_mode) {
		std::vector<uint8_t> rgbaData(m_width * m_height * 4);

		// Convert NV12 to RGBA in the main thread (producer)
		NV12ToRGBA(currentBuffer.buffer.virt_addr[0], rgbaData.data(), m_width, m_height);

		// Lock and push the converted RGBA data into the queue
		{
			std::lock_guard<std::mutex> lock(queueMutex);
			if (rgbaQueue.size() < MAX_QUEUE_SIZE) {
				rgbaQueue.push(std::move(rgbaData));
			}
		}
		queueCV.notify_one();  // Notify the consumer thread to process the RGBA data
	}
err:
	return ret;
}

int32_t VPPDisplay::GetImageFrame(ImageFrame *frame, int32_t chn, const int32_t timeout)
{
	// not support
	return 0;
}

void VPPDisplay::ReturnImageFrame(ImageFrame *frame, int32_t chn)
{
	// not support
}

int32_t VPPDisplay::SetGraphRect(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
	int32_t flush, uint32_t color, int32_t line_width)
{
	int32_t ret = 0;

	if (0 == m_display_mode) {
		x0 = (x0 < (m_width - line_width)) ? ((x0 >= 0) ? x0 : 0) : (m_width - line_width);
		y0 = (y0 < (m_height - line_width)) ? ((y0 >= 0) ? y0 : 0) : (m_height - line_width);
		x1 = (x1 < (m_width - line_width)) ? ((x1 >= 0) ? x1 : 0) : (m_width - line_width);
		y1 = (y1 < (m_height - line_width)) ? ((y1 >= 0) ? y1 : 0) : (m_height - line_width);

		if (flush) {
			//ret = vp_display_set_frame(&m_drm_ctx, 1, &m_draw_buffers);
			memset(m_draw_buffers.buffer.virt_addr[0], 0, m_width * m_height * DISPLAY_ARGB_BYTES);
		}

		vp_display_draw_rect(m_draw_buffers.buffer.virt_addr[0],
			x0, y0, x1, y1, color, 0, m_width, m_height, line_width);
	} else if (1 == m_display_mode) {
		return 0;
	}

	return ret;
}

int32_t VPPDisplay::SetGraphWord(int32_t x, int32_t y, char *str,
	int32_t flush, uint32_t color, int32_t line_width)
{
	int32_t ret = 0;
	int32_t len;
	if (str == NULL) {
		LOGE_print("string was NULL\n");
		return -1;
	}

	if (0 == m_display_mode) {
		if ((x < 0) || (x > m_width) || (y < 0) || (y > m_height) ||
			((line_width * FONT_ONE_ENCODE_WIDTH + y) > m_height)) {
			LOGE_print("parameter error, coordinate (%d, %d) string:%s line_width:%d\n",
				x, y, str, line_width);
			return -1;
		}
		if (((int)strlen(str) * line_width * FONT_ONE_ENCODE_WIDTH + x) > m_width) {
			len = (m_width - x) / (line_width * FONT_ONE_ENCODE_WIDTH);
			str[len] = '\0';
		}
		ret = vp_display_draw_word(m_draw_buffers.buffer.virt_addr[0], x, y, str, m_width, color, line_width);
	} else if (1 == m_display_mode) {

		return 0;
	}

	return ret;
}

int32_t VPPDisplay::GetDisplayRes(int32_t *width , int32_t *height)
{
	return vp_display_get_res(width , height);
}

void VPPDisplay::startProcessingThread() {
	stopProcessing = false;
	// Start the consumer thread to process RGBA queue
	std::thread(&VPPDisplay::processRGBAQueue, this).detach();
}

void VPPDisplay::stopProcessingThread() {
	stopProcessing = true;
	queueCV.notify_all();  // Ensure the thread can exit
}

// Consumer thread function: Processes the RGBA queue and updates the window
void VPPDisplay::processRGBAQueue() {
	while (!stopProcessing) {
		std::unique_lock<std::mutex> lock(queueMutex);
		// Wait until there's data in the queue or stop is requested
		queueCV.wait(lock, [] { return !rgbaQueue.empty() || stopProcessing; });

		if (stopProcessing && rgbaQueue.empty()) {
			break;
		}

		// Get the RGBA buffer from the queue
		auto rgbaData = std::move(rgbaQueue.front());
		rgbaQueue.pop();
		lock.unlock();

		EGLPreviewWindow& window = EGLPreviewWindow::getInstance(clamp(m_width / 2, 0, 720), clamp(m_height / 2, 0, 405));
		if (!window.initialize()) {
			printf("Failed to initialize display window\n");
			return;
		}

		// Update the window with the RGBA data
		if (!window.initialize()) {
			return;
		}

		window.setImageDimensions(m_width, m_height);
		window.update(rgbaData);
		window.processEvents();  // Process X11 events
	}
}

}; // namespace spdev
