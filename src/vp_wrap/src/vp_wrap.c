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

// #define J5
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <signal.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#include "utils/cJSON.h"
#include "utils/utils_log.h"
#include "utils/common_utils.h"

#include "vp_vin.h"
#include "vp_wrap.h"

#include "vp_sensors.h"

// 获取主芯片类型
static int32_t vp_get_chip_type(char *chip_type)
{
	int ret = 0;
	FILE *stream;
	char chip_id[16] = {0};

	stream = fopen("/sys/class/socinfo/chip_id", "r");
	if (!stream) {
		SC_LOGE("open fail: %s", strerror(errno));
		return -1;
	}
	ret = fread(chip_id, sizeof(char), 9, stream);
	if (ret != 9) {
		SC_LOGE("read fail: %s", strerror(errno));
		fclose(stream);
		return -1;
	}
	fclose(stream);

	// TODO: 先写死，后面有判断方法后再补充正确的逻辑
	strcpy(chip_type, "X5");

	return ret;
}

void vp_print_debug_infos(void)
{
	printf("================= VP Modules Status ====================\n");
	printf("======================== VFLOW =========================\n");
	print_file("/sys/devices/virtual/vps/flow/path_stat");
	printf("========================= SIF ==========================\n");
	print_file("/sys/devices/platform/soc/soc:cam/3d050000.sif/cim_stat");
	print_file("/sys/devices/platform/soc/soc:cam/3d020000.sif/cim_stat");
	print_file("/sys/devices/platform/soc/soc:cam/3d040000.sif/cim_stat");
	print_file("/sys/devices/platform/soc/soc:cam/3d030000.sif/cim_stat");
	printf("========================= ISP ==========================\n");
	print_file("/sys/devices/platform/soc/soc:cam/3d000000.isp/stat");
	printf("========================= VSE ==========================\n");
	print_file("/sys/devices/platform/soc/soc:cam/3d010000.vse/stat");
	printf("========================= VENC =========================\n");
	print_file("/sys/kernel/debug/vpu/venc");
	printf("========================= VDEC =========================\n");
	print_file("/sys/kernel/debug/vpu/vdec");
	printf("========================= JENC =========================\n");
	print_file("/sys/kernel/debug/jpu/jenc");

	printf("======================= Buffer =========================\n");
	print_file("/sys/devices/virtual/vps/flow/fmgr_stats");

	if (log_ctrl_level_get(NULL) == LOG_DEBUG) {
		printf("========================= ION ==========================\n");
		print_file("/sys/kernel/debug/ion/heaps/carveout");
		// print_file("/sys/kernel/debug/ion/heaps/ion_cma");
		print_file("/sys/kernel/debug/ion/ion_buf");
	}
	printf("========================= END ===========================\n");
}

void vp_normal_buf_info_print(ImageFrame *frame)
{
	if (!frame) {
		printf("ImageFrame is NULL.\n");
		return;
	}

	// 打印基本信息
	printf("ImageFrame Info:\n");
	printf("  Width: %d\n", frame->width);
	printf("  Height: %d\n", frame->height);
	printf("  Stride: %d\n", frame->stride);
	printf("  VStride: %d\n", frame->vstride);
	printf("  Frame ID: %ld\n", frame->frame_id);
	printf("  Lost Image Num: %ld\n", frame->lost_image_num);
	printf("  Exposure Time: %ld\n", frame->exp_time);
	printf("  Image Timestamp: %ld\n", frame->image_timestamp);
	printf("  Plane Count: %d\n", frame->plane_count);

	// 打印每个平面的数据
	for (int i = 0; i < frame->plane_count; ++i) {
		printf("  Plane %d:\n", i);
		printf("    Data Size: %u bytes\n", frame->data_size[i]);
		printf("    Physical Address: %llu\n", frame->pdata[i]);
		printf("    Virtual Address: %p\n", frame->data[i]);
	}
}

int32_t vp_dump_1plane_image_to_file(char *filename, uint8_t *src_buffer, uint32_t size)
{
	int image_fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);

	if (image_fd == -1) {
		SC_LOGE("Error opening file(%s)", filename);
		return -1;
	}

	ssize_t bytes_written = write(image_fd, src_buffer, size);
	close(image_fd);

	if (bytes_written != size) {
		SC_LOGE("Error writing to file");
		return -1;
	}
	return 0;
}

int32_t vp_dump_yuv_to_file(char *filename, uint8_t *src_buffer, uint32_t size)
{
	int yuv_fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);

	if (yuv_fd == -1) {
		SC_LOGE("Error opening file(%s)", filename);
		return -1;
	}

	ssize_t bytes_written = write(yuv_fd, src_buffer, size);
	close(yuv_fd);

	if (bytes_written != size) {
		SC_LOGE("Error writing to file");
		return -1;
	}

	return 0;
}

int32_t vp_dump_nv12_to_file(char *filename, uint8_t *data_y, uint8_t *data_uv,
		int width, int height)
{
	int yuv_fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);

	if (yuv_fd == -1) {
		SC_LOGE("Error opening file(%s)", filename);
		return -1;
	}

	ssize_t bytes_written = write(yuv_fd, data_y, width * height);
	if (bytes_written != width * height) {
		SC_LOGE("Error writing to file");
		close(yuv_fd);
		return -1;
	}

	bytes_written = write(yuv_fd, data_uv,  width * height / 2);
	if (bytes_written != width * height /2) {
		SC_LOGE("Error writing to file");
		close(yuv_fd);
		return -1;
	}

	close(yuv_fd);

	// SC_LOGI("Dump yuv to file(%s), size(%d) + size1(%d) succeeded\n", filename, size, size1);
	return 0;
}

int32_t vp_dump_2plane_yuv_to_file(char *filename, uint8_t *src_buffer, uint8_t *src_buffer1,
		uint32_t size, uint32_t size1)
{
	int yuv_fd = open(filename, O_WRONLY | O_CREAT | O_TRUNC, 0644);

	if (yuv_fd == -1) {
		SC_LOGE("Error opening file(%s)", filename);
		return -1;
	}

	ssize_t bytes_written = write(yuv_fd, src_buffer, size);
	if (bytes_written != size) {
		SC_LOGE("Error writing to file");
		close(yuv_fd);
		return -1;
	}

	bytes_written = write(yuv_fd, src_buffer1, size1);
	if (bytes_written != size1) {
		SC_LOGE("Error writing to file");
		close(yuv_fd);
		return -1;
	}

	close(yuv_fd);

	// SC_LOGI("Dump yuv to file(%s), size(%d) + size1(%d) succeeded\n", filename, size, size1);
	return 0;
}

// 函数声明
void vp_vin_print_hbn_frame_info_t(const hbn_frame_info_t *frame_info);
void vp_vin_print_hb_mem_graphic_buf_t(const hb_mem_graphic_buf_t *graphic_buf);

// 打印 hbn_vnode_image_t 结构体的所有字段内容
void vp_vin_print_hbn_vnode_image_t(const hbn_vnode_image_t *frame)
{
	printf("=== hbn_vnode_image ===\n");
	printf("=== Frame Info ===\n");
	vp_vin_print_hbn_frame_info_t(&(frame->info));
	printf("\n=== Graphic Buffer ===\n");
	vp_vin_print_hb_mem_graphic_buf_t(&(frame->buffer));
	printf("\n=== Metadata ===\n");
	printf("Metadata: %p\n", frame->metadata);
}

// 打印 hbn_frame_info_t 结构体的所有字段内容
void vp_vin_print_hbn_frame_info_t(const hbn_frame_info_t *frame_info) {
	printf("Frame ID: %u\n", frame_info->frame_id);
	printf("Timestamps: %lu\n", frame_info->timestamps);
	printf("tv: %ld.%06ld\n", frame_info->tv.tv_sec, frame_info->tv.tv_usec);
	printf("trig_tv: %ld.%06ld\n", frame_info->trig_tv.tv_sec, frame_info->trig_tv.tv_usec);
	printf("Frame Done: %u\n", frame_info->frame_done);
	printf("Buffer Index: %d\n", frame_info->bufferindex);
}

// 打印 hb_mem_graphic_buf_t 结构体的所有字段内容
void vp_vin_print_hb_mem_graphic_buf_t(const hb_mem_graphic_buf_t *graphic_buf) {
	printf("File Descriptors: ");
	for (int i = 0; i < MAX_GRAPHIC_BUF_COMP; i++) {
		printf("%d ", graphic_buf->fd[i]);
	}
	printf("\n");

	printf("Plane Count: %d\n", graphic_buf->plane_cnt);
	printf("Format: %d\n", graphic_buf->format);
	printf("Width: %d\n", graphic_buf->width);
	printf("Height: %d\n", graphic_buf->height);
	printf("Stride: %d\n", graphic_buf->stride);
	printf("Vertical Stride: %d\n", graphic_buf->vstride);
	printf("Is Contiguous: %d\n", graphic_buf->is_contig);

	printf("Share IDs: ");
	for (int i = 0; i < MAX_GRAPHIC_BUF_COMP; i++) {
		printf("%d ", graphic_buf->share_id[i]);
	}
	printf("\n");

	printf("Flags: %ld\n", graphic_buf->flags);

	printf("Sizes: ");
	for (int i = 0; i < MAX_GRAPHIC_BUF_COMP; i++) {
		printf("%lu ", graphic_buf->size[i]);
	}
	printf("\n");

	printf("Virtual Addresses: ");
	for (int i = 0; i < MAX_GRAPHIC_BUF_COMP; i++) {
		printf("%p ", graphic_buf->virt_addr[i]);
	}
	printf("\n");

	printf("Physical Addresses: ");
	for (int i = 0; i < MAX_GRAPHIC_BUF_COMP; i++) {
		printf("%lu ", graphic_buf->phys_addr[i]);
	}
	printf("\n");

	printf("Offsets: ");
	for (int i = 0; i < MAX_GRAPHIC_BUF_COMP; i++) {
		printf("%lu ", graphic_buf->offset[i]);
	}
	printf("\n");
}


// 根据参数类型打印不同 union 的值的函数
void vp_codec_print_media_codec_output_buffer_info(ImageFrame *frame)
{
	media_codec_buffer_t *frame_buffer = &(frame->frame_buffer);
	media_codec_output_buffer_info_t *buffer_info = &(frame->buffer_info);
	switch (frame_buffer->type) {
		case MC_VIDEO_FRAME_BUFFER:
			printf("/**\n"
				"* Define the H264/H265 output frame information.\n"
				"*/\n");
			printf("typedef struct _mc_h264_h265_output_frame_info {\n");

			// 打印每个字段的名称和值
			printf("    decode_result: %d\n", buffer_info->video_frame_info.decode_result);
			printf("    frame_display_index: %d\n", buffer_info->video_frame_info.frame_display_index);
			printf("    frame_decoded_index: %d\n", buffer_info->video_frame_info.frame_decoded_index);
			printf("    stream_start_addr: %lu\n", buffer_info->video_frame_info.stream_start_addr);
			printf("    stream_size: %d\n", buffer_info->video_frame_info.stream_size);
			printf("    nalu_type: %d\n", buffer_info->video_frame_info.nalu_type);
			printf("    err_mb_in_frame_decoded: %d\n", buffer_info->video_frame_info.err_mb_in_frame_decoded);
			printf("    total_mb_in_frame_decoded: %d\n", buffer_info->video_frame_info.total_mb_in_frame_decoded);
			printf("    err_mb_in_frame_display: %d\n", buffer_info->video_frame_info.err_mb_in_frame_display);
			printf("    total_mb_in_frame_display: %d\n", buffer_info->video_frame_info.total_mb_in_frame_display);
			printf("    display_rect: (x=%d, y=%d, width=%d, height=%d)\n", buffer_info->video_frame_info.display_rect.x_pos, buffer_info->video_frame_info.display_rect.y_pos, buffer_info->video_frame_info.display_rect.width, buffer_info->video_frame_info.display_rect.height);
			printf("    display_width: %d\n", buffer_info->video_frame_info.display_width);
			printf("    display_height: %d\n", buffer_info->video_frame_info.display_height);
			printf("    decoded_rect: (x=%d, y=%d, width=%d, height=%d)\n", buffer_info->video_frame_info.decoded_rect.x_pos, buffer_info->video_frame_info.decoded_rect.y_pos, buffer_info->video_frame_info.decoded_rect.width, buffer_info->video_frame_info.decoded_rect.height);
			printf("    aspect_rate_info: %d\n", buffer_info->video_frame_info.aspect_rate_info);
			printf("    frame_rate_numerator: %d\n", buffer_info->video_frame_info.frame_rate_numerator);
			printf("    frame_rate_denominator: %d\n", buffer_info->video_frame_info.frame_rate_denominator);
			printf("    display_poc: %d\n", buffer_info->video_frame_info.display_poc);
			printf("    decoded_poc: %d\n", buffer_info->video_frame_info.decoded_poc);
			printf("    error_reason: %d\n", buffer_info->video_frame_info.error_reason);
			printf("    warn_info: %d\n", buffer_info->video_frame_info.warn_info);
			printf("    sequence_no: %u\n", buffer_info->video_frame_info.sequence_no);
			printf("    temporal_id: %d\n", buffer_info->video_frame_info.temporal_id);
			printf("    output_flag: %d\n", buffer_info->video_frame_info.output_flag);
			printf("    ctu_size: %d\n", buffer_info->video_frame_info.ctu_size);
			printf("} mc_h264_h265_output_frame_info_t;\n");

			printf("/**\n"
				"* Define the information of video frame buffer(H264/H265/MJPEG/JPEG).\n"
				"*/\n");
			printf("typedef struct _mc_video_frame_buffer_info {\n");
			// 打印每个字段的名称和值
			printf("    vir_ptr[0]: %p\n", frame_buffer->vframe_buf.vir_ptr[0]);
			printf("    vir_ptr[1]: %p\n", frame_buffer->vframe_buf.vir_ptr[1]);
			printf("    vir_ptr[2]: %p\n", frame_buffer->vframe_buf.vir_ptr[2]);
			printf("    phy_ptr[0]: %lu\n", frame_buffer->vframe_buf.phy_ptr[0]);
			printf("    phy_ptr[1]: %lu\n", frame_buffer->vframe_buf.phy_ptr[1]);
			printf("    phy_ptr[2]: %lu\n", frame_buffer->vframe_buf.phy_ptr[2]);
			printf("    size: %u\n", frame_buffer->vframe_buf.size);
			printf("    compSize[0]: %u\n", frame_buffer->vframe_buf.compSize[0]);
			printf("    compSize[1]: %u\n", frame_buffer->vframe_buf.compSize[1]);
			printf("    compSize[2]: %u\n", frame_buffer->vframe_buf.compSize[2]);
			printf("    width: %d\n", frame_buffer->vframe_buf.width);
			printf("    height: %d\n", frame_buffer->vframe_buf.height);
			printf("    pix_fmt: %d\n", frame_buffer->vframe_buf.pix_fmt);
			printf("    stride: %d\n", frame_buffer->vframe_buf.stride);
			printf("    vstride: %d\n", frame_buffer->vframe_buf.vstride);
			printf("    fd[0]: %d\n", frame_buffer->vframe_buf.fd[0]);
			printf("    fd[1]: %d\n", frame_buffer->vframe_buf.fd[1]);
			printf("    fd[2]: %d\n", frame_buffer->vframe_buf.fd[2]);
			printf("    pts: %lu\n", frame_buffer->vframe_buf.pts);
			printf("    src_idx: %d\n", frame_buffer->vframe_buf.src_idx);
			printf("    frame_end: %d\n", frame_buffer->vframe_buf.frame_end);
			printf("    qp_map_valid: %d\n", frame_buffer->vframe_buf.qp_map_valid);
			// printf("    qp_map_array: %u\n", frame_buffer->vframe_buf.qp_map_array);
			printf("    qp_map_array_count: %u\n", frame_buffer->vframe_buf.qp_map_array_count);
			printf("    flags: %d\n", frame_buffer->vframe_buf.flags);

			printf("} mc_video_frame_buffer_info_t;\n");
			break;
		case MC_VIDEO_STREAM_BUFFER:
			printf("/**\n"
				"*  Define the coded H264 stream information.\n"
				"*/\n");
			printf("typedef struct mc_h264_h265_output_stream_info_t {\n");
			printf("    frame_index: %d\n", buffer_info->video_stream_info.frame_index);
			printf("    frame_start_addr: %lu\n", buffer_info->video_stream_info.frame_start_addr);
			printf("    frame_size: %d\n", buffer_info->video_stream_info.frame_size);
			printf("    nalu_type: %d\n", buffer_info->video_stream_info.nalu_type);
			printf("    slice_idx: %u\n", buffer_info->video_stream_info.slice_idx);
			printf("    slice_num: %u\n", buffer_info->video_stream_info.slice_num);
			printf("    dependent_slice_num: %u\n", buffer_info->video_stream_info.dependent_slice_num);
			printf("    independent_slice_num: %u\n", buffer_info->video_stream_info.independent_slice_num);
			printf("    pic_skipped: %u\n", buffer_info->video_stream_info.pic_skipped);
			printf("    intra_block_num: %u\n", buffer_info->video_stream_info.intra_block_num);
			printf("    skip_block_num: %u\n", buffer_info->video_stream_info.skip_block_num);
			printf("    avg_mb_qp: %u\n", buffer_info->video_stream_info.avg_mb_qp);
			printf("    enc_pic_byte: %u\n", buffer_info->video_stream_info.enc_pic_byte);
			printf("    enc_gop_pic_idx: %d\n", buffer_info->video_stream_info.enc_gop_pic_idx);
			printf("    enc_pic_poc: %d\n", buffer_info->video_stream_info.enc_pic_poc);
			printf("    enc_src_idx: %u\n", buffer_info->video_stream_info.enc_src_idx);
			printf("    enc_pic_cnt: %u\n", buffer_info->video_stream_info.enc_pic_cnt);
			printf("    enc_error_reason: %d\n", buffer_info->video_stream_info.enc_error_reason);
			printf("    enc_warn_info: %d\n", buffer_info->video_stream_info.enc_warn_info);
			printf("    frame_cycle: %u\n", buffer_info->video_stream_info.frame_cycle);
			printf("    temporal_id: %u\n", buffer_info->video_stream_info.temporal_id);
			printf("    longterm_ref_type: %u\n", buffer_info->video_stream_info.longterm_ref_type);
			printf("} mc_h264_h265_output_stream_info_t;\n");
			printf("/**\n"
				"* Define the information of video stream buffers(H264/H265/MJPEG/JPEG).\n"
				"*/\n");
			printf("typedef struct mc_video_stream_buffer_info_t {\n");
			printf("    vir_ptr: %p\n", frame_buffer->vstream_buf.vir_ptr);
			printf("    phy_ptr: %lu\n", frame_buffer->vstream_buf.phy_ptr);
			printf("    size: %u\n", frame_buffer->vstream_buf.size);
			printf("    pts: %lu\n", frame_buffer->vstream_buf.pts);
			printf("    fd: %d\n", frame_buffer->vstream_buf.fd);
			printf("    src_idx: %d\n", frame_buffer->vstream_buf.src_idx);
			printf("    stream_end: %d\n", frame_buffer->vstream_buf.stream_end);
			printf("} mc_video_stream_buffer_info_t;\n");
			break;
		case MC_AUDIO_FRAME_BUFFER:
			printf("Audio Frame Info:\n");
			// 打印 video_stream_info 的值
			// 注意：这里省略了打印 video_stream_info 的代码
			break;
		case MC_AUDIO_STREAM_BUFFER:
			printf("Audio Stream Info:\n");
			// 打印 video_stream_info 的值
			// 注意：这里省略了打印 video_stream_info 的代码
			break;
		default:
			printf("Unsupported Codec Buffer Type.\n");
			break;
	}
}


void fill_image_frame_from_vnode_image(ImageFrame *frame)
{
	if (!frame) return;

	// 从 vnode_image 填充 ImageFrame 的字段
	frame->frame_id = frame->vnode_image.info.frame_id;
	frame->image_timestamp = frame->vnode_image.info.timestamps;

	// 填充 width, height, stride, vstride 等字段
	frame->width = frame->vnode_image.buffer.width;
	frame->height = frame->vnode_image.buffer.height;
	frame->stride = frame->vnode_image.buffer.stride;
	frame->vstride = frame->vnode_image.buffer.vstride;
	frame->plane_count = frame->vnode_image.buffer.plane_cnt;

	for (int i = 0; i < frame->vnode_image.buffer.plane_cnt; ++i) {
		frame->pdata[i] = frame->vnode_image.buffer.phys_addr[i];
		frame->data[i] = frame->vnode_image.buffer.virt_addr[i];
		frame->data_size[i] = frame->vnode_image.buffer.size[i];
	}
}

void fill_vnode_image_from_image_frame(ImageFrame *frame)
{
	if (!frame) return;

	// 从 ImageFrame 填充到 vnode_image 的 info 字段
	frame->vnode_image.info.frame_id = frame->frame_id;
	frame->vnode_image.info.timestamps = frame->image_timestamp;

	// 填充 width, height, stride, vstride 等字段
	frame->vnode_image.buffer.width = frame->width;
	frame->vnode_image.buffer.height = frame->height;
	frame->vnode_image.buffer.stride = frame->stride;
	frame->vnode_image.buffer.vstride = frame->vstride;
	frame->vnode_image.buffer.plane_cnt = frame->plane_count;

	for (int i = 0; i < frame->plane_count; ++i) {
		frame->vnode_image.buffer.phys_addr[i] = frame->pdata[i];
		frame->vnode_image.buffer.virt_addr[i] = frame->data[i];
		frame->vnode_image.buffer.size[i] = frame->data_size[i];
	}
}
