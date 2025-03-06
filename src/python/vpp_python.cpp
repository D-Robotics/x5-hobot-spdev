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
 * D-Robotics
 * Copyright (C) 2024 D-Robotics Inc.
 */

#include <atomic>
#include <cstdbool>
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

#include <Python.h>

#include "utils_log.h"

#include "vpp_python.h"
#include "vpp_display.h"
#include "vpp_camera.h"
#include "vpp_codec.h"

using namespace spdev;
using namespace std;

#ifdef __cplusplus
extern "C"
{
#endif

	static void dump_frame_info(ImageFrame &frame)
	{
		SC_LOGE("frame_id: %ld, lost_image_num: %ld, image_timestamp: %ld,"
			" data_size: %u, exp_time: %ld\n",
			frame.frame_id, frame.lost_image_num, frame.image_timestamp,
			frame.data_size[0] + frame.data_size[1], frame.exp_time);
	}

	static int32_t __save_fame(ImageFrame &frame, string tail)
	{
		dump_frame_info(frame);
		string pic_name{
			"/userdata/image_" + to_string(frame.frame_id) + "_" +
			to_string(frame.image_timestamp) + "_" +
			to_string(frame.data_size[0] + frame.data_size[1]) + tail};

		ofstream ofs;
		ofs.open(pic_name, ofstream::binary);
		if (ofs.is_open())
		{
			ofs.write(reinterpret_cast<char *>(frame.data[0]), frame.data_size[0]);
			if (frame.plane_count > 1)
			{
				ofs.write(reinterpret_cast<char *>(frame.data[1]), frame.data_size[1]);
			}
			ofs.close();
		}

		return 0;
	}

	int save_raw_frame_data(ImageFrame &frame)
	{
		return __save_fame(frame, ".raw");
	}

	int save_yuv_frame_data(ImageFrame &frame)
	{
		return __save_fame(frame, ".yuv");
	}

	static int32_t py_obj_to_array(PyObject *obj, int32_t *array)
	{
		int32_t num = 0;

		if (obj == NULL)
		{
			num = 0;
		}
		else if (PyLong_Check(obj))
		{
			num = 1;
			array[0] = PyLong_AsLong(obj);
		}
		else if (PyList_Check(obj))
		{
			num = PyList_Size(obj);
			for (int i = 0; i < num; i++)
			{
				array[i] = PyLong_AsLong(PyList_GetItem(obj, i));
			}
		}

		return num;
	}

	static int32_t py_obj_to_size(PyObject *obj, int32_t *width, int32_t *height)
	{
		int32_t num = 0;

		if (obj == NULL)
		{
			num = 0;
		}
		else if (PyLong_Check(obj))
		{
			num = 0;
		}
		else if (PyList_Check(obj))
		{
			num = PyList_Size(obj);
			if (PyLong_Check(PyList_GetItem(obj, 0)))
			{
				if (PyList_Size(obj) < 2)
				{
					return -1;
				}
				num = 1;
				width[0] = PyLong_AsLong(PyList_GetItem(obj, 0));
				height[0] = PyLong_AsLong(PyList_GetItem(obj, 1));
			}
		}

		return num;
	}

	static int32_t py_obj_to_sizes(PyObject *obj, int32_t *width, int32_t *height)
	{
		int32_t num = 0;

		if (obj == NULL)
		{
			num = 0;
		}
		else if (PyLong_Check(obj))
		{
			num = 0;
		}
		else if (PyList_Check(obj))
		{
			num = PyList_Size(obj);
			if (PyLong_Check(PyList_GetItem(obj, 0)))
			{
				if (PyList_Size(obj) < 2)
				{
					return -1;
				}
				num = 1;
				width[0] = PyLong_AsLong(PyList_GetItem(obj, 0));
				height[0] = PyLong_AsLong(PyList_GetItem(obj, 1));
			}
			else if (PyList_Check(PyList_GetItem(obj, 0)))
			{
				for (int i = 0; i < num; i++)
				{
					if (PyList_Size(PyList_GetItem(obj, i)) < 2)
					{
						return -1;
					}
					width[i] = PyLong_AsLong(PyList_GetItem(PyList_GetItem(obj, i), 0));
					height[i] = PyLong_AsLong(PyList_GetItem(PyList_GetItem(obj, i), 1));
				}
			}
		}

		return num;
	}

	static int32_t py_obj_to_rect(PyObject *obj, int32_t *x, int32_t *y, int32_t *width, int32_t *height)
	{
		int32_t num = 0;

		if (obj == NULL)
		{
			num = 0;
		}
		else if (PyList_Check(obj))
		{
			num = PyList_Size(obj);
			if (PyLong_Check(PyList_GetItem(obj, 0)))
			{
				if (PyList_Size(obj) < 4)
				{
					return -1;
				}
				num = 1;
				x[0] = PyLong_AsLong(PyList_GetItem(obj, 0));
				y[0] = PyLong_AsLong(PyList_GetItem(obj, 1));
				width[0] = PyLong_AsLong(PyList_GetItem(obj, 2));
				height[0] = PyLong_AsLong(PyList_GetItem(obj, 3));
			}
			else if (PyList_Check(PyList_GetItem(obj, 0)))
			{
				for (int i = 0; i < num; i++)
				{
					if (PyList_Size(PyList_GetItem(obj, i)) < 4)
					{
						return -1;
					}
					x[i] = PyLong_AsLong(PyList_GetItem(PyList_GetItem(obj, i), 0));
					y[i] = PyLong_AsLong(PyList_GetItem(PyList_GetItem(obj, i), 1));
					width[i] = PyLong_AsLong(PyList_GetItem(PyList_GetItem(obj, i), 2));
					height[i] = PyLong_AsLong(PyList_GetItem(PyList_GetItem(obj, i), 3));
				}
			}
		}

		return num;
	}

	static PyObject *Camera_new(PyTypeObject *type, PyObject *args, PyObject *kw)
	{
		libsppydev_Object *self = (libsppydev_Object *)type->tp_alloc(type, 0);
		self->pobj = nullptr;
		self->pframe = nullptr;
		return (PyObject *)self;
	}

	static void Camera_dealloc(libsppydev_Object *self)
	{
		if (self->pobj)
		{
			delete (VPPCamera *)self->pobj;
			self->pobj = nullptr;
		}

		if (self->pframe)
		{
			delete (ImageFrame *)self->pframe;
			self->pframe = nullptr;
		}
		self->ob_base.ob_type->tp_free(self);
	}

	static int32_t Camera_init(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		if (self->pobj)
		{
			PyErr_SetString(PyExc_Exception, "__init__ already called");
			return -1;
		}
		self->pobj = new VPPCamera();
		self->pframe = new ImageFrame();
		self->object = VPP_CAMERA;

		return 0;
	}

	PyObject *Camera_open_cam(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		if (!(self->pobj && self->pframe)) {
			PyErr_SetString(PyExc_Exception, "camera not inited");
			return Py_BuildValue("i", -1);
		}

		int pipe_id, video_index, fps = 30, chn_num = 0;
		int raw_height = -1, raw_width = -1;
		int width[VSE_MAX_CHN_NUM], height[VSE_MAX_CHN_NUM];
		vp_sensors_parameters sensors_parameters;
		PyObject *width_obj = NULL, *height_obj = NULL, *size_obj = NULL;
		VPPCamera *cam = (VPPCamera *)self->pobj;
		static char *kwlist[] = {(char *)"pipe_id", (char *)"video_index",
			(char *)"fps", (char *)"width", (char *)"height",
			(char *)"raw_height",(char *)"raw_width",NULL};

		if (!PyArg_ParseTupleAndKeywords(args, kw, "ii|iOOii",
			kwlist, &pipe_id, &video_index, &fps,
			&width_obj, &height_obj, &raw_height, &raw_width))
			return Py_BuildValue("i", -1);
		sensors_parameters.fps = fps;
		sensors_parameters.raw_height = raw_height;
		sensors_parameters.raw_width = raw_width;
		if (size_obj != NULL) {
			chn_num = py_obj_to_size(size_obj, width, height);
		} else {
			chn_num = py_obj_to_array(width_obj, width);
			chn_num = py_obj_to_array(height_obj, height);
		}
		if (chn_num < 0) {
			PRINT("Invalid param\n");
			return Py_BuildValue("i", -1);
		}
		if (chn_num < (VSE_MAX_CHN_NUM - 1)) {
			// set 0 means default size
			width[chn_num] = 0;
			height[chn_num] = 0;
			chn_num++;
		}
		return Py_BuildValue("i",
			cam->OpenCamera(pipe_id, video_index, chn_num, width, height,
				&sensors_parameters));
	}

	PyObject *Camera_open_vps(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		if (!(self->pobj && self->pframe)) {
			PyErr_SetString(PyExc_Exception, "camera not inited");
			return Py_BuildValue("i", -1);
		}

		int pipe_id = 1, proc_mode = 0, src_width = 1920, src_height = 1080, chn_num = 0;
		int rotate[VSE_MAX_CHN_NUM] = {0};
		int dst_width[VSE_MAX_CHN_NUM] = {0}, dst_height[VSE_MAX_CHN_NUM] = {0};
		int crop_x[VSE_MAX_CHN_NUM] = {0}, crop_y[VSE_MAX_CHN_NUM] = {0};
		int crop_width[VSE_MAX_CHN_NUM] = {0}, crop_height[VSE_MAX_CHN_NUM] = {0};
		PyObject *dst_width_obj = NULL, *dst_height_obj = NULL, *rotate_obj = NULL;
		PyObject *crop_rect_obj = NULL, *src_size_obj = NULL, *dst_size_obj = NULL;
		VPPCamera *cam = (VPPCamera *)self->pobj;
		static char *kwlist[] = {(char *)"pipe_id", (char *)"process_mode",
			(char *)"src_width", (char *)"src_height",
			(char *)"dst_width", (char *)"dst_height",
			(char *)"crop_rect", (char *)"rotate",
			(char *)"src_size", (char *)"dst_size", NULL};

		if (!PyArg_ParseTupleAndKeywords(args, kw, "|iiiiOOOOOO", kwlist, &pipe_id, &proc_mode,
			&src_width, &src_height, &dst_width_obj, &dst_height_obj,
			&crop_rect_obj, &rotate_obj, &src_size_obj, &dst_size_obj))
			return Py_BuildValue("i", -1);

		if (proc_mode < 0) {
			PRINT("Invalid param\n");
			return Py_BuildValue("i", -1);
		}

		if (src_size_obj != NULL) {
			py_obj_to_size(src_size_obj, &src_width, &src_height);
		}
		if (dst_size_obj != NULL) {
			chn_num = py_obj_to_size(dst_size_obj, dst_width, dst_height);
		} else {
			chn_num = py_obj_to_array(dst_width_obj, dst_width);
			chn_num = py_obj_to_array(dst_height_obj, dst_height);
		}
		if (chn_num < (VSE_MAX_CHN_NUM - 1)) {
			// set 0 means default size
			dst_width[chn_num] = 0;
			dst_height[chn_num] = 0;
			chn_num++;
		}
		if ((proc_mode == VPS_SCALE_CROP) || (proc_mode == VPS_SCALE_ROTATE_CROP)) {
			py_obj_to_rect(crop_rect_obj, crop_x, crop_y, crop_width, crop_height);
		}
		if ((proc_mode == VPS_SCALE_ROTATE) || (proc_mode == VPS_SCALE_ROTATE_CROP)) {
			py_obj_to_array(rotate_obj, rotate);
		}

		if (chn_num < 0) {
			PRINT("Invalid param\n");
			return Py_BuildValue("i", -1);
		}

		return Py_BuildValue("i", cam->OpenVPS(pipe_id, chn_num, proc_mode, src_width, src_height,
			dst_width, dst_height, crop_x, crop_y, crop_width, crop_height, rotate));
	}

	PyObject *Camera_close_cam(libsppydev_Object *self)
	{
		if (!(self->pobj && self->pframe))
		{
			PyErr_SetString(PyExc_Exception, "camera not inited");
			Py_RETURN_NONE;
		}

		VPPCamera *cam = (VPPCamera *)self->pobj;

		cam->Close();

		Py_RETURN_NONE;
	}

	PyObject *Camera_get_img(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		if (!(self->pobj && self->pframe))
		{
			PyErr_SetString(PyExc_Exception, "camera not inited");
			Py_RETURN_NONE;
		}

		DevModule module = SP_DEV_VPS;
		int32_t width = 0, height = 0;
		VPPCamera *cam = (VPPCamera *)self->pobj;
		PyObject *img_obj = nullptr, *uv_obj = nullptr;
		static char *kwlist[] = {(char *)"module", (char *)"width", (char *)"height", NULL};

		if (!PyArg_ParseTupleAndKeywords(args, kw, "|iii", kwlist, &module, &width, &height))
			Py_RETURN_NONE;

		if (!cam->GetImageFrame(self->pframe, module, width, height, 2000))
		{
			img_obj = PyBytes_FromStringAndSize((const char *)self->pframe->data[0],
												self->pframe->data_size[0]);

			if (self->pframe->plane_count == 2)
			{
				uv_obj = PyBytes_FromStringAndSize((const char *)self->pframe->data[1],
												   self->pframe->data_size[1]);
				PyBytes_ConcatAndDel(&img_obj, uv_obj);
			}

			cam->ReturnImageFrame(self->pframe, module, width, height);

			return img_obj;
		}

		Py_RETURN_NONE;
	}

	PyObject *Camera_set_img(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		if (!(self->pobj && self->pframe))
		{
			PyErr_SetString(PyExc_Exception, "camera not inited");
			Py_RETURN_NONE;
		}

		VPPCamera *cam = (VPPCamera *)self->pobj;
		PyObject *img_obj = nullptr;
		char *kwlist[] = {(char *)"img_obj", NULL};
		int32_t size = 0;
		uint8_t *addr = NULL;

		if (!PyArg_ParseTupleAndKeywords(args, kw, "O", kwlist, &img_obj))
			Py_RETURN_NONE;

		self->pframe->width = cam->GetModuleWidth();
		self->pframe->height = cam->GetModuleHeight();
		self->pframe->stride = cam->GetModuleWidth();

		addr = (uint8_t *)PyBytes_AsString(img_obj);
		size = PyBytes_Size(img_obj);
		if (size < (self->pframe->stride * self->pframe->height * 3 / 2))
		{
			PyErr_SetString(PyExc_Exception, "camera frame size less than nv12 format size\n");
			return Py_BuildValue("i", -1);
		}

		self->pframe->data[0] = addr;
		self->pframe->data_size[0] = self->pframe->stride * self->pframe->height;
		self->pframe->data[1] = addr + self->pframe->data_size[0];
		self->pframe->data_size[1] = self->pframe->data_size[0] / 2;
		self->pframe->plane_count = 2;

		return Py_BuildValue("i", cam->SetImageFrame(self->pframe));
	}

	/// encode related

	static PyObject *Encoder_new(PyTypeObject *type, PyObject *args, PyObject *kw)
	{
		libsppydev_Object *self = (libsppydev_Object *)type->tp_alloc(type, 0);
		self->pobj = nullptr;
		self->pframe = nullptr;
		return (PyObject *)self;
	}

	static void Encoder_dealloc(libsppydev_Object *self)
	{
		if (self->pobj)
		{
			delete static_cast<VPPEncode *>(self->pobj);
			self->pobj = nullptr;
		}
		if (self->pframe)
		{
			delete static_cast<ImageFrame *>(self->pframe);
			self->pframe = nullptr;
		}

		self->ob_base.ob_type->tp_free(self);
	}

	static int32_t Encoder_init(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		if (self->pobj)
		{
			PyErr_SetString(PyExc_Exception, "__init__ already called");
			return -1;
		}
		self->pobj = static_cast<void *>(new VPPEncode());
		self->pframe = new ImageFrame();
		self->object = VPP_ENCODE;

		return 0;
	}

	static PyObject *Encoder_encode(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		if (!self->pobj)
		{
			PyErr_SetString(PyExc_Exception, "encoder not inited");
			return Py_BuildValue("i", -1);
		}

		int chn = 0, type = 0, width = 0, height = 0, bits = 8000;
		static char *kwlist[] = {(char *)"video_chn", (char *)"type",
			(char *)"width", (char *)"height", (char *)"bits", NULL};

		if (!PyArg_ParseTupleAndKeywords(args, kw, "iiii|i", kwlist, &chn, &type, &width, &height, &bits)) {
			return Py_BuildValue("i", -1);
		}

		VPPEncode *pobj = static_cast<VPPEncode *>(self->pobj);

			return Py_BuildValue("i", pobj->OpenEncode(type, width, height, bits));
	}

	static PyObject *Encoder_send_frame(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		if (!self->pobj)
		{
			PyErr_SetString(PyExc_Exception, "encoder not inited");
			return Py_BuildValue("i", -1);
		}

		PyObject *img_obj = nullptr;
		VPPEncode *pobj = static_cast<VPPEncode *>(self->pobj);
		char *kwlist[] = {(char *)"img", NULL};

		if (!PyArg_ParseTupleAndKeywords(args, kw, "O", kwlist, &img_obj))
		{
			return Py_BuildValue("i", -1);
		}

		self->pframe->width = pobj->GetModuleWidth();
		self->pframe->height = pobj->GetModuleHeight();
		self->pframe->stride = pobj->GetModuleWidth();

		if (PyBytes_Size(img_obj) < (self->pframe->stride * self->pframe->height * 3 / 2))
		{
			SC_LOGE("encode send frame size:%d less than nv12 format size of %dx%d\n",
					   PyBytes_Size(img_obj), self->pframe->stride, self->pframe->height);
			return Py_BuildValue("i", -1);
		}

		self->pframe->data[0] = (uint8_t *)PyBytes_AsString(img_obj);
		self->pframe->data_size[0] = self->pframe->stride * self->pframe->height;
		self->pframe->data[1] = self->pframe->data[0] + self->pframe->data_size[0];
		self->pframe->data_size[1] = self->pframe->data_size[0] / 2;
		self->pframe->plane_count = 2;

		return Py_BuildValue("i", pobj->SetImageFrame(self->pframe));
	}

	static PyObject *Encoder_get_frame(libsppydev_Object *self)
	{
		if (!self->pobj)
		{
			PyErr_SetString(PyExc_Exception, "encoder not inited");
			Py_RETURN_NONE;
		}

		VPPEncode *pobj = static_cast<VPPEncode *>(self->pobj);
		PyObject *img_obj = nullptr;
		int32_t ret = 0;

		ret = pobj->GetImageFrame(self->pframe);
		if ((ret == 0) && (self->pframe != nullptr))
		{
			img_obj = PyBytes_FromStringAndSize((char *)self->pframe->data[0],
												self->pframe->data_size[0]);

			pobj->ReturnImageFrame(self->pframe);

			return img_obj;
		}

		Py_RETURN_NONE;
	}

	static PyObject *Encoder_close(libsppydev_Object *self)
	{
		if (!self->pobj)
		{
			PyErr_SetString(PyExc_Exception, "encoder not inited");
			return Py_BuildValue("i", -1);
		}

		VPPEncode *pobj = static_cast<VPPEncode *>(self->pobj);

		return Py_BuildValue("i", pobj->Close());
	}

	/// Decode related

	static PyObject *Decoder_new(PyTypeObject *type, PyObject *args, PyObject *kw)
	{
		libsppydev_Object *self = (libsppydev_Object *)type->tp_alloc(type, 0);
		self->pobj = nullptr;
		self->pframe = nullptr;
		return (PyObject *)self;
	}

	static void Decoder_dealloc(libsppydev_Object *self)
	{
		if (self->pobj)
		{
			delete static_cast<VPPDecode *>(self->pobj);
			self->pobj = nullptr;
		}
		if (self->pframe)
		{
			delete static_cast<ImageFrame *>(self->pframe);
			self->pframe = nullptr;
		}

		self->ob_base.ob_type->tp_free(self);
	}

	static int32_t Decoder_init(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		if (self->pobj)
		{
			PyErr_SetString(PyExc_Exception, "__init__ already called");
			return -1;
		}

		self->pobj = static_cast<void *>(new VPPDecode());
		self->pframe = new ImageFrame();
		self->object = VPP_DECODE;

		return 0;
	}

	static PyObject *Decoder_decode(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		if (!self->pobj)
		{
			PyErr_SetString(PyExc_Exception, "decoder not inited");
			return Py_BuildValue("i", -1);
		}

		/// Use the static args, for optional args using
		static char *string = nullptr;
		static int video_chn = 0, type = 1, width = 1920, height = 1080;
		int frame_count = 0;
		int ret = 0;
		PyObject *list_obj = nullptr;
		static char *kwlist[] = {(char *)"file", (char *)"video_chn", (char *)"type",
			(char *)"width", (char *)"height", NULL};

		if (!PyArg_ParseTupleAndKeywords(args, kw, "s|iiii", kwlist,
							&string, &video_chn, &type, &width, &height)) {
			return Py_BuildValue("i", -1);
		}

		VPPDecode *pobj = static_cast<VPPDecode *>(self->pobj);

		ret = pobj->OpenDecode(type, width, height, string, &frame_count);

		list_obj = PyList_New(0);
		PyList_Append(list_obj, Py_BuildValue("i", ret));
		PyList_Append(list_obj, Py_BuildValue("i", frame_count));

		return list_obj;
	}

	static PyObject *Decoder_get_frame(libsppydev_Object *self)
	{
		if (!self->pobj)
		{
			PyErr_SetString(PyExc_Exception, "decoder not inited");
			return Py_BuildValue("i", -1);
		}

		VPPDecode *pobj = static_cast<VPPDecode *>(self->pobj);
		PyObject *img_obj = nullptr, *uv_obj = nullptr;
		int32_t ret = 0;

		ret = pobj->GetImageFrame(self->pframe);
		if ((ret == 0) && (self->pframe != nullptr))
		{
			img_obj = PyBytes_FromStringAndSize((const char *)self->pframe->data[0],
												self->pframe->data_size[0]);
			if (self->pframe->plane_count == 2)
			{
				uv_obj = PyBytes_FromStringAndSize((const char *)self->pframe->data[1],
												   self->pframe->data_size[1]);
				PyBytes_ConcatAndDel(&img_obj, uv_obj);
			}

			pobj->ReturnImageFrame(self->pframe);

			return img_obj;
		}

		Py_RETURN_NONE;
	}

	static PyObject *Decoder_send_frame(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		PyObject *img_obj = nullptr;
		VPPDecode *pobj = (VPPDecode *)self->pobj;
		int chn = -1;
		int eos = 0;
		static char *kwlist[] = {(char *)"img", (char *)"chn", (char *)"eos", NULL};

		if (!self->pobj)
		{
			PyErr_SetString(PyExc_Exception, "decode not inited");
			return Py_BuildValue("i", -1);
		}
		if (!PyArg_ParseTupleAndKeywords(args, kw, "O|ii", kwlist, &img_obj, &chn, &eos)) {
			return Py_BuildValue("i", -1);
		}

		self->pframe->data[0] = (uint8_t *)PyBytes_AsString(img_obj);
		self->pframe->data_size[0] = PyBytes_Size(img_obj);
		self->pframe->plane_count = 1;

		return Py_BuildValue("i", (pobj->SetImageFrame(self->pframe)));
	}

	static PyObject *Decoder_close(libsppydev_Object *self)
	{
		if (!self->pobj)
		{
			PyErr_SetString(PyExc_Exception, "decoder not inited");
			return Py_BuildValue("i", -1);
		}

		VPPDecode *pobj = static_cast<VPPDecode *>(self->pobj);

		return Py_BuildValue("i", pobj->Close());
	}

	static PyObject *Display_new(PyTypeObject *type, PyObject *args, PyObject *kw)
	{
		libsppydev_Object *self = (libsppydev_Object *)type->tp_alloc(type, 0);
		self->pobj = nullptr;
		self->pframe = nullptr;
		return (PyObject *)self;
	}

	static void Display_dealloc(libsppydev_Object *self)
	{
		if (self->pobj)
		{
			delete static_cast<VPPDisplay *>(self->pobj);
			self->pobj = nullptr;
		}
		if (self->pframe)
		{
			delete static_cast<ImageFrame *>(self->pframe);
			self->pobj = nullptr;
		}

		self->ob_base.ob_type->tp_free(self);
	}

	static int32_t Display_init(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		if (self->pobj)
		{
			PyErr_SetString(PyExc_Exception, "__init__ already called");
			return -1;
		}

		self->pobj = static_cast<void *>(new VPPDisplay());
		self->pframe = new ImageFrame();

		self->object = VPP_DISPLAY;

		return 0;
	}

	static PyObject *Display_display(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		int width = 1920, height = 1080, vot_chn = 0, chn_width = 0, chn_height = 0;
		int vot_intf = 0, vot_out_mode = 0;
		static char *kwlist[] = {(char *)"chn", (char *)"width", (char *)"height",
			(char *)"vot_intf", (char *)"vot_out_mode",
			(char *)"chn_width", (char *)"chn_height", NULL};

		if (!self->pobj) {
			PyErr_SetString(PyExc_Exception, "display not inited");
			return Py_BuildValue("i", -1);
		}
		if (!PyArg_ParseTupleAndKeywords(args, kw, "|iiiiiii", kwlist, &vot_chn, &width, &height, &vot_intf, &vot_out_mode, &chn_width, &chn_height)) {
			return Py_BuildValue("i", -1);
		}

		if (vot_chn != 0)
			return Py_BuildValue("i", 0);

		if ((chn_width == 0) || (chn_height == 0)) {
			chn_width = width;
			chn_height = height;
		}

		return Py_BuildValue("i", ((VPPDisplay *)(self->pobj))->OpenDisplay(width, height));
	}

	static PyObject *Display_set_img(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		PyObject *img_obj = nullptr;
		VPPDisplay *pobj = (VPPDisplay *)self->pobj;
		int chn = 0;
		static char *kwlist[] = {(char *)"img", (char *)"chn", NULL};

		if (!self->pobj) {
			PyErr_SetString(PyExc_Exception, "display not inited");
			return Py_BuildValue("i", -1);
		}
		if (!PyArg_ParseTupleAndKeywords(args, kw, "O|i", kwlist, &img_obj, &chn)) {
			return Py_BuildValue("i", -1);
		}

		self->pframe->data[0] = (uint8_t *)PyBytes_AsString(img_obj);
		self->pframe->data_size[0] = PyBytes_Size(img_obj);
		self->pframe->plane_count = 1;

		return Py_BuildValue("i", pobj->SetImageFrame(self->pframe));
	}

	static PyObject *Display_set_graph_rect(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		VPPDisplay *pobj = (VPPDisplay *)self->pobj;
		int x0, y0, x1, y1, chn = 2, flush = 0, line_width = 4;
		uint64_t color = 0xffff0000;
		static char *kwlist[] = {(char *)"x0", (char *)"y0", (char *)"x1", (char *)"y1",
			(char *)"chn", (char *)"flush", (char *)"color", (char *)"line_width", NULL};

		if (!self->pobj)
		{
			PyErr_SetString(PyExc_Exception, "display not inited");
			return Py_BuildValue("i", -1);
		}
		if (!PyArg_ParseTupleAndKeywords(args, kw, "iiii|iili", kwlist, &x0, &y0, &x1, &y1, &chn, &flush, &color, &line_width))
		{
			return Py_BuildValue("i", -1);
		}

		return Py_BuildValue("i", pobj->SetGraphRect(x0, y0, x1, y1, flush, (uint32_t)color, line_width));
	}

	static PyObject *Display_set_graph_word(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		VPPDisplay *pobj = (VPPDisplay *)self->pobj;
		int x, y, chn = 2, flush = 0, line_width = 1;
		uint64_t color = 0xffff0000;
		PyObject *str_obj = nullptr;
		static char *kwlist[] = {(char *)"x", (char *)"y", (char *)"str",
			(char *)"chn", (char *)"flush", (char *)"color", (char *)"line_width", NULL};

		if (!self->pobj)
		{
			PyErr_SetString(PyExc_Exception, "display not inited");
			return Py_BuildValue("i", -1);
		}
		if (!PyArg_ParseTupleAndKeywords(args, kw, "iiO|iili", kwlist, &x, &y, &str_obj, &chn, &flush, &color, &line_width))
		{
			return Py_BuildValue("i", -1);
		}

		return Py_BuildValue("i", pobj->SetGraphWord(x, y, PyBytes_AsString(str_obj), flush, (uint32_t)color, line_width));
	}

	static PyObject *Display_get_display_res(libsppydev_Object *self)
	{
		VPPDisplay *pobj = (VPPDisplay *)self->pobj;

		//列表的大小暂时由这里定义，维护的时候根据具体情况更改。
		int disp_w_list[20] = {0};
		int disp_h_list[20] = {0};

		pobj->GetDisplayRes(disp_w_list , disp_h_list);
		PyObject* list = PyList_New(0);
		for (int i = 0; i < sizeof(disp_w_list) / sizeof(disp_w_list[0]); i++) {
			PyObject* tuple = Py_BuildValue("(ii)", disp_w_list[i], disp_h_list[i]);
			PyList_Append(list, tuple);
			Py_DECREF(tuple);
		}
		return list;
	}

	static PyObject *Display_close(libsppydev_Object *self)
	{
		if (!self->pobj)
		{
			PyErr_SetString(PyExc_Exception, "display not inited");
			return Py_BuildValue("i", -1);
		}

		return Py_BuildValue("i", ((VPPDisplay *)self->pobj)->Close());
	}

	static PyObject *Module_bind(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		libsppydev_Object *src_obj = nullptr, *dst_obj = nullptr;
		VPPModule *src_mod = nullptr, *dst_mod = nullptr;
		char *kwlist[] = {(char *)"src", (char *)"dst", NULL};

		if (!PyArg_ParseTupleAndKeywords(args, kw, "OO", kwlist, &src_obj, &dst_obj))
		{
			return Py_BuildValue("i", -1);
		}

		src_mod = (VPPModule *)src_obj->pobj;
		dst_mod = (VPPModule *)dst_obj->pobj;

		return Py_BuildValue("i", dst_mod->BindTo(src_mod));
	}

	static PyObject *Module_unbind(libsppydev_Object *self, PyObject *args, PyObject *kw)
	{
		libsppydev_Object *src_obj = nullptr, *dst_obj = nullptr;
		VPPModule *src_mod = nullptr, *dst_mod = nullptr;
		char *kwlist[] = {(char *)"src", (char *)"dst", NULL};

		if (!PyArg_ParseTupleAndKeywords(args, kw, "OO", kwlist, &src_obj, &dst_obj))
		{
			return Py_BuildValue("i", -1);
		}

		src_mod = (VPPModule *)src_obj->pobj;
		dst_mod = (VPPModule *)dst_obj->pobj;

		return Py_BuildValue("i", dst_mod->UnBind(src_mod));
	}

#define M_DOC_STRING         \
	"bind(module, module)\n" \
	"unbind(module, module)\n"

	static const char *__g_m_doc_str = M_DOC_STRING;

	static struct PyMethodDef Camera_Methods[] = {
		{"open_cam", (PyCFunction)Camera_open_cam, METH_VARARGS | METH_KEYWORDS, "Open camera and start video stream"},
		{"open_vps", (PyCFunction)Camera_open_vps, METH_VARARGS | METH_KEYWORDS, "Open vps process"},
		{"close_cam", (PyCFunction)Camera_close_cam, METH_NOARGS, "Stop video stream and close camera"},
		{"get_img", (PyCFunction)Camera_get_img, METH_VARARGS | METH_KEYWORDS, "Get image from the channel"},
		{"set_img", (PyCFunction)Camera_set_img, METH_VARARGS | METH_KEYWORDS, "Set image to the vps"},
		{nullptr, nullptr, 0, nullptr},
	};

	static PyTypeObject libsppydev_CameraType = {
		PyVarObject_HEAD_INIT(&libsppydev_CameraType, 0) /* ob_size */
		"libsppydev.Camera",                             /* tp_name */
		sizeof(libsppydev_Object),                       /* tp_basicsize */
		0,                                               /* tp_itemsize */
		(destructor)Camera_dealloc,                      /* tp_dealloc */
		0,                                               /* tp_print */
		0,                                               /* tp_getattr */
		0,                                               /* tp_setattr */
		0,                                               /* tp_compare */
		0,                                               /* tp_repr */
		0,                                               /* tp_as_number */
		0,                                               /* tp_as_sequence */
		0,                                               /* tp_as_mapping */
		0,                                               /* tp_hash */
		0,                                               /* tp_call */
		0,                                               /* tp_str */
		0,                                               /* tp_getattro */
		0,                                               /* tp_setattro */
		0,                                               /* tp_as_buffer */
		Py_TPFLAGS_DEFAULT,                              /* tp_flags */
		"Camera object.",                                /* tp_doc */
		0,                                               /* tp_traverse */
		0,                                               /* tp_clear */
		0,                                               /* tp_richcompare */
		0,                                               /* tp_weaklistoffset */
		0,                                               /* tp_iter */
		0,                                               /* tp_iternext */
		Camera_Methods,                                  /* tp_methods */
		0,                                               /* tp_members */
		0,                                               /* tp_getset */
		0,                                               /* tp_base */
		0,                                               /* tp_dict */
		0,                                               /* tp_descr_get */
		0,                                               /* tp_descr_set */
		0,                                               /* tp_dictoffset */
		(initproc)Camera_init,                           /* tp_init */
		0,                                               /* tp_alloc */
		(newfunc)Camera_new,                             /* tp_new */
		0,                                               /* tp_free */
	};

	static PyMethodDef Encoder_methods[] = {
		{"encode", (PyCFunction)Encoder_encode, METH_VARARGS | METH_KEYWORDS, "Start encoder"},
		{"close", (PyCFunction)Encoder_close, METH_NOARGS, "Closes encoder."},
		{"send_frame", (PyCFunction)Encoder_send_frame, METH_VARARGS | METH_KEYWORDS, "Send frame to encoder"},
		{"encode_file", (PyCFunction)Encoder_send_frame, METH_VARARGS | METH_KEYWORDS, "Start encoder file"},
		{"get_frame", (PyCFunction)Encoder_get_frame, METH_NOARGS, "Get stream from encoder."},
		{"get_img", (PyCFunction)Encoder_get_frame, METH_NOARGS, "Get stream from encoder."},
		{nullptr, nullptr, 0, nullptr},
	};

	static PyTypeObject libsppydev_EncoderType = {
		PyVarObject_HEAD_INIT(&libsppydev_EncoderType, 0) /* ob_size */
		"libsppydev.Encoder",                             /* tp_name */
		sizeof(libsppydev_Object),                        /* tp_basicsize */
		0,                                                /* tp_itemsize */
		(destructor)Encoder_dealloc,                      /* tp_dealloc */
		0,                                                /* tp_print */
		0,                                                /* tp_getattr */
		0,                                                /* tp_setattr */
		0,                                                /* tp_compare */
		0,                                                /* tp_repr */
		0,                                                /* tp_as_number */
		0,                                                /* tp_as_sequence */
		0,                                                /* tp_as_mapping */
		0,                                                /* tp_hash */
		0,                                                /* tp_call */
		0,                                                /* tp_str */
		0,                                                /* tp_getattro */
		0,                                                /* tp_setattro */
		0,                                                /* tp_as_buffer */
		Py_TPFLAGS_DEFAULT,                               /* tp_flags */
		"Encoder object.",                                /* tp_doc */
		0,                                                /* tp_traverse */
		0,                                                /* tp_clear */
		0,                                                /* tp_richcompare */
		0,                                                /* tp_weaklistoffset */
		0,                                                /* tp_iter */
		0,                                                /* tp_iternext */
		Encoder_methods,                                  /* tp_methods */
		0,                                                /* tp_members */
		0,                                                /* tp_getset */
		0,                                                /* tp_base */
		0,                                                /* tp_dict */
		0,                                                /* tp_descr_get */
		0,                                                /* tp_descr_set */
		0,                                                /* tp_dictoffset */
		(initproc)Encoder_init,                           /* tp_init */
		0,                                                /* tp_alloc */
		(newfunc)Encoder_new,                             /* tp_new */
		0,                                                /* tp_free */
	};

	static PyMethodDef Decoder_methods[] = {
		{"decode", (PyCFunction)Decoder_decode, METH_VARARGS | METH_KEYWORDS, "Start decoder"},
		{"close", (PyCFunction)Decoder_close, METH_NOARGS, "Closes decoder."},
		{"get_img", (PyCFunction)Decoder_get_frame, METH_NOARGS, "Get image from decoder."},
		{"set_img", (PyCFunction)Decoder_send_frame, METH_VARARGS | METH_KEYWORDS, "Set buffer to decoder."},
		{"get_frame", (PyCFunction)Decoder_get_frame, METH_VARARGS | METH_KEYWORDS, "Set buffer to decoder."},
		{"send_frame", (PyCFunction)Decoder_send_frame, METH_VARARGS | METH_KEYWORDS, "Set buffer to decoder."},
		{nullptr, nullptr, 0, nullptr},
	};

	static PyTypeObject libsppydev_DecoderType = {
		PyVarObject_HEAD_INIT(&libsppydev_DecoderType, 0) /* ob_size */
		"libsppydev.Decoder",                             /* tp_name */
		sizeof(libsppydev_Object),                        /* tp_basicsize */
		0,                                                /* tp_itemsize */
		(destructor)Decoder_dealloc,                      /* tp_dealloc */
		0,                                                /* tp_print */
		0,                                                /* tp_getattr */
		0,                                                /* tp_setattr */
		0,                                                /* tp_compare */
		0,                                                /* tp_repr */
		0,                                                /* tp_as_number */
		0,                                                /* tp_as_sequence */
		0,                                                /* tp_as_mapping */
		0,                                                /* tp_hash */
		0,                                                /* tp_call */
		0,                                                /* tp_str */
		0,                                                /* tp_getattro */
		0,                                                /* tp_setattro */
		0,                                                /* tp_as_buffer */
		Py_TPFLAGS_DEFAULT,                               /* tp_flags */
		"Decoder object.",                                /* tp_doc */
		0,                                                /* tp_traverse */
		0,                                                /* tp_clear */
		0,                                                /* tp_richcompare */
		0,                                                /* tp_weaklistoffset */
		0,                                                /* tp_iter */
		0,                                                /* tp_iternext */
		Decoder_methods,                                  /* tp_methods */
		0,                                                /* tp_members */
		0,                                                /* tp_getset */
		0,                                                /* tp_base */
		0,                                                /* tp_dict */
		0,                                                /* tp_descr_get */
		0,                                                /* tp_descr_set */
		0,                                                /* tp_dictoffset */
		(initproc)Decoder_init,                           /* tp_init */
		0,                                                /* tp_alloc */
		(newfunc)Decoder_new,                             /* tp_new */
		0,                                                /* tp_free */
	};

	static PyMethodDef Display_methods[] = {
		{"display", (PyCFunction)Display_display, METH_VARARGS | METH_KEYWORDS, "Init display"},
		{"set_img", (PyCFunction)Display_set_img, METH_VARARGS | METH_KEYWORDS, "Set display image"},
		{"set_graph_rect", (PyCFunction)Display_set_graph_rect, METH_VARARGS | METH_KEYWORDS, "Set display grapth rect"},
		{"set_graph_word", (PyCFunction)Display_set_graph_word, METH_VARARGS | METH_KEYWORDS, "Set display grapth word"},
		{"get_display_res", (PyCFunction)Display_get_display_res, METH_NOARGS, "Get display res"},
		{"close", (PyCFunction)Display_close, METH_NOARGS, "Closes Display."},
		{nullptr, nullptr, 0, nullptr},
	};

	static PyTypeObject libsppydev_DisplayType = {
		PyVarObject_HEAD_INIT(&libsppydev_DisplayType, 0) /* ob_size */
		"libsppydev.Display",                             /* tp_name */
		sizeof(libsppydev_Object),                        /* tp_basicsize */
		0,                                                /* tp_itemsize */
		(destructor)Display_dealloc,                      /* tp_dealloc */
		0,                                                /* tp_print */
		0,                                                /* tp_getattr */
		0,                                                /* tp_setattr */
		0,                                                /* tp_compare */
		0,                                                /* tp_repr */
		0,                                                /* tp_as_number */
		0,                                                /* tp_as_sequence */
		0,                                                /* tp_as_mapping */
		0,                                                /* tp_hash */
		0,                                                /* tp_call */
		0,                                                /* tp_str */
		0,                                                /* tp_getattro */
		0,                                                /* tp_setattro */
		0,                                                /* tp_as_buffer */
		Py_TPFLAGS_DEFAULT,                               /* tp_flags */
		"Display object.",                                /* tp_doc */
		0,                                                /* tp_traverse */
		0,                                                /* tp_clear */
		0,                                                /* tp_richcompare */
		0,                                                /* tp_weaklistoffset */
		0,                                                /* tp_iter */
		0,                                                /* tp_iternext */
		Display_methods,                                  /* tp_methods */
		0,                                                /* tp_members */
		0,                                                /* tp_getset */
		0,                                                /* tp_base */
		0,                                                /* tp_dict */
		0,                                                /* tp_descr_get */
		0,                                                /* tp_descr_set */
		0,                                                /* tp_dictoffset */
		(initproc)Display_init,                           /* tp_init */
		0,                                                /* tp_alloc */
		(newfunc)Display_new,                             /* tp_new */
		0,                                                /* tp_free */
	};

	static PyMethodDef libsppydev_methods[] = {
		{"bind", (PyCFunction)Module_bind, METH_VARARGS | METH_KEYWORDS, "Bind two module."},
		{"unbind", (PyCFunction)Module_unbind, METH_VARARGS | METH_KEYWORDS, "Unbind two module."},
		{nullptr, nullptr, 0, nullptr},
	};

	/// wrap a module structure
	static struct PyModuleDef libsppydev = {
		PyModuleDef_HEAD_INIT,
		"libsppydev",  /* name of module */
		__g_m_doc_str, /* module documentation, may be nullptr */
		-1,            /* size of per-interpreter state of the module, or -1 if the module keeps state in global variables. */
		libsppydev_methods,
	};


	static PyObject* init_module(PyObject* m) {
		if (m == NULL) {
			return NULL;
		}

		PyVarObject ob_base = {1, &PyType_Type, 0};
		libsppydev_CameraType.ob_base = ob_base;
		libsppydev_EncoderType.ob_base = ob_base;
		libsppydev_DecoderType.ob_base = ob_base;
		libsppydev_DisplayType.ob_base = ob_base;

		if (PyType_Ready(&libsppydev_CameraType) < 0)
		{
			return nullptr;
		}

		if (PyType_Ready(&libsppydev_EncoderType) < 0)
		{
			return nullptr;
		}

		if (PyType_Ready(&libsppydev_DecoderType) < 0)
		{
			return nullptr;
		}

		if (PyType_Ready(&libsppydev_DisplayType) < 0)
		{
			return nullptr;
		}

		Py_INCREF(&libsppydev_CameraType);
		Py_INCREF(&libsppydev_EncoderType);
		Py_INCREF(&libsppydev_DecoderType);
		Py_INCREF(&libsppydev_DisplayType);

		PyModule_AddObject(m, "Camera", (PyObject *)&libsppydev_CameraType);
		PyModule_AddObject(m, "Encoder", (PyObject *)&libsppydev_EncoderType);
		PyModule_AddObject(m, "Decoder", (PyObject *)&libsppydev_DecoderType);
		PyModule_AddObject(m, "Display", (PyObject *)&libsppydev_DisplayType);

		return m;
	}

	/// init the module is a py module
	PyMODINIT_FUNC PyInit_libsppydev(void)
	{
		PyObject *m;
		m = PyModule_Create(&libsppydev);
		return init_module(m);
	}

	PyMODINIT_FUNC PyInit_libsrcampy(void)
	{
		PyObject *m;
		m = PyModule_Create(&libsppydev);
		return init_module(m);
	}

#ifdef __cplusplus
}
#endif /* extern "C" */
