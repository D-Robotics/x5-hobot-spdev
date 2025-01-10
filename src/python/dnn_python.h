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

// Copyright (c) 2024 D-Robotics.All Rights Reserved.

#ifndef _PYTHON_DNN_INFERENCE_H_
#define _PYTHON_DNN_INFERENCE_H_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "dnn/hb_dnn.h"
#include "dnn/hb_sys.h"
#include "dnn/hb_dnn_ext.h"

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <structmember.h>
#include <object.h>
#include <numpy/arrayobject.h>

#ifdef __cplusplus
extern "C" {
#endif

extern std::string TensorType[];
extern std::map<int, std::string> Layout;

#define ALIGN_16(v) ((v + (16 - 1)) / 16 * 16)

void HB_CHECK_SUCCESS(int32_t value, char *errmsg)
{
    /*value can be call of function*/
    int32_t ret_code = value;
    if (ret_code != 0)
    {
        printf("[BPU MSG] %s, return code:%d\n", errmsg, ret_code);
    }
}

// statement may involves function calls
#define RETURN_IF_FAILED(statement) \
    do {                            \
    auto code_ = (statement);       \
    if ((code_) != 0) {             \
        return code_;               \
    }                               \
    } while (0)


// C 结构体模拟 hbDNNTensorProperties 类
typedef struct {
    PyObject_HEAD;
    int32_t tensor_type;
    int32_t dtype;
    int32_t layout;
    int32_t shape_size;
    int32_t* shape;
    hbDNNTensorShape * aligned_shape;
    hbDNNTensorShape * valid_shape;
    int32_t scale_len;
    float* scale_data;
} TensorProperties;

// 定义 pyDNNTensor 类型
typedef struct {
    PyObject_HEAD;
    hbDNNTensorProperties properties;
    void *buffer;
    char name[64];           // 名称
} PyDNNTensor;

typedef struct {
    PyObject_HEAD;
    char name[128];
    hbPackedDNNHandle_t m_packed_dnn_handle;
    hbDNNHandle_t m_dnn_handle;
    int32_t m_input_count;
    hbDNNTensor *m_inputs;
    int32_t m_output_count;
    hbDNNTensor *m_outputs;
    int32_t m_estimate_latency;
} Model_Object;

#ifdef __cplusplus
}
#endif /* extern "C" */

#endif  // _PYTHON_DNN_INFERENCE_H_
