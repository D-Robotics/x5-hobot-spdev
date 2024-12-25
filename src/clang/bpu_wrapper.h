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
 * COPYRIGHT NOTICE
 * Copyright 2020 Horizon Robotics, Inc.
 * All rights reserved.
 ***************************************************************************/
#ifndef BPU_WRAPPER_H_
#define BPU_WRAPPER_H_

#include "sp_bpu.h"
#include "dnn/hb_dnn.h"

#ifdef __cplusplus
extern "C"
{
#endif

bpu_module *hb_bpu_predict_init(const char *model_file_name);

int hb_bpu_init_tensors(bpu_module *bpu_handle, hbDNNTensor *output_tensors);
int hb_bpu_deinit_tensor(hbDNNTensor *tensor, int32_t len);
int hb_bpu_start_predict(bpu_module *bpu_handle, char *frame_buffer);
int hb_bpu_predict_unint(bpu_module *handle);
#ifdef __cplusplus
}
#endif

#endif // BPU_WRAPPER_H_