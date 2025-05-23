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

#include <stdint.h>
#include "dnn/hb_sys.h"
#include "dnn/hb_dnn.h"
#include "sp_bpu.h"
#include "bpu_wrapper.h"
bpu_module *sp_init_bpu_module(const char *model_file_name)
{
    auto bpu_handle = hb_bpu_predict_init(model_file_name);
    return bpu_handle;
}

int sp_bpu_start_predict(bpu_module *bpu_handle, char *addr)
{
    if (bpu_handle)
    {
        return hb_bpu_start_predict(bpu_handle, addr);
    }
    return -1;
}

int sp_release_bpu_module(bpu_module *bpu_handle)
{
    if (bpu_handle)
    {
        return hb_bpu_predict_unint(bpu_handle);
    }
    return -1;
}
int sp_init_bpu_tensors(bpu_module *bpu_handle, hbDNNTensor *output_tensors)
{
    if (bpu_handle)
    {
        return hb_bpu_init_tensors(bpu_handle, output_tensors);
    }
    return -1;
}

int sp_deinit_bpu_tensor(hbDNNTensor *tensor, int32_t len)
{
    if (tensor)
    {
        return hb_bpu_deinit_tensor(tensor, len);
    }
    return -1;
}