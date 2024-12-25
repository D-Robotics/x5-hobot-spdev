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

// Copyright (c) 2020 D-Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of D-Robotics Inc. This is proprietary information owned by
// D-Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of D-Robotics Inc.

#ifndef _POST_PROCESS_CENTERNET_POST_PROCESS_H_
#define _POST_PROCESS_CENTERNET_POST_PROCESS_H_

#include "dnn/hb_dnn.h"

#ifdef __cplusplus
  extern "C"{
#endif

typedef struct {
	int height;
	int width;
	int ori_height;
	int ori_width;
	float score_threshold; // 0.35
	float nms_threshold; // 0.65
	int nms_top_k; // 500
	int is_pad_resize;
} CenternetPostProcessInfo_t;

/**
 * Post process
 * @param[in] tensor: Model output tensors
 * @param[in] image_tensor: Input image tensor
 * @param[out] perception: Perception output data
 * @return 0 if success
 */
char* CenternetPostProcess(CenternetPostProcessInfo_t *post_info);

void CenternetdoProcess(hbDNNTensor *nms_tensor, hbDNNTensor *wh_tensor, hbDNNTensor *reg_tensor, CenternetPostProcessInfo_t *post_info, int layer);

void Centernet_resnet101_doProcess(hbDNNTensor *nms_tensor, hbDNNTensor *wh_tensor, hbDNNTensor *reg_tensor, CenternetPostProcessInfo_t *post_info, int layer);


#ifdef __cplusplus
}
#endif

#endif  // _POST_PROCESS_CENTERNET_POST_PROCESS_H_