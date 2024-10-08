// Copyright (c) 2020 D-Robotics.All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of D-Robotics Inc. This is proprietary information owned by
// D-Robotics Inc. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of D-Robotics Inc.

#ifndef _POST_PROCESS_UNET_POST_PROCESS_H_
#define _POST_PROCESS_UNET_POST_PROCESS_H_

#include "dnn/hb_dnn.h"

#ifdef __cplusplus
  extern "C"{
#endif

typedef struct {
	int height;
	int width;
	int ori_height;
	int ori_width;
	float score_threshold; // 0.5
	float nms_threshold; // 0.60
	int nms_top_k; // 500
	int is_pad_resize;
} UnetPostProcessInfo_t;

  /**
   * Post process
   * @param[in] tensor: Model output tensors
   * @param[in] image_tensor: Input image tensor
   * @param[out] perception: Perception output data
   * @return 0 if success
   */
  char* UnetPostProcess(UnetPostProcessInfo_t *post_info);

  void UnetdoProcess(hbDNNTensor *tensors, UnetPostProcessInfo_t *post_info, int layer);

#ifdef __cplusplus
}
#endif

#endif  // _POST_PROCESS_UNET_POST_PROCESS_H_