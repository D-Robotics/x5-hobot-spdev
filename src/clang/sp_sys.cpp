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
 * @Date: 2023-03-05 16:55:56
 * @LastEditTime: 2023-03-05 16:59:38
 ***************************************************************************/
#include <sys/stat.h>
#include <thread>
#include <iostream>
#include <fstream>
#include <string>
#include <time.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <string.h>

#include "sp_sys.h"
#include "vpp_module.h"

using namespace std;
using namespace spdev;

int sp_module_bind(void *src, int32_t src_type, void *dst, int32_t dst_type)
{
    return ((VPPModule *)dst)->BindTo((VPPModule *)src);
}

int sp_module_unbind(void *src, int32_t src_type, void *dst, int32_t dst_type)
{
    return ((VPPModule *)dst)->UnBind((VPPModule *)src);
}