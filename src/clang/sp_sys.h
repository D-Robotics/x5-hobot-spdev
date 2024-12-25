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

#ifndef SP_SYS_H_
#define SP_SYS_H_
#define SP_MTYPE_VIO     0
#define SP_MTYPE_ENCODER 1
#define SP_MTYPE_DECODER 2
#define SP_MTYPE_DISPLAY 3
#ifdef __cplusplus
extern "C"
{
#endif
int sp_module_bind(void *src, int32_t src_type, void *dst, int32_t dst_type);
int sp_module_unbind(void *src, int32_t src_type, void *dst, int32_t dst_type);
#ifdef __cplusplus
}
#endif /* End of #ifdef __cplusplus */

#endif /* SP_SYS_H_ */