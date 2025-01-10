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

#ifndef SP_DISPLAY_H_
#define SP_DISPLAY_H_

#ifdef __cplusplus
extern "C"
{
#endif

    void *sp_init_display_module();
    void sp_release_display_module(void *obj);
    int32_t sp_start_display(void *obj, int32_t chn,
        int32_t width, int32_t height);
    int32_t sp_stop_display(void *obj);
    int32_t sp_display_set_image(void *obj,
        char *addr, int32_t size, int32_t chn);
    int32_t sp_display_draw_rect(void *obj,
        int32_t x0, int32_t y0, int32_t x1, int32_t y1,
        int32_t chn, int32_t flush, int32_t color, int32_t line_width);
    int32_t sp_display_draw_string(void *obj,
        int32_t x, int32_t y, char *str,
        int32_t chn, int32_t flush, int32_t color, int32_t line_width);
    void sp_get_display_resolution(int32_t *width, int32_t *height);

#ifdef __cplusplus
}
#endif /* End of #ifdef __cplusplus */

#endif /* SP_DISPLAY_H_ */