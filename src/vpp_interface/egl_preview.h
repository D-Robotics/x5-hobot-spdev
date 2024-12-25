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
 *
 * Copyright (C) 2024 D-Robotics Inc.
 * All rights reserved.
 * Author:
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _EGL_PREVIEW_H__
#define _EGL_PREVIEW_H__

#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <EGL/egl.h>
#include <GLES2/gl2.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <thread>

using namespace std;

namespace spdev
{
    class EGLPreviewWindow {
    public:
        static EGLPreviewWindow& getInstance(int width = 1920, int height = 1080) {
            static EGLPreviewWindow instance(width, height);
            return instance;
        }
        EGLPreviewWindow(const EGLPreviewWindow&) = delete;
        EGLPreviewWindow& operator=(const EGLPreviewWindow&) = delete;

        void initializeViewport();
        bool initialize();
        void show(const std::vector<uint8_t>& rgba_data);
        void close();
        void updateTextureSize();
        void setImageDimensions(int width, int height);
        void resize(int new_width, int new_height);
        void update(const std::vector<uint8_t>& rgba_data);
        void processEvents();

    private:
        EGLPreviewWindow(int width, int height);
        ~EGLPreviewWindow();

        static bool isInitialized;
        Display* x_display;
        Window x_window;
        EGLDisplay egl_display;
        EGLContext egl_context;
        EGLSurface egl_surface;
        int width;
        int height;
        int image_width;
        int image_height;

        GLuint program;
        GLuint rgba_tex;
        int tex_width, tex_height;

        bool initialize_shaders();
        void draw();
        void upload_texture(const std::vector<uint8_t>& rgba_data);
    };
}; // namespace spdev

#endif // _EGL_PREVIEW_H__
