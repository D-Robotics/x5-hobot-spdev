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
 * @Copyright 2024 D-Robotics, Inc.
 * @All rights reserved.
 * @Date: 2023-01-30 11:27:41
 * @LastEditTime: 2024-09-09 16:38:23
 ***************************************************************************/

#include <cstdint>
#include <fcntl.h>
#include <stdio.h>
#include <sys/mman.h>
#include <unistd.h>
#include <linux/fb.h>
#include <stdlib.h>
#include <stdbool.h>

#include <egl_preview.h>

using namespace std;

namespace spdev
{

bool EGLPreviewWindow::isInitialized = false;

EGLPreviewWindow::EGLPreviewWindow(int width, int height)
	: width(width), height(height), x_display(nullptr), x_window(0),
	  egl_display(EGL_NO_DISPLAY), egl_context(EGL_NO_CONTEXT), egl_surface(EGL_NO_SURFACE),
	  program(0), rgba_tex(0) {}

EGLPreviewWindow::~EGLPreviewWindow() {
	close();
}

void EGLPreviewWindow::initializeViewport() {
	glViewport(0, 0, width, height);
}

bool EGLPreviewWindow::initialize() {
	if (isInitialized) {
		return true;
	}

	x_display = XOpenDisplay(nullptr);
	if (!x_display) {
		std::cerr << "Failed to open X display" << std::endl;
		return false;
	}

	Window root = DefaultRootWindow(x_display);
	XSetWindowAttributes swa;
	swa.event_mask = ExposureMask | PointerMotionMask | KeyPressMask | StructureNotifyMask;
	x_window = XCreateWindow(x_display, root, 0, 0, width, height, 0, CopyFromParent, InputOutput, CopyFromParent, CWEventMask, &swa);
	XMapWindow(x_display, x_window);
	XStoreName(x_display, x_window, "EGL Preview");

	egl_display = eglGetDisplay((EGLNativeDisplayType)x_display);
	if (egl_display == EGL_NO_DISPLAY) {
		std::cerr << "Failed to get EGL display" << std::endl;
		return false;
	}

	if (!eglInitialize(egl_display, nullptr, nullptr)) {
		std::cerr << "Failed to initialize EGL" << std::endl;
		return false;
	}

	EGLint configAttribs[] = {
		EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
		EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,
		EGL_NONE
	};
	EGLConfig config;
	EGLint numConfigs;
	eglChooseConfig(egl_display, configAttribs, &config, 1, &numConfigs);

	egl_surface = eglCreateWindowSurface(egl_display, config, (EGLNativeWindowType)x_window, nullptr);
	if (egl_surface == EGL_NO_SURFACE) {
		std::cerr << "Failed to create EGL surface" << std::endl;
		return false;
	}

	EGLint contextAttribs[] = {EGL_CONTEXT_CLIENT_VERSION, 2, EGL_NONE};
	egl_context = eglCreateContext(egl_display, config, EGL_NO_CONTEXT, contextAttribs);
	if (egl_context == EGL_NO_CONTEXT) {
		std::cerr << "Failed to create EGL context" << std::endl;
		return false;
	}

	eglMakeCurrent(egl_display, egl_surface, egl_surface, egl_context);

	if (!initialize_shaders()) {
		return false;
	}

	initializeViewport();

	isInitialized = true;
	return true;
}

bool EGLPreviewWindow::initialize_shaders() {
	const char* vertex_shader_src = R"(
		attribute vec4 position;
		attribute vec2 texcoord;
		varying vec2 v_texcoord;

		void main() {
			gl_Position = position;
			v_texcoord = vec2(texcoord.x, 1.0 - texcoord.y);  // Flip Y coordinate
		}
	)";

	const char* fragment_shader_src = R"(
		precision mediump float;
		varying vec2 v_texcoord;
		uniform sampler2D rgba_texture;

		void main() {
			// Sample the RGBA texture
			vec4 rgba = texture2D(rgba_texture, v_texcoord);
			gl_FragColor = rgba;
		}
	)";

	GLuint vertex_shader = glCreateShader(GL_VERTEX_SHADER);
	glShaderSource(vertex_shader, 1, &vertex_shader_src, nullptr);
	glCompileShader(vertex_shader);

	GLuint fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
	glShaderSource(fragment_shader, 1, &fragment_shader_src, nullptr);
	glCompileShader(fragment_shader);

	program = glCreateProgram();
	glAttachShader(program, vertex_shader);
	glAttachShader(program, fragment_shader);
	glLinkProgram(program);

	glDeleteShader(vertex_shader);
	glDeleteShader(fragment_shader);

	glUseProgram(program);

	const float vertices[] = {
		-1.0f, -1.0f, 0.0f, 0.0f,
		 1.0f, -1.0f, 1.0f, 0.0f,
		-1.0f,  1.0f, 0.0f, 1.0f,
		 1.0f,  1.0f, 1.0f, 1.0f,
	};

	const GLushort indices[] = { 0, 1, 2, 2, 3, 0 };

	GLuint vbo;
	glGenBuffers(1, &vbo);
	glBindBuffer(GL_ARRAY_BUFFER, vbo);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	GLint position_loc = glGetAttribLocation(program, "position");
	glEnableVertexAttribArray(position_loc);
	glVertexAttribPointer(position_loc, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (const void*)0);

	GLint texcoord_loc = glGetAttribLocation(program, "texcoord");
	glEnableVertexAttribArray(texcoord_loc);
	glVertexAttribPointer(texcoord_loc, 2, GL_FLOAT, GL_FALSE, 4 * sizeof(float), (const void*)(2 * sizeof(float)));

	glGenTextures(1, &rgba_tex);
	glBindTexture(GL_TEXTURE_2D, rgba_tex);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	return true;
}

void EGLPreviewWindow::updateTextureSize() {
	glBindTexture(GL_TEXTURE_2D, rgba_tex);

	// Update texture size
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);

	// Set texture parameters
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	tex_width = width;
	tex_height = height;
}

void EGLPreviewWindow::resize(int new_width, int new_height) {
	width = new_width;
	height = new_height;
	initializeViewport();
	updateTextureSize(); // Update the texture size
}

void EGLPreviewWindow::upload_texture(const std::vector<uint8_t>& rgba_data) {
	glBindTexture(GL_TEXTURE_2D, rgba_tex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, image_width, image_height, 0, GL_RGBA, GL_UNSIGNED_BYTE, rgba_data.data());
}

void EGLPreviewWindow::setImageDimensions(int width, int height) {
	image_width = width;
	image_height = height;
	updateTextureSize(); // Optionally update texture size if necessary
}

void EGLPreviewWindow::show(const std::vector<uint8_t>& rgba_data) {
	upload_texture(rgba_data);

	XEvent xev;
	while (true) {
		XNextEvent(x_display, &xev);
		if (xev.type == Expose) {
			draw();
		} else if (xev.type == ConfigureNotify) { // Window resize
			XConfigureEvent xce = xev.xconfigure;
			resize(xce.width, xce.height);
		} else if (xev.type == KeyPress) {
			break;
		}
	}
}

void EGLPreviewWindow::update(const std::vector<uint8_t>& rgba_data) {
	upload_texture(rgba_data);
	draw();  // Draw the current texture
}

void EGLPreviewWindow::processEvents() {
	XEvent xev;
	while (XPending(x_display) > 0) {
		XNextEvent(x_display, &xev);
		if (xev.type == ConfigureNotify) { // Window resize
			XConfigureEvent xce = xev.xconfigure;
			resize(xce.width, xce.height);
		} else if (xev.type == KeyPress) {
			close(); // Close window on key press
			exit(0); // Exit the application
		}
	}
}

void EGLPreviewWindow::draw() {
	glClear(GL_COLOR_BUFFER_BIT);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);

	eglSwapBuffers(egl_display, egl_surface);
}

void EGLPreviewWindow::close() {
	if (egl_display != EGL_NO_DISPLAY) {
		eglMakeCurrent(egl_display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
		if (egl_context != EGL_NO_CONTEXT) {
			eglDestroyContext(egl_display, egl_context);
		}
		if (egl_surface != EGL_NO_SURFACE) {
			eglDestroySurface(egl_display, egl_surface);
		}
		eglTerminate(egl_display);
	}
	if (x_window) {
		XDestroyWindow(x_display, x_window);
	}
	if (x_display) {
		XCloseDisplay(x_display);
	}
}
}; // namespace spdev
