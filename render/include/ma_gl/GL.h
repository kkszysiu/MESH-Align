#pragma once
// Single place that pulls in the OpenGL 3.3 core headers.
// macOS ships GL symbols in the framework (no loader); elsewhere we use glad.
#if defined(__APPLE__)
#ifndef GL_SILENCE_DEPRECATION
#define GL_SILENCE_DEPRECATION
#endif
#include <OpenGL/gl3.h>
#else
#include <glad/gl.h>
#endif
