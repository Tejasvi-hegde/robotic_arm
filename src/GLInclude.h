// GLInclude.h - single place that pulls in OpenGL / GLU / freeGLUT.
// On Windows <windows.h> must come before the GL headers.
#pragma once

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

#include <GL/freeglut.h>   // also includes <GL/gl.h> and <GL/glu.h>
