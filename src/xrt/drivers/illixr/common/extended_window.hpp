

#pragma once

#include <cerrno>
#include <cassert>
#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl32.h>
#include <GLES3/gl3ext.h>
#include <GLES3/gl3platform.h>
#include "phonebook.hpp"
#include "global_module_defs.hpp"
#include <cerrno>
#include <cassert>
#include <initializer_list>
#include <memory>
#include <cstdlib>
#include <cstring>
#include <jni.h>
#include <android/log.h>
#include <android/native_window.h>

#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "extended-window", __VA_ARGS__))

namespace ILLIXR {
    class xlib_gl_extended_window : public phonebook::service {
    public:
        int width;
        int height;
        EGLDisplay display;
        EGLSurface surface;
        EGLContext context;
        ANativeWindow *my_window;

        xlib_gl_extended_window(int _width, int _height, EGLContext egl_context, ANativeWindow *window) {
            my_window = window;
            width = _width;
            height = _height;
            display = eglGetDisplay(EGL_DEFAULT_DISPLAY);
            EGLint major_version, minor_version;
            eglInitialize(display, &major_version, &minor_version);
            LOGI("EGL Initialized with major version : %d minor version: %d", major_version, minor_version);

            const EGLint attribs[] = {
                    //EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
                    EGL_RENDERABLE_TYPE,EGL_OPENGL_ES2_BIT,
                    EGL_BLUE_SIZE, 8,
                    EGL_GREEN_SIZE, 8,
                    EGL_RED_SIZE, 8,
                    EGL_ALPHA_SIZE, 0,
                    // EGL_DEPTH_SIZE, 24,
                    EGL_NONE
            };

            EGLint w, h, format;
            EGLint numConfigs;
            EGLConfig config = nullptr;
            eglChooseConfig(display, attribs, &config, 1, &numConfigs);
            std::unique_ptr < EGLConfig[] > supportedConfigs(new EGLConfig[numConfigs]);
            assert(supportedConfigs);
            eglChooseConfig(display, attribs, supportedConfigs.get(), numConfigs, &numConfigs);
            assert(numConfigs);
            auto i = 0;
            for (; i < numConfigs; i++) {
                auto &cfg = supportedConfigs[i];
                EGLint r, g, b, d;
                if (eglGetConfigAttrib(display, cfg, EGL_RED_SIZE, &r) &&
                    eglGetConfigAttrib(display, cfg, EGL_GREEN_SIZE, &g) &&
                    eglGetConfigAttrib(display, cfg, EGL_BLUE_SIZE, &b) &&
                    eglGetConfigAttrib(display, cfg, EGL_DEPTH_SIZE, &d) &&
                    r == 8 && g == 8 && b == 8 && d == 24) {

                    config = supportedConfigs[i];
                    break;
                }
            }
            if (i == numConfigs) {
                config = supportedConfigs[0];
            }

            if (config == nullptr) {
                return;
            }
            eglGetConfigAttrib(display, config, EGL_NATIVE_VISUAL_ID, &format);
            if (window != nullptr) {
                LOGI("window is not nullptr");
            }
            else{
                LOGI("window is nullptr");
            }
            try {
                surface = eglCreateWindowSurface(display, config, my_window, nullptr);
            }
            catch (const std::exception& e) {
                return;
            }
            EGLint ctxattrb[] = {
                    EGL_CONTEXT_CLIENT_VERSION, 2,
                    EGL_NONE
            };
            context = eglCreateContext(display, config, egl_context, ctxattrb);

            eglQuerySurface(display, surface, EGL_WIDTH, &w);
            eglQuerySurface(display, surface, EGL_HEIGHT, &h);
        }

        ~xlib_gl_extended_window() {
            if (display != EGL_NO_DISPLAY) {
                eglMakeCurrent(display, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
                if (context != EGL_NO_CONTEXT) {
                    eglDestroyContext(display, context);
                }
                if (surface != EGL_NO_SURFACE) {
                    eglDestroySurface(display, surface);
                }
                eglTerminate(display);
            }
            display = EGL_NO_DISPLAY;
            context = EGL_NO_CONTEXT;
            surface = EGL_NO_SURFACE;
        }
    };
}