// Copyright 2020-2021, The Board of Trustees of the University of Illinois.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  ILLIXR HMD
 * @author RSIM Group <illixr@cs.illinois.edu>
 * @ingroup drv_illixr
 */

#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <dlfcn.h>
#include <alloca.h>
#include <string>
#include <sstream>

#include "math/m_api.h"
#include "xrt/xrt_device.h"
#include "util/u_var.h"
#include "util/u_misc.h"
#include "util/u_debug.h"
#include "util/u_device.h"
#include "util/u_time.h"
#include "util/u_distortion_mesh.h"

#include "illixr_component.h"
#include "common/dynamic_lib.hpp"
#include "common/runtime.hpp"
#include "../auxiliary/android/android_globals.h"
#include <android/native_window_jni.h>
#include "os/os_threading.h"
#include "math/m_imu_3dof.h"
#include <xrt/xrt_config_android.h>
#include <android/sensor.h>
#include <Eigen/Core>

#define POLL_RATE_USEC (1000L / 60) * 1000

#define LOGD(...) ((void)__android_log_print(ANDROID_LOG_INFO, "illixr_device", __VA_ARGS__))

#define ILLIXR_MONADO 1

/*
 *
 * Structs and defines.
 *
 */

struct illixr_hmd
{
	struct xrt_device base;
    struct os_thread_helper oth;
	struct xrt_pose pose;
    ASensorManager *sensor_manager;
    const ASensor *accelerometer;
    const ASensor *gyroscope;
    const ASensor *sensor_info;
    ASensorEventQueue *event_queue;

	bool print_spew;
	bool print_debug;

    struct
    {
        //! Lock for last and fusion.
        struct os_mutex lock;
        struct m_imu_3dof fusion;
    };

	const char *path;
	const char *comp;
	ILLIXR::dynamic_lib *runtime_lib;
	ILLIXR::runtime *runtime;
};


/*
 *
 * Functions
 *
 */

// Callback for the Android sensor event queue
static int
android_sensor_callback(int fd, int events, void *data)
{
    struct illixr_hmd *d = (struct illixr_hmd *)data;

    if (d->accelerometer == NULL || d->gyroscope == NULL)
        return 1;

    ASensorEvent event;
    struct xrt_vec3 gyro;
    struct xrt_vec3 accel;
    while (ASensorEventQueue_getEvents(d->event_queue, &event, 1) > 0) {

        switch (event.type) {
            case ASENSOR_TYPE_ACCELEROMETER: {
                accel.x = event.acceleration.y;
                accel.y = -event.acceleration.x;
                accel.z = event.acceleration.z;

                //ANDROID_TRACE(d, "accel %ld %.2f %.2f %.2f", event.timestamp, accel.x, accel.y, accel.z);
                //LOGD("accel %ld %.2f %.2f %.2f", event.timestamp, accel.x, accel.y, accel.z);

                break;
            }
            case ASENSOR_TYPE_GYROSCOPE: {
                gyro.x = -event.data[1];
                gyro.y = event.data[0];
                gyro.z = event.data[2];

                //ANDROID_TRACE(d, "gyro %ld %.2f %.2f %.2f", event.timestamp, gyro.x, gyro.y, gyro.z);
                //LOGD( "gyro %ld %.2f %.2f %.2f", event.timestamp, gyro.x, gyro.y, gyro.z);

                // TODO: Make filter handle accelerometer
                struct xrt_vec3 null_accel;

                // Lock last and the fusion.
                os_mutex_lock(&d->lock);

                m_imu_3dof_update(&d->fusion, event.timestamp, &null_accel, &gyro);
                unsigned long long timestamp = event.timestamp;
                //LOGD( "imu timestamp = %d", timestamp);

                //write_imu_data(timestamp, accel, gyro);
                //LOGD( "write_imu_data");

                // Now done.
                os_mutex_unlock(&d->lock);
            }
//            case ASENSOR_TYPE_ADDITIONAL_INFO: {
//                switch (event.additional_info.type) {
//                    case ASENSOR_ADDITIONAL_INFO_BEGIN: {
//                        LOGD("additional info begin .. ");
//                    }
//                    default:
//                        LOGD("sensor additinoal info default %d", event.additional_info.serial);
//                }
//            }
            default: //ANDROID_TRACE(d, "Unhandled event type %d", event.type);
                     ;//LOGD( "Unhandled event type %d", event.type);
        }
    }
    return 1;
}

static inline int32_t
android_get_sensor_poll_rate(const struct illixr_hmd *d)
{
    const float freq_multiplier = 1.0f / 3.0f;
    return (d == NULL) ? POLL_RATE_USEC
                       : (int32_t)(d->base.hmd->screens[0].nominal_frame_interval_ns * freq_multiplier * 0.001f);
}

static void *
android_run_thread(void *ptr)
{
    struct illixr_hmd *d = (struct illixr_hmd *)ptr;
    const int32_t poll_rate_usec = android_get_sensor_poll_rate(d);
    LOGD("ANDROID RUN THREAD");
#if __ANDROID_API__ >= 26
    d->sensor_manager = ASensorManager_getInstanceForPackage(XRT_ANDROID_PACKAGE);
#else
    d->sensor_manager = ASensorManager_getInstance();
#endif
    d->accelerometer = ASensorManager_getDefaultSensor(d->sensor_manager, ASENSOR_TYPE_ACCELEROMETER);
    d->gyroscope = ASensorManager_getDefaultSensor(d->sensor_manager, ASENSOR_TYPE_GYROSCOPE);
    d->sensor_info = ASensorManager_getDefaultSensor(d->sensor_manager,  ASENSOR_TYPE_GYROSCOPE_UNCALIBRATED);

    ALooper *looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);

    d->event_queue = ASensorManager_createEventQueue(d->sensor_manager, looper, ALOOPER_POLL_CALLBACK,
                                                     android_sensor_callback, (void *)d);

    // Start sensors in case this was not done already.
    if (d->accelerometer != NULL) {
        LOGD("start accelerometer");
        ASensorEventQueue_enableSensor(d->event_queue, d->accelerometer);
        ASensorEventQueue_setEventRate(d->event_queue, d->accelerometer, poll_rate_usec);
    }
    if (d->gyroscope != NULL) {
        LOGD("start gyroscope");
        ASensorEventQueue_enableSensor(d->event_queue, d->gyroscope);
        ASensorEventQueue_setEventRate(d->event_queue, d->gyroscope, poll_rate_usec);
    }
    if (d->sensor_info != NULL) {
        LOGD("start additional info");
        ASensorEventQueue_enableSensor(d->event_queue, d->sensor_info);
        ASensorEventQueue_setEventRate(d->event_queue, d->sensor_info, poll_rate_usec);
    }

    int ret = 0;
    while (d->oth.running && ret != ALOOPER_POLL_ERROR) {
        ret = ALooper_pollAll(0, NULL, NULL, NULL);
        //LOGD("oth running %d", ret);
    }

    return NULL;
}

static inline struct illixr_hmd *
illixr_hmd(struct xrt_device *xdev)
{
	return (struct illixr_hmd *)xdev;
}

DEBUG_GET_ONCE_BOOL_OPTION(illixr_spew, "ILLIXR_PRINT_SPEW", false)
DEBUG_GET_ONCE_BOOL_OPTION(illixr_debug, "ILLIXR_PRINT_DEBUG", false)

#define DH_SPEW(dh, ...)                                                                                               \
	do {                                                                                                           \
		if (dh->print_spew) {                                                                                  \
			fprintf(stderr, "%s - ", __func__);                                                            \
			fprintf(stderr, __VA_ARGS__);                                                                  \
			fprintf(stderr, "\n");                                                                         \
		}                                                                                                      \
	} while (false)

#define DH_DEBUG(dh, ...)                                                                                              \
	do {                                                                                                           \
		if (dh->print_debug) {                                                                                 \
			fprintf(stderr, "%s - ", __func__);                                                            \
			fprintf(stderr, __VA_ARGS__);                                                                  \
			fprintf(stderr, "\n");                                                                         \
		}                                                                                                      \
	} while (false)

#define DH_ERROR(dh, ...)                                                                                              \
	do {                                                                                                           \
		fprintf(stderr, "%s - ", __func__);                                                                    \
		fprintf(stderr, __VA_ARGS__);                                                                          \
		fprintf(stderr, "\n");                                                                                 \
	} while (false)

static void
illixr_hmd_destroy(struct xrt_device *xdev)
{
	struct illixr_hmd *dh = illixr_hmd(xdev);
	dh->runtime->stop();
	delete dh->runtime;
	delete dh->runtime_lib;

    // Destroy the thread object.
    os_thread_helper_destroy(&dh->oth);

    // Now that the thread is not running we can destroy the lock.
    os_mutex_destroy(&dh->lock);

	// Remove the variable tracking.
	u_var_remove_root(dh);

	u_device_free(&dh->base);
}

static void
illixr_hmd_update_inputs(struct xrt_device *xdev)
{
	// Empty
}

static void
illixr_hmd_get_tracked_pose(struct xrt_device *xdev,
                            enum xrt_input_name name,
                            uint64_t at_timestamp_ns,
                            struct xrt_space_relation *out_relation)
{
	if (name != XRT_INPUT_GENERIC_HEAD_POSE) {
		DH_ERROR(illixr_hmd(xdev), "unknown input name");
		return;
	}

	out_relation->pose.orientation = illixr_read_pose().orientation;
    //out_relation->pose = illixr_read_pose();

    LOGD("ILLIXR orientation %f %f %f %f", out_relation->pose.orientation.w, out_relation->pose.orientation.x, out_relation->pose.orientation.y, out_relation->pose.orientation.z);

    struct illixr_hmd *d = illixr_hmd(xdev);
    LOGD("MOnado orientation %f %f %f %f", d->fusion.rot.w, d->fusion.rot.x, d->fusion.rot.y, d->fusion.rot.z);

    //out_relation->pose.orientation = d->fusion.rot;

	out_relation->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT | XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT |
	    XRT_SPACE_RELATION_POSITION_VALID_BIT | XRT_SPACE_RELATION_POSITION_TRACKED_BIT);
}

static void
illixr_hmd_get_view_poses(struct xrt_device *xdev,
                          const struct xrt_vec3 *default_eye_relation,
                          uint64_t at_timestamp_ns,
                          uint32_t view_count,
                          struct xrt_space_relation *out_head_relation,
                          struct xrt_fov *out_fovs,
                          struct xrt_pose *out_poses)
{
	u_device_get_view_poses(xdev, default_eye_relation, at_timestamp_ns, view_count, out_head_relation, out_fovs,
	                        out_poses);
}

std::vector<std::string>
split(const std::string &s, char delimiter)
{
	std::vector<std::string> tokens;
	std::string token;
	std::istringstream tokenStream{s};
	while (std::getline(tokenStream, token, delimiter)) {
		tokens.push_back(token);
	}
	return tokens;
}

static int
illixr_rt_launch(struct illixr_hmd *dh, const char *path, const char *comp)
{
    //JavaVM* vm = (JavaVM*)android_globals_get_vm();
    ANativeWindow* window = (ANativeWindow*) android_globals_get_window_illixr();
    //ANativeWindow_acquire(window);
//    JavaVM* jvm = (JavaVM*)android_globals_get_vm();
//    JNIEnv* myNewEnv;
//    jvm->AttachCurrentThread(reinterpret_cast<JNIEnv **>((void **) &myNewEnv), NULL);
//    ANativeWindow* window = ANativeWindow_fromSurface(myNewEnv, ANativeWindow_toSurface(myNewEnv, window1));
    if(window != NULL)
        LOG("Anative window is not null");
	dh->runtime_lib = new ILLIXR::dynamic_lib{ILLIXR::dynamic_lib::create(std::string{path})};
    LOG("Before getting runtime factory");
	dh->runtime = dh->runtime_lib->get<ILLIXR::runtime *(*)(EGLContext appGLCtx, ANativeWindow *window)>("runtime_factory")(EGL_NO_CONTEXT, window);
    LOG("After getting runtime factory");
    dh->runtime->load_so(split(std::string{comp}, ':'));
	dh->runtime->load_plugin_factory((ILLIXR::plugin_factory)illixr_monado_create_plugin);
	return 0;
}

extern "C" struct xrt_device *
illixr_hmd_create(const char *path_in, const char *comp_in)
{
	struct illixr_hmd *dh;
	enum u_device_alloc_flags flags =
	    (enum u_device_alloc_flags)(U_DEVICE_ALLOC_HMD | U_DEVICE_ALLOC_TRACKING_NONE);
	dh = U_DEVICE_ALLOCATE(struct illixr_hmd, flags, 1, 0);
	dh->base.update_inputs = illixr_hmd_update_inputs;
	dh->base.get_tracked_pose = illixr_hmd_get_tracked_pose;
	dh->base.get_view_poses = illixr_hmd_get_view_poses;
	dh->base.destroy = illixr_hmd_destroy;
	dh->base.name = XRT_DEVICE_GENERIC_HMD;
	dh->base.device_type = XRT_DEVICE_TYPE_HMD;

	size_t idx = 0;
	dh->base.hmd->blend_modes[idx++] = XRT_BLEND_MODE_OPAQUE;
	dh->base.hmd->blend_mode_count = idx;

	dh->pose.orientation.w = 1.0f; // All other values set to zero.
	dh->print_spew = debug_get_bool_option_illixr_spew();
	dh->print_debug = debug_get_bool_option_illixr_debug();
	dh->path = path_in;
	dh->comp = comp_in;

	// Print name.
	snprintf(dh->base.str, XRT_DEVICE_NAME_LEN, "ILLIXR");
	snprintf(dh->base.serial, XRT_DEVICE_NAME_LEN, "ILLIXR");

	// Setup input.
	dh->base.inputs[0].name = XRT_INPUT_GENERIC_HEAD_POSE;

	// Setup info.
	struct u_device_simple_info info;
	info.display.w_pixels = ILLIXR::display_params::width_pixels;
	info.display.h_pixels = ILLIXR::display_params::height_pixels;
	info.display.w_meters = 0.14f;
	info.display.h_meters = 0.07f;
	info.lens_horizontal_separation_meters = 0.13f / 2.0f;
	info.lens_vertical_position_meters = 0.07f / 2.0f;
	info.fov[0] = 85.0f * (M_PI / 180.0f);
	info.fov[1] = 85.0f * (M_PI / 180.0f);

	if (!u_device_setup_split_side_by_side(&dh->base, &info)) {
		DH_ERROR(dh, "Failed to setup basic device info");
		illixr_hmd_destroy(&dh->base);
		return NULL;
	}

	// Setup variable tracker.
	u_var_add_root(dh, "ILLIXR", true);
	u_var_add_pose(dh, &dh->pose, "pose");

	if (dh->base.hmd->distortion.preferred == XRT_DISTORTION_MODEL_NONE) {
		// Setup the distortion mesh.
		u_distortion_mesh_set_none(&dh->base);
	}

    m_imu_3dof_init(&dh->fusion, M_IMU_3DOF_USE_GRAVITY_DUR_20MS);

    int ret = os_mutex_init(&dh->lock);
    LOGD("LOCK RET IS %d", ret);

    if (ret != 0) {
        LOGD("Failed to init mutex!");
        illixr_hmd_destroy(&dh->base);
        return 0;
    }
    ret = os_thread_helper_init(&dh->oth);
    LOGD("RET IS %d", ret);
    ret = os_thread_helper_start(&dh->oth, android_run_thread, dh);
    if (ret != 0) {
        DH_ERROR(dh, "Failed to start thread ILLIXR");
        illixr_hmd_destroy(&dh->base);
        return NULL;
    }

    u_var_add_root(dh, "Android phone", true);
    u_var_add_ro_vec3_f32(dh, &dh->fusion.last.accel, "last.accel");
    u_var_add_ro_vec3_f32(dh, &dh->fusion.last.gyro, "last.gyro");

	// start ILLIXR runtime
	if (illixr_rt_launch(dh, dh->path, dh->comp) != 0) {
		DH_ERROR(dh, "Failed to load ILLIXR Runtime");
		illixr_hmd_destroy(&dh->base);
	}

	return &dh->base;
}
