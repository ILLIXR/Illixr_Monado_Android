extern "C" {
//#include "ogl/ogl_api.h"
//#include <GLFW/glfw3.h>
#include "xrt/xrt_device.h"
}

#include <iostream>
#include <array>

#include <android/log.h>
#include <vulkan/vulkan_core.h>
#include "common/plugin.hpp"
#include "common/phonebook.hpp"
#include "common/switchboard.hpp"
#include "common/data_format.hpp"
#include "common/pose_prediction.hpp"
#include "common/relative_clock.hpp"
#include "../../include/xrt/xrt_handles.h"
#include "common/common_lock.hpp"
#include <Eigen/Core>
#include <android/sensor.h>
#include <opencv2/core/mat.hpp>
#include <mutex>
#include <semaphore.h>

#define ILLIXR_MONADO 1
#define LOGI(...) ((void)__android_log_print(ANDROID_LOG_INFO, "comp-layer", __VA_ARGS__))

using namespace ILLIXR;

static constexpr duration VSYNC_PERIOD {freq2period(60.0)};

/// Dummy plugin class for an instance during phonebook registration
class illixr_plugin : public plugin {
public:
    illixr_plugin(std::string name_, phonebook* pb_)
            : plugin{name_, pb_}
            , sb{pb->lookup_impl<switchboard>()}
            , sb_pose{pb->lookup_impl<pose_prediction>()}
            , cl{pb->lookup_impl<common_lock>()}
            , _m_clock{pb->lookup_impl<RelativeClock>()}
            , sb_image_handle{sb->get_writer<image_handle>("image_handle")}
            , sb_imu{sb->get_writer<imu_type>("imu")}
            , sb_cam{sb->get_writer<cam_type>("cam")}
            , sb_illixr_signal{sb->get_reader<illixr_signal>("illixr_signal")}
            , sb_semaphore_handle{sb->get_writer<semaphore_handle>("semaphore_handle")}
            , sb_eyebuffer{sb->get_writer<rendered_frame>("eyebuffer")}
            , sb_vsync_estimate{sb->get_reader<switchboard::event_wrapper<time_point>>("vsync_estimate")}
    {}

    const std::shared_ptr<switchboard> sb;
    const std::shared_ptr<pose_prediction> sb_pose;
    const std::shared_ptr<common_lock> cl;
    std::shared_ptr<RelativeClock> _m_clock;
    switchboard::writer<image_handle> sb_image_handle;
    switchboard::writer<imu_type> sb_imu;
    switchboard::writer<cam_type> sb_cam;
    switchboard::reader<illixr_signal> sb_illixr_signal;
    switchboard::writer<semaphore_handle> sb_semaphore_handle;
    switchboard::writer<rendered_frame> sb_eyebuffer;
    switchboard::reader<switchboard::event_wrapper<time_point>> sb_vsync_estimate;
    fast_pose_type prev_pose; /* stores a copy of pose each time illixr_read_pose() is called */
    time_point sample_time; /* when prev_pose was stored */
    std::optional<ullong>     _m_first_imu_time;
    std::optional<time_point> _m_first_real_time_imu;
    std::mutex imu_write_lock;
};

static illixr_plugin* illixr_plugin_obj = nullptr;
int prev_counter = 0;

extern "C" plugin* illixr_monado_create_plugin(phonebook* pb) {
    // "borrowed" from common/plugin.hpp PLUGIN_MAIN
    illixr_plugin_obj = new illixr_plugin {"illixr_plugin", pb};
    illixr_plugin_obj->start();
    return illixr_plugin_obj;
}

extern "C" void wait_for_illixr_signal() {
    LOGI("Wait for illixr signal");
    //illixr_plugin_obj->cl->get_lock();
    sem_wait(&illixr_plugin_obj->cl->sem_illixr);
    LOGI("Wait for illixr signal done");
}

extern "C" void done_signal_illixr() {
    LOGI("Done illixr signal");
    //illixr_plugin_obj->cl->release_lock();
    sem_post(&illixr_plugin_obj->cl->sem_monado);
    LOGI("Done illixr signal done");
}

extern "C" void write_imu_data(ullong ts, xrt_vec3 accel, xrt_vec3 gyro) {
    if(!illixr_plugin_obj)
        return;
    std::lock_guard<std::mutex> lock(illixr_plugin_obj->imu_write_lock);
    //illixr_plugin_obj->imu_write_lock.lock();
    Eigen::Vector3f la = {accel.x, accel.y, accel.z};
    Eigen::Vector3f av = {gyro.x, gyro.y, gyro.z};
    ullong imu_time = static_cast<ullong>(ts);
    //LOGI("IMU TIMESTAMP = %d and %d", ts, imu_time);
    if (!illixr_plugin_obj->_m_first_imu_time) {
        illixr_plugin_obj->_m_first_imu_time      = imu_time;
        illixr_plugin_obj->_m_first_real_time_imu = illixr_plugin_obj->_m_clock->now();
    }
    time_point imu_time_point{*illixr_plugin_obj->_m_first_real_time_imu + std::chrono::nanoseconds(imu_time - *illixr_plugin_obj->_m_first_imu_time)};

//    cv::Mat ir_left = cv::Mat::zeros(cv::Size(100, 100), CV_64FC1);
//    cv::Mat ir_right = cv::Mat::zeros(cv::Size(100, 100), CV_64FC1);
//    illixr_plugin_obj->sb_cam.put(illixr_plugin_obj->sb_cam.allocate<cam_type>({imu_time_point, ir_left, ir_right}));

    illixr_plugin_obj->sb_imu.put(illixr_plugin_obj->sb_imu.allocate<imu_type>
    ({imu_time_point
      , av.cast<double>()
      , la.cast<double>()})
    );
    //LOGI("DONE IMU WRITE");
    //illixr_plugin_obj->imu_write_lock.unlock();
    return;
}

extern "C" struct xrt_pose illixr_read_pose() {
    assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");

    if (!illixr_plugin_obj->sb_pose->fast_pose_reliable()) {
        std::cerr << "Pose not reliable yet; returning best guess" << std::endl;
    }
    struct xrt_pose ret;
    const fast_pose_type fast_pose = illixr_plugin_obj->sb_pose->get_fast_pose();
    const pose_type pose = fast_pose.pose;

    // record when the pose was read for use in write_frame
    illixr_plugin_obj->sample_time = illixr_plugin_obj->_m_clock->now();
    LOGI("Illixr orientation in component: %f, %f, %f, %f", pose.orientation.w(), pose.orientation.x(), pose.orientation.y(), pose.orientation.z());
    ret.orientation.x = pose.orientation.x();
    ret.orientation.y = pose.orientation.y();
    ret.orientation.z = pose.orientation.z();
    ret.orientation.w = pose.orientation.w();
    ret.position.x = pose.position.x();
    ret.position.y = pose.position.y();
    ret.position.z = pose.position.z();

    // store pose in static variable for use in write_frame
    illixr_plugin_obj->prev_pose = fast_pose; // copy member variables

    return ret;
}

extern "C" void illixr_publish_vk_image_handle(int fd, int64_t format, size_t size, uint32_t width, uint32_t height, uint32_t num_images, int usage) {    assert(illixr_plugin_obj != nullptr && "illixr_plugin_obj must be initialized first.");
    assert(illixr_plugin_obj != nullptr && "illixr_plugin_obj must be initialized first.");
    LOGI("PRINT FD %d", fd);
    swapchain_usage image_usage;
    switch (usage) {
        case 0: {
            image_usage = swapchain_usage::LEFT_SWAPCHAIN;
            break;
        }
        case 1: {
            image_usage = swapchain_usage::RIGHT_SWAPCHAIN;
            break;
        }
        case 2: {
            image_usage = swapchain_usage::LEFT_RENDER;
            break;
        }
        case 3: {
            image_usage = swapchain_usage::RIGHT_RENDER;
            break;
        }
        default: {
            assert(false && "Invalid swapchain usage!");
        }
    }

    illixr_plugin_obj->sb_image_handle.put(illixr_plugin_obj->sb_image_handle.allocate<image_handle>(
            image_handle {
                    fd,
                    format,
                    size,
                    width,
                    height,
                    num_images,
                    image_usage
            }
    ));
}

extern "C" void illixr_publish_vk_semaphore_handle(int fd, int usage) {
    assert(illixr_plugin_obj != nullptr && "illixr_plugin_obj must be initialized first.");

    semaphore_usage sem_usage;
    switch (usage) {
        case 0: {
            sem_usage = semaphore_usage::LEFT_RENDER_COMPLETE;
            break;
        }
        case 1: {
            sem_usage = semaphore_usage::RIGHT_RENDER_COMPLETE;
            break;
        }
        default: {
            assert(false && "Invalid swapchain usage!");
        }
    }

    illixr_plugin_obj->sb_semaphore_handle.put(illixr_plugin_obj->sb_semaphore_handle.allocate<semaphore_handle>(
            semaphore_handle {
                    fd,
                    sem_usage
            }
    ));
}

//extern "C" void illixr_publish_vk_image_handle(int fd, int64_t format, size_t size, uint32_t width, uint32_t height, uint32_t num_images, uint32_t swapchain_index) {
//    assert(illixr_plugin_obj != nullptr && "illixr_plugin_obj must be initialized first.");
//    illixr_plugin_obj->sb_image_handle.put(illixr_plugin_obj->sb_image_handle.allocate<image_handle>(
//            image_handle {
//                    fd,
//                    format,
//                    size,
//                    width,
//                    height,
//                    num_images,
//                    swapchain_index
//            }
//    ));
//}

extern "C" void illixr_publish_vk_buffer_handle(AHardwareBuffer *ahardware_buffer, int64_t format, size_t size, uint32_t width, uint32_t height, uint32_t num_images, uint32_t  usage) {
    assert(illixr_plugin_obj != nullptr && "illixr_plugin_obj must be initialized first.");
    if(ahardware_buffer == NULL)
        LOGI("HARDWARE BUFFER NULL IN MONADO .. %s", "hjj");
    LOGI("NOT NULL..%d",usage);
    #if defined(XRT_GRAPHICS_SYNC_HANDLE_IS_FD)
    LOGI("XRT ..%d", 1);
    #endif
    //std::lock_guard<std::mutex> lock(illixr_plugin_obj->imu_write_lock);

    swapchain_usage image_usage;
    switch (usage) {
        case 0: {
            image_usage = swapchain_usage::LEFT_SWAPCHAIN;
            break;
        }
        case 1: {
            image_usage = swapchain_usage::RIGHT_SWAPCHAIN;
            break;
        }
        case 2: {
            image_usage = swapchain_usage::LEFT_RENDER;
            break;
        }
        case 3: {
            image_usage = swapchain_usage::RIGHT_RENDER;
            break;
        }
        default: {
            assert(false && "Invalid swapchain usage!");
        }
    }

    illixr_plugin_obj->sb_image_handle.put(illixr_plugin_obj->sb_image_handle.allocate<image_handle>(
            image_handle {
                    ahardware_buffer,
                    format,
                    size,
                    width,
                    height,
                    num_images,
                    image_usage
            }
    ));
}

//extern "C" void illixr_publish_vk_buffer_handle(AHardwareBuffer *ahardware_buffer, int64_t format, size_t size, uint32_t width, uint32_t height, uint32_t num_images, uint32_t swapchain_index) {
//    LOGI("illixr vk publish handle .. ");
//    assert(illixr_plugin_obj != nullptr && "illixr_plugin_obj must be initialized first.");
//    LOGI("illixr vk publish handle .. ");
//    //TODO
//    illixr_plugin_obj->sb_image_handle.put(illixr_plugin_obj->sb_image_handle.allocate<image_handle>(
//            image_handle {
//                    ahardware_buffer,
//                    format,
//                    size,
//                    width,
//                    height,
//                    num_images,
//                    swapchain_index
//            }
//    ));
//}

extern "C" void illixr_write_frame(GLuint left,
                                   GLuint right) {
    assert(illixr_plugin_obj != nullptr && "illixr_plugin_obj must be initialized first.");
    LOGI("ILLIXR WRITE FRAME ..");
    static unsigned int buffer_to_use = 0U;

    illixr_plugin_obj->sb_eyebuffer.put(illixr_plugin_obj->sb_eyebuffer.allocate<rendered_frame>(
            rendered_frame {
                    std::array<GLuint, 2>{ left, right },
                    std::array<GLuint, 2>{ buffer_to_use, buffer_to_use }, // .data() deleted FIXME
                    illixr_plugin_obj->prev_pose,
                    illixr_plugin_obj->sample_time,
                    illixr_plugin_obj->_m_clock->now()
            }
    ));

    buffer_to_use = (buffer_to_use == 0U) ? 1U : 0U;
}

extern "C" int64_t illixr_get_vsync_ns() {
    assert(illixr_plugin_obj != nullptr && "illixr_plugin_obj must be initialized first.");

    switchboard::ptr<const switchboard::event_wrapper<time_point>> vsync_estimate = illixr_plugin_obj->sb_vsync_estimate.get_ro_nullable();

    time_point target_time = vsync_estimate == nullptr ? illixr_plugin_obj->_m_clock->now() + VSYNC_PERIOD : **vsync_estimate;

    return std::chrono::nanoseconds{target_time.time_since_epoch()}.count();
}

extern "C" int64_t illixr_get_now_ns() {
    //assert(illixr_plugin_obj && "illixr_plugin_obj must be initialized first.");
    return std::chrono::duration_cast<std::chrono::nanoseconds>((illixr_plugin_obj->_m_clock->now()).time_since_epoch()).count();
}
