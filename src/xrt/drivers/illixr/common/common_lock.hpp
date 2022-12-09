//
// Created by madhuparna on 10/19/22.
//

#ifndef ILLIXR_NATIVE_ACTIVITY_COMMON_LOCK_HPP
#define ILLIXR_NATIVE_ACTIVITY_COMMON_LOCK_HPP

#endif //ILLIXR_NATIVE_ACTIVITY_COMMON_LOCK_HPP

#include "data_format.hpp"
#include "phonebook.hpp"
#include <mutex>

using namespace ILLIXR;

class common_lock : public phonebook::service {
public:
    std::mutex lock;
    virtual void     get_lock()                            = 0;
    virtual void     release_lock()                        = 0;
    virtual ~common_lock() { }
};
