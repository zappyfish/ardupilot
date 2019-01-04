//
// Created by Liam Kelly on 11/9/18.
//

#include "time_manager.h"

time_manager& time_manager::get_instance() {
    static time_manager instance;
    return instance;
}

time_manager::time_manager() : m_start_time(duration_cast<milliseconds>(system_clock::now().time_since_epoch())) {

}

time_manager::~time_manager() {}

size_t time_manager::get_relative_time() {
    milliseconds ms_now = duration_cast<milliseconds>(
            system_clock::now().time_since_epoch()
    );

    return (ms_now - m_start_time).count();
}

void time_manager::reset_clock() {
    m_start_time = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
}