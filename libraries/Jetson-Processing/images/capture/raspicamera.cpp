//
// Created by Liam Kelly on 11/11/18.
//

#ifdef __linux__

#include "raspicamera.h"
#include "soft_error_entry.h"
#include "data_logger.h"

raspicamera::raspicamera() {
    m_raspicam.set(8, CV_8UC3); // TOOD: figure out why the heck I had to hard code in 8 instead of using CV_CAP_PROP_FORMAT (because it wasn't found)
    if(!m_raspicam.open()) {
        std::string category = "camera";
        std::string description = "could not open camera";
        std::map<std::string, std::string> metadata;
        metadata["file"] = "raspicam.cpp";
        metadata["method"] = "constructor";
        soft_error_entry *s_error = new soft_error_entry(category, description, metadata);
        data_logger::get_instance().save_log_entry(s_error);
    }
}

raspicamera::~raspicamera() {}

image_ptr raspicamera::capture_image() {
    cv::Mat result;
    m_raspicam.grab();
    m_raspicam.retrieve(result);
    return std::make_shared<image>(result);
}

#endif
