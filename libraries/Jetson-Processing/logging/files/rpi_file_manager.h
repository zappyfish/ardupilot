//
// Created by Liam Kelly on 11/8/18.
//

#ifndef NVIDIA_RPI_FILE_MANAGER_H
#define NVIDIA_RPI_FILE_MANAGER_H

#include <ostream>
#include <boost/filesystem.hpp>
#include <boost/lambda/bind.hpp>
#include <boost/iostreams/stream.hpp>
#include <boost/iostreams/device/file.hpp>
#include "file_manager.h"
#include <opencv2/opencv.hpp>
#include <ctime>

using namespace boost::filesystem;
using namespace boost::lambda;
namespace io = boost::iostreams;

class rpi_file_manager : public file_manager {

public:

    /**
     * We need this constructor which takes a custom root directory so that we can run tests and not mess up
     * our flight data file system.
     * @param root_dir
     */
    rpi_file_manager(std::string root_dir);
    rpi_file_manager();
    ~rpi_file_manager();

    size_t get_flight_number();
    void write_line_to_file(std::string file_name, std::string line);
    void write_image(image &img);
    void multiline_write(std::string file_name, std::string lines[], size_t num_lines);
    void multiline_write(std::string file_name, std::vector<std::string> lines);

private:

    static const std::string DIRECTORY_PATH[];
    static const int PATH_LENGTH;

    size_t m_flight_number;
    std::string m_flight_path;

    void create_flight_info();
};


#endif //NVIDIA_RPI_FILE_MANAGER_H
