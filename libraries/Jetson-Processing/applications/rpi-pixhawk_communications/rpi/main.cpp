//
// Created by Liam Kelly on 11/15/18.
//

#include "image_buffer.h"
#include "raspicamera.h"
#include "rpi_file_manager.h"
#include "data_logger.h"
#include "flight_packet.h"
#include "time_manager.h"
#include "pixhawk_entry.h"
#include "rpi_uart.h"
#include "packet_manager.h"
#include "test_uart.h"

int main() {
    size_t start_time = time_manager::get_instance().get_relative_time();
    rpi_file_manager *f_manager = new rpi_file_manager();
    data_logger::get_instance().start_flight_session(f_manager);

#ifdef __linux__
    rpi_uart *uart = new rpi_uart(rpi_uart::UART_MAIN, 115200);
#else
    test_uart *uart = new test_uart();
#endif
    packet_manager::get_instance().set_uart(uart);

    while (time_manager::get_instance().get_relative_time() - start_time < 10000) {
        size_t time = time_manager::get_instance().get_relative_time();
        flight_packet *packet = new flight_packet(time, time * 2, time * 3, time * 4, time * 5, time * 6);
        packet_manager::get_instance().send_packet(packet);
        size_t num_packets;
        if ((num_packets = packet_manager::get_instance().check_packets(flight_packet::PACKET_NAME))) {
            for (size_t i = 0; i < num_packets; i++) {
                pixhawk_entry *entry = new pixhawk_entry(packet_manager::get_instance().get_packet(flight_packet::PACKET_NAME).get());
                data_logger::get_instance().save_log_entry(entry);
            }
        }
    }

    std::cout << "done\n";

    return 0;
}