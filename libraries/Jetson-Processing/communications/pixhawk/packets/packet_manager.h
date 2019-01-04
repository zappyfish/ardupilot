//
// Created by Liam Kelly on 11/12/18.
//

#ifndef NVIDIA_PACKET_MANAGER_H
#define NVIDIA_PACKET_MANAGER_H

#include "pixhawk_packet.h"
#include <queue>
#include <cstddef>
#if defined(__linux__)|| defined(_WIN32) || defined(__APPLE__)
#include "data_logger.h"
#include "soft_error_entry.h"
#endif
#include "../uart/uart.h"

class packet_manager {

public:

    ~packet_manager();

    static packet_manager& get_instance();

    void send_packet(pixhawk_packet *packet);
    void set_uart(uart* uart_bus);

    /**
     * Will parse ALL current packets in UART into queues and return the number of packet_type packets available
     * @param packet_type
     * @return -1 for unknown packet_type, number of packets available otherwise
     */
    int check_packets(const char* packet_type);

    pixhawk::packet_ptr get_packet(const char* packet_type);

private:

    typedef struct {
        std::queue<pixhawk::packet_ptr> packets;
        const char * name;
    } packet_queue;

    packet_manager();

    size_t parse_for_packet_data(const std::vector<char> &buffer_data, size_t start_ind);
    pixhawk::packet_ptr create_packet(const char* packet_type, std::vector<const char*> keys, std::vector<const char*> values);

    uart *m_uart;

    // This maps packet type to a queue of those packets
    std::vector<packet_queue*> m_packet_queues;

    packet_queue *find_packet_queue(const char * name);

};


#endif //NVIDIA_PACKET_MANAGER_H
