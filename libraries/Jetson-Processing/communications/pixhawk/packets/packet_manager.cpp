//
// Created by Liam Kelly on 11/12/18.
//

#include "packet_manager.h"
#include "flight_packet.h"
#include "directions_packet.h"
#include "test_packet.h"
#if defined(__linux__)|| defined(_WIN32) || defined(__APPLE__)
#include "data_logger.h"
#endif

packet_manager& packet_manager::get_instance() {
    static packet_manager instance;
    return instance;
}

packet_manager::packet_manager() : m_uart(nullptr) {
    // TODO: Add a line here for each packet type
    m_packet_queues[directions_packet::PACKET_NAME] = new packet_queue();
    m_packet_queues[test_packet::PACKET_NAME] = new packet_queue();
    m_packet_queues[flight_packet::PACKET_NAME] = new packet_queue();
}

packet_manager::~packet_manager() {
    if (m_uart != nullptr) {
        delete m_uart;
        m_uart = nullptr;
    }
    for (auto const &pair: m_packet_queues) {
        delete pair.second; // The packet queue itself
    }
}

void packet_manager::send_packet(pixhawk_packet *packet) {
    std::vector<char> serialized = packet->serialize();
    m_uart->write_buffer(serialized);
    delete packet;
}

int packet_manager::check_packets(const char* packet_type) {
    if (m_uart == nullptr) {
        return -1;
    }

    std::vector<char> buffer_data;
    m_uart->read_buffer(buffer_data);

    size_t start_ind = 0;

    while (start_ind < buffer_data.size()) {
        start_ind = parse_for_packet_data(buffer_data, start_ind);
    }

    if (m_packet_queues.find(packet_type) == m_packet_queues.end()) {
        return -1;
    }

    return m_packet_queues[packet_type]->size();
}

/**
 * This is kinda dangerous b/c it returns nullptr, so just be careful when you call it; be sure to check the result!
 * @return
 */
pixhawk::packet_ptr packet_manager::get_packet(const char* packet_type) {
    if (!m_packet_queues[packet_type]->empty()) {
        pixhawk::packet_ptr packet = m_packet_queues[packet_type]->front();
        m_packet_queues[packet_type]->pop();
        return packet;
    } else {
        return nullptr;
    }
}

size_t packet_manager::parse_for_packet_data(const std::vector<char> &buffer_data, size_t start_ind) {
    char checksum = 0;
    std::vector<char> next_token;
    std::vector<char> packet_name;
    std::vector<const char*> keys;
    std::vector<const char*> values;
    bool got_name = false;
    bool is_key = true;
    char c;
    while (start_ind < buffer_data.size()) {
        c = buffer_data.at(start_ind++);
        if (!got_name) {
            if (c != pixhawk_packet::PACKET_START) {
                packet_name.push_back(c);
            } else {
                got_name = true;
            }
        } else {
            switch (c) {
                case pixhawk_packet::PACKET_END: {
                    char *token = new char[next_token.size() + 1];
                    std::copy(next_token.begin(), next_token.end(), token);
                    values.push_back(token); // Guaranteed to be a value if it's the last token
                    // Validate checksum here
                    if (start_ind >= buffer_data.size() || buffer_data.at(start_ind++) != checksum) {
#if defined(__linux__)|| defined(_WIN32) || defined(__APPLE__)
                        std::string category = "packet_manager";
                        std::string description = "checksum did not match";
                        std::map<std::string, std::string> metadata;
                        metadata["file"] = "packet_manager.cpp";
                        metadata["method"] = "parse_for_packet_data";
                        std::string serialized_line(buffer_data.begin(),buffer_data.end());
                        metadata["serialized"] = serialized_line;
                        soft_error_entry *s_error = new soft_error_entry(category, description, metadata);
                        data_logger::get_instance().save_log_entry(s_error);
#endif
                    } else {
                        char *name = new char[packet_name.size() + 1];
                        std::copy(packet_name.begin(), packet_name.end(), name);
                        name[packet_name.size()] = '\0';
                        pixhawk::packet_ptr packet = create_packet(name, keys, values);
                        if (packet != nullptr) {
                            m_packet_queues[packet->get_packet_type()]->push(packet);
                        }
                        delete[] name;
                    }
                    for (size_t k = 0; k < keys.size(); k++) {
                        delete[] keys.at(k);
                    }
                    for (size_t v = 0; v < values.size(); v++) {
                        delete[] values.at(v);
                    }
                    return start_ind;
                }

                case pixhawk_packet::DELIMITER: {
                    char *token = new char[next_token.size() + 1];
                    std::copy(next_token.begin(), next_token.end(), token);
                    if (is_key) {
                        // We're recording a value and reached the end, so store the key-value pair and reset
                        keys.push_back(token);
                    } else {
                        values.push_back(token);
                    }
                    next_token.clear();
                    is_key = !is_key;
                    break;
                }

                default: {
                    next_token.push_back(c);
                    checksum = checksum ^ c;
                    break;
                }
            }
        }
    }
    for (size_t k = 0; k < keys.size(); k++) {
        delete[] keys.at(k);
    }
    for (size_t v = 0; v < values.size(); v++) {
        delete[] values.at(v);
    }
    return start_ind;
}

/**
 * Add a case here for each type of packet
 * @param packet_type
 * @param fields
 * @return a packet_ptr if there were no issues, nullptr if there was problem (invalid name)
 */
pixhawk::packet_ptr packet_manager::create_packet(const char* packet_type, std::vector<const char*> keys, std::vector<const char*> values) {
    if (std::strcmp(packet_type, directions_packet::PACKET_NAME) == 0) {
        return std::make_shared<directions_packet>(keys, values);
    // TODO: add else-if's as required for other packet types
    } else if (std::strcmp(packet_type, test_packet::PACKET_NAME) == 0) {
        return std::make_shared<test_packet>(keys, values);
    } else if (std::strcmp(packet_type, flight_packet::PACKET_NAME) == 0) {
        return std::make_shared<flight_packet>(keys, values);
    } else {
#if defined(__linux__)|| defined(_WIN32) || defined(__APPLE__)
            std::string category = "packet_manager";
            std::string description = "packet name not found";
            std::map<std::string, std::string> metadata;
            metadata["file"] = "packet_manager.cpp";
            metadata["method"] = "parse_for_packet_data";
            metadata["name_given"] = packet_type;
            soft_error_entry *s_error = new soft_error_entry(category, description, metadata);
            data_logger::get_instance().save_log_entry(s_error);
#endif
        return nullptr;
    }
}

void packet_manager::set_uart(uart *uart_bus) {
    m_uart = uart_bus;
}
