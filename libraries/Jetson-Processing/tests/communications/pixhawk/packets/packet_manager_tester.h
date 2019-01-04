//
// Created by Liam Kelly on 11/13/18.
//

#ifndef NVIDIA_PACKET_MANAGER_TESTER_H
#define NVIDIA_PACKET_MANAGER_TESTER_H

#include "packet_manager.h"
#include "test_packet.h"
#include "directions_packet.h"
#include "test_uart.h"
#include "flight_packet.h"
#include "pixhawk_entry.h"

/**
 * In this test case, we ensure that the packet manager correctly returns the number of packets available of a given type.
 * For example, if it's fed invalid packet data, it doesn't record that packets are available. Also, if it has packets,
 * it only returns packets available if it has packets of that specific packet type.
 */
TEST_CASE("Packet Manager Test Gets Number of Packets Available Correctly", "[packet_manager]") {
    test_uart *uart = new test_uart();

    packet_manager::get_instance().set_uart(uart);

    REQUIRE(packet_manager::get_instance().check_packets("jfkdlsjflsd") == -1); // Invalid packet_type
    REQUIRE(packet_manager::get_instance().check_packets(test_packet::PACKET_NAME) == 0); // Valid packet, but none recorded

    std::vector<char> data;
    std::string invalid_string;
    invalid_string.append(test_packet::PACKET_NAME);
    invalid_string.append(":key1,val1,key2,val2,not_terminated_correctly");
    data = std::vector<char>(invalid_string.begin(), invalid_string.end());
    uart->write_buffer(data);

    invalid_string.clear();

    REQUIRE(packet_manager::get_instance().check_packets(test_packet::PACKET_NAME) == 0);

    invalid_string.append(test_packet::PACKET_NAME);
    invalid_string.append(":correct,packet,but,incorrect,checksum,lelz;*");

    data = std::vector<char>(invalid_string.begin(), invalid_string.end());

    uart->write_buffer(data);

    REQUIRE(packet_manager::get_instance().check_packets(test_packet::PACKET_NAME) == 0);

    std::vector<const char*> keys;
    std::vector<const char*> values;

    keys.push_back("lmao");
    values.push_back("8");

    keys.push_back("memes");
    values.push_back("imtired");

    test_packet *packet = new test_packet(keys, values);

    std::vector<char> s = packet->serialize();

    packet_manager::get_instance().send_packet(packet); // This works EXCLUSIVELY for test_uart

    REQUIRE(packet_manager::get_instance().check_packets(directions_packet::PACKET_NAME) == 0); // Wrong packet type
    REQUIRE(packet_manager::get_instance().check_packets(test_packet::PACKET_NAME) == 1); // Correct packet type

    // Grab the packet, make sure size is updated
    packet_manager::get_instance().get_packet(test_packet::PACKET_NAME);
    REQUIRE(packet_manager::get_instance().check_packets(test_packet::PACKET_NAME) == 0);
}


TEST_CASE("Packet Manager Test Data Sent and Received Correctly", "[packet_manager]") {
    test_uart *uart = new test_uart();

    packet_manager::get_instance().set_uart(uart);

    std::vector<const char*> keys;
    std::vector<const char*> values;

    keys.push_back("lmao");
    values.push_back("8");

    keys.push_back("memes");
    values.push_back("imtired");

    test_packet *sent = new test_packet(keys, values);
    std::vector<char> s_v = sent->serialize();

    packet_manager::get_instance().send_packet(sent);

    packet_manager::get_instance().check_packets(test_packet::PACKET_NAME);
    pixhawk::packet_ptr received = packet_manager::get_instance().get_packet(test_packet::PACKET_NAME);

    std::vector<char> r_v = received->serialize();
    REQUIRE(s_v.size() == r_v.size());

    for (size_t i = 0; i < s_v.size(); i++) {
        REQUIRE(s_v.at(i) == r_v.at(i));
    }
}

TEST_CASE("Packet Manager Test Can Reconstruct Classes", "[packet_manager]") {
    test_uart *uart = new test_uart();

    packet_manager::get_instance().set_uart(uart);

    flight_packet *sent = new flight_packet(1, 2, 3, 4, 5, 6);

    packet_manager::get_instance().send_packet(sent);

    packet_manager::get_instance().check_packets(flight_packet::PACKET_NAME);
    pixhawk::packet_ptr received = packet_manager::get_instance().get_packet(flight_packet::PACKET_NAME);

    pixhawk_entry entry(received.get());

    REQUIRE(true);
}

#endif //NVIDIA_PACKET_MANAGER_TESTER_H