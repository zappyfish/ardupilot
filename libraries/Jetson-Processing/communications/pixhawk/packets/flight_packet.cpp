//
// Created by Liam Kelly on 11/15/18.
//

#include "flight_packet.h"
#include <string>
#include <cstdio>
#include <cstring>

const size_t flight_packet::NUM_KEYS = 6;
const size_t flight_packet::PRECISION = 10;

const char* flight_packet::PACKET_NAME = "flight_packet";
const char* flight_packet::KEYS[] = {"x", "y", "z", "roll", "pitch", "yaw"};

flight_packet::flight_packet(std::vector<const char*> keys, std::vector<const char*> values) {
    for (size_t i = 0; i < NUM_KEYS; i++) {
        m_keys.push_back(KEYS[i]);
        char * value = new char[std::strlen(values[i]) + 1];
        std::strcpy(value, values[i]);
        m_values.push_back(value);
    }
}

flight_packet::flight_packet(double x, double y, double z, double roll, double pitch, double yaw) {
    for (size_t i = 0; i < NUM_KEYS; i ++) {
        m_keys.push_back(KEYS[i]);
    }
    char *value = new char[PRECISION + 2];
    m_values.push_back(to_string(x, value));
    value = new char[PRECISION + 2];
    m_values.push_back(to_string(y, value));
    value = new char[PRECISION + 2];
    m_values.push_back(to_string(z, value));
    value = new char[PRECISION + 2];
    m_values.push_back(to_string(roll, value));
    value = new char[PRECISION + 2];
    m_values.push_back(to_string(pitch, value));
    value = new char[PRECISION + 2];
    m_values.push_back(to_string(yaw, value));
}

flight_packet::~flight_packet() {
    for (size_t i = 0; i < m_values.size(); i++) {
        delete[] m_values[i];
    }
}

double flight_packet::get_x() {
    return std::atof(m_values.at(0));
}

double flight_packet::get_y() {
    return std::atof(m_values.at(1));
}

double flight_packet::get_z() {
    return std::atof(m_values.at(2));
}

double flight_packet::get_roll() {
    return std::atof(m_values.at(3));
}

double flight_packet::get_pitch() {
    return std::atof(m_values.at(4));
}

double flight_packet::get_yaw() {
    return std::atof(m_values.at(5));
}

const char* flight_packet::get_packet_type() {
    return flight_packet::PACKET_NAME;
}


const char* flight_packet::to_string(double &value, char *buffer) {
    // Use precision here
    std::snprintf(buffer, PRECISION + 1, "%10f", value);
    return buffer;
}
