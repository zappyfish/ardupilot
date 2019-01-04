//
// Created by Liam Kelly on 11/12/18.
//

#include "directions_packet.h"

const char* directions_packet::PACKET_NAME = "directions";

const char* directions_packet::get_packet_type() {
    return directions_packet::PACKET_NAME;
}

directions_packet::directions_packet(vector<const char*> keys, vector<const char*> values) {}