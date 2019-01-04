//a
// Created by Liam Kelly on 11/15/18.
//

#include "pixhawk_entry.h"

const std::string pixhawk_entry::ENTRY_ID = "PIXHAWK_DATA";

pixhawk_entry::pixhawk_entry(flight_packet *packet) {
    add_data("x", packet->get_x());
    add_data("y", packet->get_y());
    add_data("z", packet->get_z());
    add_data("roll", packet->get_roll());
    add_data("pitch", packet->get_pitch());
    add_data("yaw", packet->get_yaw());
}

pixhawk_entry::pixhawk_entry(pixhawk_packet *packet) : pixhawk_entry(static_cast<flight_packet*>(packet)) {}

std::string pixhawk_entry::get_name() {
    return pixhawk_entry::ENTRY_ID;
}