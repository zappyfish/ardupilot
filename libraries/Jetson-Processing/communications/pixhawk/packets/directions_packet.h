//
// Created by Liam Kelly on 11/12/18.
//

#ifndef NVIDIA_DIRECTIONS_PACKET_H
#define NVIDIA_DIRECTIONS_PACKET_H

#include "pixhawk_packet.h"

using namespace std;

class directions_packet : public pixhawk_packet {

public:

    static const char* PACKET_NAME;

    directions_packet(vector<const char*> keys, vector<const char*> values);

    bool is_heading_new();
    bool is_pose_new();
    bool has_reached_target();

    const char* get_packet_type();

private:

    bool new_heading_data_available;
    bool new_pose_data_available;
    bool reached_target;
};


#endif //NVIDIA_DIRECTIONS_PACKET_H
