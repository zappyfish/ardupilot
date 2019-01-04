//
// Created by Liam Kelly on 11/15/18.
//

#ifndef ARDUCOPTER_FLIGHT_PACKET_H
#define ARDUCOPTER_FLIGHT_PACKET_H

#include "pixhawk_packet.h"
#include <cstdlib>

class flight_packet : public pixhawk_packet {

public:

    static const char* PACKET_NAME;

    flight_packet(std::vector<const char*> keys, std::vector<const char*> values);
    flight_packet(double x, double y, double z, double roll, double pitch, double yaw);
    ~flight_packet();

    double get_roll();
    double get_pitch();
    double get_yaw();

    double get_x();
    double get_y();
    double get_z();


    const char* get_packet_type();

private:

    static const size_t NUM_KEYS;
    static const size_t PRECISION;
    const char* to_string(double &value, char *buffer);

    static const char *KEYS[];

};


#endif //ARDUCOPTER_FLIGHT_PACKET_H