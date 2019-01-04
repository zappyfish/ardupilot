//
// Created by Liam Kelly on 10/27/18.
//

#ifndef NVIDIA_RF_PACKET_H
#define NVIDIA_RF_PACKET_H

#include <vector>

using namespace std;

class rf_packet {

    rf_packet(vector<unsigned char> &serialized);
    ~rf_packet();

    vector<unsigned char> serialize();
};


#endif //NVIDIA_RF_PACKET_H
