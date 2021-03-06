#include "Copter.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/packet_manager.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/flight_packet.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/uart/pixhawk_uart.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/ack_packet.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/arming_packet.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/gps_values_packet.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/mode_packet.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/beacon_packet.h"

#define VADL_SERVO_CHANNEL 5

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    pixhawk_uart *uart = new pixhawk_uart();
    packet_manager::get_instance().set_uart(uart);

    acknowledge_callback.name = ack_packet::PACKET_NAME;
    acknowledge_callback.callback = &Copter::vadl_arming_callback;
    acknowledge_callback.args = this;

    packet_manager::get_instance().set_packet_callback(&acknowledge_callback);

    // Now create a gps destination callback
    destination_set_callback.name = gps_values_packet::PACKET_NAME;
    destination_set_callback.callback = &Copter::vadl_destination_callback;
    destination_set_callback.args = this;

    packet_manager::get_instance().set_packet_callback(&destination_set_callback);

}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    const Vector3f &pos = inertial_nav.get_position(); // 0 is imu instance
    int16_t roll = ahrs.roll_sensor;
    int16_t pitch = ahrs.pitch_sensor;
    uint16_t yaw = ahrs.yaw_sensor;
    // Vector3f accel = ins.get_accel();
    flight_packet *packet = new flight_packet(pos.x, pos.y, pos.z, roll, pitch, yaw); // TODO: change me
    packet_manager::get_instance().send_packet(packet);

    // Check packets every 10ms
    packet_manager::get_instance().check_packets();

    // Check to see if beacon is being deployed
    int range = RC_INPUT_MAX_PULSEWIDTH - RC_INPUT_MIN_PULSEWIDTH;
    float percent_pwm_beacon = ((float)(hal.rcin->read(VADL_SERVO_CHANNEL) - RC_INPUT_MIN_PULSEWIDTH)) / range;
    if (percent_pwm_beacon >= 0.5) {
        // Send transmit gps coords
        beacon_packet *beacon = new beacon_packet();
        packet_manager::get_instance().send_packet(beacon);
    }
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
    // put your 50Hz code here
    // Vector3f pos = inertial_nav.get_position();
//    const Vector3f &accel = ins.get_accel(0); // 0 is imu instance
//    int16_t roll = ahrs.roll_sensor;
//    int16_t pitch = ahrs.pitch_sensor;
//    uint16_t yaw = ahrs.yaw_sensor;
//    flight_packet *packet = new flight_packet(accel.x, accel.y, accel.z, roll, pitch, yaw);
//    packet_manager::get_instance().send_packet(packet);
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP

void Copter::userhook_SuperSlowLoop()
{
    // packet_manager::get_instance().check_packets();
    // put your 1Hz code here
    if (shouldSendArmingPacket.load()) {
        arming_packet* jetson_arming_packet = new arming_packet(true);
        packet_manager::get_instance().send_packet(jetson_arming_packet);
    } else if (shouldSendDisarmingPacket.load()) {
        arming_packet* jetson_disarming_packet = new arming_packet(false);
        packet_manager::get_instance().send_packet(jetson_disarming_packet);
    }

    mode_packet *packet;
    if (flightmode == &(mode_target_auto)) {
        packet = new mode_packet(true, true);
    } else if (flightmode == &(mode_gps_auto)) {
        packet = new mode_packet(true, false);
    } else {
        packet = new mode_packet(false, false);
    }
    packet_manager::get_instance().send_packet(packet);

    // Send location update
    const Vector3f &pos = inertial_nav.get_position(); // 0 is imu instance
    gps_values_packet *gps_packet = new gps_values_packet(pos.x, pos.y);
    packet_manager::get_instance().send_packet(gps_packet);
}
#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(uint8_t ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(uint8_t ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(uint8_t ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::vadl_arming_callback(const char *packet_type, std::vector<const char *> keys, std::vector<const char *> values, void *args) {
    Copter* vadl_copter = static_cast<Copter*>(args);

    if (vadl_copter->shouldSendArmingPacket.load()) { // Not sure if this is right
        vadl_copter->shouldSendArmingPacket = false;
    } else if (vadl_copter->shouldSendDisarmingPacket.load()) {
        vadl_copter->shouldSendDisarmingPacket = false;
    }
}

void Copter::vadl_destination_callback(const char *packet_type, std::vector<const char *> keys,
                                       std::vector<const char *> values, void *args) {
    Copter *vadl_copter = static_cast<Copter *>(args);

    gps_values_packet received_packet(keys, values);

    if (received_packet.get_x() != 0 || received_packet.get_y() != 0) {
        vadl_copter->has_destination = true;
        vadl_copter->rf_destination.x = received_packet.get_x();
        vadl_copter->rf_destination.y = received_packet.get_y();
    }
}
