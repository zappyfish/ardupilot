#include "Copter.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/packet_manager.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/flight_packet.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/uart/pixhawk_uart.h"

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    pixhawk_uart *uart = new pixhawk_uart();
    packet_manager::get_instance().set_uart(uart);
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
    const Vector3f &accel = ins.get_accel(0); // 0 is imu instance
    int16_t roll = ahrs.roll_sensor;
    int16_t pitch = ahrs.pitch_sensor;
    uint16_t yaw = ahrs.yaw_sensor;
    flight_packet *packet = new flight_packet(accel.x * 1000, accel.y * 1000, accel.z  * 1000, roll, pitch, yaw);
    packet_manager::get_instance().send_packet(packet);
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
    // put your 1Hz code here
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
