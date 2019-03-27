//
// Created by Liam Kelly on 2/10/19.
//
#include "Copter.h"
#include <AP_Math/AP_Math.h>

#include "mode.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/packet_manager.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/directions_packet.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/packet_manager.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/mode_packet.h"

#define VADL_TEST_GPS_DISTANCE_CM 1000
#define VADL_GPS_MODE_MAX_SPEED 50

bool Copter::ModeGPSAuto::init(bool ignore_checks) {

    mode_packet* packet = new mode_packet(true, false);
    packet_manager::get_instance().send_packet(packet);

    target_z = inertial_nav.get_altitude();
    const Vector3f &pos = inertial_nav.get_position(); // 0 is imu instance
    go_to_x = pos.x + VADL_TEST_GPS_DISTANCE_CM;
    go_to_y = pos.y;

//    pos_control->set_alt_target_to_current_alt();
//    pos_control->set_xy_target(go_to_x, go_to_y);

    Vector3f destination;
    destination.x = go_to_x;
    destination.y = go_to_y;
    destination.z = target_z;

    wp_nav->set_wp_destination(destination, false); // TODO: check on the second arg, terrain_alt flag

    return true;
}

void Copter::ModeGPSAuto::de_init() {
    mode_packet* packet = new mode_packet(false, false);
    packet_manager::get_instance().send_packet(packet);
}

void Copter::ModeGPSAuto::run() {
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), 0); // 0 for target yaw rate
}


float Copter::ModeGPSAuto::get_speed_cm() {
    const Vector3f &pos = inertial_nav.get_position(); // 0 is imu instance

    float dist_x = go_to_x - pos.x;
    float dist_y = go_to_y - pos.y;

    float dist = safe_sqrt((dist_x * dist_x) + (dist_y * dist_y));

    float speed = (dist / VADL_TEST_GPS_DISTANCE_CM) * VADL_GPS_MODE_MAX_SPEED;
    if (speed > VADL_GPS_MODE_MAX_SPEED) {
        speed = VADL_GPS_MODE_MAX_SPEED;
    }
    return speed;
}