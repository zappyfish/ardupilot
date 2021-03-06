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
#define VADL_GPS_MODE_MAX_SPEED 200

bool Copter::ModeGPSAuto::init(bool ignore_checks) {

    mode_packet* packet = new mode_packet(true, false);
    packet_manager::get_instance().send_packet(packet);


    copter.initial_destination.set_alt_cm(inertial_nav.get_altitude(), Location_Class::ALT_FRAME::ALT_FRAME_ABOVE_HOME);
    wp_nav->set_wp_destination(copter.initial_destination);

//    if (!copter.has_destination.load() || !copter.has_switched_into_gps_mode_once.load()) {
//        wp_nav->set_wp_destination(copter.initial_destination);
//        copter.has_switched_into_gps_mode_once = true;
//    } else {
//        copter.rf_destination.z = inertial_nav.get_altitude();
//        wp_nav->set_wp_destination(copter.rf_destination, false);
//    }

    return true;
}

void Copter::ModeGPSAuto::de_init() {
    mode_packet* packet = new mode_packet(false, false);
    packet_manager::get_instance().send_packet(packet);
}

void Copter::ModeGPSAuto::run() {
    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // Set speed with linear slowdown as target is approached
    pos_control->set_speed_xy(get_speed_cm());

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    // call z-axis position controller (wpnav should have already updated its alt target)
    pos_control->update_z_controller();

    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), 0); // 0 for target yaw rate
}


float Copter::ModeGPSAuto::get_speed_cm() {

    float dist = wp_nav->get_wp_distance_to_destination();

    float speed = (dist / VADL_TEST_GPS_DISTANCE_CM) * VADL_GPS_MODE_MAX_SPEED;
    if (speed > VADL_GPS_MODE_MAX_SPEED) {
        speed = VADL_GPS_MODE_MAX_SPEED;
    }
    return speed;
}