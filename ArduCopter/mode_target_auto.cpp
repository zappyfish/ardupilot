//
// Created by Liam Kelly on 2/10/19.
//
#include "Copter.h"

#include "mode.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/packet_manager.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/directions_packet.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/packet_manager.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/mode_packet.h"

#define SAW_TARGET_MIN_ALTITUDE 500
#define DID_NOT_SEE_TARGET_MAX_ALTITUDE 1000
#define SAW_TARGET_CLIMB_RATE -100
#define DID_NOT_SEE_TARGET_CLIMB_RATE 100
#define RUN_CALLS_PER_SECOND 400
#define SQUARE_TARGET_ALTITUDE 600
#define TEST_ANGLE 600
#define AUTONOMOUS_DESCEND_RATE -150

bool Copter::ModeTargetAuto::init(bool ignore_checks) {
    // Superclass method??
    pos_control->set_alt_target_to_current_alt();

    // Setup callback here

    should_land = false;
    saw_target = false;
    should_go_up = false;
    m_run_count = 0;

    target_location_callback.name = directions_packet::PACKET_NAME;
    target_location_callback.args = this;
    target_location_callback.callback = &ModeTargetAuto::target_packet_callback;

    packet_manager::get_instance().set_packet_callback(&target_location_callback);
    search_for_target();

    pos_control->set_speed_z(-400, 400);

    mode_packet* packet = new mode_packet(true);
    packet_manager::get_instance().send_packet(packet);

    return true;
}

void Copter::ModeTargetAuto::de_init() {
    packet_manager::get_instance().remove_packet_callback(&target_location_callback);
    mode_packet* packet = new mode_packet(false);
    packet_manager::get_instance().send_packet(packet);
}

void Copter::ModeTargetAuto::square() {
    float target_pitch;
    float target_roll;
    if (m_run_count < 3 * RUN_CALLS_PER_SECOND) {
        target_pitch = TEST_ANGLE;
        target_roll = 0;
    } else if (m_run_count < 6 * RUN_CALLS_PER_SECOND) {
        target_pitch = 0;
        target_roll = TEST_ANGLE;
    } else if (m_run_count < 9 * RUN_CALLS_PER_SECOND) {
        target_pitch = -TEST_ANGLE;
        target_roll = 0;
    } else if (m_run_count < 12 * RUN_CALLS_PER_SECOND) {
        target_pitch = 0;
        target_roll = -TEST_ANGLE;
    } else {
        m_run_count = 0;
        target_pitch = TEST_ANGLE;
        target_roll = 0;
    }
    m_run_count++;
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, 0);

    pos_control->set_alt_target_from_climb_rate_ff(0, G_Dt, false); // TODO: Should this be 0?
}

void Copter::ModeTargetAuto::run() {
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);
//    if (inertial_nav.get_position().z >= DID_NOT_SEE_TARGET_MAX_ALTITUDE) {
//        should_go_up = false;
//    } else if (inertial_nav.get_position().z <= SAW_TARGET_MIN_ALTITUDE) {
//        should_go_up = true;
//    }
//    float target_climb_rate = SAW_TARGET_CLIMB_RATE;
//    if (should_go_up) {
//        target_climb_rate = DID_NOT_SEE_TARGET_CLIMB_RATE;
//    }
//    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
//    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0);
    pos_control->set_alt_target_from_climb_rate_ff(0, G_Dt, false);
    if (saw_target) {
        if (should_land) {
            land();
        } else {
            move_to_target();
        }
    } else {
        search_for_target();
    }
    // square();
    pos_control->update_z_controller();
}

void Copter::ModeTargetAuto::move_to_target() {
    float target_roll, target_pitch, target_yaw_rate;
    target_yaw_rate = 0; // Don't yaw

    set_lean_angles(target_roll, target_pitch);
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
//
//    hal.console->printf("roll: %f, pitch: %f", target_roll, target_pitch);
//
//    target_roll = 1000;
//    target_pitch = 1000;
//
//    float target_climb_rate = SAW_TARGET_CLIMB_RATE;
//
//    if (inertial_nav.get_position().z <= SAW_TARGET_MIN_ALTITUDE) {
//        target_climb_rate = 0;
//    }
//
//    pos_control->set_alt_target_from_climb_rate_ff(0, G_Dt, false);

    // pos_control->set_accel_z(-400);

    // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
}

void Copter::ModeTargetAuto::set_lean_angles(float &target_roll, float &target_pitch) {
    // TODO: use x and y here
    // update angle targets that will be passed to stabilize controller


    // target_pitch = -TEST_ANGLE *((target_y / 5.0) * (target_y / 5.0)) ;
//    if (target_y > 0) {
//        target_pitch = -TEST_ANGLE;
//    } else {
//        target_pitch = TEST_ANGLE;
//    }
//    // target_roll = TEST_ANGLE* ((target_x / 5.0) * (target_x / 5.0));
//    if (target_x > 0) {
//        target_roll = TEST_ANGLE;
//    } else {
//        target_roll = -TEST_ANGLE;
//    }
    target_pitch = -target_y * 5;
    target_roll = target_x * 5;
}

void Copter::ModeTargetAuto::land() {
    // TODO: complete me
    // TODO: set velocity to 0
    float target_roll, target_pitch, target_yaw_rate;
    target_yaw_rate = 0; // Don't yaw

    set_lean_angles(target_roll, target_pitch);
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
    pos_control->set_alt_target_from_climb_rate_ff(AUTONOMOUS_DESCEND_RATE, G_Dt, false);
//    float target_climb_rate = DID_NOT_SEE_TARGET_CLIMB_RATE;
//
//    if (inertial_nav.get_position().z >= DID_NOT_SEE_TARGET_MAX_ALTITUDE) {
//        target_climb_rate = 0;
//    }
//
//    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    // pos_control->set_accel_z(250);

}

void Copter::ModeTargetAuto::search_for_target() {
    // TODO: Complete me
    // TODO: set velocity to 0
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0);
    pos_control->set_alt_target_from_climb_rate_ff(0, G_Dt, false);
//    float target_climb_rate = DID_NOT_SEE_TARGET_CLIMB_RATE;
//
//    if (inertial_nav.get_position().z >= DID_NOT_SEE_TARGET_MAX_ALTITUDE) {
//        target_climb_rate = 0;
//    }
//
//    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    // pos_control->set_accel_z(250);
}

void Copter::ModeTargetAuto::set_target(float x, float y) {
    target_x = x;
    target_y = y;
}

void Copter::ModeTargetAuto::target_packet_callback(const char *packet_type, std::vector<const char *> keys,
                                            std::vector<const char *> values, void *args) {
    directions_packet packet(keys, values);
    ModeTargetAuto* mode = static_cast<ModeTargetAuto*>(args);
    mode->set_target(packet.get_x(), packet.get_y());
    mode->saw_target = packet.get_saw_target();
    mode->should_land = packet.get_should_land();
}