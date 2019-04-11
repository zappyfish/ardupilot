//
// Created by Liam Kelly on 2/10/19.
//
#include "Copter.h"

#include "mode.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/packet_manager.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/directions_packet.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/packet_manager.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/mode_packet.h"

#define RUN_CALLS_PER_SECOND 400
#define SQUARE_TARGET_ALTITUDE 600
#define TEST_ANGLE 600
#define AUTONOMOUS_DESCEND_RATE -75
#define MAX_AUTO_ANGLE 1000

bool Copter::ModeTargetAuto::init(bool ignore_checks) {
    // Superclass method??
    pos_control->set_alt_target_to_current_alt();

    // Setup callback here

    should_land = false;
    saw_target = false;
    should_go_up = false;
    m_run_count = 0;

    auto_roll = 0;
    auto_pitch = 0;

    target_location_callback.name = directions_packet::PACKET_NAME;
    target_location_callback.args = this;
    target_location_callback.callback = &ModeTargetAuto::target_packet_callback;

    packet_manager::get_instance().set_packet_callback(&target_location_callback);
    search_for_target();

    pos_control->set_speed_z(-400, 400);

    mode_packet* packet = new mode_packet(true, true);
    packet_manager::get_instance().send_packet(packet);

    return true;
}

void Copter::ModeTargetAuto::de_init() {
    packet_manager::get_instance().remove_packet_callback(&target_location_callback);
    mode_packet* packet = new mode_packet(false, false);
    packet_manager::get_instance().send_packet(packet);
    saw_target = false;
    should_land = false;
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
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(auto_roll, auto_pitch, 0);
    pos_control->set_alt_target_from_climb_rate_ff(0, G_Dt, false);
}

void Copter::ModeTargetAuto::set_lean_angles(float angle_x, float angle_y) {
    // TODO: use x and y here
    // update angle targets that will be passed to stabilize controller
    if (angle_y > MAX_AUTO_ANGLE) {
        auto_pitch = MAX_AUTO_ANGLE;
    } else if (angle_y < -MAX_AUTO_ANGLE) {
        auto_pitch = -MAX_AUTO_ANGLE;
    } else {
        auto_pitch = angle_y; // This is now totally governed (including sign) by the image processing controller
    }

    if (angle_x > MAX_AUTO_ANGLE) {
        auto_roll = MAX_AUTO_ANGLE;
    } else if (angle_x < -MAX_AUTO_ANGLE) {
        auto_roll = -MAX_AUTO_ANGLE;
    } else {
        auto_roll = angle_x;
    }
}

void Copter::ModeTargetAuto::land() {
    // TODO: complete me
    // TODO: set velocity to 0
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(auto_roll, auto_pitch, 0);
    pos_control->set_alt_target_from_climb_rate_ff(AUTONOMOUS_DESCEND_RATE, G_Dt, false);
}

void Copter::ModeTargetAuto::search_for_target() {
    // TODO: Complete me
    // TODO: set velocity to 0
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(0, 0, 0);
    pos_control->set_alt_target_from_climb_rate_ff(0, G_Dt, false);
}

void Copter::ModeTargetAuto::target_packet_callback(const char *packet_type, std::vector<const char *> keys,
                                            std::vector<const char *> values, void *args) {
    directions_packet packet(keys, values);
    ModeTargetAuto* mode = static_cast<ModeTargetAuto*>(args);
    mode->set_lean_angles(packet.get_x(), packet.get_y());
    mode->saw_target = packet.get_saw_target();
    mode->should_land = packet.get_should_land();
}