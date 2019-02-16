//
// Created by Liam Kelly on 2/10/19.
//
#include "Copter.h"

#include "mode.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/packet_manager.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/directions_packet.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/packet_manager.h"

bool Copter::ModeTargetAuto::init(bool ignore_checks) {
    // Superclass method??
    pos_control->set_alt_target_to_current_alt();

    // Setup callback here

    should_land = false;
    saw_target = false;

    target_location_callback.name = directions_packet::PACKET_NAME;
    target_location_callback.args = this;
    target_location_callback.callback = &ModeTargetAuto::target_packet_callback;

    packet_manager::get_instance().set_packet_callback(&target_location_callback);
    search_for_target();

    pos_control->set_speed_z(-400, 400);

    return true;
}

void Copter::ModeTargetAuto::de_init() {
    packet_manager::get_instance().remove_packet_callback(&target_location_callback);
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
    pos_control->update_z_controller();
    hal.console->printf("z: %f\n", inertial_nav.get_position().z);
}

void Copter::ModeTargetAuto::move_to_target() {
//    float target_roll, target_pitch, target_yaw_rate;
//    target_yaw_rate = 0; // Don't yaw
//
//    set_lean_angles(target_roll, target_pitch);
//
//    hal.console->printf("roll: %f, pitch: %f", target_roll, target_pitch);
//
//    target_roll = 1000;
//    target_pitch = 1000;

    float target_climb_rate = -400;

    if (inertial_nav.get_position().z <= 20) {
        target_climb_rate = 0;
    }

    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);

    // pos_control->set_accel_z(-400);

    // attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);
}

void Copter::ModeTargetAuto::set_lean_angles(float &target_roll, float &target_pitch) {
    // TODO: use x and y here
    // update angle targets that will be passed to stabilize controller

    // accel_forward = target_y (speed) - current_y (speed)
    // accel_right = target_x (speed) - current_x (speed)

    Vector3f current_vel = pos_control->get_desired_velocity();
    // Vector2f ground_speed = ahrs.groundspeed_vector();

    float current_vel_mag = safe_sqrt((current_vel.x * current_vel.x) + (current_vel.y * current_vel.y));
    float desired_vel_mag = safe_sqrt((target_x * target_x) + (target_y * target_y));

    float accel_forward = 0;
    float accel_right = 0;

    // in result, x is forward, y is right
    Vector2f earth_xy_velocity;
    earth_xy_velocity.x = current_vel.x;
    earth_xy_velocity.y = current_vel.y;
    Vector2f body_frame_current_vel_xy = ahrs.rotate_earth_to_body2D(earth_xy_velocity);

    if (desired_vel_mag > 0.0) {
        float scale_factor = (float)fabsf(current_vel_mag / desired_vel_mag);
        accel_forward = (target_y * scale_factor) - (float)body_frame_current_vel_xy.x;
        accel_right = (target_x * scale_factor) - (float)body_frame_current_vel_xy.y;
    }

    target_pitch = atanf(-accel_forward/(GRAVITY_MSS * 100.0f))*(18000.0f/M_PI);
    float cos_pitch_target = cosf(target_pitch*M_PI/18000.0f);
    target_roll = atanf(accel_right*cos_pitch_target/(GRAVITY_MSS * 100.0f))*(18000.0f/M_PI);
}

void Copter::ModeTargetAuto::land() {
    // TODO: complete me
    // TODO: set velocity to 0
    float target_climb_rate = 400;

    if (inertial_nav.get_position().z >= 40) {
        target_climb_rate = 0;
    }

    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
    // pos_control->set_accel_z(250);
}

void Copter::ModeTargetAuto::search_for_target() {
    // TODO: Complete me
    // TODO: set velocity to 0
    float target_climb_rate = 400;

    if (inertial_nav.get_position().z >= 40) {
        target_climb_rate = 0;
    }

    pos_control->set_alt_target_from_climb_rate_ff(target_climb_rate, G_Dt, false);
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