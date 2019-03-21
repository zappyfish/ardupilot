//
// Created by Liam Kelly on 2/10/19.
//
#include "Copter.h"

#include "mode.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/packet_manager.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/directions_packet.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/packet_manager.h"
#include "../libraries/Jetson-Processing/communications/pixhawk/packets/mode_packet.h"


bool Copter::ModeGPSAuto::init(bool ignore_checks)

    mode_packet* packet = new mode_packet(true);
    packet_manager::get_instance().send_packet(packet);

    return true;
}

void Copter::ModeTargetAuto::de_init() {
    mode_packet* packet = new mode_packet(false);
    packet_manager::get_instance().send_packet(packet);
}

void Copter::ModeGPSAuto::run() {
    motors->set_desired_spool_state(AP_Motors::DESIRED_THROTTLE_UNLIMITED);

    // Go to desired gps
    if (has_destination) {
        pos_control->set_xy_target(destination_x, destination_y);
    } else {
        // TODO: loiter
        precision_loiter_xy();
    }

    pos_control->update_z_controller();
}

void Copter::ModeGPSAuto::precision_loiter_xy() {
    loiter_nav->clear_pilot_desired_acceleration();
    Vector2f target_pos, target_vel_rel;
    if (!copter.precland.get_target_position_cm(target_pos)) {
        target_pos.x = inertial_nav.get_position().x;
        target_pos.y = inertial_nav.get_position().y;
    }
    if (!copter.precland.get_target_velocity_relative_cms(target_vel_rel)) {
        target_vel_rel.x = -inertial_nav.get_velocity().x;
        target_vel_rel.y = -inertial_nav.get_velocity().y;
    }
    pos_control->set_xy_target(target_pos.x, target_pos.y);
    pos_control->override_vehicle_velocity_xy(-target_vel_rel);
}
