#![allow(dead_code)]
use crate::octree::octree;
use crate::data_reader::structor::LaserPoint;
use crate::calculator::mavlink_args::MavlinkArgs;
use crate::calculator::coordinate_switch::mid360_to_frd;

fn distance_calculator(
    p1: [f32; 3],
    p2: [f32; 3],
) -> f32 {
    let x = p1[0] - p2[0];
    let y = p1[1] - p2[1];
    let z = p1[2] - p2[2];
    return (x * x + y * y + z * z).sqrt();
}

pub fn crash_warn_for_point(
    point: LaserPoint,
    warn_trigger_distance: f32,
) -> bool {
    let x = point.x;
    let y = point.y;
    let z = point.z;

    let distance = distance_calculator([x, y, z], [0.0, 0.0, 0.0]);
    if distance < warn_trigger_distance {
        return true;
    }
    return false;
}

pub fn crash_warn_for_octree(
    octree_input: &octree::Octree,
    warn_trigger_distance: f32,
) -> (bool, Vec<(f32, [f32; 3])>) {
    // TODO: change input to map
    let octree_map = octree_input.octree_to_map();
    let mut result = false;
    let mut obstacle_list: Vec<(f32, [f32; 3])> = Vec::new();

    for (_, points)  in octree_map {
        for point in points {
            let x = point.x;
            let y = point.y;
            let z = point.z;
            let distance = distance_calculator([x, y, z], [0.0, 0.0, 0.0]);
            if distance < warn_trigger_distance {
                result = true;
                obstacle_list.push((distance, [x, y, z]));
            }
        }
    }
    return (result, obstacle_list);
}

// TODO: implement speed_factor
pub fn obstacle_avoidance(
    obstacle_list: &Vec<(f32, [f32; 3])>,
    _warn_trigger_distance: f32,
    //mavlink_args: &MavlinkArgs,
) -> MavlinkArgs {
    const EPSILON: f32 = 1e-6;
    const MAX_SPEED: f32 = 2.0;
    let mut result = MavlinkArgs::new(
        0,
        1,
        1,
        9,
        0b0000001000000000,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    );
    if obstacle_list.is_empty() {
        return result;
    }
    let mut sorted_obstacle_list = obstacle_list.clone();
    sorted_obstacle_list.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    let (mut sum_x, mut sum_y, mut sum_z) = (0.0, 0.0, 0.0);
    for &(distance, [x, y, z]) in &sorted_obstacle_list {
        let weight = 1.0 / (distance.powi(2) + EPSILON);
        sum_x += -x * weight;
        sum_y += -y * weight;
        sum_z += -z * weight;
    }

    let _minimum_distance = sorted_obstacle_list[0].0;
    // let speed_factor = (1.0 - (warn_trigger_distance / minimum_distance)).clamp(0.0, 1.0);
    // let speed = MAX_SPEED * speed_factor;
    let speed = MAX_SPEED;

    let magnitude = (sum_x.powi(2) + sum_y.powi(2) + sum_z.powi(2)).sqrt();
    if magnitude < EPSILON {
        let (_, [x, y, z]) = sorted_obstacle_list[0];
        let dir_mag = distance_calculator([x, y, z], [0.0, 0.0, 0.0]);
        if dir_mag < EPSILON {
            result.type_mask = 0b010111111111;
            result.yaw_rate = 0.5; // TODO: stop after a while
            return result;
        }
        else {
            result.type_mask = 0b0000001000000000;
            let vx = speed * -x / dir_mag;
            let vy = speed * -y / dir_mag; // Change O-XYZ to O-FRD
            let vz = speed * -z / dir_mag;
            let (vx, vy, vz) = mid360_to_frd(vx, vy, vz);
            result.vx = vx;
            result.vy = vy;
            result.vz = vz;
            return result;
        }
    }
    else {
        result.type_mask = 0b0000001000000000;
        let vx = speed * sum_x / magnitude;
        let vy = speed * sum_y / magnitude; // Change O-XYZ to O-FRD
        let vz = speed * sum_z / magnitude;
        let (vx, vy, vz) = mid360_to_frd(vx, vy, vz);
        result.vx = vx;
        result.vy = vy;
        result.vz = vz;
        return result;
    }
}

// fn flu_to_frd(mavlink_msg: &MavlinkArgs) -> MavlinkArgs {
//     let mut result = MavlinkArgs::new(
//         mavlink_msg.time_boot_ms,
//         mavlink_msg.target_system,
//         mavlink_msg.target_component,
//         mavlink_msg.coordinate_frame,
//         mavlink_msg.type_mask,
//         mavlink_msg.x,
//         mavlink_msg.y,
//         mavlink_msg.z,
//         mavlink_msg.vx,
//         mavlink_msg.vy,
//         mavlink_msg.vz,
//         mavlink_msg.afx,
//         mavlink_msg.afy,
//         mavlink_msg.afz,
//         mavlink_msg.yaw,
//         mavlink_msg.yaw_rate,
//     );
//     let vx = mavlink_msg.vx;
//     let vy = mavlink_msg.vy;
//     let vz = mavlink_msg.vz;
//     let yaw = mavlink_msg.yaw;
//     let yaw_rate = mavlink_msg.yaw_rate;
//     result.vx = vx * yaw.cos() - vy * yaw.sin();
//     result.vy = vx * yaw.sin() + vy * yaw.cos();
//     result.vz = vz;
//     result.yaw = yaw;
//     result.yaw_rate = yaw_rate;
//     return result;
// }