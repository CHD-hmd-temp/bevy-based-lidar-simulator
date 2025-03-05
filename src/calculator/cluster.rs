#![allow(dead_code)]
#![allow(unused_imports)]
use std::collections::HashMap;
use crate::octree::octree::Octree;
use crate::data_reader::structor::{ LaserPoint, Point3 };
use crate::calculator::mavlink_args::MavlinkArgs;
use crate::calculator::coordinate_switch::mid360_to_frd;

fn get_size(boundary: f32, max_depth: u32) -> f32 {
    let mut size = boundary * 2.0;
    for _ in 0..max_depth {
        size /= 2.0;
    }
    size
}

