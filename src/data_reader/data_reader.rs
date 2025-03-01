#![allow(dead_code)]

use std::error::Error;
use std::fs::File;
use std::io::Read;
use csv::Reader;
//use serde::Deserialize;
use std::collections::HashMap;
use crate::data_reader::structor::LaserPoint;

// #[allow(non_snake_case)]
// #[derive(Debug, Deserialize, Clone)]
// pub struct LaserPoint {
//     #[serde(rename = "Reflectivity")]
//     pub reflectivity: u8,
//     #[serde(rename = "X")]
//     pub x: f32,
//     #[serde(rename = "Y")]
//     pub y: f32,
//     #[serde(rename = "Z")]
//     pub z: f32,
//     #[serde(skip)]
//     _remainder: String,
// }

// impl LaserPoint {
//     pub fn new(x: f32, y: f32, z: f32, reflectivity: u8) -> Self {
//         Self {
//             x,
//             y,
//             z,
//             reflectivity,
//         }
//     }
// }

/// Read the laser points from a CSV file and return a vector of `LaserPoint`s.
pub fn read_laser_points(file_path: &str, boundary: f32) -> Result<Vec<LaserPoint>, Box<dyn Error>> {
    let path = std::path::Path::new(file_path);
    let mut file = File::open(&path)?;
    let mut contents = String::new();
    file.read_to_string(&mut contents)?;

    let mut reader = Reader::from_reader(contents.as_bytes());
    let mut points = Vec::new();
    for result in reader.deserialize() {
        let point: LaserPoint = result?;
        if env_sensor(&point, boundary) {
            points.push(point);
        }
    }

    Ok(points)
}

pub fn divide_points(
    points: Vec<LaserPoint>,
) -> HashMap<u8, Vec<LaserPoint>> {
    let mut grouped_points: HashMap<u8, Vec<LaserPoint>> = HashMap::new();

    for point in points {
        grouped_points
            .entry(point.reflectivity) // Get the group corresponding to the Reflectivity
            .or_insert_with(Vec::new) // Initialize a new vector if it doesn't exist
            .push(point);
    }

    grouped_points
}

pub fn env_sensor(point: &LaserPoint, boundary: f32) -> bool {
    point.x > -boundary && point.x < boundary && point.y > -boundary && point.y < boundary && point.z > -boundary && point.z < boundary
}

pub fn voxel_grid_filter(points: &[LaserPoint], voxel_size: f32) -> Vec<LaserPoint> {
    if voxel_size < 0.05 {
        return points.to_vec();
    }
    type VoxelKey = (i32, i32, i32);
    let mut voxel_map: HashMap<VoxelKey, Vec<&LaserPoint>> = HashMap::new();

    for point in points {
        let idx_x = (point.x / voxel_size).floor() as i32;
        let idx_y = (point.y / voxel_size).floor() as i32;
        let idx_z = (point.z / voxel_size).floor() as i32;
        let key = (idx_x, idx_y, idx_z);

        voxel_map.entry(key).or_default().push(point);
    }

    voxel_map
        .into_values()
        .filter_map(|points| {
            if points.is_empty() {
                return None;
            }

            let total = points.len() as f32;
            let sum_x = points.iter().map(|p| p.x).sum::<f32>();
            let sum_y = points.iter().map(|p| p.y).sum::<f32>();
            let sum_z = points.iter().map(|p| p.z).sum::<f32>();
            let sum_refl = points.iter().map(|p| p.reflectivity as f32).sum::<f32>();

            Some(LaserPoint::new(
                sum_x / total,
                sum_y / total,
                sum_z / total,
                (sum_refl / total).round() as u8,
            ))
        })
        .collect()
}