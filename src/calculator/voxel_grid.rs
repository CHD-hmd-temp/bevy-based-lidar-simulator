#![allow(dead_code)]
use crate::data_reader::structor::LaserPoint;
use std::collections::HashMap;

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