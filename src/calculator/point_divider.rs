use crate::data_reader::structor::LaserPoint;
use std::collections::HashMap;

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