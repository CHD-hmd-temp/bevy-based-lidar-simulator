#![allow(dead_code)]

use std::collections::HashMap;
use bevy::ecs::system::Resource;

use crate::data_reader::structor::LaserPoint;

#[derive(Debug)]
#[derive(PartialEq)]
pub enum Occupancy {
    Free,
    Occupied,
}

#[derive(Debug)]
pub enum OctreeNode {
    Internal {
        bounds: [[f32; 3]; 2],
        center: [f32; 3],
        depth: u32,
        children: [Box<OctreeNode>; 8],
    },
    Leaf {
        bounds: [[f32; 3]; 2],
        center: [f32; 3],
        depth: u32,
        occupancy: Occupancy,
        reflectivity: [u32; 2],
    }
}

impl OctreeNode {
    /// Cast a ray into the octree and return the distance to the closest hit
    pub fn cast_ray(
        &self,
        origin: [f32; 3],
        direction: [f32; 3],
        max_distance: f32,
        current_min: &mut Option<f32>,
    ) -> Option<f32> {
        let bbox = self.bounds();
        let (t_enter_raw, t_exit) = match Self::aabb_ray_intersection(bbox, origin, direction) {
            Some((enter, exit)) => (enter, exit),
            None => return None,
        };

        // Clamp t_enter to 0.0 if the ray starts inside the AABB
        let t_enter = t_enter_raw.max(0.0);
        if t_enter > max_distance || t_exit < 0.0 {
            return None;
        }

        match self {
            OctreeNode::Internal { children, .. } => {
                let current_max = current_min.map_or(max_distance, |t| t.min(max_distance));
                let mut candidates = Vec::new();

                // Collect potential child candidates with their enter times
                for child in children.iter() {
                    let child_bbox = child.bounds();
                    if let Some((child_enter_raw, child_exit)) =
                        Self::aabb_ray_intersection(child_bbox, origin, direction)
                    {
                        let child_enter = child_enter_raw.max(0.0);
                        // Skip children that are too far or behind the ray
                        if child_enter > current_max || child_exit < 0.0 {
                            continue;
                        }
                        candidates.push((child_enter, child));
                    }
                }

                // Sort children by their enter time to process closer ones first
                candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

                let mut closest = None;
                for (child_enter, child) in candidates {
                    // Update the current max distance based on the closest found so far
                    let current_max = current_min.map_or(max_distance, |t| t.min(max_distance));
                    if child_enter > current_max {
                        continue;
                    }

                    if let Some(t) = child.cast_ray(origin, direction, current_max, current_min) {
                        // Update closest if this child provides a closer hit
                        if closest.is_none() || t < closest.unwrap() {
                            closest = Some(t);
                        }
                    }
                }

                closest
            }

            OctreeNode::Leaf { occupancy, .. } => {
                if *occupancy == Occupancy::Occupied && t_enter <= max_distance {
                    // Update the current_min if this hit is closer
                    match current_min {
                        Some(min) if t_enter < *min => {
                            *current_min = Some(t_enter);
                            Some(t_enter)
                        }
                        None => {
                            *current_min = Some(t_enter);
                            Some(t_enter)
                        }
                        _ => None,
                    }
                } else {
                    None
                }
            }
        }
    }

    /// Get the AABB bounds of the node
    pub fn bounds(&self) -> &[[f32; 3]; 2] {
        match self {
            OctreeNode::Leaf { bounds, .. } => bounds,
            OctreeNode::Internal { bounds, .. } => bounds,
        }
    }

    /// Determine if a ray intersects with an AABB and compute the intersection parameters
    fn aabb_ray_intersection(bbox: &[[f32; 3]; 2], origin: [f32; 3], direction: [f32; 3]) -> Option<(f32, f32)> {
        let mut t_enter = -std::f32::INFINITY;
        let mut t_leave = std::f32::INFINITY;

        for i in 0..3 {
            let dir = direction[i];
            if dir.abs() < std::f32::EPSILON {
                // Ray is parallel to the axis; check if origin is within the AABB
                if origin[i] < bbox[0][i] || origin[i] > bbox[1][i] {
                    return None;
                }
                continue;
            }

            let inv_dir = 1.0 / dir;
            let t1 = (bbox[0][i] - origin[i]) * inv_dir;
            let t2 = (bbox[1][i] - origin[i]) * inv_dir;

            let (t_1, t_2) = if t1 < t2 { (t1, t2) } else { (t2, t1) };

            t_enter = t_enter.max(t_1);
            t_leave = t_leave.min(t_2);

            if t_enter > t_leave {
                return None;
            }
        }

        Some((t_enter, t_leave))
    }
}

#[derive(Resource)]
pub struct Octree {
    root: OctreeNode,
}

impl Octree {
    pub fn new(
        bounds: [[f32; 3]; 2]
    ) -> Self {
        Octree {
            root: OctreeNode::Leaf {
                bounds,
                center: [
                    (bounds[0][0] + bounds[1][0]) / 2.0,
                    (bounds[0][1] + bounds[1][1]) / 2.0,
                    (bounds[0][2] + bounds[1][2]) / 2.0,
                ],
                depth: 0,
                occupancy: Occupancy::Free,
                reflectivity: [0, 0],
            }
        }
    }

    /// Expose the root node for visualization
    pub fn get_root_mut(&mut self) -> &mut OctreeNode {
        &mut self.root
    }

    pub fn insert(
        &mut self,
        point: [f32; 3],
        max_depth: u32,
        point_reflectivity: u8
    ) -> Result<(), String> {
        //log::trace!("Inserting point {:?} into octree", point);
        Self::insert_internal(&mut self.root, point, 0, max_depth, point_reflectivity)
    }

    fn insert_internal(
        node: &mut OctreeNode,
        point: [f32; 3],
        current_depth: u32,
        max_depth: u32,
        point_reflectivity: u8
    ) -> Result<(), String> {
        // point out-of-bounds check
        let bounds = match node {
            OctreeNode::Leaf { bounds, .. } => bounds,
            OctreeNode::Internal { bounds, .. } => bounds,
        };
        let epsilon = 0.00001;
        for i in 0..3 {
            if point[i] < bounds[0][i] - epsilon || point[i] > bounds[1][i] + epsilon {
                //return Err("Point out of bounds".to_string());
                return Ok(());
            }
        }

        match node {
            OctreeNode::Internal { center, children, .. } => {
                let index = Self::get_index(center, point);
                let child = &mut children[index];
                Self::insert_internal(child, point, current_depth + 1, max_depth, point_reflectivity)
            }
            #[allow(unused_variables)]
            OctreeNode::Leaf { depth, occupancy, reflectivity, .. } => {
                if *occupancy == Occupancy::Occupied {
                    //log::trace!("Point {:?} already occupied at depth {}", point, depth);
                    reflectivity[0] += point_reflectivity as u32;
                    reflectivity[1] += 1;
                    return Ok(());
                }
                else {
                    if current_depth < max_depth {
                        //log::debug!("Splitting leaf node at depth {}", current_depth);
                        Self::split(node, current_depth)?;
                        Self::insert_internal(node, point, current_depth, max_depth, point_reflectivity)
                    }
                    else {
                        //log::trace!("Marking node occupied at depth {}", current_depth);
                        *occupancy = Occupancy::Occupied;
                        reflectivity[0] += point_reflectivity as u32;
                        reflectivity[1] += 1;
                        Ok(())
                    }
                }

            }
        }
    }

    fn get_index(
        center: &[f32; 3],
        point: [f32; 3]
    ) -> usize {
        let mut index = 0;
        for i in 0..3 {
            if point[i] > center[i] {
                index |= 1 << i;
            }
        }
        index
    }

    fn split(
        node: &mut OctreeNode,
        current_depth: u32
    ) -> Result<(), String> {
        let center = match *node {
            OctreeNode::Leaf { center, .. } => center,
            _ => return Err("Cannot split non-leaf node".to_string()),
        };
        let bounds = match *node {
            OctreeNode::Leaf { bounds, .. } => bounds,
            _ => return Err("Cannot split non-leaf node".to_string()),
        };
        let children = match node {
            OctreeNode::Leaf { .. } => Self::create_children(&bounds, &center, current_depth),
            _ => return Err("Cannot split non-leaf node".to_string()),
        };

        *node = OctreeNode::Internal {
            bounds: bounds,
            center: center,
            depth: current_depth,
            children,
        };

        Ok(())
    }

    fn create_children(
        parent_bounds: &[[f32; 3]; 2],
        parent_center: &[f32; 3],
        parent_depth: u32
    ) -> [Box<OctreeNode>; 8] {
        let mut children: [Box<OctreeNode>; 8] = [
            Box::new(OctreeNode::Leaf { bounds: [[0.0; 3]; 2],center: [0.0; 3], occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0] }),
            Box::new(OctreeNode::Leaf { bounds: [[0.0; 3]; 2],center: [0.0; 3], occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0] }),
            Box::new(OctreeNode::Leaf { bounds: [[0.0; 3]; 2],center: [0.0; 3], occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0] }),
            Box::new(OctreeNode::Leaf { bounds: [[0.0; 3]; 2],center: [0.0; 3], occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0] }),
            Box::new(OctreeNode::Leaf { bounds: [[0.0; 3]; 2],center: [0.0; 3], occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0] }),
            Box::new(OctreeNode::Leaf { bounds: [[0.0; 3]; 2],center: [0.0; 3], occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0] }),
            Box::new(OctreeNode::Leaf { bounds: [[0.0; 3]; 2],center: [0.0; 3], occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0] }),
            Box::new(OctreeNode::Leaf { bounds: [[0.0; 3]; 2],center: [0.0; 3], occupancy: Occupancy::Free, depth: 0, reflectivity: [0, 0] }),
        ];
        
        for i in 0..8 {
            let child_bounds = Self::calculate_child_bounds(&parent_bounds, &parent_center, i);
            children[i] = Box::new(OctreeNode::Leaf {
                bounds: child_bounds,
                center: [
                    (child_bounds[0][0] + child_bounds[1][0]) / 2.0,
                    (child_bounds[0][1] + child_bounds[1][1]) / 2.0,
                    (child_bounds[0][2] + child_bounds[1][2]) / 2.0,
                ],
                depth: parent_depth + 1,
                reflectivity: [0, 0],
                occupancy: Occupancy::Free,
            });
        }

        children
    }
    
    fn calculate_child_bounds(
        parent_bounds: &[[f32; 3]; 2],
        parent_center: &[f32; 3],
        index: usize,
    ) -> [[f32; 3]; 2] {
        let x_sign = if (index & 1) != 0 { 1.0 } else { -1.0 };
        let y_sign = if (index & 2) != 0 { 1.0 } else { -1.0 };
        let z_sign = if (index & 4) != 0 { 1.0 } else { -1.0 };
        let epsilon = 0.00001;

        let child_size = (parent_bounds[1][0] - parent_bounds[0][0]) / 2.0;
        let offset = [
            x_sign * child_size / 2.0,
            y_sign * child_size / 2.0,
            z_sign * child_size / 2.0,
        ];

        let child_min = [
        parent_center[0] + offset[0] - child_size / 2.0 - epsilon,
        parent_center[1] + offset[1] - child_size / 2.0 - epsilon,
        parent_center[2] + offset[2] - child_size / 2.0 - epsilon,
        ];
        let child_max = [
            parent_center[0] + offset[0] + child_size / 2.0 + epsilon,
            parent_center[1] + offset[1] + child_size / 2.0 + epsilon,
            parent_center[2] + offset[2] + child_size / 2.0 + epsilon,
        ];
        [child_min, child_max]
    }

    pub fn octree_to_map(&self) -> HashMap<u32, Vec<LaserPoint>> {
        let mut meshes = HashMap::new();
        Self::octree_to_map_internal(&self.root, &mut meshes);
        meshes
    }

    fn octree_to_map_internal(node: &OctreeNode, meshes: &mut HashMap<u32, Vec<LaserPoint>>) {
        match node {
            OctreeNode::Internal { children, .. } => {
                for child in children.iter() {
                    Self::octree_to_map_internal(child, meshes);
                }
            }
            OctreeNode::Leaf { bounds, depth, occupancy, reflectivity, .. } => {
                if *occupancy == Occupancy::Occupied {
                    let center = [
                        (bounds[0][0] + bounds[1][0]) / 2.0,
                        (bounds[0][1] + bounds[1][1]) / 2.0,
                        (bounds[0][2] + bounds[1][2]) / 2.0,
                    ];
                    let reflectivity = Self::node_reflectivity_calculator(reflectivity);
                    let data = LaserPoint::new(center[0], center[1], center[2], reflectivity);
                    meshes.entry(*depth).or_insert_with(Vec::new).push(data);
                }
            }
        }
    }

    fn node_reflectivity_calculator(reflectivity: &[u32; 2]) -> u8 {
        let total = reflectivity[1] as f32;
        let sum = reflectivity[0] as f32;
        (sum / total).round() as u8
    }

    pub fn refresh(&mut self) {
        let mut new_root = OctreeNode::Leaf {
            bounds: [[-10.0; 3], [10.0; 3]],
            center: [0.0; 3],
            depth: 0,
            occupancy: Occupancy::Free,
            reflectivity: [0, 0],
        };
        std::mem::swap(&mut self.root, &mut new_root);
        self.root = new_root;
    }


    //TODO: Implement ray casting
    pub fn cast_ray(&self, origin: [f32; 3], direction: [f32; 3], max_distance: f32) -> Option<f32> {
        self.root.cast_ray(origin, direction, max_distance, &mut None)
    }

    pub fn optimize(&mut self) {
        Self::optimize_recursive_internal(&mut self.root);
    }

    fn optimize_recursive_internal(node: &mut OctreeNode) {
        if let OctreeNode::Internal { children, .. } = node {
            for child in children.iter_mut() {
                Self::optimize_recursive_internal(child);
            }
        }
        Self::try_merge_node(node);
    }

    fn try_merge_node(node: &mut OctreeNode) {
        if let OctreeNode::Internal { bounds, center, depth, children } = node {
            let all_free = children.iter().all(|c| Self::is_fully_free(c));
            let all_occupied = children.iter().all(|c| Self::is_fully_occupied(c));

            if all_free || all_occupied {
                let reflectivity = Self::merge_reflectivity(children);
                *node = OctreeNode::Leaf {
                    bounds: *bounds,
                    center: *center,
                    depth: *depth,
                    occupancy: if all_free { Occupancy::Free } else { Occupancy::Occupied },
                    reflectivity,
                };
            }
        }
    }

    /// Recursively merges reflectivity values from child nodes
    /// 
    /// # Arguments
    /// * `children` - Slice of boxed OctreeNode children
    /// 
    /// # Returns
    /// An array containing the sum of reflectivity values [sum_reflectivity, sum_point_number]
    fn merge_reflectivity(children: &[Box<OctreeNode>]) -> [u32; 2] {
        children.iter().fold([0, 0], |acc, child| {
            match child.as_ref() {
                OctreeNode::Leaf { reflectivity, .. } => {
                    [acc[0] + reflectivity[0], acc[1] + reflectivity[1]]
                },
                OctreeNode::Internal { children, .. } => {
                    let child_reflectivity = Self::merge_reflectivity(children);
                    [acc[0] + child_reflectivity[0], acc[1] + child_reflectivity[1]]
                }
            }
        })
    }

    fn is_fully_occupied(node: &OctreeNode) -> bool {
        match node {
            OctreeNode::Leaf { occupancy, .. } => *occupancy == Occupancy::Occupied,
            OctreeNode::Internal { children, .. } => children.iter().all(|c| Self::is_fully_occupied(c)),
        }
    }

    fn is_fully_free(node: &OctreeNode) -> bool {
        match node {
            OctreeNode::Leaf { occupancy, .. } => *occupancy == Occupancy::Free,
            OctreeNode::Internal { children, .. } => children.iter().all(|c| Self::is_fully_free(c)),
        }
    }
}