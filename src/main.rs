mod octree;
mod data_reader;
mod visualization;
mod calculator;


fn main() {
    #![allow(unused_variables)]
    // let boundary = data_reader::io::read_with_default(
    //     "Boundary:",
    //     10.0,
    //     None
    // );
    // let if_use_voxel = data_reader::io::read_with_default(
    //     "Voxel Grid Filter? (Y/N):",
    //     'Y',
    //     None
    // );
    // let voxel_size = match if_use_voxel {
    //     'Y' | 'y' => data_reader::io::read_with_default(
    //         "Voxel Size: (Minimum 0.05, or may not function properly)",
    //         0.08,
    //         None
    //     ) as f32,
    //     _ => 0.0
    // };
    // let max_depth = data_reader::io::read_with_default(
    //     "Max Depth:",
    //     7,
    //     None
    // );
    // let frame_integration_time = data_reader::io::read_with_default(
    //     "Frame Integration Time (ms):",
    //     500,
    //     None
    // ) as u64;

    // let mut octree = octree::creat_octree::creat_octree_from_udp(
    //     boundary,
    //     max_depth,
    //     voxel_size,
    //     frame_integration_time
    // );

    // octree.optimize();
    visualization::rendering_components_octree::run_bevy();
    

}
