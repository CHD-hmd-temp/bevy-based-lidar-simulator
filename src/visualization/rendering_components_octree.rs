#![allow(dead_code)]
#![allow(unused_imports)]
use std::net::UdpSocket;
use bevy::ecs::query;
use bevy::{gizmos, prelude::*};
use bevy::state::commands;
use bevy_flycam::prelude::*;
use bevy::color::palettes::css::GOLD;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, DiagnosticsStore};
use crate::data_reader::structor::LaserPoint;
use crate::data_reader::udp_reader;
use crate::data_reader::data_reader;
use crate::data_reader::io;
use crate::visualization::color_calculator;
use crate::octree::{self, octree::*, creat_octree};
use crate::calculator::{crash_detector, mavlink_args}; 
use crate::calculator::coordinate_switch::{frd_to_bevy, mid360_to_bevy};
//use crate::data_reader::structor::LaserPoint;

#[derive(Component)]
struct Ground;

// #[derive(Component)]
// struct LaserPoint;

#[derive(Component)]
struct FpsText;

#[derive(Resource)]
struct Boundary(pub f32);

#[derive(Resource)]
struct MaxDepth(pub u32);

#[derive(Resource)]
struct VoxelSize(pub f32);

#[derive(Resource)]
struct FrameIntegrationTime(pub u64);

#[derive(Component)]
struct OctreeEntity;

#[derive(Resource)]
pub struct VelocityVector(pub Vec3);

pub fn run_bevy() {
    // let boundary = io::read_with_default(
    //     "Boundary:",
    //     10.0,
    //     None
    // );
    // let if_use_voxel = io::read_with_default(
    //     "Voxel Grid Filter? (Y/N):",
    //     'Y',
    //     None
    // );
    // let voxel_size = match if_use_voxel {
    //     'Y' | 'y' => io::read_with_default(
    //         "Voxel Size: (Minimum 0.05, or may not function properly)",
    //         0.08,
    //         None
    //     ) as f32,
    //     _ => 0.0
    // };
    // let max_depth = io::read_with_default(
    //     "Max Depth:",
    //     7,
    //     None
    // );
    // let frame_integration_time = io::read_with_default(
    //     "Frame Integration Time (ms):",
    //     500,
    //     None
    // ) as u64;
    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "WorldWithoutMortis".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins( FrameTimeDiagnosticsPlugin)
        .add_plugins(NoCameraPlayerPlugin)
        .insert_resource(MovementSettings {
            sensitivity: 0.00009,
            speed: 3.0,
        })
        .insert_resource(VelocityVector(Vec3::ZERO))
        //.insert_resource(octree)
        .add_systems(Startup,
            |commands: Commands,
            meshes: ResMut<Assets<Mesh>>,
            materials: ResMut<Assets<StandardMaterial>>,
            // boundary: Res<Boundary>,
            // max_depth: Res<MaxDepth>,
            // voxel_size: Res<VoxelSize>,
            // frame_integration_time: Res<FrameIntegrationTime>
            | {
            setup_bevy(
                commands,
                meshes,
                materials,
                // boundary,
                // max_depth,
                // voxel_size,
                // frame_integration_time
            );
        })
        .add_systems(Update, text_update_system)
        .add_systems(Update,
            |commands: Commands,
            meshes: ResMut<Assets<Mesh>>,
            materials: ResMut<Assets<StandardMaterial>>,
            velocity: ResMut<VelocityVector>,
            query: Query<'_, '_, Entity, With<OctreeEntity>>|
            octree_update_system(commands, meshes, materials, velocity, query))
        .add_systems(Update, draw_gizmos)
        .run();
}

fn setup_bevy(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    // boundary: Res<Boundary>,
    // max_depth: Res<MaxDepth>,
    // voxel_size: Res<VoxelSize>,
    // frame_integration_time: Res<FrameIntegrationTime>
) {
    // let boundary = boundary.0;
    // let max_depth = max_depth.0;
    // let voxel_size = voxel_size.0;
    // let frame_integration_time = frame_integration_time.0;
    let boundary = 10.0;
    let max_depth = 7;
    let voxel_size = 0.08;
    let frame_integration_time = 100;
    let mut octree = creat_octree::creat_octree_from_udp(boundary, max_depth, voxel_size, frame_integration_time);
    octree.optimize();
    let leaves = octree.octree_to_map();

    // let warn_trigger_distance = io::read_with_default(
    //     "Warning Trigger Distance:",
    //     0.5,
    //     None
    // ) as f32;

    //let tup_obstacle_result = crash_detector::crash_warn_for_octree(&octree, warn_trigger_distance);
    //println!("Crash Warning: {}", tup_obstacle_result.0);
    //let mavlink_message = crash_detector::obstacle_avoidance(&tup_obstacle_result.1, warn_trigger_distance);

    for (depth, group) in leaves {
        let grouped_pixel_points = data_reader::divide_points(group);
        let cube_mesh = meshes.add(Mesh::from(
            Cuboid::new(
                get_size(boundary, depth),
                get_size(boundary, depth),
                get_size(boundary, depth)
            )
        ));

        for (reflectivity, group) in &grouped_pixel_points {
            let material = materials.add(StandardMaterial {
                emissive: color_calculator::reflectivity_to_color(*reflectivity).into(),
                ..default()
            });

            for point in group {
                let (x, y, z) = mid360_to_bevy(point.x, point.y, point.z);
                commands.spawn((
                    //LaserPoint,
                    Mesh3d(cube_mesh.clone()), // Reuse the same mesh
                    MeshMaterial3d(material.clone()), // Reuse the same material
                    Transform::from_translation(Vec3::new(x, y, z)),
                    OctreeEntity,
                ));
            }
        }
    };

    // Add a camera at [0, 0, 2] and look at front
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(0., 1.5, 4.).looking_at(Vec3::ZERO, Vec3::Y),
        //FlyCam,
    ));

    // Add a red sphere to represent the drone at [0, 0, 0]
    let sphere_mesh = meshes.add(Sphere::new(0.1));
    let material = materials.add(StandardMaterial {
        emissive: Color::srgb_u8(255, 0, 0).into(),
        ..default()
    });
    commands.spawn((
        Mesh3d(sphere_mesh),
        MeshMaterial3d(material),
        Transform::from_translation(Vec3::new(0.0, 0.0, 0.0)),
    )); 

    // plane
    commands.spawn((
        Mesh3d(meshes.add(Plane3d::default().mesh().size(20., 20.))),
        MeshMaterial3d(materials.add(StandardMaterial {
            base_color: Color::srgba(1.0, 1.0, 1.0, 0.5), // White with 50% transparency
            alpha_mode: AlphaMode::Blend, // Enables transparency blending
            ..Default::default()
        })),
        Ground,
    ));

    // Text with multiple sections
    commands
        .spawn((
            Text::new("FPS: "),
            Node {
                position_type: PositionType::Absolute,
                bottom:Val::Px(12.0),
                left: Val::Px(12.0),
                ..default()
            },
        ))
        .with_child((
            TextSpan::default(),
            TextColor(GOLD.into()),
            FpsText,
        ));
}

fn text_update_system(
    diagnostics: Res<DiagnosticsStore>,
    mut query: Query<&mut TextSpan, With<FpsText>>,
) {
    for mut span in &mut query {
        if let Some(fps) = diagnostics.get(&FrameTimeDiagnosticsPlugin::FPS) {
            if let Some(value) = fps.smoothed() {
                // Update the value of the second section
                **span = format!("{value:.2}");
            }
        }
    }
}

fn octree_update_system(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    mut velocity: ResMut<VelocityVector>,
    query: Query<Entity, With<OctreeEntity>>,
) {
    for entity in query.iter() {
        commands.entity(entity).despawn();
    }
    let boundary = 10.0;
    let max_depth = 7;
    let voxel_size = 0.08;
    let frame_integration_time = 100;
    let warn_trigger_distance = 0.5;
    let mut octree = creat_octree::creat_octree_from_udp(boundary, max_depth, voxel_size, frame_integration_time);

    octree.optimize();

    let leaves = octree.octree_to_map();
    for (depth, group) in leaves {
        let grouped_pixel_points = data_reader::divide_points(group);
        let cube_mesh = meshes.add(Mesh::from(
            Cuboid::new(
                get_size(10.0, depth),
                get_size(10.0, depth),
                get_size(10.0, depth)
            )
        ));

        for (reflectivity, group) in &grouped_pixel_points {
            let material = materials.add(StandardMaterial {
                emissive: color_calculator::reflectivity_to_color(*reflectivity).into(),
                ..default()
            });

            for point in group {
                let (x, y, z) = mid360_to_bevy(point.x, point.y, point.z);
                commands.spawn((
                    //LaserPoint,
                    Mesh3d(cube_mesh.clone()), // Reuse the same mesh
                    MeshMaterial3d(material.clone()), // Reuse the same material
                    Transform::from_translation(Vec3::new(x, y, z)),
                    OctreeEntity,
                ));
            }
        }
    };

    // Crash detection
    let tup_obstacle_result = crash_detector::crash_warn_for_octree(&octree, warn_trigger_distance);
    //println!("Crash Warning: {}", tup_obstacle_result.0);
    let mavlink_message = crash_detector::obstacle_avoidance(&tup_obstacle_result.1, warn_trigger_distance);
    velocity.0 = match mavlink_message.type_mask {
        0b0000001000000000 => {
            let (x, y, z) = frd_to_bevy(mavlink_message.vx, mavlink_message.vy, mavlink_message.vz);
            Vec3::new(x, y, z)
        }
        
        _ => Vec3::ZERO,
    };
}

fn draw_gizmos(
    mut gizmos: Gizmos,
    velocity: Res<VelocityVector>,
) {
    use std::f32::consts::PI;
    gizmos.line(
        Vec3::ZERO,
        velocity.0,
        Color::srgb_u8(255, 0, 0),
    );
    gizmos.grid(
        Quat::from_rotation_x(PI / 2.),
        UVec2::splat(20),
        Vec2::new(2., 2.),
        // Light gray
        LinearRgba::gray(0.65),
    );
}

fn get_size(boundary: f32, max_depth: u32) -> f32 {
    let mut size = boundary * 2.0;
    for _ in 0..max_depth {
        size /= 2.0;
    }
    size
}
