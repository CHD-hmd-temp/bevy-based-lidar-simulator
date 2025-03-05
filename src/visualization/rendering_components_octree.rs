#![allow(dead_code)]
#![allow(unused_imports)]
use std::net::UdpSocket;
use std::path;
use std::io::Write;
use bevy::ecs::query;
use bevy::{gizmos, prelude::*};
use bevy::state::commands;
use bevy_flycam::prelude::*;
use bevy::color::palettes::css::GOLD;
use bevy::diagnostic::{FrameTimeDiagnosticsPlugin, DiagnosticsStore};
use crate::data_reader::structor::{ LaserPoint, Point3};
use crate::data_reader::udp_reader;
use crate::data_reader::data_reader;
use crate::data_reader::io;
use crate::visualization::color_calculator;
use crate::octree::{self, octree::*, creat_octree};
use crate::calculator::{crash_detector, mavlink_args}; 
use crate::calculator::coordinate_switch::{frd_to_bevy, mid360_to_bevy};
use crate::calculator::apf;
use crate::calculator::apf::ApfConfig;
//use crate::data_reader::structor::LaserPoint;

#[derive(Component)]
struct Ground;

#[derive(Component)]
struct FpsText;

#[derive(Component)]
struct OctreeEntity;

#[derive(Resource)]
struct FrameIntegrationTime(pub u64);

#[derive(Resource)]
pub struct VelocityVector(pub Vec3);

#[derive(Resource)]
pub struct Path(pub Vec<Point3>);

#[derive(Resource)]
pub struct OctreeConfig {
    boundary: f32,
    max_depth: u32,
    voxel_size: f32,
    frame_integration_time: u64,
}

pub fn run_bevy() {
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
        .insert_resource(OctreeConfig {
            boundary: 10.0,
            max_depth: 7,
            voxel_size: 0.08,
            frame_integration_time: 100,
        })
        .insert_resource(ApfConfig {
            k_att: 2.5,
            k_rep: 2.5,
            d0: 0.7,
            epsilon: 0.1,
            max_steps: 500,
            step_size: 0.1,
        })
        .insert_resource(VelocityVector(Vec3::ZERO))
        .insert_resource(Path(Vec::new()))
        .add_systems(Startup,
            |commands: Commands,
            meshes: ResMut<Assets<Mesh>>,
            materials: ResMut<Assets<StandardMaterial>>,
            octree_config: Res<OctreeConfig>
            | {
            setup_bevy(
                commands,
                meshes,
                materials,
                octree_config,
            );
        })
        .add_systems(Update, text_update_system)
        .add_systems(Update,
            |commands: Commands,
            meshes: ResMut<Assets<Mesh>>,
            materials: ResMut<Assets<StandardMaterial>>,
            velocity: ResMut<VelocityVector>,
            path: ResMut<Path>,
            octree_config: Res<OctreeConfig>,
            apf_config: Res<ApfConfig>,
            query: Query<'_, '_, Entity, With<OctreeEntity>>|
            octree_update_system(
                commands,
                meshes,
                materials,
                velocity,
                path,
                octree_config,
                apf_config,
                query
            ))
        .add_systems(Update, draw_gizmos)
        .run();
}

fn setup_bevy(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    octree_config: Res<OctreeConfig>,
) {
    let boundary = octree_config.boundary;
    let max_depth = octree_config.max_depth;
    let voxel_size = octree_config.voxel_size;
    let frame_integration_time = octree_config.frame_integration_time;
    let mut octree = creat_octree::creat_octree_from_udp(boundary, max_depth, voxel_size, frame_integration_time);
    octree.optimize();
    let leaves = octree.octree_to_map();

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
    // commands.spawn((
    //     Mesh3d(meshes.add(Plane3d::default().mesh().size(20., 20.))),
    //     MeshMaterial3d(materials.add(StandardMaterial {
    //         base_color: Color::srgba(1.0, 1.0, 1.0, 0.5), // White with 50% transparency
    //         alpha_mode: AlphaMode::Blend, // Enables transparency blending
    //         ..Default::default()
    //     })),
    //     Ground,
    // ));

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
    mut path: ResMut<Path>,
    octree_config: Res<OctreeConfig>,
    apf_config: Res<ApfConfig>,
    query: Query<Entity, With<OctreeEntity>>,
) {
    for entity in query.iter() {
        commands.entity(entity).despawn();
    }
    let boundary = octree_config.boundary;
    let max_depth = octree_config.max_depth;
    let voxel_size = octree_config.voxel_size;
    let frame_integration_time = octree_config.frame_integration_time;
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
                    Mesh3d(cube_mesh.clone()), // Reuse the same mesh
                    MeshMaterial3d(material.clone()), // Reuse the same material
                    Transform::from_translation(Vec3::new(x, y, z)),
                    OctreeEntity,
                ));
            }
        }
    };

    // APF palnning
    let start = Point3::new(0.0, 0.0, 0.0);
    let goal_mid360 = (5.0, 0.0, 0.0);
    let goal = Point3::new(goal_mid360.0, goal_mid360.1, goal_mid360.2);

    let config = ApfConfig {
        k_att: apf_config.k_att,
        k_rep: apf_config.k_rep,
        d0: apf_config.d0,
        epsilon: apf_config.epsilon,
        max_steps: apf_config.max_steps,
        step_size: apf_config.step_size,
    };

    let _warn_trigger_distance = apf_config.d0;

    let apf_path = apf::apf_plan(start, goal, &octree, config);
    let vec = match apf_path {
        Ok(path) => {
            path
        }
        Err(e) => {
            println!("Error: {:?}", e);
            Vec::new()
        }
    };
    path.0 = vec;

    // Crash detection
    // let tup_obstacle_result = crash_detector::crash_warn_for_octree(&octree, warn_trigger_distance);
    // let mavlink_message = crash_detector::obstacle_avoidance(&tup_obstacle_result.1, warn_trigger_distance);
    // velocity.0 = match mavlink_message.type_mask {
    //     0b0000001000000000 => {
    //         let (x, y, z) = frd_to_bevy(mavlink_message.vx, mavlink_message.vy, mavlink_message.vz);
    //         Vec3::new(x, y, z)
    //     }
    //     _ => Vec3::ZERO,
    // };

    velocity.0 = Vec3::ZERO;
}

fn draw_gizmos(
    mut gizmos: Gizmos,
    velocity: Res<VelocityVector>,
    path: Res<Path>,
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
    if path.0.len() > 1 {
        for i in 0..path.0.len() - 1 {
            let (x, y, z) = mid360_to_bevy(path.0[i].x, path.0[i].y, path.0[i].z);
            let (x1, y1, z1) = mid360_to_bevy(path.0[i + 1].x, path.0[i + 1].y, path.0[i + 1].z);
            gizmos.line(
                Vec3::new(x, y, z),
                Vec3::new(x1, y1, z1),
                Color::srgb_u8(0, 255, 0),
            );
        }
    }
}

fn get_size(boundary: f32, max_depth: u32) -> f32 {
    let mut size = boundary * 2.0;
    for _ in 0..max_depth {
        size /= 2.0;
    }
    size
}
