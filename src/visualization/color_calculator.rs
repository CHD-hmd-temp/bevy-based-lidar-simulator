use bevy::prelude::*;

fn interpolate_color(start: (u8, u8, u8), end: (u8, u8, u8), factor: f32) -> (u8, u8, u8) {
    let r = (start.0 as f32 + factor * (end.0 as f32 - start.0 as f32)).round() as u8;
    let g = (start.1 as f32 + factor * (end.1 as f32 - start.1 as f32)).round() as u8;
    let b = (start.2 as f32 + factor * (end.2 as f32 - start.2 as f32)).round() as u8;
    (r, g, b)
}

pub fn reflectivity_to_rgb(reflectivity: u8) -> (u8, u8, u8) {
    let ranges = [
        (0, (0, 0, 255)),       // Blue
        (85, (0, 255, 255)),    // Cyan
        (170, (255, 255, 0)),   // Yellow
        (255, (255, 0, 0)),     // Red
    ];

    for i in 0..ranges.len() - 1 {
        let (start_ref, start_color) = ranges[i];
        let (end_ref, end_color) = ranges[i + 1];
        if reflectivity as usize <= end_ref {
            let factor = (reflectivity as f32 - start_ref as f32) / (end_ref as f32 - start_ref as f32);
            return interpolate_color(start_color, end_color, factor);
        }
    }
    (255, 0, 0) // Fallback to Red
}

pub fn reflectivity_to_color(reflectivity: u8) -> Color {
    let reflectivity = reflectivity_to_rgb(reflectivity);
    Color::srgb_u8(reflectivity.0, reflectivity.1, reflectivity.2)
}
