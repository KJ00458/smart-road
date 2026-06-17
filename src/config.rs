use std::time::Duration;

pub const WINDOW_W: u32 = 900;
pub const WINDOW_H: u32 = 900;
pub const TARGET_FPS: f64 = 60.0;

pub const ROAD_COLOR: (u8, u8, u8) = (50, 50, 50);
pub const LANE_LINE_COLOR: (u8, u8, u8) = (220, 220, 80);
pub const GRASS_COLOR: (u8, u8, u8) = (34, 139, 34);
pub const SIDEWALK_COLOR: (u8, u8, u8) = (180, 170, 150);

pub const ROAD_WIDTH: f64 = 300.0;
pub const LANE_COUNT: usize = 3;
pub const LANE_WIDTH: f64 = ROAD_WIDTH / LANE_COUNT as f64;

pub const INTERSECTION_X: f64 = (WINDOW_W as f64 - ROAD_WIDTH) / 2.0;
pub const INTERSECTION_Y: f64 = (WINDOW_H as f64 - ROAD_WIDTH) / 2.0;

pub const VEHICLE_W: f64 = 36.0;
pub const VEHICLE_H: f64 = 60.0;

pub const SPEED_LOW: f64 = 60.0;
pub const SPEED_MED: f64 = 120.0;
pub const SPEED_HIGH: f64 = 200.0;

pub const SAFE_DISTANCE: f64 = 70.0;
pub const STOP_DISTANCE: f64 = 10.0;

pub const ACCEL: f64 = 160.0;
pub const DECEL: f64 = 240.0;

pub const KEY_COOLDOWN: Duration = Duration::from_millis(400);
pub const RANDOM_SPAWN_INTERVAL: Duration = Duration::from_millis(600);

pub const STATS_BG_COLOR: (u8, u8, u8) = (15, 15, 30);
pub const STATS_TEXT_COLOR: (u8, u8, u8) = (220, 220, 255);
pub const STATS_TITLE_COLOR: (u8, u8, u8) = (100, 200, 255);
