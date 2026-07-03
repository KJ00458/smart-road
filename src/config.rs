use std::time::Duration;

pub const WINDOW_W: u32 = 900;
pub const WINDOW_H: u32 = 900;
pub const TARGET_FPS: f64 = 60.0;

// Road / grid
pub const TILE: f64 = 50.0;              // one lane width = 50 px
pub const LANE_COUNT: usize = 6;          // 3 inbound + 3 outbound
pub const ROAD_W: f64 = TILE * LANE_COUNT as f64;  // 300 px
pub const IX: f64 = (WINDOW_W as f64 - ROAD_W) / 2.0; // intersection left edge  = 300
pub const IY: f64 = (WINDOW_H as f64 - ROAD_W) / 2.0; // intersection top edge   = 300

// Vehicle
pub const V_W: f64 = 34.0;
pub const V_H: f64 = 44.0;
pub const SAFE_DIST: f64 = V_H * 2.2;

// Speeds  (px/s)
pub const SPD_FAST:   f64 = 200.0;
pub const SPD_NORMAL: f64 = 150.0;
pub const SPD_SLOW:   f64 = 100.0;
pub const SPD_VSLOW:  f64 =  60.0;
pub const SPD_STOP:   f64 =   0.0;

// Spawn offset off-screen
pub const SPAWN_OFF: f64 = 55.0;

// Colours
pub const COL_GRASS:  (u8,u8,u8) = (34, 120, 34);
pub const COL_ROAD:   (u8,u8,u8) = (50, 50, 50);
pub const COL_YELLOW: (u8,u8,u8) = (220, 200, 60);
pub const COL_WHITE:  (u8,u8,u8) = (240, 240, 240);
pub const COL_INTER:  (u8,u8,u8) = (60, 60, 60);

// HUD
pub const COL_HUD_BG:    (u8,u8,u8,u8) = (10, 10, 25, 210);
pub const COL_HUD_TITLE: (u8,u8,u8)    = (100, 210, 255);
pub const COL_HUD_VAL:   (u8,u8,u8)    = (100, 255, 150);
pub const COL_HUD_DIM:   (u8,u8,u8)    = (130, 130, 155);
pub const COL_HUD_WARN:  (u8,u8,u8)    = (255, 170, 50);
pub const COL_HUD_ON:    (u8,u8,u8)    = (80,  255, 120);
pub const COL_HUD_OFF:   (u8,u8,u8)    = (200, 70,  70);

pub const KEY_CD: Duration = Duration::from_millis(350);
pub const RAND_CD: Duration = Duration::from_millis(480);
