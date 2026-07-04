use std::time::Duration;

pub const WINDOW_W: u32 = 1100;
pub const WINDOW_H: u32 = 1100;
pub const FPS: f64 = 60.0;

// Grid: 6 lanes x 60px each = 360px road width, centred in 1100px window
pub const TILE: f64 = 60.0;
pub const LANES: usize = 6;
pub const ROAD: f64 = TILE * LANES as f64; // 360
pub const IX: f64 = (WINDOW_W as f64 - ROAD) / 2.0; // 370
pub const IY: f64 = (WINDOW_H as f64 - ROAD) / 2.0; // 370

// Vehicle box
pub const VW: f64 = 36.0;
pub const VH: f64 = 48.0;

// Three speeds (px/s) — deliberately slow so you can watch the logic
pub const SPD_SLOW:   f64 =  28.0;
pub const SPD_MED:    f64 =  70.0;
pub const SPD_FAST:   f64 = 130.0;

// Safe following gap (px)
pub const GAP:       f64 = VH * 2.8;   // ~134 px
pub const STOP_GAP:  f64 = VH * 1.3;   // ~62 px

// Sensor cone
pub const SENSOR_HALF_W: f64 = VW * 0.65;
pub const SENSOR_RANGE:  f64 = 260.0;

// Conflict / priority thresholds
pub const CONFLICT_DIST:  f64 = TILE * 1.3;
pub const PRIORITY_DIST:  f64 = 260.0;

// Crash threshold: two vehicles this close = crash
pub const CRASH_DIST: f64 = VH * 0.85;

// Off-screen spawn/despawn margin — cars must travel fully off screen
pub const OFF: f64 = 80.0;

// Timers
pub const KEY_CD:  Duration = Duration::from_millis(400);
pub const RAND_CD: Duration = Duration::from_millis(550);

// ── Colours ────────────────────────────────────────────────────────────────
pub const C_GRASS:  (u8,u8,u8) = (22, 78, 22);
pub const C_ROAD:   (u8,u8,u8) = (38, 38, 38);
pub const C_INTER:  (u8,u8,u8) = (48, 48, 48);
pub const C_YELLOW: (u8,u8,u8) = (210,185, 40);
pub const C_WHITE:  (u8,u8,u8) = (230,230,230);

pub const C_HUD_BG:    (u8,u8,u8,u8) = ( 8,  8, 18, 230);
pub const C_HUD_TITLE: (u8,u8,u8)    = ( 80,190,255);
pub const C_HUD_VAL:   (u8,u8,u8)    = ( 80,245,140);
pub const C_HUD_DIM:   (u8,u8,u8)    = (100,100,130);
pub const C_HUD_WARN:  (u8,u8,u8)    = (255,140,  0);
pub const C_HUD_CRASH: (u8,u8,u8)    = (255, 50, 50);
pub const C_HUD_ON:    (u8,u8,u8)    = ( 60,245,110);
pub const C_HUD_OFF:   (u8,u8,u8)    = (200, 50,  50);
