use std::time::Duration;

pub const WINDOW_W: u32 = 900;
pub const WINDOW_H: u32 = 900;
pub const FPS: f64 = 60.0;

// Grid: 6 lanes x 50px each = 300px road width, centred in 900px window
pub const TILE: f64 = 50.0;
pub const LANES: usize = 6;
pub const ROAD: f64 = TILE * LANES as f64; // 300
pub const IX: f64 = (WINDOW_W as f64 - ROAD) / 2.0; // 300  (left edge of intersection)
pub const IY: f64 = (WINDOW_H as f64 - ROAD) / 2.0; // 300  (top  edge of intersection)

// Vehicle box
pub const VW: f64 = 34.0;
pub const VH: f64 = 44.0;

// Three speeds (px/s)  — FAST is deliberately high so cars zoom through
pub const SPD_SLOW:   f64 =  60.0;
pub const SPD_MED:    f64 = 150.0;
pub const SPD_FAST:   f64 = 320.0;   // max / free-running speed

// Safe following gap (px)
pub const GAP: f64 = VH * 2.5;       // ~110 px
pub const STOP_GAP: f64 = VH * 1.2;  // hard stop

// Off-screen spawn margin
pub const OFF: f64 = 60.0;

// Timers
pub const KEY_CD:  Duration = Duration::from_millis(350);
pub const RAND_CD: Duration = Duration::from_millis(460);

// Colours
pub const C_GRASS:  (u8,u8,u8) = (34, 110, 34);
pub const C_ROAD:   (u8,u8,u8) = (50,  50, 50);
pub const C_INTER:  (u8,u8,u8) = (62,  62, 62);
pub const C_YELLOW: (u8,u8,u8) = (220,195, 50);
pub const C_WHITE:  (u8,u8,u8) = (240,240,240);

pub const C_HUD_BG:    (u8,u8,u8,u8) = (10, 10, 25, 215);
pub const C_HUD_TITLE: (u8,u8,u8)    = (100,210,255);
pub const C_HUD_VAL:   (u8,u8,u8)    = (100,255,150);
pub const C_HUD_DIM:   (u8,u8,u8)    = (120,120,145);
pub const C_HUD_WARN:  (u8,u8,u8)    = (255,160, 40);
pub const C_HUD_ON:    (u8,u8,u8)    = ( 80,255,120);
pub const C_HUD_OFF:   (u8,u8,u8)    = (200, 60, 60);
