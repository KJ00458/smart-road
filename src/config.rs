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

// Vehicle box (matches Golden76z tile sizes scaled to our grid)
pub const VW: f64 = TILE;         // 60px  (full tile width)
pub const VH: f64 = TILE;         // 60px  (full tile height)

// ── Speeds ── (Golden76z exact values, px/s)
pub const SPD_ALMOST_STOP: f64 =  30.0;
pub const SPD_VERY_SLOW:   f64 =  60.0;
pub const SPD_SLOW:        f64 = 100.0;
pub const SPD_NORMAL:      f64 = 150.0;
pub const SPD_FAST:        f64 = 200.0;

// ── Hitbox projection sizes ── (Golden76z exact values, px ahead of vehicle)
pub const HB_BIG:        f64 = 400.0;
pub const HB_MEDIUM:     f64 = 300.0;
pub const HB_SMALL:      f64 = 225.0;
pub const HB_VERY_SMALL: f64 = 100.0;
pub const HB_ALMOST_STOP: f64 = 51.0;
pub const HB_STOP:       f64 =  50.0;  // 1px smaller than ALMOST_STOP
                                        // if even this overlaps = truly blocked

// Half-width of the forward hitbox rectangle
pub const HB_HALF_W: f64 = VW * 0.5;

// Same-lane following safe gap = 4 * vehicle width (Golden76z: SAFE_DISTANCE)
pub const SAFE_DISTANCE: f64 = 4.0 * VW; // 240px
pub const STOP_GAP:      f64 = VW * 1.0; // 60px  — hard stop threshold

// ── Velocity cooldown (Golden76z: VELOCITY_COOLDOWN = 20ms) ──
// Speed may only change once per this interval per vehicle.
// Prevents per-frame flicker that allows cars to phase through each other.
pub const VELOCITY_COOLDOWN: Duration = Duration::from_millis(20);

// Spawn gap: minimum distance between same-lane cars at spawn
pub const SPAWN_GAP: f64 = SAFE_DISTANCE * 2.0;

// Crash threshold (used only for stats, not for stopping)
pub const CRASH_DIST: f64 = VH * 0.7;

// Off-screen exit margin
pub const OFF: f64 = 80.0;

// Input cooldowns
pub const KEY_CD:  Duration = Duration::from_millis(400);
pub const RAND_CD: Duration = Duration::from_millis(1200); // Golden76z SPAWN_COOLDOWN

// Intersection box bounds
pub const IX_L: f64 = IX;
pub const IX_R: f64 = IX + ROAD;
pub const IX_T: f64 = IY;
pub const IX_B: f64 = IY + ROAD;

// Sensor / conflict ranges
pub const SENSOR_RANGE: f64 = HB_BIG + VH;
pub const CONFLICT_DIST: f64 = TILE * 1.5;

// ── Colours ──
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
