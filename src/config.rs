use std::time::Duration;

pub const WINDOW_W: u32 = 900;
pub const WINDOW_H: u32 = 900;
pub const TARGET_FPS: f64 = 60.0;

pub const ROAD_COLOR: (u8, u8, u8) = (50, 50, 50);
pub const LANE_LINE_COLOR: (u8, u8, u8) = (220, 220, 80);
pub const CENTER_LINE_COLOR: (u8, u8, u8) = (255, 255, 255);
pub const GRASS_COLOR: (u8, u8, u8) = (34, 139, 34);

// 6 lanes total (3 inbound + 3 outbound) per road arm
pub const LANE_COUNT: usize = 6;
pub const INBOUND_LANES: usize = 3;
pub const ROAD_WIDTH: f64 = 300.0;
pub const LANE_WIDTH: f64 = ROAD_WIDTH / LANE_COUNT as f64; // 50 px per lane

pub const INTERSECTION_X: f64 = (WINDOW_W as f64 - ROAD_WIDTH) / 2.0; // 300.0
pub const INTERSECTION_Y: f64 = (WINDOW_H as f64 - ROAD_WIDTH) / 2.0; // 300.0

pub const VEHICLE_W: f64 = 36.0;
pub const VEHICLE_H: f64 = 46.0;

pub const SPEED_LOW: f64 = 60.0;
pub const SPEED_MED: f64 = 120.0;
pub const SPEED_HIGH: f64 = 200.0;

pub const SAFE_DISTANCE: f64 = 80.0;
pub const STOP_DISTANCE: f64 = 12.0;

pub const ACCEL: f64 = 160.0;
pub const DECEL: f64 = 280.0;

pub const KEY_COOLDOWN: Duration = Duration::from_millis(400);
pub const RANDOM_SPAWN_INTERVAL: Duration = Duration::from_millis(500);

pub const STATS_BG_COLOR: (u8, u8, u8) = (15, 15, 30);
pub const STATS_TEXT_COLOR: (u8, u8, u8) = (220, 220, 255);
pub const STATS_TITLE_COLOR: (u8, u8, u8) = (100, 200, 255);

// Spawn offset: how far off-screen vehicles start
pub const SPAWN_OFFSET: f64 = 80.0;

// ── Entry side of the intersection ──────────────────────────────────────────
// Inbound lanes for North arm (coming from top, moving South):
//   lane 0 = West turn  (rightmost of inbound = left side of northbound road)
//   lane 1 = Forward
//   lane 2 = East turn  (leftmost of inbound)
// Mirror logic applies for other arms.
// Outbound lanes occupy indices 3-5 and are used by exiting vehicles.

// Turning destination X (for North/South arms turning East/West)
// and Y (for East/West arms turning North/South) — snapped like reference.
// These are the pixel Y/X at which the vehicle snaps direction.

// North arm inbound lane X positions (vehicle moving South into intersection)
// Inbound lanes are the RIGHT half of the northbound road column (x: IX .. IX+ROAD_WIDTH/2)
pub fn inbound_x_for_south_arm(lane: usize) -> f64 {
    // South arm (entering from bottom, moving North): right 3 lanes of vertical road
    // Lanes 0..2 from center outward
    INTERSECTION_X + ROAD_WIDTH / 2.0 + (lane as f64 + 0.5) * LANE_WIDTH
}

pub fn inbound_x_for_north_arm(lane: usize) -> f64 {
    // North arm (entering from top, moving South): left 3 lanes
    INTERSECTION_X + (2 - lane as isize) as f64 * LANE_WIDTH + LANE_WIDTH / 2.0
}

pub fn inbound_y_for_east_arm(lane: usize) -> f64 {
    // East arm (entering from right, moving West): top 3 lanes
    INTERSECTION_Y + (lane as f64 + 0.5) * LANE_WIDTH
}

pub fn inbound_y_for_west_arm(lane: usize) -> f64 {
    // West arm (entering from left, moving East): bottom 3 lanes
    INTERSECTION_Y + ROAD_WIDTH / 2.0 + (lane as f64 + 0.5) * LANE_WIDTH
}
