//! Hard-coded waypoint paths — RIGHT-HAND driving, 6-lane grid.
//!
//! Window: 900x900. Road strip: 300px wide, starts at 300, ends at 600.
//! Lane centres:  300+0*50+25=325  375  425  475  525  575
//!
//! RIGHT-HAND RULE — each arm uses its RIGHT half for inbound traffic:
//!
//!   North arm (moving SOUTH): inbound cols 3,4,5  → x = 475, 525, 575
//!     lane[0]=right-turn(East)  x=475
//!     lane[1]=straight          x=525
//!     lane[2]=left-turn(West)   x=575
//!
//!   South arm (moving NORTH): inbound cols 2,1,0  → x = 425, 375, 325
//!     lane[0]=right-turn(West)  x=425
//!     lane[1]=straight          x=375
//!     lane[2]=left-turn(East)   x=325
//!
//!   West arm (moving EAST):  inbound rows 3,4,5  → y = 475, 525, 575
//!     lane[0]=right-turn(North) y=475
//!     lane[1]=straight          y=525
//!     lane[2]=left-turn(South)  y=575
//!
//!   East arm (moving WEST):  inbound rows 2,1,0  → y = 425, 375, 325
//!     lane[0]=right-turn(South) y=425
//!     lane[1]=straight          y=375
//!     lane[2]=left-turn(North)  y=325
//!
//! Outbound (exit) lanes are the mirror — left half for North/South, top half for East/West:
//!   North outbound (going North/up):   cols 0,1,2  → x=325,375,425
//!   South outbound (going South/down): cols 3,4,5  → x=475,525,575  (same as N inbound — different y zone)
//!   West  outbound (going West/left):  rows 0,1,2  → y=325,375,425
//!   East  outbound (going East/right): rows 3,4,5  → y=475,525,575
//!
//! Spawn/exit off-screen: top=240, bottom=660, left=240, right=660

use crate::vehicle::{Arm, Turn};

// ─── NORTH arm (spawn top y=240, travel South) ──────────────────────────────

// Right turn → col3 (x=475) south, pivot at row2 (y=425), exit East (x=660) on row2
const PATH_N_E: &[(f64,f64)] = &[
    (475.0, 240.0),
    (475.0, 425.0),
    (660.0, 425.0),
];
// Straight   → col4 (x=525) straight south, exit bottom (y=660)
const PATH_N_F: &[(f64,f64)] = &[
    (525.0, 240.0),
    (525.0, 660.0),
];
// Left turn  → col5 (x=575) south, pivot at row5 (y=575), exit West (x=240) on row5
const PATH_N_W: &[(f64,f64)] = &[
    (575.0, 240.0),
    (575.0, 575.0),
    (240.0, 575.0),
];

// ─── SOUTH arm (spawn bottom y=660, travel North) ───────────────────────────

// Right turn → col2 (x=425) north, pivot at row3 (y=475), exit West (x=240) on row3
const PATH_S_E: &[(f64,f64)] = &[
    (425.0, 660.0),
    (425.0, 475.0),
    (240.0, 475.0),
];
// Straight   → col1 (x=375) straight north, exit top (y=240)
const PATH_S_F: &[(f64,f64)] = &[
    (375.0, 660.0),
    (375.0, 240.0),
];
// Left turn  → col0 (x=325) north, pivot at row0 (y=325), exit East (x=660) on row0
const PATH_S_W: &[(f64,f64)] = &[
    (325.0, 660.0),
    (325.0, 325.0),
    (660.0, 325.0),
];

// ─── WEST arm (spawn left x=240, travel East) ───────────────────────────────

// Right turn → row3 (y=475) east, pivot at col3 (x=475), exit North (y=240) on col3
const PATH_W_E: &[(f64,f64)] = &[
    (240.0, 475.0),
    (475.0, 475.0),
    (475.0, 240.0),
];
// Straight   → row4 (y=525) straight east, exit right (x=660)
const PATH_W_F: &[(f64,f64)] = &[
    (240.0, 525.0),
    (660.0, 525.0),
];
// Left turn  → row5 (y=575) east, pivot at col5 (x=575), exit South (y=660) on col5
const PATH_W_W: &[(f64,f64)] = &[
    (240.0, 575.0),
    (575.0, 575.0),
    (575.0, 660.0),
];

// ─── EAST arm (spawn right x=660, travel West) ──────────────────────────────

// Right turn → row2 (y=425) west, pivot at col2 (x=425), exit South (y=660) on col2
const PATH_E_E: &[(f64,f64)] = &[
    (660.0, 425.0),
    (425.0, 425.0),
    (425.0, 660.0),
];
// Straight   → row1 (y=375) straight west, exit left (x=240)
const PATH_E_F: &[(f64,f64)] = &[
    (660.0, 375.0),
    (240.0, 375.0),
];
// Left turn  → row0 (y=325) west, pivot at col0 (x=325), exit North (y=240) on col0
const PATH_E_W: &[(f64,f64)] = &[
    (660.0, 325.0),
    (325.0, 325.0),
    (325.0, 240.0),
];

// ─── Selector ────────────────────────────────────────────────────────────────

pub fn get_path(arm: Arm, turn: Turn) -> &'static [(f64,f64)] {
    match (arm, turn) {
        (Arm::North, Turn::East)    => PATH_N_E,
        (Arm::North, Turn::Forward) => PATH_N_F,
        (Arm::North, Turn::West)    => PATH_N_W,
        (Arm::South, Turn::East)    => PATH_S_E,
        (Arm::South, Turn::Forward) => PATH_S_F,
        (Arm::South, Turn::West)    => PATH_S_W,
        (Arm::West,  Turn::East)    => PATH_W_E,
        (Arm::West,  Turn::Forward) => PATH_W_F,
        (Arm::West,  Turn::West)    => PATH_W_W,
        (Arm::East,  Turn::East)    => PATH_E_E,
        (Arm::East,  Turn::Forward) => PATH_E_F,
        (Arm::East,  Turn::West)    => PATH_E_W,
    }
}
