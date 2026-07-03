//! Hard-coded waypoint paths for every (Arm, Turn) combination.
//! All coordinates are plain f64 literals — no const fn float arithmetic.

use crate::vehicle::{Arm, Turn};

// Grid constants (mirrored from config, spelled out as literals)
// Window: 900x900. Road: 300px wide (6 lanes * 50px). Starts at x/y = 300.
// Lane centres:  col(i) = 300 + i*50 + 25
//   col0=325  col1=375  col2=425  col3=475  col4=525  col5=575
// Row centres:   row(i) = 300 + i*50 + 25  (same values, different axis)
//   row0=325  row1=375  row2=425  row3=475  row4=525  row5=575
// Off-screen spawn: top=240, bottom=660, left=240, right=660

// ─── NORTH arm (spawn top, move South) ────────────────────────────────────
// Left turn  → inbound col0 (x=325), pivot at row4 (y=525), exit West (x=240)
const PATH_N_W: &[(f64,f64)] = &[
    (325.0, 240.0),
    (325.0, 525.0),
    (240.0, 525.0),
];
// Straight    → inbound col1 (x=375), exit bottom (y=660)
const PATH_N_F: &[(f64,f64)] = &[
    (375.0, 240.0),
    (375.0, 660.0),
];
// Right turn  → inbound col2 (x=425), pivot at row2 (y=425), exit East (x=660)
const PATH_N_E: &[(f64,f64)] = &[
    (425.0, 240.0),
    (425.0, 425.0),
    (660.0, 425.0),
];

// ─── SOUTH arm (spawn bottom, move North) ─────────────────────────────────
// Left turn  → inbound col5 (x=575), pivot at row1 (y=375), exit East (x=660)
const PATH_S_W: &[(f64,f64)] = &[
    (575.0, 660.0),
    (575.0, 375.0),
    (660.0, 375.0),
];
// Straight   → inbound col4 (x=525), exit top (y=240)
const PATH_S_F: &[(f64,f64)] = &[
    (525.0, 660.0),
    (525.0, 240.0),
];
// Right turn → inbound col3 (x=475), pivot at row3 (y=475), exit West (x=240)
const PATH_S_E: &[(f64,f64)] = &[
    (475.0, 660.0),
    (475.0, 475.0),
    (240.0, 475.0),
];

// ─── WEST arm (spawn left, move East) ─────────────────────────────────────
// Left turn  → inbound row5 (y=575), pivot at col1 (x=375), exit South (y=660)
const PATH_W_W: &[(f64,f64)] = &[
    (240.0, 575.0),
    (375.0, 575.0),
    (375.0, 660.0),
];
// Straight   → inbound row4 (y=525), exit right (x=660)
const PATH_W_F: &[(f64,f64)] = &[
    (240.0, 525.0),
    (660.0, 525.0),
];
// Right turn → inbound row3 (y=475), pivot at col4 (x=525), exit North (y=240)
const PATH_W_E: &[(f64,f64)] = &[
    (240.0, 475.0),
    (525.0, 475.0),
    (525.0, 240.0),
];

// ─── EAST arm (spawn right, move West) ────────────────────────────────────
// Left turn  → inbound row0 (y=325), pivot at col4 (x=525), exit South (y=660)
const PATH_E_W: &[(f64,f64)] = &[
    (660.0, 325.0),
    (525.0, 325.0),
    (525.0, 660.0),
];
// Straight   → inbound row1 (y=375), exit left (x=240)
const PATH_E_F: &[(f64,f64)] = &[
    (660.0, 375.0),
    (240.0, 375.0),
];
// Right turn → inbound row2 (y=425), pivot at col1 (x=375), exit North (y=240)
const PATH_E_E: &[(f64,f64)] = &[
    (660.0, 425.0),
    (375.0, 425.0),
    (375.0, 240.0),
];

// ─── Selector ─────────────────────────────────────────────────────────────

pub fn get_path(arm: Arm, turn: Turn) -> &'static [(f64,f64)] {
    match (arm, turn) {
        (Arm::North, Turn::West)    => PATH_N_W,
        (Arm::North, Turn::Forward) => PATH_N_F,
        (Arm::North, Turn::East)    => PATH_N_E,
        (Arm::South, Turn::West)    => PATH_S_W,
        (Arm::South, Turn::Forward) => PATH_S_F,
        (Arm::South, Turn::East)    => PATH_S_E,
        (Arm::West,  Turn::West)    => PATH_W_W,
        (Arm::West,  Turn::Forward) => PATH_W_F,
        (Arm::West,  Turn::East)    => PATH_W_E,
        (Arm::East,  Turn::West)    => PATH_E_W,
        (Arm::East,  Turn::Forward) => PATH_E_F,
        (Arm::East,  Turn::East)    => PATH_E_E,
    }
}
