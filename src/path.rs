//! Hard-coded waypoint paths — RIGHT-HAND driving, 6-lane grid.
//!
//! Window 900x900. Road 300px wide (IX=300..600, IY=300..600).
//! Lane centres at IX+25, IX+75, IX+125, IX+175, IX+225, IX+275
//!            = 325, 375, 425, 475, 525, 575
//!
//! RIGHT-HAND convention — inbound on the RIGHT half:
//!
//!  North arm (spawn y=240, moving SOUTH):
//!    Right  → lane col3 x=475  → turn East  (exit y=425)
//!    Fwd    → lane col4 x=525  → straight S (exit y=660)
//!    Left   → lane col5 x=575  → turn West  (exit y=575)
//!
//!  South arm (spawn y=660, moving NORTH):
//!    Right  → lane col2 x=425  → turn West  (exit y=475)
//!    Fwd    → lane col1 x=375  → straight N (exit y=240)
//!    Left   → lane col0 x=325  → turn East  (exit y=325)
//!
//!  West arm (spawn x=240, moving EAST):
//!    Right  → lane row3 y=475  → turn North (exit x=475)
//!    Fwd    → lane row4 y=525  → straight E (exit x=660)
//!    Left   → lane row5 y=575  → turn South (exit x=575)
//!
//!  East arm (spawn x=660, moving WEST):
//!    Right  → lane row2 y=425  → turn South (exit x=425)
//!    Fwd    → lane row1 y=375  → straight W (exit x=240)
//!    Left   → lane row0 y=325  → turn North (exit x=325)
//!
//! All paths include an off-screen spawn point and an off-screen exit point
//! so vehicles are created/destroyed outside the visible road.

use crate::vehicle::{Arm, Turn};

// ─── NORTH arm ──────────────────────────────────────────────────────────────
const PATH_N_R: &[(f64,f64)] = &[(475.0,240.0),(475.0,425.0),(660.0,425.0)];
const PATH_N_F: &[(f64,f64)] = &[(525.0,240.0),(525.0,660.0)];
const PATH_N_L: &[(f64,f64)] = &[(575.0,240.0),(575.0,575.0),(240.0,575.0)];

// ─── SOUTH arm ──────────────────────────────────────────────────────────────
const PATH_S_R: &[(f64,f64)] = &[(425.0,660.0),(425.0,475.0),(240.0,475.0)];
const PATH_S_F: &[(f64,f64)] = &[(375.0,660.0),(375.0,240.0)];
const PATH_S_L: &[(f64,f64)] = &[(325.0,660.0),(325.0,325.0),(660.0,325.0)];

// ─── WEST arm ───────────────────────────────────────────────────────────────
const PATH_W_R: &[(f64,f64)] = &[(240.0,475.0),(475.0,475.0),(475.0,240.0)];
const PATH_W_F: &[(f64,f64)] = &[(240.0,525.0),(660.0,525.0)];
const PATH_W_L: &[(f64,f64)] = &[(240.0,575.0),(575.0,575.0),(575.0,660.0)];

// ─── EAST arm ───────────────────────────────────────────────────────────────
const PATH_E_R: &[(f64,f64)] = &[(660.0,425.0),(425.0,425.0),(425.0,660.0)];
const PATH_E_F: &[(f64,f64)] = &[(660.0,375.0),(240.0,375.0)];
const PATH_E_L: &[(f64,f64)] = &[(660.0,325.0),(325.0,325.0),(325.0,240.0)];

// ─── Selector ────────────────────────────────────────────────────────────────
pub fn get_path(arm: Arm, turn: Turn) -> &'static [(f64,f64)] {
    match (arm, turn) {
        (Arm::North, Turn::Right)   => PATH_N_R,
        (Arm::North, Turn::Forward) => PATH_N_F,
        (Arm::North, Turn::Left)    => PATH_N_L,
        (Arm::South, Turn::Right)   => PATH_S_R,
        (Arm::South, Turn::Forward) => PATH_S_F,
        (Arm::South, Turn::Left)    => PATH_S_L,
        (Arm::West,  Turn::Right)   => PATH_W_R,
        (Arm::West,  Turn::Forward) => PATH_W_F,
        (Arm::West,  Turn::Left)    => PATH_W_L,
        (Arm::East,  Turn::Right)   => PATH_E_R,
        (Arm::East,  Turn::Forward) => PATH_E_F,
        (Arm::East,  Turn::Left)    => PATH_E_L,
    }
}
