//! Hard-coded waypoint paths — RIGHT-HAND driving, 6-lane grid.
//!
//! Window 1100x1100. Road 360px wide (IX=370..730, IY=370..730).
//! Lane centres: IX+30, IX+90, IX+150, IX+210, IX+270, IX+330
//!             = 400, 460, 520, 580, 640, 700
//!
//! Spawn/exit off-screen at margin OFF=80:
//!   North/South: y = IY-OFF = 290  and  y = IY+ROAD+OFF = 810
//!   West/East  : x = IX-OFF = 290  and  x = IX+ROAD+OFF = 810
//!
//! RIGHT-HAND inbound = RIGHT half of road:
//!  North arm (moving SOUTH, spawn y=290):
//!    Right   col3 x=580 → pivot at row2 y=520 → exit East  x=810
//!    Forward col4 x=640 → straight S → exit y=810
//!    Left    col5 x=700 → pivot at row5 y=700 → exit West  x=290
//!
//!  South arm (moving NORTH, spawn y=810):
//!    Right   col2 x=520 → pivot at row3 y=580 → exit West  x=290
//!    Forward col1 x=460 → straight N → exit y=290
//!    Left    col0 x=400 → pivot at row0 y=400 → exit East  x=810
//!
//!  West arm (moving EAST, spawn x=290):
//!    Right   row3 y=580 → pivot at col3 x=580 → exit North y=290
//!    Forward row4 y=640 → straight E → exit x=810
//!    Left    row5 y=700 → pivot at col5 x=700 → exit South y=810
//!
//!  East arm (moving WEST, spawn x=810):
//!    Right   row2 y=520 → pivot at col2 x=520 → exit South y=810
//!    Forward row1 y=460 → straight W → exit x=290
//!    Left    row0 y=400 → pivot at col0 x=400 → exit North y=290

use crate::vehicle::{Arm, Turn};

// ─── NORTH ─────────────────────────────────────────────────────────────────
const PATH_N_R: &[(f64,f64)] = &[(580.0,290.0),(580.0,520.0),(810.0,520.0)];
const PATH_N_F: &[(f64,f64)] = &[(640.0,290.0),(640.0,810.0)];
const PATH_N_L: &[(f64,f64)] = &[(700.0,290.0),(700.0,700.0),(290.0,700.0)];

// ─── SOUTH ─────────────────────────────────────────────────────────────────
const PATH_S_R: &[(f64,f64)] = &[(520.0,810.0),(520.0,580.0),(290.0,580.0)];
const PATH_S_F: &[(f64,f64)] = &[(460.0,810.0),(460.0,290.0)];
const PATH_S_L: &[(f64,f64)] = &[(400.0,810.0),(400.0,400.0),(810.0,400.0)];

// ─── WEST ──────────────────────────────────────────────────────────────────
const PATH_W_R: &[(f64,f64)] = &[(290.0,580.0),(580.0,580.0),(580.0,290.0)];
const PATH_W_F: &[(f64,f64)] = &[(290.0,640.0),(810.0,640.0)];
const PATH_W_L: &[(f64,f64)] = &[(290.0,700.0),(700.0,700.0),(700.0,810.0)];

// ─── EAST ──────────────────────────────────────────────────────────────────
const PATH_E_R: &[(f64,f64)] = &[(810.0,520.0),(520.0,520.0),(520.0,810.0)];
const PATH_E_F: &[(f64,f64)] = &[(810.0,460.0),(290.0,460.0)];
const PATH_E_L: &[(f64,f64)] = &[(810.0,400.0),(400.0,400.0),(400.0,290.0)];

// ─── Selector ──────────────────────────────────────────────────────────────
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
