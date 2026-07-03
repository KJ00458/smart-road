//! Hard-coded waypoint paths for every (Arm, Turn) combination.
//! Each path is a &'static [(f64,f64)] slice the vehicle follows in order.
//! A vehicle advances to the next waypoint when it is within SNAP pixels.

use crate::config::*;

pub const SNAP: f64 = 6.0; // px — close enough to advance waypoint

// ── Helpers: named lane centre coordinates ───────────────────────────────
// Columns (X centres), left→right inside the 300-px road strip
//   col(0)=IX+25  col(1)=IX+75  col(2)=IX+125
//   col(3)=IX+175 col(4)=IX+225 col(5)=IX+275
// Rows (Y centres), top→bottom inside the 300-px road strip (same offsets)
//
// Inbound/outbound assignments:
//   North arm (going South): inbound cols 0-1-2  (left half)
//   South arm (going North): inbound cols 5-4-3  (right half, mirrored)
//   West  arm (going East):  inbound rows 5-4-3  (bottom half)
//   East  arm (going West):  inbound rows 0-1-2  (top half)
//
// Lane index per turn:  West(left)=0  Forward=1  East(right)=2

const fn col(i: usize) -> f64 { IX + i as f64 * TILE + TILE / 2.0 }
const fn row(i: usize) -> f64 { IY + i as f64 * TILE + TILE / 2.0 }

// Spawn / exit positions (off-screen)
const fn top(x: f64)    -> f64 { IY - OFF }
const fn bot(x: f64)    -> f64 { IY + ROAD + OFF }
const fn left(y: f64)   -> f64 { IX - OFF }
const fn right(y: f64)  -> f64 { IX + ROAD + OFF }

// ── Waypoint tables ───────────────────────────────────────────────────────
//
// Format: list of (x, y) waypoints from spawn to exit.
// The vehicle is placed at waypoint[0] and removed after passing waypoint[last].

// ─── NORTH arm (entering from top, moving South) ──────────────────────

// North → West (left turn): inbound col0, exit top of col5 heading North
pub const PATH_N_W: &[(f64,f64)] = &[
    (col(0), top(0.0)),           // spawn above
    (col(0), row(4)),             // deep into intersection
    (col(5), row(4)),             // pivot to outbound row4 going right… wait: exit West side
    // Actually exit left side — West exit uses col5 going North out the West arm
    // Corrected: left turn from North goes out the West arm (left side of screen)
    (col(0), row(4)),
    (left(0.0), row(4)),          // exit off left edge
];

// North → Forward: inbound col1, exit bottom of col4 heading South
pub const PATH_N_F: &[(f64,f64)] = &[
    (col(1), top(0.0)),
    (col(1), bot(0.0)),
];

// North → East (right turn): inbound col2, exit right edge at row2
pub const PATH_N_E: &[(f64,f64)] = &[
    (col(2), top(0.0)),
    (col(2), row(2)),             // enter intersection, snap right
    (right(0.0), row(2)),        // exit right edge
];

// ─── SOUTH arm (entering from bottom, moving North) ─────────────────────

// South → West (left turn): inbound col5 going North, exit right edge at row3
pub const PATH_S_W: &[(f64,f64)] = &[
    (col(5), bot(0.0)),
    (col(5), row(1)),
    (right(0.0), row(1)),
];

// South → Forward: inbound col4, exit top of col3
pub const PATH_S_F: &[(f64,f64)] = &[
    (col(4), bot(0.0)),
    (col(4), top(0.0)),
];

// South → East (right turn): inbound col3 going North, exit left edge at row3
pub const PATH_S_E: &[(f64,f64)] = &[
    (col(3), bot(0.0)),
    (col(3), row(3)),
    (left(0.0), row(3)),
];

// ─── WEST arm (entering from left, moving East) ────────────────────────

// West → West (left turn): inbound row5, exit bottom edge at col1
pub const PATH_W_W: &[(f64,f64)] = &[
    (left(0.0), row(5)),
    (col(1), row(5)),
    (col(1), bot(0.0)),
];

// West → Forward: inbound row4, exit right edge at row4
pub const PATH_W_F: &[(f64,f64)] = &[
    (left(0.0), row(4)),
    (right(0.0), row(4)),
];

// West → East (right turn): inbound row3, exit top edge at col4
pub const PATH_W_E: &[(f64,f64)] = &[
    (left(0.0), row(3)),
    (col(4), row(3)),
    (col(4), top(0.0)),
];

// ─── EAST arm (entering from right, moving West) ───────────────────────

// East → West (left turn): inbound row0, exit bottom edge at col4
pub const PATH_E_W: &[(f64,f64)] = &[
    (right(0.0), row(0)),
    (col(4), row(0)),
    (col(4), bot(0.0)),
];

// East → Forward: inbound row1, exit left edge at row1
pub const PATH_E_F: &[(f64,f64)] = &[
    (right(0.0), row(1)),
    (left(0.0), row(1)),
];

// East → East (right turn): inbound row2, exit top edge at col1
pub const PATH_E_E: &[(f64,f64)] = &[
    (right(0.0), row(2)),
    (col(1), row(2)),
    (col(1), top(0.0)),
];

// ── Path selector ─────────────────────────────────────────────────────────

use crate::vehicle::{Arm, Turn};

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
