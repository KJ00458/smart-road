//! Hard-coded waypoint paths — RIGHT-HAND driving, 6-lane grid.
//!
//! Window 1100×1100.
//! Road strip: IX=370..730 (horiz), IY=370..730 (vert), width=360px.
//! 6 lanes of 60px each. Lane centres (left→right / top→bottom):
//!   col/row 0→400, 1→460, 2→520, 3→580, 4→640, 5→700
//!
//! RIGHT-HAND TRAFFIC RULES (like the US):
//!   - Cars drive on the RIGHT side of the road.
//!   - Looking at a road arm, INBOUND = the lanes on the RIGHT half.
//!   - OUTBOUND (exit) = the lanes on the LEFT half.
//!
//! ┌─────────────────────────────────────────────────────────────────┐
//! │  NORTH arm — cars enter from TOP, travel SOUTH (↓):           │
//! │    RIGHT side of southward travel = columns with HIGH x        │
//! │    col 3 x=580  right-turn lane   → exits East  (row 2 y=520) │
//! │    col 4 x=640  straight lane     → exits South (y=810+OFF)   │
//! │    col 5 x=700  left-turn lane    → exits West  (row 5 y=700) │
//! │    Spawn y = IY - OFF = 290  (off-screen above)               │
//! │                                                                 │
//! │  SOUTH arm — cars enter from BOTTOM, travel NORTH (↑):        │
//! │    RIGHT side of northward travel = columns with LOW x         │
//! │    col 2 x=520  right-turn lane   → exits West  (row 5 y=700) │
//! │    col 1 x=460  straight lane     → exits North (y=IY-OFF)    │
//! │    col 0 x=400  left-turn lane    → exits East  (row 2 y=520) │
//! │    Spawn y = IY + ROAD + OFF = 810 (off-screen below)         │
//! │                                                                 │
//! │  WEST arm — cars enter from LEFT, travel EAST (→):            │
//! │    RIGHT side of eastward travel = rows with HIGH y            │
//! │    row 3 y=580  right-turn lane   → exits North (col 3 x=580) │
//! │    row 4 y=640  straight lane     → exits East  (x=IX+ROAD+OFF)│
//! │    row 5 y=700  left-turn lane    → exits South (col 5 x=700) │
//! │    Spawn x = IX - OFF = 290  (off-screen left)                │
//! │                                                                 │
//! │  EAST arm — cars enter from RIGHT, travel WEST (←):           │
//! │    RIGHT side of westward travel = rows with LOW y             │
//! │    row 2 y=520  right-turn lane   → exits South (col 2 x=520) │
//! │    row 1 y=460  straight lane     → exits West  (x=IX-OFF)    │
//! │    row 0 y=400  left-turn lane    → exits North (col 0 x=400) │
//! │    Spawn x = IX + ROAD + OFF = 810 (off-screen right)         │
//! └─────────────────────────────────────────────────────────────────┘
//!
//! Exit waypoints go 80px PAST the road edge so cars fully leave screen.
//! Outbound (exit) lane centres:
//!   North outbound (going ↑): cols 0,1,2  x=400,460,520
//!   South outbound (going ↓): cols 3,4,5  x=580,640,700
//!   West  outbound (going ←): rows 0,1,2  y=400,460,520
//!   East  outbound (going →): rows 3,4,5  y=580,640,700

use crate::vehicle::{Arm, Turn};
use crate::config::{IX, IY, ROAD, OFF};

// Precomputed geometry constants (same values as config)
const SPAWN_N: f64 = IY - OFF;           // 290  (top, off-screen)
const SPAWN_S: f64 = IY + ROAD + OFF;    // 810  (bottom, off-screen)
const SPAWN_W: f64 = IX - OFF;           // 290  (left, off-screen)
const SPAWN_E: f64 = IX + ROAD + OFF;    // 810  (right, off-screen)

// Exit coordinates (80px past road edge)
const EXIT_N: f64 = IY - OFF;            // 290  (exit upward)
const EXIT_S: f64 = IY + ROAD + OFF;     // 810  (exit downward)
const EXIT_W: f64 = IX - OFF;            // 290  (exit leftward)
const EXIT_E: f64 = IX + ROAD + OFF;     // 810  (exit rightward)

// Inbound lane centres — RIGHT-HAND traffic
// North/South inbound cols: 3=580, 4=640, 5=700
const N_RT: f64 = 580.0; // North right-turn inbound col
const N_ST: f64 = 640.0; // North straight    inbound col
const N_LT: f64 = 700.0; // North left-turn   inbound col
// South inbound cols: 2=520, 1=460, 0=400
const S_RT: f64 = 520.0; // South right-turn
const S_ST: f64 = 460.0; // South straight
const S_LT: f64 = 400.0; // South left-turn
// West inbound rows: 3=580, 4=640, 5=700
const W_RT: f64 = 580.0;
const W_ST: f64 = 640.0;
const W_LT: f64 = 700.0;
// East inbound rows: 2=520, 1=460, 0=400
const E_RT: f64 = 520.0;
const E_ST: f64 = 460.0;
const E_LT: f64 = 400.0;

// Outbound (exit) lane centres — the LEFT half of each arm
// North outbound (cars going ↑ out): cols 0,1,2 → x=400,460,520
// South outbound (cars going ↓ out): cols 3,4,5 → x=580,640,700  (same x as N inbound)
// West  outbound (cars going ← out): rows 0,1,2 → y=400,460,520
// East  outbound (cars going → out): rows 3,4,5 → y=580,640,700  (same y as W inbound)
//
// Turn target lane selection (matching destination arm's outbound lanes):
//   N→E right turn  exits on East outbound row nearest = row 2 y=520
//   N→S straight    exits on South outbound col 4 x=640 (same x, just continues)
//   N→W left turn   exits on West outbound row 5 y=700 (closest left-turn target)
//
//   S→W right turn  exits on West outbound row 5 y=700  ← rightmost outbound for westbound
//   S→N straight    exits on North outbound col 1 x=460 (same x)
//   S→E left turn   exits on East outbound row 2 y=520
//
//   W→N right turn  exits on North outbound col 3 x=580 ← nearest outbound northward
//   W→E straight    exits on East outbound row 4 y=640 (same y)
//   W→S left turn   exits on South outbound col 5 x=700
//
//   E→S right turn  exits on South outbound col 2 x=520 ← nearest outbound southward
//   E→W straight    exits on West outbound row 1 y=460 (same y)
//   E→N left turn   exits on North outbound col 0 x=400

// ─── NORTH arm (spawn top y=290, travel South ↓) ─────────────────────────
// Right turn → stay right, pivot into East outbound row 2 (y=520), exit right
const PATH_N_R: &[(f64,f64)] = &[
    (N_RT, SPAWN_N),   // spawn off-screen top, inbound col 3
    (N_RT, E_RT),      // travel south to pivot row (y=520)
    (EXIT_E, E_RT),    // exit eastward off-screen right
];
// Straight → continue south through intersection, exit bottom
const PATH_N_F: &[(f64,f64)] = &[
    (N_ST, SPAWN_N),   // spawn off-screen top, inbound col 4
    (N_ST, EXIT_S),    // travel straight south and exit
];
// Left turn → cross intersection, pivot into West outbound row 5 (y=700), exit left
const PATH_N_L: &[(f64,f64)] = &[
    (N_LT, SPAWN_N),   // spawn off-screen top, inbound col 5
    (N_LT, W_LT),      // travel south to pivot row (y=700)
    (EXIT_W, W_LT),    // exit westward off-screen left
];

// ─── SOUTH arm (spawn bottom y=810, travel North ↑) ──────────────────────
// Right turn → stay right, pivot into West outbound row 5 (y=700), exit left
const PATH_S_R: &[(f64,f64)] = &[
    (S_RT, SPAWN_S),   // spawn off-screen bottom, inbound col 2
    (S_RT, W_LT),      // travel north to pivot row (y=700)
    (EXIT_W, W_LT),    // exit westward
];
// Straight → continue north, exit top
const PATH_S_F: &[(f64,f64)] = &[
    (S_ST, SPAWN_S),   // spawn off-screen bottom, inbound col 1
    (S_ST, EXIT_N),    // travel straight north and exit
];
// Left turn → cross intersection, pivot into East outbound row 2 (y=520), exit right
const PATH_S_L: &[(f64,f64)] = &[
    (S_LT, SPAWN_S),   // spawn off-screen bottom, inbound col 0
    (S_LT, E_RT),      // travel north to pivot row (y=520)
    (EXIT_E, E_RT),    // exit eastward
];

// ─── WEST arm (spawn left x=290, travel East →) ──────────────────────────
// Right turn → stay right, pivot into North outbound col 3 (x=580), exit top
const PATH_W_R: &[(f64,f64)] = &[
    (SPAWN_W, W_RT),   // spawn off-screen left, inbound row 3
    (N_RT, W_RT),      // travel east to pivot col (x=580)
    (N_RT, EXIT_N),    // exit northward
];
// Straight → continue east, exit right
const PATH_W_F: &[(f64,f64)] = &[
    (SPAWN_W, W_ST),   // spawn off-screen left, inbound row 4
    (EXIT_E, W_ST),    // travel straight east and exit
];
// Left turn → cross intersection, pivot into South outbound col 5 (x=700), exit bottom
const PATH_W_L: &[(f64,f64)] = &[
    (SPAWN_W, W_LT),   // spawn off-screen left, inbound row 5
    (N_LT, W_LT),      // travel east to pivot col (x=700)
    (N_LT, EXIT_S),    // exit southward
];

// ─── EAST arm (spawn right x=810, travel West ←) ─────────────────────────
// Right turn → stay right, pivot into South outbound col 2 (x=520), exit bottom
const PATH_E_R: &[(f64,f64)] = &[
    (SPAWN_E, E_RT),   // spawn off-screen right, inbound row 2
    (S_RT, E_RT),      // travel west to pivot col (x=520)
    (S_RT, EXIT_S),    // exit southward
];
// Straight → continue west, exit left
const PATH_E_F: &[(f64,f64)] = &[
    (SPAWN_E, E_ST),   // spawn off-screen right, inbound row 1
    (EXIT_W, E_ST),    // travel straight west and exit
];
// Left turn → cross intersection, pivot into North outbound col 0 (x=400), exit top
const PATH_E_L: &[(f64,f64)] = &[
    (SPAWN_E, E_LT),   // spawn off-screen right, inbound row 0
    (S_LT, E_LT),      // travel west to pivot col (x=400)
    (S_LT, EXIT_N),    // exit northward
];

// ─── Selector ─────────────────────────────────────────────────────────────
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
