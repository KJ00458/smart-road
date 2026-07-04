//! Hard-coded waypoint paths — RIGHT-HAND driving (US style), 6-lane grid.
//!
//! Window 1100×1100.
//! Road strip: IX=370..730 (horiz), IY=370..730 (vert), width=360px.
//! 6 lanes of 60px each. Lane centres:
//!   col/row 0→400, 1→460, 2→520, 3→580, 4→640, 5→700
//!
//! RIGHT-HAND TRAFFIC (drive on the right, like the US):
//!   When you face the direction of travel, INBOUND lanes are on YOUR RIGHT.
//!
//! ┌──────────────────────────────────────────────────────────────────────┐
//! │  NORTH arm — cars enter TOP, travel SOUTH (↓):                      │
//! │    Facing south, right side = LOW x columns                          │
//! │    col 0 x=400  right-turn lane  → exits East  (row 3, y=580)       │
//! │    col 1 x=460  straight lane    → exits South (y=810+OFF)           │
//! │    col 2 x=520  left-turn lane   → exits West  (row 5, y=700)       │
//! │    Spawn y = IY - OFF = 290  (off-screen above)                      │
//! │                                                                       │
//! │  SOUTH arm — cars enter BOTTOM, travel NORTH (↑):                   │
//! │    Facing north, right side = HIGH x columns                         │
//! │    col 5 x=700  right-turn lane  → exits West  (row 2, y=520)       │
//! │    col 4 x=640  straight lane    → exits North (y=IY-OFF)            │
//! │    col 3 x=580  left-turn lane   → exits East  (row 5, y=700)       │
//! │    Spawn y = IY + ROAD + OFF = 810 (off-screen below)               │
//! │                                                                       │
//! │  WEST arm — cars enter LEFT, travel EAST (→):                       │
//! │    Facing east, right side = HIGH y rows                             │
//! │    row 3 y=580  right-turn lane  → exits North (col 0, x=400)       │
//! │    row 4 y=640  straight lane    → exits East  (x=IX+ROAD+OFF)      │
//! │    row 5 y=700  left-turn lane   → exits South (col 2, x=520)       │
//! │    Spawn x = IX - OFF = 290  (off-screen left)                       │
//! │                                                                       │
//! │  EAST arm — cars enter RIGHT, travel WEST (←):                      │
//! │    Facing west, right side = LOW y rows                              │
//! │    row 0 y=400  right-turn lane  → exits North (col 5, x=700)       │
//! │    row 1 y=460  straight lane    → exits West  (x=IX-OFF)            │
//! │    row 2 y=520  left-turn lane   → exits South (col 3, x=580)       │
//! │    Spawn x = IX + ROAD + OFF = 810 (off-screen right)               │
//! └──────────────────────────────────────────────────────────────────────┘

use crate::vehicle::{Arm, Turn};
use crate::config::{IX, IY, ROAD, OFF};

// Spawn points (off-screen)
const SPAWN_N: f64 = IY - OFF;           // 290
const SPAWN_S: f64 = IY + ROAD + OFF;    // 810
const SPAWN_W: f64 = IX - OFF;           // 290
const SPAWN_E: f64 = IX + ROAD + OFF;    // 810

// Exit points (off-screen, 80px past road edge)
const EXIT_N: f64 = IY - OFF;            // 290
const EXIT_S: f64 = IY + ROAD + OFF;     // 810
const EXIT_W: f64 = IX - OFF;            // 290
const EXIT_E: f64 = IX + ROAD + OFF;     // 810

// ── Inbound lane centres (RIGHT-HAND: right side of direction of travel) ──

// North inbound (travelling south ↓ → right side = LOW x)
const N_RT: f64 = 400.0; // col 0 — right turn
const N_ST: f64 = 460.0; // col 1 — straight
const N_LT: f64 = 520.0; // col 2 — left turn

// South inbound (travelling north ↑ → right side = HIGH x)
const S_RT: f64 = 700.0; // col 5 — right turn
const S_ST: f64 = 640.0; // col 4 — straight
const S_LT: f64 = 580.0; // col 3 — left turn

// West inbound (travelling east → → right side = HIGH y)
const W_RT: f64 = 580.0; // row 3 — right turn
const W_ST: f64 = 640.0; // row 4 — straight
const W_LT: f64 = 700.0; // row 5 — left turn

// East inbound (travelling west ← → right side = LOW y)
const E_RT: f64 = 400.0; // row 0 — right turn
const E_ST: f64 = 460.0; // row 1 — straight
const E_LT: f64 = 520.0; // row 2 — left turn

// ── NORTH arm (spawn top y=290, travel South ↓) ───────────────────────────
// Right turn → col 0 (x=400), pivot into East outbound row 3 (y=580)
const PATH_N_R: &[(f64,f64)] = &[
    (N_RT, SPAWN_N),   // (400, 290) spawn
    (N_RT, W_RT),      // (400, 580) pivot row — East outbound row 3
    (EXIT_E, W_RT),    // (810, 580) exit east
];
// Straight → col 1 (x=460), continue south
const PATH_N_F: &[(f64,f64)] = &[
    (N_ST, SPAWN_N),   // (460, 290)
    (N_ST, EXIT_S),    // (460, 810)
];
// Left turn → col 2 (x=520), cross, pivot into West outbound row 5 (y=700)
const PATH_N_L: &[(f64,f64)] = &[
    (N_LT, SPAWN_N),   // (520, 290)
    (N_LT, W_LT),      // (520, 700) pivot row — West outbound row 5
    (EXIT_W, W_LT),    // (290, 700) exit west
];

// ── SOUTH arm (spawn bottom y=810, travel North ↑) ────────────────────────
// Right turn → col 5 (x=700), pivot into West outbound row 2 (y=520)
const PATH_S_R: &[(f64,f64)] = &[
    (S_RT, SPAWN_S),   // (700, 810) spawn
    (S_RT, E_LT),      // (700, 520) pivot row — West outbound row 2
    (EXIT_W, E_LT),    // (290, 520) exit west
];
// Straight → col 4 (x=640), continue north
const PATH_S_F: &[(f64,f64)] = &[
    (S_ST, SPAWN_S),   // (640, 810)
    (S_ST, EXIT_N),    // (640, 290)
];
// Left turn → col 3 (x=580), cross, pivot into East outbound row 5 (y=700)
const PATH_S_L: &[(f64,f64)] = &[
    (S_LT, SPAWN_S),   // (580, 810)
    (S_LT, W_LT),      // (580, 700) pivot row — East outbound row 5
    (EXIT_E, W_LT),    // (810, 700) exit east
];

// ── WEST arm (spawn left x=290, travel East →) ────────────────────────────
// Right turn → row 3 (y=580), pivot into North outbound col 0 (x=400)
const PATH_W_R: &[(f64,f64)] = &[
    (SPAWN_W, W_RT),   // (290, 580) spawn
    (N_RT, W_RT),      // (400, 580) pivot col — North outbound col 0
    (N_RT, EXIT_N),    // (400, 290) exit north
];
// Straight → row 4 (y=640), continue east
const PATH_W_F: &[(f64,f64)] = &[
    (SPAWN_W, W_ST),   // (290, 640)
    (EXIT_E, W_ST),    // (810, 640)
];
// Left turn → row 5 (y=700), cross, pivot into South outbound col 2 (x=520)
const PATH_W_L: &[(f64,f64)] = &[
    (SPAWN_W, W_LT),   // (290, 700)
    (N_LT, W_LT),      // (520, 700) pivot col — South outbound col 2
    (N_LT, EXIT_S),    // (520, 810) exit south
];

// ── EAST arm (spawn right x=810, travel West ←) ───────────────────────────
// Right turn → row 0 (y=400), pivot into North outbound col 5 (x=700)
const PATH_E_R: &[(f64,f64)] = &[
    (SPAWN_E, E_RT),   // (810, 400) spawn
    (S_RT, E_RT),      // (700, 400) pivot col — North outbound col 5
    (S_RT, EXIT_N),    // (700, 290) exit north
];
// Straight → row 1 (y=460), continue west
const PATH_E_F: &[(f64,f64)] = &[
    (SPAWN_E, E_ST),   // (810, 460)
    (EXIT_W, E_ST),    // (290, 460)
];
// Left turn → row 2 (y=520), cross, pivot into South outbound col 3 (x=580)
const PATH_E_L: &[(f64,f64)] = &[
    (SPAWN_E, E_LT),   // (810, 520)
    (S_LT, E_LT),      // (580, 520) pivot col — South outbound col 3
    (S_LT, EXIT_S),    // (580, 810) exit south
];

// ── Selector ──────────────────────────────────────────────────────────────
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
